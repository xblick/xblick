"""Every program declares itself on the network: which program a Zenoh session is, so that the terminal interface
names it by its program and its camera or node, not by the keys it declared.

    session = open_from_args(args)                    # declares the program (session.open_session)
    declaration.add_name(session, "test/camera/yolox")  # node.Reporter and the publishers do it themselves
    programs = declaration.query(session)             # zid -> Program, of every session that declares itself

A session opened by ``session.open_session`` (and so by ``open_from_args``, every ``xblick`` command and every node
on the library) declares at once, below ``xblick/sessions/<its zid>`` (``topics.SESSIONS``, protocol.hpp):

* the liveliness token ``.../program`` while the session is open: a program appears and disappears at once (a killed
  one when its links close, which the kernel does for a killed process);
* a queryable on the same key that answers the program record at once (JSON, ``record``): program, its version and
  the core's, its camera or node names, hostname, the whole command line, process id, start time.

The command line is visible to everything that reaches the network: paths and arguments included, never the
environment. Secrets do not belong in arguments. A program of Durchblick before 0.5.9, or another Zenoh program,
declares nothing; the interface shows ``–`` for it. ``xblick router`` carries the same record as its Zenoh metadata
(router.py), which the admin space shows.
"""

from __future__ import annotations

import dataclasses
import json
import os
import socket
import threading
import weakref
from importlib import metadata

import zenoh

from . import _core, topics

LANGUAGE = "python"
SERVE_PERIOD_S = 0.02  # how often the declaration looks for queries: an answer waits at most this much longer
ALL = f"{topics.SESSIONS}/*/{topics.PROGRAM}"  # every program's key


def key(zid: str) -> str:
    """``xblick/sessions/<zid>/program``: a session's declaration (token and queryable)."""
    return f"{topics.SESSIONS}/{zid}/{topics.PROGRAM}"


def zid_of(key_expr: str) -> str | None:
    """The zid of a declaration's key, None for another key."""
    prefix, suffix = topics.SESSIONS + "/", "/" + topics.PROGRAM
    if not (key_expr.startswith(prefix) and key_expr.endswith(suffix)):
        return None
    zid = key_expr[len(prefix):-len(suffix)]
    return zid if zid and "/" not in zid else None


def command_line() -> list[str]:
    """The process's whole command line, interpreter included (``/proc/self/cmdline``); nothing of the
    environment."""
    with open("/proc/self/cmdline", "rb") as f:
        return [a.decode(errors="replace") for a in f.read().split(b"\0")[:-1]]


def started_ns() -> int | None:
    """When the process started, in ns since the epoch (``/proc``: its start in clock ticks after the boot); None
    where the system does not say."""
    try:
        with open("/proc/self/stat") as f:
            fields = f.read().rsplit(")", 1)[1].split()  # after the command's name, which may hold spaces
        with open("/proc/stat") as f:
            boot = next(int(line.split()[1]) for line in f if line.startswith("btime "))
    except (OSError, StopIteration, IndexError, ValueError):
        return None
    ticks = int(fields[19])  # field 22 of stat(5); fields[0] is field 3
    return boot * 1_000_000_000 + ticks * 1_000_000_000 // os.sysconf("SC_CLK_TCK")


def program_version(program: str) -> str | None:
    """The version of a program: the package's for ``xblick`` and its commands (``xblick view``), else the version of
    the distribution whose console script has the program's name (a node: ``xblick-yolox``); None when no installed
    distribution names it (a script started by its path)."""
    first = program.split(" ", 1)[0]
    if first == "xblick":
        return _core.__version__
    for entry in metadata.entry_points(group="console_scripts", name=first):
        if entry.dist is not None:
            return entry.dist.version
    return None


def program_record(program: str, version: str | None) -> dict:
    """The record of this process as ``program`` (without the session's zid and with no names yet)."""
    return {"v": topics.PROGRAM_VERSION, "kind": "program", "program": program, "version": version,
            "xblick": _core.__version__, "language": LANGUAGE, "names": [], "host": socket.gethostname(),
            "command": command_line(), "pid": os.getpid(), "started_ns": started_ns()}


_declarations: weakref.WeakValueDictionary[str, Declaration] = weakref.WeakValueDictionary()  # zid -> its own


class Declaration:
    """A session's declaration (the module's docstring): the token and the queryable on ``key(zid)``, the record
    (``record``) with ``names`` added by ``add_name``. ``close`` undeclares both (the session's close does too)."""

    def __init__(self, session: zenoh.Session, program: str, version: str | None = None, names=()):
        self.zid = str(session.info.zid())
        self.key = key(self.zid)
        self._lock = threading.Lock()
        self._record = {"zid": self.zid, **program_record(program, version)}
        self._payload = b""
        for name in names:
            self.add_name(name)
        self._encode()
        # a channel, answered by a daemon thread of its own (as parameters.py): a Python callback would run in
        # zenoh-python's thread, which is no daemon, and a program that ends with an exception before it closes its
        # session would wait for it for good
        self._queryable = session.declare_queryable(self.key)
        self._stop = threading.Event()
        self._server = threading.Thread(target=self._serve, name=f"declaration {self.zid[:8]}", daemon=True)
        self._server.start()
        self._token = session.liveliness().declare_token(self.key)
        _declarations[self.zid] = self

    def record(self) -> dict:
        with self._lock:
            return json.loads(self._payload)

    def add_name(self, name: str):
        """A camera or node name of the program (``test/camera``, ``test/camera/yolox``), once."""
        name = name.strip("/")
        with self._lock:
            if name and name not in self._record["names"]:
                self._record["names"].append(name)
                self._encode()

    def _encode(self):
        self._payload = json.dumps(self._record, separators=(",", ":")).encode()

    def _serve(self):
        """The queries, answered as they come (looked for every SERVE_PERIOD_S), until close() or until the session
        closed (the channel closes with it: Zenoh's ZError "channel is empty and closed")."""
        while not self._stop.is_set():
            while True:
                try:
                    query = self._queryable.try_recv()
                except zenoh.ZError as error:
                    if "closed" in str(error):
                        return
                    raise
                if query is None:
                    break
                self._answer(query)
            self._stop.wait(SERVE_PERIOD_S)

    def _answer(self, query: zenoh.Query):
        # answered at once, and the query dropped right after (the receiver rule, AGENTS.md)
        with self._lock:
            payload = self._payload
        query.reply(self.key, payload, encoding=zenoh.Encoding.APPLICATION_JSON)
        query.drop()

    def close(self):
        if _declarations.get(self.zid) is self:
            del _declarations[self.zid]
        self._stop.set()
        self._token.undeclare()
        self._queryable.undeclare()
        self._server.join(1.0)


def of(session) -> Declaration | None:
    """The declaration of a session opened in this process (``open_session``), None for one that does not declare."""
    try:
        return _declarations.get(str(session.info.zid()))
    except zenoh.ZError:  # a closed session
        return None


def add_name(session, name: str) -> bool:
    """Adds a camera or node name to the declaration of ``session``; False when the session does not declare (opened
    without ``open_session``, or with ``declare=False``)."""
    declaration = of(session)
    if declaration is None:
        return False
    declaration.add_name(name)
    return True


@dataclasses.dataclass
class Program:
    """A program's record as it answered (``record``); ``host`` and ``names`` as the program said."""

    zid: str
    program: str
    version: str | None = None
    xblick: str = ""
    language: str = ""
    names: list[str] = dataclasses.field(default_factory=list)
    host: str = ""
    command: list[str] = dataclasses.field(default_factory=list)
    pid: int | None = None
    started_ns: int | None = None

    @classmethod
    def from_record(cls, record: dict, zid: str | None = None) -> Program:
        """ValueError for a record that is not one (another kind, a version this one does not read, no program)."""
        if not isinstance(record, dict) or record.get("kind") != "program":
            raise ValueError(f"not a program record: {str(record)[:80]}")
        if record.get("v") != topics.PROGRAM_VERSION:
            raise ValueError(f"a program record of version {record.get('v')} (this Durchblick reads "
                             f"{topics.PROGRAM_VERSION})")
        if not isinstance(record.get("program"), str) or not record["program"]:
            raise ValueError("a program record without its program")
        zid = zid if zid is not None else record.get("zid", "")
        return cls(zid, record["program"], record.get("version"), str(record.get("xblick") or ""),
                   str(record.get("language") or ""), [str(n) for n in record.get("names") or []],
                   str(record.get("host") or ""), [str(a) for a in record.get("command") or []], record.get("pid"),
                   record.get("started_ns"))

    @property
    def name(self) -> str:
        """Its camera or node names, joined; empty without one."""
        return ", ".join(self.names)


def query(session: zenoh.Session, timeout: float = 0.5) -> tuple[dict[str, Program], list[str]]:
    """(zid -> Program of every session that answered its declaration, the replies that were none: what they were).
    Costs the programs one reply each."""
    from .session import replies

    programs, errors = {}, []
    for reply in replies(session, ALL, timeout):
        if reply.ok is None:
            continue
        key_expr = str(reply.ok.key_expr)
        zid = zid_of(key_expr)
        try:
            if zid is None:
                raise ValueError("not a declaration's key")
            programs[zid] = Program.from_record(json.loads(reply.ok.payload.to_bytes()), zid)
        except ValueError as error:
            errors.append(f"{key_expr}: {error}")
    return programs, errors


def alive(session: zenoh.Session, timeout: float = 0.5) -> set[str]:
    """The zids whose declaration token is alive."""
    from .session import replies

    return {z for r in replies(session, ALL, timeout, liveliness=True) if r.ok is not None
            and (z := zid_of(str(r.ok.key_expr))) is not None}
