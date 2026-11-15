"""``xblick clean``: the rings on this PC (/dev/shm/xblick-<16 hex>), who made them, and which ones a killed
publisher left behind.

A publisher removes its ring when it ends. A publisher that is killed (SIGKILL, a crash) cannot, and its ring stays
in /dev/shm until the PC restarts. The command judges each ring by its creator, never by a name: a publisher may run
several times on purpose. Since 0.5.12 a ring records its creator (process id and start time) and the time of its
last frame, and its creator holds a lock on it for its lifetime (cpp/include/xblick/ring.hpp). A ring whose lock this
command can take has no living creator.

    xblick clean                          # list the rings and what --remove would remove; removes nothing
    xblick clean --remove                 # remove the rings whose creator is gone and that no process maps
    xblick clean --remove --force xblick-0123456789abcdef  # also this named ring without a creator record

It never touches a ring of another user, a ring whose creator lives, or a ring it cannot judge. A ring made by
xblick 0.5.11 or older has no creator record and holds no lock: its state is "unknown", and only ``--force`` with
its name removes it, when no process maps it.
"""

from __future__ import annotations

import fcntl
import os
import re
import struct
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path

from . import _core

SHM_DIR = Path("/dev/shm")  # where glibc's shm_open keeps the objects on Linux
NAME = re.compile(r"xblick-[0-9a-f]{16}")
MAGIC = b"DBLKRING"
# the control header, ring.hpp: magic, version, slot_count, slot_size, data_offset, kind, creator_pid,
# creator_start, last_update_ns, reserved
HEADER = struct.Struct("<8sIIQQIIQQ8s")
KINDS = {0: "host", 1: "cuda"}

LIVING = "living"  # the lock is taken and the recorded creator lives
HELD = "held"  # the lock is taken by a process other than the recorded creator (a forked child), or no record
GONE = "gone"  # the lock is free and the record names a creator that has ended
UNKNOWN = "unknown"  # no creator record (xblick 0.5.11 or older) and no lock
OTHER_USER = "other user"
UNJUDGED = "cannot judge"  # not a ring, unreadable, or contradicting itself


@dataclass
class Ring:
    """One ring file and the verdict on it."""

    name: str  # xblick-<16 hex>, as in /dev/shm
    state: str
    detail: str = ""
    kind: str = "?"
    size: int = 0
    creator_pid: int = 0
    creator_alive: bool = False
    last_update_ns: int = 0
    mappers: list[int] = field(default_factory=list)  # processes other than the creator that map it
    removed: bool = False

    def removable(self, force: bool) -> bool:
        """True if ``--remove`` may remove it; ``force``: the ring was named and ``--force`` given."""
        if self.state == GONE:
            return not self.mappers or force
        return self.state == UNKNOWN and not self.mappers and force


def ring_name(given: str) -> str:
    """``xblick-<16 hex>`` from a name as a user gives it: bare, with a leading / (a descriptor's) or a path in
    /dev/shm; ValueError otherwise."""
    name = given[len(str(SHM_DIR)) + 1:] if given.startswith(str(SHM_DIR) + "/") else given.lstrip("/")
    if not NAME.fullmatch(name):
        raise ValueError(f"{given!r} is not a ring's name (xblick-<16 hex digits>)")
    return name


def mappers() -> dict[str, list[int]]:
    """The processes that map each ring, from /proc/<pid>/maps of the processes this user may read. Rings are made
    with mode 0600, so only this user's processes (and root's) can map them."""
    found: dict[str, list[int]] = {}
    marker = f" {SHM_DIR}/"
    for entry in os.scandir("/proc"):
        if not entry.name.isdigit():
            continue
        try:
            with open(f"/proc/{entry.name}/maps", encoding="utf-8", errors="replace") as maps:
                names = set()
                for line in maps:
                    at = line.find(marker)
                    if at >= 0:
                        name = line[at + len(marker):].split()[0]
                        if NAME.fullmatch(name):
                            names.add(name)
        except OSError:  # ended meanwhile, or another user's
            continue
        for name in names:
            found.setdefault(name, []).append(int(entry.name))
    return found


def _alive(pid: int, start: int) -> bool:
    """True if process pid exists and started at start (so it is not a later process with the same pid)."""
    try:
        return _core.process_start_time(pid) == start
    except RuntimeError:  # RingError: no such process
        return False


def _command(pid: int) -> str:
    try:
        return Path(f"/proc/{pid}/comm").read_text().strip()
    except OSError:
        return "?"


def judge(name: str, mapped: dict[str, list[int]], remove=None) -> Ring:
    """The verdict on ring ``name`` (in /dev/shm). ``remove(ring) -> bool``: called with the creator lock held when the
    lock could be taken; True removes the ring before the lock is released, so no creator can come between."""
    path = SHM_DIR / name
    try:
        st = os.lstat(path)
    except FileNotFoundError:
        return Ring(name, UNJUDGED, "gone meanwhile")
    if st.st_uid != os.geteuid():
        return Ring(name, OTHER_USER, f"uid {st.st_uid}", size=st.st_size)
    try:
        fd = os.open(path, os.O_RDONLY | os.O_CLOEXEC | os.O_NOFOLLOW)
    except OSError as error:
        return Ring(name, UNJUDGED, f"cannot open: {error.strerror}", size=st.st_size)
    try:
        data = os.pread(fd, HEADER.size, 0)
        if len(data) < HEADER.size:
            return Ring(name, UNJUDGED, f"{len(data)} bytes, no ring header (being created?)", size=st.st_size)
        magic, version, _, _, _, kind, pid, start, last, _ = HEADER.unpack(data)
        if magic != MAGIC:
            return Ring(name, UNJUDGED, "not a Durchblick ring", size=st.st_size)
        ring = Ring(name, UNJUDGED, kind=KINDS.get(kind, f"kind {kind}"), size=st.st_size, creator_pid=pid,
                    last_update_ns=last)
        if version != 1:
            ring.detail = f"ring version {version}, this xblick knows 1"
            return ring
        try:
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            locked = True  # by us: no living creator holds it
        except BlockingIOError:
            locked = False
        ring.creator_alive = pid != 0 and _alive(pid, start)
        ring.mappers = [p for p in mapped.get(name, []) if not (ring.creator_alive and p == pid)]
        if not locked:
            ring.state = LIVING if ring.creator_alive else HELD
            if ring.state == HELD:
                ring.detail = ("lock taken, no creator record" if pid == 0 else
                               "lock taken, its creator ended: a child that inherited it (fork without exec)?")
        elif pid == 0:
            ring.state, ring.detail = UNKNOWN, "no creator record: made by xblick 0.5.11 or older"
        elif ring.creator_alive:
            ring.detail = "lock free, but its creator lives"
        else:
            ring.state = GONE
        if locked and remove is not None and remove(ring):
            os.unlink(path)
            ring.removed = True
        return ring
    finally:
        os.close(fd)  # releases our lock, after an unlink


def _age(ns: int, now_ns: int) -> str:
    if ns == 0:
        return "-"
    seconds = max(now_ns - ns, 0) / 1e9
    for unit, size in (("d", 86400), ("h", 3600), ("min", 60)):
        if seconds >= size:
            return f"{seconds / size:.1f} {unit} ago"
    return f"{seconds:.1f} s ago"


def _size(n: int) -> str:
    for unit, size in (("GB", 1e9), ("MB", 1e6), ("kB", 1e3)):
        if n >= size:
            return f"{n / size:.1f} {unit}"
    return f"{n} B"


def _state(ring: Ring) -> str:
    text = {LIVING: "creator living", GONE: "creator gone", UNKNOWN: "unknown", HELD: "lock held",
            OTHER_USER: "another user's", UNJUDGED: "cannot judge"}[ring.state]
    if ring.detail:
        text += f" ({ring.detail})"
    if ring.mappers:
        text += ", mapped by pid " + ", ".join(map(str, sorted(ring.mappers)))
    return text


def run(names: list[str], remove: bool, force: bool, out=sys.stdout) -> int:
    """``xblick clean``: list the rings (``names``: only these), remove with ``remove``. Exit code 1 if a named ring
    does not exist or a removal failed."""
    prog = "xblick clean"
    if force and not (remove and names):
        print(f"{prog}: --force needs --remove and the rings named", file=sys.stderr)
        return 1
    try:
        chosen = [ring_name(n) for n in names]
    except ValueError as error:
        print(f"{prog}: {error}", file=sys.stderr)
        return 1
    missing = [n for n in chosen if not (SHM_DIR / n).exists()]
    if missing:
        print(f"{prog}: no such ring in {SHM_DIR}: {', '.join(missing)}", file=sys.stderr)
        return 1
    if not chosen:
        chosen = sorted(e.name for e in os.scandir(SHM_DIR) if NAME.fullmatch(e.name))
    mapped = mappers()
    rings, failed = [], 0
    for name in chosen:
        try:
            rings.append(judge(name, mapped, (lambda r: r.removable(force)) if remove else None))
        except OSError as error:  # the unlink
            print(f"{prog}: {name}: {error.strerror}", file=sys.stderr)
            failed += 1
    if not rings:
        print(f"no ring in {SHM_DIR}", file=out)
        return 1 if failed else 0
    now = time.time_ns()
    rows = [("RING", "KIND", "SIZE", "CREATOR", "LAST FRAME", "ACTION", "STATE")]
    for r in rings:
        creator = "-" if r.creator_pid == 0 else (
            f"pid {r.creator_pid} ({_command(r.creator_pid)})" if r.creator_alive else f"pid {r.creator_pid} (ended)")
        action = "removed" if r.removed else "would remove" if not remove and r.removable(force) else "keep"
        rows.append((r.name, r.kind, _size(r.size), creator, _age(r.last_update_ns, now), action, _state(r)))
    widths = [max(len(row[i]) for row in rows) for i in range(len(rows[0]) - 1)]
    for row in rows:
        print("  ".join(c.ljust(w) for c, w in zip(row, widths)) + "  " + row[-1], file=out)
    gone = [r for r in rings if r.removed or (not remove and r.removable(force))]
    total = _size(sum(r.size for r in gone))
    if remove:
        print(f"removed {len(gone)} of {len(rings)} rings ({total})", file=out)
    elif gone:
        print(f"{len(gone)} of {len(rings)} rings left behind by a creator that is gone ({total}); "
              f"xblick clean --remove removes them", file=out)
    else:
        print(f"nothing to remove among {len(rings)} rings", file=out)
    return 1 if failed else 0
