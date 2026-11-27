"""The terminal interface: ``xblick`` without a command. A Textual app (textual.textualize.io) on one
Zenoh session that shows what runs, for any node: nothing in it knows of a particular node.

* **Connect** (``c``): this PC's router, endpoints (``--connect``/``--listen``, e.g. ``tcp/robot-pc:7447``) or a Zenoh
  configuration file (``--config``, e.g. ``robot-pc.json5``); again at any time, to the same or another.
* **Topics**: every key a session publishes (from the routers' admin space), and with ``m`` the traffic of all of
  them (rate, size, type, last stamp: subscribing to everything, as ``xblick list --traffic``, which makes the
  publishers produce every representation). Enter on a topic subscribes to it alone and shows its last message.
* **Nodes**: every program (one session each) by router (tui_nodes.py): its name, program, kind, what it reads and
  publishes, from what it declared (declaration.py) and the routers' admin space; the routers with their links; the
  reports on ``<camera>/diagnostics`` and the record and play node's status. A row selected: its detail.
* **Graph**: the sessions as boxes and the data between them as arrows (tui_graph.py); a session selected (arrow
  keys, a click) or the mouse over an edge names the keys, how they are taken and what the session reports.
* **Rerun**: everything ``xblick rerun`` can show, an entry each, as a tree by camera and node (``rerun_entries``:
  info records and the admin space; the cameras that exist only as ROS topics by their namespace, marked ROS, from
  the session's ROS graph, never our programs' ROS copies); the marked entries are its selection, a camera's transforms marked with its
  first entry (``toggle_mark``), a marked entry whose publisher ended shown as gone. Options and marks as presets
  with a name; launched as a child process (stopped with SIGTERM, as its Ctrl-C) on the interface's connection.
  While it runs its options are fixed and the marks go to its live parameters (``xblick/<name>``).

The routers' admin space (``survey.query_network``) has the sessions and their declarations; without a router (only
peers) the interface says so and shows what the streams' info records and the traffic tell. In a session in ROS 2
(the default) the Graph also titles a session that declares no program of ours but publishes camera topics in ROS's
graph (a ROS camera driver) by its ROS nodes, marked ROS, with those topics in its box (``Discovery.ros_publishers``).
"""

from __future__ import annotations

import collections
import contextlib
import dataclasses
import functools
import importlib.util
import io
import json
import os
import shlex
import signal
import socket
import subprocess
import sys
import threading
import time
from pathlib import Path
from typing import ClassVar

import zenoh
from rich.console import Group
from rich.pretty import Pretty
from rich.table import Table
from rich.text import Text
from textual import events
from textual.app import App, ComposeResult
from textual.binding import Binding
from textual.containers import Horizontal, Vertical, VerticalScroll
from textual.screen import ModalScreen
from textual.widgets import (Button, DataTable, Footer, Header, Input, Label, RadioButton, RadioSet, RichLog, Select,
                             Static, TabbedContent, TabPane, Tree)
from textual.worker import get_current_worker

from . import declaration, diagnostics, parameters, ros, survey, topics, tui_nodes
from . import session as zsession
from .camera import find_streams
from .paths import config_dir, expand_path
from .tui_graph import LABEL, GraphEdge, GraphGroup, GraphNode, GraphView
from .tui_parameters import ParametersPane

PRESETS_VERSION = 1
UNREACHABLE_S = 5.0  # a peer without a link to it for so long is not reachable from here
MIN_SIZE = (80, 24)  # columns, rows: below it the interface says so instead of a broken layout
WIDE = 120  # from this width the Rerun tab's lists stand side by side
GRAPH_HELP = Text("arrow keys or a click: a node and its edges (out green, in cyan); the mouse over an edge or a "
                  "box: its keys and how they are taken; ▶ data flows into a box, ◀ out of it (a cycle); a line: one "
                  "topic, how it is taken and, while m measures, its messages/s as this interface receives them; ▷ in "
                  "a box: a topic nobody takes; one band per router, its links on the left; shift+arrows or a drag: "
                  "move the view, 0: back to its start", style="dim")
# the kinds of the Rerun tab's entries: those of xblick rerun's selection (rerun_bridge.KINDS), and a camera's
# transforms (tf_static and tf), which come with what they place unless left out (--no-transforms)
RERUN_KINDS = ("bases", "clouds", "detections", "segmentations", "trajectories", "bodies", "imu", "diagnostics")
TRANSFORMS = "transforms"
PLACED_KINDS = ("bases", "clouds", "detections", "segmentations", "bodies")  # rerun_bridge.PLACED_KINDS
ENTRY_ORDER = {"bases": 0, "clouds": 1, TRANSFORMS: 2, "imu": 3, "diagnostics": 4, "segmentations": 5,
               "detections": 6, "bodies": 7, "trajectories": 8}  # within a branch of the tree
ROS_MARK = " (ROS)"  # after the branch of a ROS camera (its topics' namespace) and a ROS node's name in the Graph


def _short(zid: str) -> str:
    return zid[:8]


def short_names(labels: dict[str, str]) -> dict[str, str]:
    """Sessions' names without what another session's name says already: below the longest other name that is a
    prefix of it (``sam3`` below ``test/camera``, ``tapnext/tracks``), else whole (``robot``,
    ``xblick/recorder/status``); ``subscribes <prefix>`` likewise after its word. Two names that would be the same
    stay whole."""
    def split(label: str) -> tuple[str, list[str]]:
        word, _, rest = label.partition(" ") if label.startswith("subscribes ") else ("", "", label)
        return word, rest.split("/")

    tokens = {tuple(split(label)[1]) for label in labels.values() if not label.startswith("subscribes ")}
    names = {}
    for zid, label in labels.items():
        word, own = split(label)
        cut = max((len(t) for t in tokens if len(t) < len(own) and tuple(own[:len(t)]) == t), default=0)
        names[zid] = " ".join(filter(None, (word, "/".join(own[cut:]))))
    count: dict[str, int] = {}
    for name in names.values():
        count[name] = count.get(name, 0) + 1
    return {zid: name if count[name] == 1 else labels[zid] for zid, name in names.items()}


def _clock(ns: int | None) -> str:
    if ns is None:
        return ""
    return time.strftime("%H:%M:%S", time.localtime(ns / 1e9)) + f".{ns // 1_000_000 % 1000:03d}"


# ------------------------------------------------------------------------------------- connection and presets


@dataclasses.dataclass
class Connection:
    """A Zenoh connection as the commands take it: nothing (this PC's router), ``connect``/``listen`` endpoints in
    ``mode`` peer or client, or a configuration file (absolute, so that a preset works from anywhere). A peer that only
    connects is refused, unlike in the commands: there it listens on a loopback port of this PC, which peers on other
    PCs or behind a tunnel cannot reach (the router does not route between peers), so it would receive nothing from
    them."""

    connect: list[str] = dataclasses.field(default_factory=list)
    listen: list[str] = dataclasses.field(default_factory=list)
    config: str | None = None
    mode: str = "peer"

    def __post_init__(self):
        if self.config and (self.connect or self.listen or self.mode != "peer"):
            raise ValueError("give either a Zenoh config file or connect/listen endpoints and a mode, not both")
        if self.mode not in ("peer", "client"):
            raise ValueError(f"mode {self.mode!r}: peer or client")
        if self.mode == "client" and (self.listen or not self.connect):
            raise ValueError("client mode: an endpoint to connect to (the router), nothing to listen on")
        # what a person types (robot-pc, 192.0.2.10:7447) as Zenoh endpoints, shown so in the status line
        self.connect = [zsession.endpoint(e) for e in self.connect]
        self.listen = [zsession.endpoint(e) for e in self.listen]
        if self.mode == "peer" and self.connect and not self.listen:
            raise ValueError("a peer that only connects is refused: it would listen on a loopback port only, which "
                             "peers on other PCs or behind a tunnel cannot reach (the router does not route between "
                             "peers): listen too (tcp/127.0.0.1:0 on this PC) or use client mode")
        if self.config:
            path = expand_path(self.config).resolve()
            if not path.is_file():
                raise FileNotFoundError(f"Zenoh config file {self.config}: no such file")
            self.config = str(path)

    def arguments(self) -> list[str]:
        """The commands' ``--connect``, ``--listen``, ``--config`` for it."""
        if self.config:
            return ["--config", self.config]
        return ((["--zenoh-mode", "client"] if self.mode == "client" else [])
                + (["--connect", *self.connect] if self.connect else [])
                + (["--listen", *self.listen] if self.listen else []))

    def describe(self) -> str:
        if self.config:
            return f"config {self.config}"
        if not (self.connect or self.listen):
            return f"this PC's router ({zsession.ROUTER})"
        return " ".join(self.arguments())

    def open(self) -> zenoh.Session:
        return zsession.open_session(connect=self.connect, listen=self.listen, mode=self.mode, config_file=self.config,
                                     program="xblick")

    def to_dict(self) -> dict:
        return dataclasses.asdict(self)

    @classmethod
    def from_dict(cls, d: dict) -> Connection:
        return cls(list(d.get("connect") or []), list(d.get("listen") or []), d.get("config"), d.get("mode", "peer"))


@dataclasses.dataclass
class RerunSelection:
    """What ``xblick rerun`` is to show and how: its selection (streams as ``bases``, point clouds, detections, label
    images, trajectories, bodies, IMU, diagnostics, the cameras whose transforms are left out), options, the viewer
    (``spawn``, ``serve`` with a port, ``save`` with a file, ``connect-viewer`` with a URL), further arguments, and the
    Zenoh connection. Nothing selected: rerun shows everything it finds."""

    def selected(self) -> dict[str, list[str]]:
        """The selection by kind (RERUN_KINDS)."""
        return {kind: list(getattr(self, kind)) for kind in RERUN_KINDS}

    def node_key(self) -> str:
        """The key of the node it runs as (``xblick/<name>``, ``--name`` among the further arguments)."""
        from . import cli

        return f"xblick/{cli.parse(['rerun', *self.check()]).name}"

    bases: list[str] = dataclasses.field(default_factory=list)
    clouds: list[str] = dataclasses.field(default_factory=list)
    detections: list[str] = dataclasses.field(default_factory=list)
    segmentations: list[str] = dataclasses.field(default_factory=list)
    trajectories: list[str] = dataclasses.field(default_factory=list)
    bodies: list[str] = dataclasses.field(default_factory=list)
    imu: list[str] = dataclasses.field(default_factory=list)
    diagnostics: list[str] = dataclasses.field(default_factory=list)
    no_transforms: list[str] = dataclasses.field(default_factory=list)  # cameras whose transforms are left out
    prefix: str = ""
    stale_after: float | None = None  # None: rerun's default
    cloud_rate: float | None = None
    viewer: str = "spawn"
    viewer_value: str = ""
    extra: str = ""  # further arguments, as on a command line
    connection: Connection = dataclasses.field(default_factory=Connection)

    VIEWERS: ClassVar[tuple[str, ...]] = ("spawn", "serve", "save", "connect-viewer")

    def arguments(self) -> list[str]:
        """``xblick rerun``'s arguments (after ``rerun``)."""
        if self.viewer not in self.VIEWERS:
            raise ValueError(f"viewer {self.viewer!r}: one of {', '.join(self.VIEWERS)}")
        if self.viewer in ("save", "connect-viewer") and not self.viewer_value:
            raise ValueError(f"--{self.viewer} needs a {'file' if self.viewer == 'save' else 'URL'}")
        args = [f"--{self.viewer}"] + ([self.viewer_value] if self.viewer_value else [])
        args += list(self.bases)
        for option, keys in (("--detections", self.detections), ("--segmentation", self.segmentations),
                             ("--points", self.clouds), ("--trajectory", self.trajectories),
                             ("--bodies", self.bodies), ("--imu", self.imu), ("--diagnostics", self.diagnostics),
                             ("--no-transforms", self.no_transforms)):
            for key in keys:
                args += [option, key]
        if self.prefix:
            args += ["--prefix", self.prefix]
        if self.stale_after is not None:
            args += ["--stale-after", f"{self.stale_after:g}"]
        if self.cloud_rate is not None:
            args += ["--cloud-rate", f"{self.cloud_rate:g}"]
        args += shlex.split(self.extra)
        return args + self.connection.arguments()  # last: --connect takes several endpoints

    def check(self) -> list[str]:
        """The arguments, checked by the command's own parser; a wrong one is a ValueError with argparse's reason."""
        from . import cli

        args = self.arguments()
        reason = io.StringIO()
        with contextlib.redirect_stderr(reason), contextlib.redirect_stdout(io.StringIO()):
            try:
                cli.parse(["rerun", *args])
            except SystemExit:
                raise ValueError(reason.getvalue().strip().splitlines()[-1]) from None
        return args

    def command(self) -> list[str]:
        """The child process's command line (this interpreter's Durchblick)."""
        return [sys.executable, "-m", "xblick.cli", "rerun", *self.check()]

    def to_dict(self) -> dict:
        d = dataclasses.asdict(self)
        d["connection"] = self.connection.to_dict()
        return d

    @classmethod
    def from_dict(cls, d: dict) -> RerunSelection:
        fields = {f.name for f in dataclasses.fields(cls)}
        unknown = set(d) - fields
        if unknown:
            raise ValueError(f"a preset with fields {sorted(unknown)} this version does not know")
        d = dict(d)
        d["connection"] = Connection.from_dict(d.get("connection") or {})
        return cls(**d)


def presets_path() -> Path:
    """``$XDG_CONFIG_HOME/xblick/rerun_presets.json`` (``~/.config`` without it; ``paths.config_dir``)."""
    return config_dir() / "rerun_presets.json"


class Presets:
    """Named RerunSelections in a JSON file (``{"v": 1, "presets": {name: selection}}``). A file of another version
    or form is refused, not overwritten."""

    def __init__(self, path: Path | str | None = None):
        self.path = Path(path) if path is not None else presets_path()

    def load(self) -> dict[str, RerunSelection]:
        if not self.path.exists():
            return {}
        try:
            data = json.loads(self.path.read_text())
        except ValueError as error:
            raise ValueError(f"{self.path}: no presets file ({error}); move it away") from None
        if not isinstance(data, dict) or data.get("v") != PRESETS_VERSION or not isinstance(data.get("presets"), dict):
            raise ValueError(f"{self.path}: not a presets file of version {PRESETS_VERSION}; move it away")
        out = {}
        for name, d in data["presets"].items():
            try:
                out[name] = RerunSelection.from_dict(d)
            except (TypeError, ValueError, OSError) as error:
                raise ValueError(f"{self.path}: preset {name!r}: {error}") from None
        return out

    def _write(self, presets: dict[str, RerunSelection]):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        data = {"v": PRESETS_VERSION, "presets": {n: s.to_dict() for n, s in sorted(presets.items())}}
        part = self.path.with_suffix(".part")
        part.write_text(json.dumps(data, indent=1) + "\n")
        part.replace(self.path)  # whole or not at all

    def save(self, name: str, selection: RerunSelection):
        if not name.strip():
            raise ValueError("a preset needs a name")
        selection.arguments()  # a preset that cannot run is not saved
        presets = self.load()
        presets[name.strip()] = selection
        self._write(presets)

    def delete(self, name: str):
        presets = self.load()
        if name not in presets:
            raise KeyError(f"no preset {name!r} in {self.path}")
        del presets[name]
        self._write(presets)


# ------------------------------------------------------------------------------------------------- the screens


class ConnectScreen(ModalScreen[Connection | None]):
    """The connection to open: this PC's router, endpoints, or a configuration file. It opens on the endpoints with
    the cursor in the address (the interface starts on this PC's router, and the dialog is opened to change that), or
    on the config file when that is the current connection. Typing into a field chooses its way; an address alone
    (``robot-pc``) is ``tcp/robot-pc:7447`` in client mode."""

    FIELDS: ClassVar = {"connect-endpoints": 1, "listen": 1, "config": 2}  # the input -> the way it belongs to

    BINDINGS: ClassVar = [("escape", "cancel", "Cancel")]

    def __init__(self, current: Connection, hint: str = ""):
        super().__init__()
        self.current = current
        self.hint = hint  # why the dialog opened by itself

    def compose(self) -> ComposeResult:
        c = self.current
        mode = 2 if c.config else 1
        with VerticalScroll(id="connect-dialog", can_focus=False):  # the keys go to the choice first
            yield Label("Zenoh connection", classes="title")
            with RadioSet(id="connect-mode", compact=True):
                for i, text in enumerate((f"this PC's router ({zsession.ROUTER})", "endpoints", "config file")):
                    yield RadioButton(text, value=i == mode, compact=True)
            yield Label("endpoints: connect to (several: spaces; host alone: tcp, port 7447)")
            yield Input(" ".join(c.connect), placeholder="e.g. robot-pc or tcp/192.0.2.10:7447",
                        id="connect-endpoints", compact=True)
            yield Label("endpoints: mode")
            yield Select([("client: everything through the router (a viewer, another PC)", "client"),
                          ("peer: listening too (on this PC: tcp/127.0.0.1:0)", "peer")],
                         value=c.mode if (c.connect or c.listen) else "client", allow_blank=False, id="zenoh-mode",
                         compact=True)
            yield Label("endpoints, peer: listen on")
            yield Input(" ".join(c.listen), placeholder="e.g. tcp/127.0.0.1:0", id="listen", compact=True)
            yield Label("config file")
            yield Input(c.config or "", placeholder="a Zenoh JSON5 file, e.g. robot-pc.json5", id="config",
                        compact=True)
            yield Static(Text(self.hint, style="yellow"), id="connect-error")
            with Horizontal(id="connect-buttons"):
                yield Button("Connect", variant="primary", id="connect-ok", compact=True)
                yield Button("Cancel", id="connect-cancel", compact=True)

    def on_mount(self):
        self.query_one("#config" if self.current.config else "#connect-endpoints", Input).focus()

    def on_input_changed(self, event: Input.Changed):
        way = self.FIELDS.get(event.input.id)
        radios = self.query_one("#connect-mode", RadioSet)
        if way is not None and event.value.strip() and radios.pressed_index != way:
            list(radios.query(RadioButton))[way].value = True

    def on_button_pressed(self, event: Button.Pressed):
        if event.button.id == "connect-cancel":
            self.dismiss(None)
        elif event.button.id == "connect-ok":
            self._submit()

    def on_input_submitted(self, event: Input.Submitted):
        self._submit()

    def _submit(self):
        mode = self.query_one("#connect-mode", RadioSet).pressed_index
        connect = self.query_one("#connect-endpoints", Input).value.split()
        listen = self.query_one("#listen", Input).value.split()
        zenoh_mode = self.query_one("#zenoh-mode", Select).value
        config = self.query_one("#config", Input).value.strip()
        try:
            if mode == 0:
                connection = Connection()
            elif mode == 1:
                if not (connect or listen):
                    raise ValueError("endpoints: give at least one to connect to (or to listen on)")
                connection = Connection(connect, listen, mode=zenoh_mode)
            else:
                if not config:
                    raise ValueError("config file: give its path")
                connection = Connection(config=config)
        except (ValueError, OSError) as error:
            self.query_one("#connect-error", Static).update(Text(str(error), style="bold red"))
            return
        self.dismiss(connection)

    def action_cancel(self):
        self.dismiss(None)


@dataclasses.dataclass
class TopicRow:
    """A row of the Topics tab: a key (``key``) or a prefix of keys (``key`` None) with the keys below it
    (``below``). ``path`` is the row's id (a prefix ends with ``/``); ``show_pub``/``show_sub``: the publishers or
    subscribers stand on this row (not on a prefix above it, which has the same for every key below)."""

    path: str
    label: str
    key: str | None
    below: list[str]
    show_pub: bool = True
    show_sub: bool = True

    def common(self, cells: dict[str, tuple[frozenset, frozenset]]) -> tuple[frozenset, frozenset]:
        pubs = {cells[k][0] for k in self.below}
        subs = {cells[k][1] for k in self.below}
        return (next(iter(pubs)) if len(pubs) == 1 else frozenset(),
                next(iter(subs)) if len(subs) == 1 else frozenset())


def topic_tree(keys, collapsed: set[str] = frozenset(), cells: dict | None = None) -> list[TopicRow]:
    """The keys as a directory tree, depth first: a prefix with one child and no key of its own joins it
    (``test/camera/``), guides (``├─``, ``└─``) show the depth, a prefix in ``collapsed`` hides what is below it
    (``▸``, with the number of keys). A key that is also a prefix (``.../image_raw`` and ``.../image_raw/shm``) is one
    row. A ring's key ends with its place, the host's id and the GPU's UUID (topics.py): the tree shows the first 8
    characters of each. With ``cells`` (key -> publishers, subscribers), a key's or prefix's sessions stand only on the highest row
    below which every key has the same."""
    root: dict = {}
    for key in keys:
        node = root
        place = topics.ring_place(key)  # a ring's key ends with its host's id and its GPU's: shown by 8 characters
        tokens = key.split("/") if place is None else place[0].split("/") + [c[:8] for c in place[1].split("/")]
        for token in tokens:
            node = node.setdefault(token, {})
        node["\0"] = key  # a key ends here
    rows: list[TopicRow] = []

    def leaves(node) -> list[str]:
        out = [node["\0"]] if "\0" in node else []
        for token, child in node.items():
            if token != "\0":
                out += leaves(child)
        return out

    def walk(node, prefix: str, guide: str, last: bool, top: bool, shown: tuple[bool, bool]):
        names = [prefix]
        # join a chain of prefixes with one child and no key of their own
        while "\0" not in node and len(node) == 1:
            token, node = next(iter(node.items()))
            names.append(token)
        path = "/".join(n for n in names if n)
        children = sorted(t for t in node if t != "\0")
        key = node.get("\0")
        is_prefix = bool(children)
        row_path = path + "/" if is_prefix else path
        folded = is_prefix and row_path in collapsed
        name = path[len(guide_path[0]):].lstrip("/") if not top else path
        mark = ("▸ " if folded else "▾ ") if is_prefix else ""
        branch = "" if top else ("└─ " if last else "├─ ")
        below = leaves(node)
        label = guide + branch + mark + name + ("/" if is_prefix else "") + (f"  ({len(below)} keys)" if folded else "")
        row = TopicRow(row_path, label, key, below)
        if cells is not None:
            pubs = {cells[k][0] for k in below}
            subs = {cells[k][1] for k in below}
            row.show_pub = not shown[0] and (len(pubs) == 1 or not is_prefix)
            row.show_sub = not shown[1] and (len(subs) == 1 or not is_prefix)
            shown = (shown[0] or (row.show_pub and is_prefix), shown[1] or (row.show_sub and is_prefix))
        rows.append(row)
        if folded:
            return
        saved = guide_path[0]
        guide_path[0] = path
        inner = guide + ("" if top else ("   " if last else "│  "))
        for n, token in enumerate(children):
            walk(node[token], path + "/" + token if path else token, inner, n == len(children) - 1, False, shown)
        guide_path[0] = saved

    guide_path = [""]
    for token in sorted(t for t in root if t != "\0"):
        guide_path[0] = ""
        walk(root[token], token, "", True, True, (False, False))
    return rows


def pose_topics(network: survey.Network | None) -> list[str]:
    """The keys for ``xblick rerun --trajectory``: a camera's ``<camera>/tf`` and a node's ``<camera>/<node>/tf``.
    Not ``<robot>/tf``: its frames form a tree below the world, and the bridge takes poses in the world only."""
    if network is None:
        return []
    return sorted(k for k in network.publishers() if k.endswith("/" + topics.TF) and k.count("/") >= 2)


@dataclasses.dataclass(frozen=True)
class RerunEntry:
    """One thing ``xblick rerun`` can show, as the Rerun tab lists it: its kind (RERUN_KINDS or TRANSFORMS), its key
    (the topic; for the transforms the camera), where it sits in the tree (camera, node: empty for the camera's own)
    under its label, and what the viewer makes of it (``note``)."""

    kind: str
    key: str
    camera: str
    node: str
    label: str
    note: str = ""

    @property
    def id(self) -> str:
        return f"{self.kind}\t{self.key}"


def rerun_entry(kind: str, key: str, note: str = "") -> RerunEntry:
    """An entry of a kind and key, placed by its key: ``<namespace>/<camera>``, then a node's part, then the name. A
    ROS topic (a leading /) below its namespace, marked ROS (``ROS_MARK``): ``/rgbd/color/image_raw`` is
    ``image_raw`` of ``/rgbd/color (ROS)``."""
    from .rerun_bridge import camera_of_entry

    if kind == TRANSFORMS:
        return RerunEntry(kind, key, key, "", "tf, tf_static", note)
    if ros.is_topic(key):
        namespace, _, label = key.rpartition("/")
        return RerunEntry(kind, key, (namespace or "/") + ROS_MARK, "", label, note)
    camera = camera_of_entry(kind, key) if kind != "trajectories" else "/".join(key.split("/")[:2])
    rest = key[len(camera) + 1:].split("/")
    label = rest[-1] + (" (path)" if kind == "trajectories" else "")
    return RerunEntry(kind, key, camera, "/".join(rest[:-1]), label, note)


def placed(kind: str, key: str) -> bool:
    """An entry a camera's transforms place: of PLACED_KINDS, not a ROS topic (a ROS camera has no transforms of ours
    in the bridge)."""
    return kind in PLACED_KINDS and not ros.is_topic(key)


def rerun_entries(streams: list[tuple[str, dict]], network: survey.Network | None,
                  ros_cameras: dict[str, list[str]] | None = None) -> list[RerunEntry]:
    """Everything ``xblick rerun`` can show of what was found: from the info records (``streams``: image and point
    cloud streams, label images, detections) and from the routers' admin space (``network``: bodies, IMU, diagnostics,
    poses, the cameras' transforms); a camera's transforms for every camera with something they place; the cameras
    that exist only as ROS topics (``ros_cameras``, ros.camera_streams: their image and PointCloud2 topics). A note
    says what the viewer makes of each, as far as the records tell (an image's frame, a detection's type)."""
    from .cli import _bridged

    by_base = dict(streams)
    found = _bridged(streams)
    images: dict[str, list[str]] = {}  # camera -> its image streams, colour before 16-bit
    for base in found["bases"]:
        images.setdefault(rerun_entry("bases", base).camera, []).append(base)
    for camera, bases in images.items():
        bases.sort(key=lambda b: (by_base[b].get("encoding") in ("16UC1", "mono16"), b))

    def over(key: str) -> str:
        """The image a label image is drawn over: the camera's stream of its frame."""
        entry = rerun_entry("segmentations", key)
        frame = by_base[key].get("frame_id")
        image = next((b for b in images.get(entry.camera, []) if by_base[b].get("frame_id") == frame), None)
        return f"drawn over {image[len(entry.camera) + 1:]}" if image else "drawn over the image of its frame"

    detections = {"vision_msgs/msg/Detection2DArray": "boxes over the image of their frame",
                  "vision_msgs/msg/Detection3DArray": "boxes in 3D and over the images",
                  "vision_msgs/msg/Classification": "a text log"}
    entries = [rerun_entry("bases", b) for b in found["bases"]]
    entries += [rerun_entry("clouds", c, "points in 3D") for c in found["clouds"]]
    entries += [rerun_entry("segmentations", s, over(s)) for s in found["segmentations"]]
    entries += [rerun_entry("detections", d, detections.get(by_base[d].get("type"), "")) for d in found["detections"]]
    published = set(network.publishers()) if network is not None else set()
    answered = {k for n in network.nodes.values() for k in n.queryables} if network is not None else set()
    for key in sorted(published):
        if key.count("/") < 2:
            continue
        last = key.rpartition("/")[2]
        if last == "bodies":
            entries.append(rerun_entry("bodies", key, "skeletons in 3D and over the images"))
        elif last == topics.IMU:
            entries.append(rerun_entry("imu", key, "time series"))
        elif last == topics.DIAGNOSTICS:
            entries.append(rerun_entry("diagnostics", key, "a text log and time series"))
    entries += [rerun_entry("trajectories", k, "poses as a path in the world") for k in pose_topics(network)]
    ros_cameras = ros_cameras or {}
    entries += [rerun_entry("bases", t, "a ROS topic, no transforms") for t in ros_cameras.get("images", [])]
    entries += [rerun_entry("clouds", t, "points in 3D, a ROS topic, no transforms")
                for t in ros_cameras.get("clouds", [])]
    cameras = sorted({e.camera for e in entries if placed(e.kind, e.key)})
    for camera in cameras:
        with_any = {topics.tf_static(camera), topics.tf(camera)} & (published | answered)
        entries.append(rerun_entry(TRANSFORMS, camera, "places the camera in 3D" + (
            "" if with_any or network is None else ", none published now")))
    return entries


def rerun_counts(entries: list[RerunEntry]) -> str:
    """``3 streams, 2 results, 4 more topics``: a camera's own streams, the nodes' results, the rest."""
    streams = sum(1 for e in entries if e.kind in ("bases", "clouds") and not e.node)
    results = sum(1 for e in entries if e.node or e.kind == "bodies")
    return f"{_plural(streams, 'stream')}, {_plural(results, 'result')}, {len(entries) - streams - results} more"


def rerun_groups(entries: list[RerunEntry]) -> dict[str, dict[str, list[RerunEntry]]]:
    """The tree: camera -> node ("" first: the camera's own) -> its entries, each sorted."""
    groups: dict[str, dict[str, list[RerunEntry]]] = {}
    for e in entries:
        groups.setdefault(e.camera, {}).setdefault(e.node, []).append(e)
    return {camera: {node: sorted(group[node], key=lambda e: (ENTRY_ORDER[e.kind], e.label))
                     for node in sorted(group)} for camera, group in sorted(groups.items())}


def toggle_mark(marks: set[str], entry: RerunEntry, entries: list[RerunEntry]) -> set[str]:
    """The marks after Space on ``entry``: marked or not; a camera's transforms come with the first entry of that
    camera they place and go with its last, and can be unmarked themselves."""
    marks = set(marks)
    by_camera = {e.id for e in entries if e.camera == entry.camera and placed(e.kind, e.key)}
    transforms = rerun_entry(TRANSFORMS, entry.camera).id
    if entry.id in marks:
        marks.discard(entry.id)
        if placed(entry.kind, entry.key) and not (marks & by_camera):
            marks.discard(transforms)
    else:
        if placed(entry.kind, entry.key) and not (marks & by_camera):
            marks.add(transforms)
        marks.add(entry.id)
    return marks


def marks_of(selection: RerunSelection) -> set[str]:
    """The marks of a selection (a preset): its entries, and the transforms of every camera of what they place but
    those left out."""
    marks = {f"{kind}\t{key}" for kind, keys in selection.selected().items() for key in keys}
    cameras = {rerun_entry(kind, key).camera for kind, keys in selection.selected().items() for key in keys
               if placed(kind, key)}
    return marks | {rerun_entry(TRANSFORMS, c).id for c in cameras if c not in selection.no_transforms}


def selected_of(marks: set[str]) -> tuple[dict[str, list[str]], list[str]]:
    """(the selection by kind, the cameras whose transforms are left out) of the marks."""
    chosen: dict[str, list[str]] = {kind: [] for kind in RERUN_KINDS}
    transforms = set()
    for mark in sorted(marks):
        kind, key = mark.split("\t")
        if kind == TRANSFORMS:
            transforms.add(key)
        else:
            chosen[kind].append(key)
    cameras = sorted({rerun_entry(kind, key).camera for kind in PLACED_KINDS for key in chosen[kind]
                      if placed(kind, key)})
    return chosen, [c for c in cameras if c not in transforms]


def entry_text(entry: RerunEntry, marked: bool, note: str, width: int, gone: str = "") -> Text:
    """A leaf of the tree: its mark, its label padded to ``width``, its note (or ``gone``, in red)."""
    text = Text("[x] " if marked else "[ ] ", style="bold" if marked else "")
    text.append(entry.label.ljust(width) + "  ")
    if gone:
        text.append(gone, style="red")
    elif note:
        text.append(note, style="dim")
    return text


@dataclasses.dataclass
class Discovery:
    """One look at the network (in the discovery thread)."""

    network: survey.Network | None = None
    streams: list[tuple[str, dict]] = dataclasses.field(default_factory=list)
    tokens: list[str] = dataclasses.field(default_factory=list)
    statuses: dict[str, dict] = dataclasses.field(default_factory=dict)  # xblick/.../status -> its JSON
    program_errors: list[str] = dataclasses.field(default_factory=list)  # replies that were no program record
    error: str | None = None
    unreachable: list[str] = dataclasses.field(default_factory=list)  # peers this peer has no link to (zids)
    origins: list[str] = dataclasses.field(default_factory=list)  # the routers this interface is linked to (zids)
    took_s: float = 0.0  # how long this look took
    drawn_over: dict[str, list[str]] | None = None  # the running xblick rerun's: result -> its images (None: unknown)
    # in a session in ROS 2: the cameras that exist only as ROS topics (ros.camera_streams), and the ROS publishers of
    # camera topics (ros.CAMERA_TYPES) of sessions that are no programs of ours (ROS camera drivers)
    ros_cameras: dict[str, list[str]] = dataclasses.field(default_factory=dict)
    ros_publishers: list[ros.Token] = dataclasses.field(default_factory=list)


def statuses(session: zenoh.Session, timeout: float = 0.5) -> dict[str, dict]:
    """What Durchblick's programs answer on their status keys below ``xblick/`` (JSON), by key: every record and
    play node's (``xblick/<name>/status``), every MPS daemon's (``xblick/mps/<name>/status``)."""
    out = {}
    for reply in zsession.replies(session, "xblick/**/status", timeout):
        if reply.ok is not None:
            out[str(reply.ok.key_expr)] = json.loads(reply.ok.payload.to_bytes())
    return out


class RerunTree(Tree):
    """The Rerun tab's entries by camera and node. Space, Enter or a click on an entry marks or unmarks it
    (``Tree.NodeSelected``, handled by the app); on a camera or node it folds or unfolds."""

    BINDINGS: ClassVar = [Binding("space", "select_cursor", "Mark", show=False)]

    def __init__(self, label: str, **kwargs):
        super().__init__(label, **kwargs)
        self.show_root = False
        self.auto_expand = True


class XblickApp(App):
    """The interface on one session at a time; ``connection`` is opened at the start (default: this PC's router)."""

    TITLE = "Durchblick"
    HORIZONTAL_BREAKPOINTS: ClassVar = [(0, "-too-narrow"), (MIN_SIZE[0], "-narrow"), (WIDE, "-wide")]
    VERTICAL_BREAKPOINTS: ClassVar = [(0, "-too-short"), (MIN_SIZE[1], "-tall")]
    CSS = """
    #status { height: auto; max-height: 3; padding: 0 1; background: $boost; }
    /* the tabs take what is left: taller (auto) they made the screen scroll, the status line under the header */
    #main { height: 1fr; }
    #tabs { height: 1fr; }
    #tabs > ContentSwitcher, #tabs TabPane { height: 1fr; }
    DataTable { height: 1fr; }
    .detail { height: 1fr; border-top: solid $primary; padding: 0 1; }
    #report-table { border-top: solid $primary; }
    #graph-scroll { height: 3fr; overflow-x: auto; }
    #tab-topics DataTable { height: 2fr; }
    #tab-topics .detail { height: 1fr; }
    #rerun-top { height: 2fr; }
    #rerun-left { width: 1fr; height: 1fr; }
    #rerun-found, #rerun-options { height: auto; padding: 0 1; background: $boost; }
    #rerun-streams { height: 1fr; }
    #rerun-form { width: 1fr; height: 1fr; }
    /* narrower than WIDE: the Rerun tab's lists one above the other */
    Screen.-narrow #rerun-top { layout: vertical; }
    #rerun-command { height: auto; padding: 0 1; background: $boost; }
    #rerun-buttons, #preset-buttons { height: auto; }
    #rerun-log { height: 1fr; min-height: 3; border-top: solid $primary; }
    #too-small { display: none; height: 1fr; content-align: center middle; text-style: bold; }
    Screen.-too-narrow #main, Screen.-too-short #main { display: none; }
    Screen.-too-narrow #too-small, Screen.-too-short #too-small { display: block; }
    ConnectScreen { align: center middle; }
    #connect-dialog { width: 100%; max-width: 90; height: auto; max-height: 100%; padding: 0 1;
                      border: thick $primary; background: $surface; }
    #connect-buttons { height: auto; }
    /* the dialog opens with the cursor in a field: the choice's dot alone says the way, not Textual's blurred cursor
       on the first one */
    #connect-mode:blur > RadioButton.-selected > .toggle--label { background: transparent; }
    #connect-error { height: auto; }
    """
    BINDINGS: ClassVar = [("c", "connect", "Connect"), ("m", "measure", "Measure traffic"),
                          ("escape", "close_detail", "Close topic"), ("1", "tab('tab-topics')", "Topics"),
                          ("2", "tab('tab-nodes')", "Nodes"), ("3", "tab('tab-graph')", "Graph"),
                          ("4", "tab('tab-rerun')", "Rerun"), ("5", "tab('tab-params')", "Parameters"),
                          ("r", "refresh", "Update now"),
                          ("q", "quit", "Quit")]
    MAIN: ClassVar = {"tab-topics": "#topic-table", "tab-nodes": "#node-table", "tab-graph": "#graph",
                      "tab-rerun": "#rerun-streams",
                      "tab-params": "#param-nodes"}  # what a tab focuses: keys (arrows, Enter) go to it

    def __init__(self, connection: Connection | None = None, presets: Presets | None = None,
                 discover_every: float = 2.0):
        super().__init__()
        self.connection = connection or Connection()
        self.presets = presets or Presets()
        self.discover_every = discover_every
        self.session: zenoh.Session | None = None
        self.generation = 0  # of the session; results of an older one are dropped
        # the opening thread hands its result over under this lock; after the end (on_unmount) it closes it itself
        self._open_lock = threading.Lock()
        self._unmounted = False
        self._handed: dict[int, zenoh.Session] = {}  # generation -> a session opened, not yet taken by the loop
        self.discovery = Discovery()
        self.meter: survey.TrafficMeter | None = None  # every topic (m)
        self.detail: survey.TrafficMeter | None = None  # the selected topic
        self.detail_key: str | None = None
        self._before: tuple[float, dict] | None = None  # the last snapshot, for rates
        self._rates: dict[str, tuple[float, float]] = {}
        self._ages: dict[str, tuple] = {}  # key -> ((median, p95) ms at send or None, the same at receive)
        # (camera, name) -> (received ns, status, the sender's zid or None before 0.3.29)
        self._reports: dict[tuple[str, str], tuple[int, object, str | None]] = {}
        self._reports_lock = threading.Lock()
        self._report_errors: list[str] = []
        self._diagnostics_subscriber = None
        self._programs_subscriber = None  # the declarations' tokens: a program came or went, look again at once
        self._selected_node: str | None = None  # a row key of the Nodes tab (node\t<zid>, router\t<zid>)
        self._collapsed: set[str] = set()
        self._topic_rows: dict[str, TopicRow] = {}  # the Topics tab's folded prefixes (ending with /)
        self._graph_hovered = None  # what the mouse is over in the graph
        self._auto_connect = self.connection == Connection()  # nothing found on this PC: the dialog, once
        self._discover_now = threading.Event()  # r: look again now, not at the next period
        self._discovered_at: float | None = None
        self._rerun: subprocess.Popen | None = None
        self.rerun_output: list[str] = []  # the child's lines, as in the log
        self._rerun_entries: list[RerunEntry] = []  # what the Rerun tab lists, of the last look
        self._rerun_shown = None  # what the tree was built from
        self._rerun_leaves: dict = {}  # entry id -> its node of the tree
        self._rerun_marks: set[str] = set()  # the entries marked (their ids)
        self._rerun_seen: dict[str, float] = {}  # entry id -> when it was last found (time.time)
        self._rerun_folded: set[str] = set()  # the tree's branches folded by the user (camera, camera/node)
        self._rerun_key: str | None = None  # the running xblick rerun's node, while it takes its selection live
        self._rerun_live = False  # the running xblick rerun takes a changed selection (started with one)
        self._rerun_set_lock = threading.Lock()  # one set of its selection at a time, the newest marks
        self._looking = False  # r pressed: a look at the network is under way
        self.uses: dict[str, list[survey.Use]] = {}  # representation key -> who takes it, how (this interface left out)
        self._measure_after_open = False  # measuring when the connection changed: again on the next
        self.error: str | None = None

    # ------------------------------------------------------------------------------------------------ layout

    def compose(self) -> ComposeResult:
        yield Header()
        yield Static(f"terminal too small: it needs at least {MIN_SIZE[0]} x {MIN_SIZE[1]}", id="too-small")
        with Vertical(id="main"):
            yield Static(id="status")
            with TabbedContent(id="tabs"):
                with TabPane("Topics", id="tab-topics"):
                    yield DataTable(id="topic-table", cursor_type="row")
                    with VerticalScroll(classes="detail"):
                        yield Static("Enter on a topic: its publishers, subscribers and last message",
                                     id="topic-detail")
                with TabPane("Nodes", id="tab-nodes"):
                    # no padding of its own: a group's head is one band of the headers' colour (tui_nodes.py)
                    yield DataTable(id="node-table", cursor_type="row", show_header=False, cell_padding=0,
                                    cursor_background_priority="css")
                    yield DataTable(id="report-table", cursor_type="row")
                    with VerticalScroll(classes="detail"):
                        yield Static("a node or a router: what it declared and reports", id="node-detail")
                with TabPane("Graph", id="tab-graph"):
                    with VerticalScroll(id="graph-scroll"):
                        yield GraphView(id="graph")
                    with VerticalScroll(classes="detail", id="graph-detail-scroll"):
                        yield Static(GRAPH_HELP, id="graph-detail")
                with TabPane("Rerun", id="tab-rerun"):
                    with Horizontal(id="rerun-top"):
                        with Vertical(id="rerun-left"):
                            yield Static(id="rerun-found")
                            yield RerunTree("found", id="rerun-streams")
                        with VerticalScroll(id="rerun-form"):
                            yield Static(id="rerun-options")
                            yield Label("prefix (nothing marked: only what is below it)")
                            yield Input(id="rerun-prefix", placeholder="e.g. test", compact=True)
                            yield Label("--stale-after (s)")
                            yield Input(id="rerun-stale-after", placeholder="default", type="number", compact=True)
                            yield Label("--cloud-rate (clouds/s)")
                            yield Input(id="rerun-cloud-rate", placeholder="default", type="number", compact=True)
                            yield Label("viewer")
                            yield Select([(v, v) for v in RerunSelection.VIEWERS], value="spawn", allow_blank=False,
                                         id="rerun-viewer", compact=True)
                            yield Input(id="rerun-viewer-value", placeholder="serve: port; save: file; "
                                        "connect-viewer: URL", compact=True)
                            yield Label("further arguments")
                            yield Input(id="rerun-extra", placeholder="e.g. --report 2", compact=True)
                            yield Label("presets")
                            yield Select([], prompt="preset", id="preset-select", compact=True)
                            yield Input(id="preset-name", placeholder="name to save as", compact=True)
                            with Horizontal(id="preset-buttons"):
                                yield Button("Save", id="preset-save", compact=True)
                                yield Button("Load", id="preset-load", compact=True)
                                yield Button("Delete", id="preset-delete", variant="error", compact=True)
                    yield Static(id="rerun-command")
                    with Horizontal(id="rerun-buttons"):
                        yield Button("Launch xblick rerun", variant="primary", id="rerun-launch", compact=True)
                        yield Button("Stop", id="rerun-stop", disabled=True, compact=True)
                        yield Button("Update list now (r)", id="rerun-refresh", compact=True)
                    yield RichLog(id="rerun-log", max_lines=2000)
                with TabPane("Parameters", id="tab-params"):
                    yield ParametersPane(id="params")
        yield Footer()

    def on_mount(self):
        topics_table = self.query_one("#topic-table", DataTable)
        # what changes first, after the key: on a narrow terminal the rest is a horizontal scroll away
        # ages: median / p95 over the last second; at send on the publisher's clock alone, at receive with the way
        # here and both clocks
        for label, key in (("topic", "key"), ("Hz", "hz"), ("published by", "by"), ("subscribed by", "subs"),
                           ("taken as", "transport"), ("kB", "kb"), ("age at send ms", "send"),
                           ("age at receive ms", "age"), ("type", "type"), ("stamp", "stamp")):
            topics_table.add_column(label, key=key)
        nodes = self.query_one("#node-table", DataTable)
        for key in tui_nodes.COLUMNS:
            nodes.add_column(key, key=key)
        reports = self.query_one("#report-table", DataTable)
        for label, key in (("topic", "topic"), ("name", "name"), ("level", "level"), ("message", "message"),
                           ("age s", "age"), ("session", "session")):
            reports.add_column(label, key=key)
        self._load_presets()
        self._update_command()
        self.query_one("#topic-table", DataTable).focus()
        self.set_interval(1.0, self._tick)
        self._open(self.connection)

    def action_tab(self, tab: str):
        """Shows a tab with its main widget focused: a pane's widget taking the focus makes its pane the active one."""
        self.query_one(self.MAIN[tab]).focus()

    def on_tabbed_content_tab_activated(self, event: TabbedContent.TabActivated):
        """A tab clicked: its main widget takes the keys (arrows, Enter)."""
        main = self.MAIN.get(event.pane.id)
        if main is not None and self.focused not in event.pane.query(main):
            self.query_one(main).focus()

    # --------------------------------------------------------------------------------------------- the session

    def action_connect(self):
        self.push_screen(ConnectScreen(self.connection), self._connect_to)

    def _connect_to(self, connection: Connection | None):
        if connection is not None:
            self._open(connection)

    def _open(self, connection: Connection):
        """Closes the session (in the thread that opens the next) and opens ``connection``'s."""
        old = self._close_session()
        self.connection = connection
        self.generation += 1
        self.error = None
        self._status(f"connecting: {connection.describe()}")
        # A thread of its own, not a Textual worker: the app's end waits for its workers (asyncio's default executor),
        # and zenoh.open cannot be interrupted, so the interface could not end while a connect is pending. Not a
        # daemon either: a daemon thread inside Zenoh at the interpreter's exit aborts the process. The process ends
        # when the open returns; the thread then closes what it opened.
        threading.Thread(target=self._open_thread, args=(connection, old, self.generation), name="xblick open").start()

    def _open_thread(self, connection: Connection, old: zenoh.Session | None, generation: int):
        if old is not None:
            zsession.close(old)
        try:
            session = connection.open()
        except Exception as error:  # noqa: BLE001 - shown: what Zenoh said, nothing opened
            with self._open_lock:
                self._call_soon(functools.partial(self._open_failed, generation, f"{type(error).__name__}: {error}"))
            return
        with self._open_lock:
            if self._call_soon(functools.partial(self._opened, generation)):
                self._handed[generation] = session  # taken by _opened, or closed by on_unmount
                return
        zsession.close(session)  # the app has ended

    def _call_soon(self, callback) -> bool:
        """Has the event loop call ``callback``, without waiting for it (call_from_thread would wait for good if the
        loop stopped first); False when the app has ended. Under ``_open_lock``: on_unmount, which takes it, comes
        before the loop stops."""
        return not self._unmounted and self.post_message(events.Callback(callback))

    def _open_failed(self, generation: int, error: str):
        if generation == self.generation:
            self.error = f"not connected ({self.connection.describe()}): {error}"
            self._status()
            self.notify(self.error, severity="error", timeout=10)
            self._offer_connect()

    def _offer_connect(self):
        """Started on this PC's router and found nothing there: the connect dialog, once, on the address."""
        if self._auto_connect and not isinstance(self.screen, ConnectScreen):
            self._auto_connect = False
            self.push_screen(ConnectScreen(self.connection, "nothing found on this PC's router: give the address of "
                                           f"another (a host alone: tcp, port {zsession.DEFAULT_PORT})"),
                             self._connect_to)

    def action_refresh(self):
        """Looks at the network again now (streams, sessions, tokens), instead of at the next period."""
        self._discover_now.set()
        self._looking = True
        self._show_rerun_header()

    def _opened(self, generation: int):
        with self._open_lock:
            session = self._handed.pop(generation, None)
        if session is None:  # closed by on_unmount
            return
        if generation != self.generation:  # another connection was asked for meanwhile
            threading.Thread(target=zsession.close, args=(session,), name="xblick close").start()
            return
        self.session = session
        self.discovery = Discovery()
        with self._reports_lock:
            self._reports.clear()
        self._diagnostics_subscriber = session.declare_subscriber("**/" + topics.DIAGNOSTICS, self._on_diagnostics)
        self._programs_subscriber = session.liveliness().declare_subscriber(
            declaration.ALL, lambda sample: self._discover_now.set())
        self.run_worker(functools.partial(self._discover_thread, session, generation), thread=True,
                        group="discover", exclusive=True)
        self._status()

    def _close_session(self) -> zenoh.Session | None:
        """Undeclares the interface's subscribers; the session, to be closed by the caller (or None)."""
        self.workers.cancel_group(self, "discover")
        for meter in (self.meter, self.detail):
            if meter is not None:
                meter.close()
        measuring = self.meter is not None
        self.meter = self.detail = None
        self._measure_after_open = measuring
        if self._diagnostics_subscriber is not None:
            self._diagnostics_subscriber.undeclare()
            self._diagnostics_subscriber = None
        if self._programs_subscriber is not None:
            self._programs_subscriber.undeclare()
            self._programs_subscriber = None
        session, self.session = self.session, None
        return session

    def _on_diagnostics(self, sample: zenoh.Sample):
        try:
            camera, array = diagnostics.read(sample)
            sender = diagnostics.sender(sample)
        except Exception as error:  # noqa: BLE001 - shown in the reports: a publisher of another version
            with self._reports_lock:
                self._report_errors = [f"{sample.key_expr}: {error}"]
            return
        now = time.time_ns()
        with self._reports_lock:
            for status in array.status:
                self._reports[(camera, status.name)] = (now, status, sender)

    def _discover_thread(self, session: zenoh.Session, generation: int):
        worker = get_current_worker()
        missing_since: dict[str, float] = {}  # peers the admin space shows and this peer has no link to, since when
        while not worker.is_cancelled:
            started = time.monotonic()
            try:
                found = Discovery(survey.query_network(session, 1.0), find_streams(session, "", 1.0),
                                  survey.liveliness_tokens(session, 0.5), statuses(session, 0.5))
                found.network.programs, found.program_errors = declaration.query(session, 0.5)
                found.origins = sorted(str(z) for z in session.info.routers_zid())
                found.unreachable = self._unreachable(session, found.network, missing_since)
                found.drawn_over = self._rerun_drawn_over(session)
                if getattr(session, "ros", None) is not None:  # its graph made at the first look, filled after
                    graph = session.ros.graph
                    found.ros_cameras = ros.camera_streams(graph)
                    found.ros_publishers = [t for t in graph.publishers() if t.type_name in ros.CAMERA_TYPES
                                            and not graph.by_xblick(t.zid)]
                found.took_s = time.monotonic() - started
            except Exception as error:  # noqa: BLE001 - shown; discovery goes on
                if worker.is_cancelled:
                    return
                found = Discovery(error=f"{type(error).__name__}: {error}")
            if worker.is_cancelled:
                return
            try:
                self.call_from_thread(self._discovered, generation, found)
            except RuntimeError:  # the app has ended
                return
            while (not worker.is_cancelled and time.monotonic() - started < self.discover_every
                   and not self._discover_now.is_set()):
                time.sleep(0.05)
            self._discover_now.clear()

    def _unreachable(self, session: zenoh.Session, network, missing_since: dict[str, float]) -> list[str]:
        """As a peer: the peers of the admin space it has had no link to for UNREACHABLE_S (their locators are not
        reachable from here, e.g. another host's loopback over a tunnel: their streams' queries go unanswered)."""
        if self.connection.mode != "peer" or self.connection.config or network is None:
            return []
        own = str(session.info.zid())
        linked = {str(z) for z in session.info.peers_zid()}
        now = time.monotonic()
        missing = {z for z, n in network.nodes.items() if n.whatami == "peer" and z != own and z not in linked}
        for zid in list(missing_since):
            if zid not in missing:
                del missing_since[zid]
        for zid in missing:
            missing_since.setdefault(zid, now)
        return sorted(z for z in missing if now - missing_since[z] >= UNREACHABLE_S)

    def _discovered(self, generation: int, found: Discovery):
        if generation != self.generation or self._ended():
            return
        self.discovery = found
        self._discovered_at = time.time()
        self.uses = self._find_uses()
        if self._auto_connect:
            own = str(self.session.info.zid()) if self.session else ""
            others = [z for z, n in (found.network.nodes.items() if found.network else ()) if n.whatami != "router"
                      and z != own]
            if found.network is None or not found.network.routers or not (others or found.streams):
                self._offer_connect()
            else:
                self._auto_connect = False
        self._status()
        self._show_nodes()
        self._show_graph()
        self._show_rerun_streams()
        suffix = "/" + parameters.P.PARAMETERS_TOKEN  # the nodes offering parameters: their liveliness tokens
        self.query_one(ParametersPane).show_nodes(sorted(t[:-len(suffix)] for t in found.tokens if t.endswith(suffix)),
                                                  self.session)
        if self._measure_after_open and self.session is not None and self.meter is None:
            self._measure_after_open = False
            self.action_measure()

    def _status(self, text: str | None = None):
        status = self.query_one("#status", Static)
        if text is not None:
            status.update(Text(text, style="yellow"))
            return
        if self.error:
            status.update(Text(self.error, style="bold red"))
            return
        d = self.discovery
        line, counts = Text(), Text()
        if d.network is None:
            line.append(f"{self.connection.describe()}  ")
            line.append("looking..." if d.error is None else "", style="yellow")
        elif not d.network.routers:
            line.append(f"{self.connection.describe()}  ")
            line.append("no router answered (admin space): nodes and graph unknown; connect to a router",
                        style="bold yellow")
        else:
            line.append(self._connected_to())
            nodes = sum(1 for _, members in d.network.groups(d.origins) for _ in members)
            counts.append(f"{_plural(len(d.network.routers), 'router')}, {_plural(nodes, 'node')}, "
                          f"{_plural(len(d.streams), 'stream')}")
        if d.error:
            line.append(f"  discovery failed: {d.error}", style="bold red")
        if d.unreachable:
            line.append(f"  {len(d.unreachable)} peer(s) not reachable from here (their addresses are not this "
                        "host's: a tunnel?): connect as a client (c, client mode; xblick --connect <router>)",
                        style="bold yellow")
        if self.meter is not None:
            line.append("  measuring every topic (m: stop)", style="magenta")
        grid = Table.grid(expand=True)
        grid.add_column(ratio=1)
        grid.add_column(justify="right", no_wrap=True)
        grid.add_row(line, counts)
        status.update(grid)

    def _connected_to(self) -> str:
        """``connected to router <id> on <host> (<address>)``: the router this interface is linked to (several: each),
        and the address it was given (this PC's router: 127.0.0.1:7447)."""
        network, origins = self.discovery.network, self.discovery.origins
        address = (f"config {self.connection.config}" if self.connection.config
                   else " ".join(self.connection.connect) or zsession.ROUTER)
        if not origins:
            return f"not linked to a router ({self.connection.describe()})  "
        routers = ", ".join(f"{_short(z)} on {network.routers[z].host or '–'}" if z in network.routers else _short(z)
                            for z in origins)
        return f"connected to router {routers} ({address})  "

    # ---------------------------------------------------------------------------------------------- the topics

    def action_measure(self):
        """Subscribes to every topic (rates, sizes, types), or stops it."""
        if self.meter is not None:
            self.meter.close()
            self.meter = None
            self._rates, self._before = {}, None
            self._show_graph()  # its lines without the rates at once
        elif self.session is None:
            self.notify("not connected", severity="error")
        else:
            self.meter = survey.TrafficMeter(self.session, "**")
            self.notify("subscribed to every topic: publishers now produce every representation (m to stop)")
        self._status()

    def action_close_detail(self):
        if self.detail is not None:
            self.detail.close()
        self.detail, self.detail_key = None, None
        self.query_one("#topic-detail", Static).update("Enter on a topic: its publishers, subscribers and last message")

    def _find_uses(self) -> dict[str, list[survey.Use]]:
        network = self.discovery.network
        if network is None:
            return {}
        own = [str(self.session.info.zid())] if self.session else []
        return {key: uses for key in network.publishers() if (uses := network.uses(key, leave_out=own))}

    def _stream_uses(self, base: str) -> list[survey.Use]:
        return [u for key, uses in sorted(self.uses.items()) if key.startswith(base + "/") for u in uses]

    def _labels(self) -> dict[str, str]:
        """A name per session: what it declared (its names, else its program), else what its keys share."""
        network = self.discovery.network
        own = str(self.session.info.zid()) if self.session else ""
        labels = {}
        if network is not None:
            for zid in set(network.nodes) | set(network.programs):
                program = network.program(zid)
                node = network.nodes.get(zid)
                name = tui_nodes.name_of(network, zid)
                labels[zid] = (name if name != tui_nodes.DASH else program.program if program is not None
                               else node.label if node is not None else _short(zid))
        labels[own] = "this interface"
        return labels

    def _ended(self) -> bool:
        """The app is ending: its widgets are being removed while timers and the discovery thread still call in."""
        try:
            return not self.is_running or not self.screen_stack[0].query("#topic-table")
        except IndexError:  # no screen any more
            return True

    def _tick(self):
        if self._ended():
            return
        if self.meter is not None:
            now, snapshot = time.monotonic(), self.meter.snapshot()
            if self._before is not None:
                self._rates = survey.rates(self._before[1], snapshot, now - self._before[0])
            self._before = (now, snapshot)
        # the ages of the last second, by key: of every topic measured, else of the selected one
        ages = self.detail.take_ages() if self.detail is not None else {}
        if self.meter is not None:
            ages.update(self.meter.take_ages())
        self._ages = {key: (survey.median_p95(sends), survey.median_p95(receives))
                      for key, (sends, receives) in ages.items()}
        self._show_topics()
        self._show_reports()
        self._show_detail()
        self._show_rerun_header()  # "updated N s ago"
        if self.meter is not None:  # the rates on the graph's lines
            self._show_graph()

    def _show_topics(self):
        """The keys as a tree (``topic_tree``): a row per key and per prefix, a prefix's publishers and subscribers
        once on it when every key below has the same; rates, sizes and ages where measured."""
        table = self.query_one("#topic-table", DataTable)
        network = self.discovery.network
        publishers = network.publishers() if network else {}
        traffic = self.meter.snapshot() if self.meter is not None else {}
        if self.detail is not None:
            traffic.update(self.detail.snapshot())
        labels = short_names(self._labels())
        own = str(self.session.info.zid()) if self.session else ""
        # a router subscribes in the admin space for the sessions behind it, which stand here themselves
        subscriptions = ([(n.zid, zenoh.KeyExpr(s)) for n in network.nodes.values()
                          if n.zid != own and n.zid not in network.routers for s in n.subscribes] if network else [])
        now = time.time_ns()
        keys = set(publishers) | set(traffic)

        def names(zids) -> str:
            return ", ".join(sorted({labels.get(z, _short(z)) for z in zids}))

        def subscribers(key: str) -> frozenset:
            expr = zenoh.KeyExpr(key)
            return frozenset(z for z, s in subscriptions if z not in publishers.get(key, ()) and s.intersects(expr))

        cells = {key: (frozenset(publishers.get(key, ())), subscribers(key)) for key in keys}
        rows = {}
        self._topic_rows = {row.path: row for row in topic_tree(keys, self._collapsed, cells)}
        for row in self._topic_rows.values():
            if row.key is None:  # a prefix: its sessions when the same for every key below it
                pubs, subs = row.common(cells)
                rows[row.path] = (Text(row.label, style="bold"), "", Text(names(pubs) if row.show_pub else ""),
                                  Text(names(subs) if row.show_sub else ""), "", "", "", "", "", "")
                continue
            key = row.key
            t = traffic.get(key)
            hz, size = self._rates.get(key, (None, None))
            send, receive = self._ages.get(key, (None, None))
            pubs, subs = cells[key]
            rows[row.path] = (Text(row.label), f"{hz:.1f}" if hz is not None else "",
                              Text(names(pubs) if row.show_pub else ""), Text(names(subs) if row.show_sub else ""),
                              Text(_count(self.uses.get(key, []))), f"{size / 1e3:.1f}" if size else "",
                              _ms_pair(send), _ms_pair(receive) if receive else (
                                  _ms((now - t.stamp_ns) / 1e6) if t and t.stamp_ns else ""),
                              t.encoding.removeprefix("application/cdr;") if t else "", _clock(t.stamp_ns) if t else "")
        _sync(table, rows, ordered=True)

    def on_data_table_row_selected(self, event: DataTable.RowSelected):
        if event.data_table.id == "topic-table":
            row = self._topic_rows.get(event.row_key.value)
            if row is None:
                return
            if row.key is None:  # a prefix: folded or unfolded
                self._collapsed ^= {row.path}
                self._show_topics()
            else:
                self._select_topic(row.key)
        elif event.data_table.id == "node-table":
            self._selected_node = event.row_key.value
            self._show_node_detail()

    def on_data_table_row_highlighted(self, event: DataTable.RowHighlighted):
        """The Nodes tab's cursor on a node or a router: its detail at once."""
        if event.data_table.id == "node-table" and event.row_key is not None:
            kind = str(event.row_key.value).partition("\t")[0]
            if kind in ("node", "router"):
                self._selected_node = event.row_key.value
                self._show_node_detail()

    def on_resize(self, event: events.Resize):
        if not self._ended():
            self.call_after_refresh(self._show_nodes)  # the columns follow the width

    def _select_topic(self, key: str):
        if self.session is None:
            return
        if self.detail is not None:
            self.detail.close()
        self.detail_key = key
        self.detail = survey.TrafficMeter(self.session, key)
        self.detail.keep.add(key)
        self._show_detail()

    def _show_detail(self):
        if self.detail is None or self.detail_key is None:
            return
        key = self.detail_key
        network = self.discovery.network
        labels = self._labels()
        text = Text()
        text.append(key + "\n", style="bold")
        if network is not None:
            pubs = network.publishers().get(key, set())
            text.append("published by: " + (", ".join(f"{labels.get(z)} ({_short(z)})" for z in sorted(pubs))
                                            or "no session the routers know") + "\n")
            expr = zenoh.KeyExpr(key)
            subs = sorted({f"{labels.get(n.zid)} ({s})" for n in network.nodes.values() for s in n.subscribes
                           if zenoh.KeyExpr(s).intersects(expr)})
            text.append("subscribed by: " + (", ".join(subs) or "nobody") + "\n")
        # the message before the long parts (the stream's record, the consumers' ways): a small pane shows it first
        tail = Text()
        for base, record in self.discovery.streams:
            if key.startswith(base + "/"):
                text.append(f"offers {', '.join(record.get('transports', [])) or 'nothing it says'}; in use: "
                            f"{_count(self._stream_uses(base)) or 'nothing'}\n")
                tail.append(f"stream {base}: {json.dumps(record)}\n")
        if key in self.uses:
            tail.append("taken by:\n", style="bold")
            for use in self.uses[key]:
                tail.append(f"  {labels.get(use.consumer, _short(use.consumer))} ({use.expr}): {use.summary}, "
                            f"{use.detail}\n")
        t = self.detail.snapshot().get(key)
        last = self.detail.last(key)
        widget = self.query_one("#topic-detail", Static)
        if t is None or last is None:
            text.append("waiting for a message...\n", style="yellow")
            widget.update(Group(text, tail))
            return
        text.append(f"{t.count} messages since selected, last {len(last[1]) / 1e3:.1f} kB, {t.encoding}, "
                    f"stamp {_clock(t.stamp_ns) or 'none'}\n")
        recent = self.detail.recent(key)[-5:]
        if recent:  # per sample: at send exact (the publisher's clock), at receive with the way and both clocks
            text.append("age ms at send / at receive, the last samples: " + ", ".join(
                f"{_ms(s / 1e6) if s is not None else '-'} / {_ms(r / 1e6) if r is not None else '-'}"
                for _, s, r in recent) + "\n")
        try:
            body = Pretty(survey.content(*last), expand_all=False)
        except Exception as error:  # noqa: BLE001 - shown: a message this version cannot read
            body = Text(f"cannot show it: {type(error).__name__}: {error}", style="bold red")
        widget.update(Group(text, body, tail))

    # ----------------------------------------------------------------------------------------------- the nodes

    def _show_nodes(self):
        """Every node by router (tui_nodes.node_rows), the columns cut to the table's width."""
        if self._ended():
            return
        network = self.discovery.network
        table = self.query_one("#node-table", DataTable)
        rows = {}
        if network is not None and (network.routers or network.programs):
            own = str(self.session.info.zid()) if self.session else ""
            width = max(table.scrollable_content_region.width, self.size.width - 4, sum(tui_nodes.MIN.values()))
            head = table.get_component_rich_style("datatable--header")
            found, widths = tui_nodes.node_rows(network, own, self.discovery.origins, width, socket.gethostname())
            rows = {key: tui_nodes.styled(row, head) for key, row in found.items()}
            if [c.width for c in table.columns.values()] != widths:  # fixed widths: an auto width never shrinks
                cursor = table.cursor_row
                table.clear(columns=True)
                for key, w in zip(tui_nodes.COLUMNS, widths):
                    table.add_column(key, key=key, width=w)
                _sync(table, rows, ordered=True)
                if rows:
                    table.move_cursor(row=min(cursor, len(rows) - 1))
        _sync(table, rows, ordered=True)
        self._show_node_detail()

    def _show_reports(self):
        table = self.query_one("#report-table", DataTable)
        now = time.time_ns()
        rows = {}
        with self._reports_lock:
            reports = dict(self._reports)
            errors = list(self._report_errors)
        labels = self._labels()
        for (camera, name), (received, status, sender) in reports.items():
            level = diagnostics.level_name(status.level)
            style = {"OK": "green", "WARN": "yellow", "ERROR": "bold red", "STALE": "dim"}.get(level, "")
            rows[f"{camera}\t{name}"] = (Text(topics.diagnostics(camera)), Text(name), Text(level, style=style),
                                         Text(status.message), f"{(now - received) / 1e9:.1f}",
                                         Text(_sender(sender, labels)))
        for key, status in self.discovery.statuses.items():
            state = status.get("state", "running" if status.get("kind") == "mps" else "")
            rows[key] = (Text(key), Text(str(status.get("name", ""))), Text(str(state)), Text(_status_line(status)),
                         "", "")
        for error in errors:
            rows["error\t" + error] = (Text("error", style="bold red"), "", "", Text(error, style="red"), "", "")
        _sync(table, rows)

    def _show_node_detail(self):
        selected = self._selected_node
        network = self.discovery.network
        widget = self.query_one("#node-detail", Static)
        if selected is None or network is None:
            return
        kind, _, zid = selected.partition("\t")
        if kind == "router":
            if zid in network.routers:
                widget.update(tui_nodes.router_detail(network, zid, self.discovery.origins))
            else:
                widget.update(Text(f"router {zid}: gone", style="yellow"))
            return
        if zid not in network.nodes and zid not in network.programs:
            widget.update(Text(f"{zid}: gone", style="yellow"))
            return
        readers: dict[str, list[str]] = {}
        for _pub, key, sub, _expr in network.connections():
            readers.setdefault(key, []).append(sub)
        text = tui_nodes.node_detail(network, zid, readers, self._labels())
        node = network.nodes.get(zid)
        if node is None:
            widget.update(text)
            return
        with self._reports_lock:
            reports = dict(self._reports)
        own = sorted((c, n, s) for (c, n), (_, s, sender) in reports.items() if sender == zid)
        if own:
            text.append("reports:\n", style="bold")
            for camera, name, status in own:
                values = ", ".join(f"{kv.key} {kv.value}" for kv in status.values)
                text.append(f"  {topics.diagnostics(camera)} {name} {diagnostics.level_name(status.level)}: "
                            f"{status.message}" + (f"  [{values}]" if values else "") + "\n")
        cameras = {k[: -len(topics.DIAGNOSTICS) - 1] for k in node.publishes if k.endswith("/" + topics.DIAGNOSTICS)}
        unknown = sorted(f"{topics.diagnostics(c)} {n}" for (c, n), (_, _, sender) in reports.items()
                         if sender is None and c in cameras)
        if unknown:
            text.append("reports on its diagnostics keys without a sender (Durchblick before 0.3.29), any of them "
                        "may be its own: " + ", ".join(unknown) + "\n", style="yellow")
        labels = self._labels()
        for base, record in self.discovery.streams:
            if any(k.startswith(base + "/") for k in node.publishes):
                text.append(f"stream {base} offers {', '.join(record.get('transports', []))}; in use: "
                            f"{_count(self._stream_uses(base)) or 'nothing'}\n")
        given = [(key, u) for key in sorted(node.publishes) for u in self.uses.get(key, [])]
        if given:
            text.append("its keys taken by:\n", style="bold")
            text.append("".join(f"  {key} -> {labels.get(u.consumer, _short(u.consumer))}: {u.summary}\n"
                                for key, u in given))
        takes = [(key, u) for key, uses in sorted(self.uses.items()) for u in uses if u.consumer == zid]
        if takes:
            text.append("takes:\n", style="bold")
            text.append("".join(f"  {key}: {u.summary}, {u.detail}\n" for key, u in takes))
        for key in sorted(node.queryables):
            if key in self.discovery.statuses:
                text.append(f"{key}:\n{json.dumps(self.discovery.statuses[key], indent=1)}\n")
        widget.update(text)

    # ----------------------------------------------------------------------------------------------- the graph

    def _graph_parts(self) -> tuple[list[GraphNode], list[GraphEdge], dict[tuple[str, str], list[tuple[str, str]]],
                                    list[GraphGroup], list[tuple[str, str]]]:
        """The graph's boxes (nodes but the routers and this interface: titled by their name, else their program; in
        them the program and the topics nobody takes), its edges (one per topic from its publisher to each reader,
        labelled with the topic, how it is taken and its rate while measured), per pair of nodes its keys with how each
        is taken, the boxes' groups (one per router, as the Nodes tab orders them) and the links between routers.

        A rate is the messages per second of the topic's key as this interface receives them: only while ``m``
        measures every topic (the traffic meter), of the key the reader takes (the highest of a topic's keys). A
        node's own report (diagnostics) is one rate per node, so it stays in the detail."""
        network = self.discovery.network
        if network is None:
            return [], [], {}, [], []
        own = str(self.session.info.zid()) if self.session else ""
        keys: dict[tuple[str, str], list[tuple[str, str]]] = {}
        topics_of: dict[tuple[str, str, str], list[tuple[str, str]]] = {}  # (pub, sub, topic) -> (key, taken)
        names = {zid: tui_nodes.name_of(network, zid) for zid in network.nodes}
        for pub, key, sub, _expr in network.connections(leave_out=[own, *network.routers]):
            taken = next((u.summary for u in self.uses.get(key, []) if u.consumer == sub), "")
            pair = keys.setdefault((pub, sub), [])
            if (key, taken) not in pair:
                pair.append((key, taken))
                topic = _topic_name(key, names.get(pub, tui_nodes.DASH))
                topics_of.setdefault((pub, sub, topic), []).append((key, taken))
        nodes = []
        titles = {}
        ros_nodes, ros_topics = self._ros_parts()
        for zid in network.nodes:
            name = tui_nodes.name_of(network, zid)
            program = network.program(zid)
            titles[zid] = name if name != tui_nodes.DASH else (
                program.program if program else ", ".join(ros_nodes[zid]) + ROS_MARK if zid in ros_nodes
                else _short(zid))
        short = short_names(titles)
        groups = []
        for router, members in network.groups(self.discovery.origins, leave_out=[own]):
            members = [z for z in members if z in network.nodes]
            if router is None:
                groups.append(GraphGroup("\0none", "without a router", members))
            else:
                groups.append(GraphGroup(router, f"router {_short(router)}  {network.routers[router].host or '–'}",
                                    members))
        for n in sorted((n for n in network.nodes.values() if n.zid != own and n.whatami != "router"),
                        key=lambda n: (titles[n.zid], n.zid)):
            program = network.program(n.zid)
            name = names[n.zid]
            lines = [program.program if program is not None else "program –"] if name != tui_nodes.DASH else []
            if name == tui_nodes.DASH and program is None and n.zid in ros_topics:  # a ROS node: its camera topics
                shown_ros = ros_topics[n.zid]
                lines += [f"▷ {t}" for t in shown_ros[:4]] + ([f"▷ {len(shown_ros) - 4} more ROS topics"]
                                                             if len(shown_ros) > 4 else [])
            taken = {topic for (pub, _sub, topic) in topics_of if pub == n.zid}
            unread = [k for k in sorted(n.publishes) if not declaration.zid_of(k)
                      and k.rsplit("/", 1)[-1] not in tui_nodes.LEFT_OUT
                      and _topic_name(k, name) not in taken]
            by_topic: dict[str, list[str]] = {}
            for k in unread:
                by_topic.setdefault(_topic_name(k, name), []).append(k)
            shown = sorted(by_topic)
            for topic in shown[:4]:  # what nobody takes, with its rate while measured
                lines.append(f"▷ {topic}" + _rate_text(max((self._rates.get(k, (0.0, 0))[0] for k in by_topic[topic]),
                                                           default=0.0), " "))
            if len(shown) > 4:
                lines.append(f"▷ {len(shown) - 4} more nobody takes")
            nodes.append(GraphNode(n.zid, short[n.zid], lines))
        edges = []
        self._graph_edge_keys = []
        for (pub, sub, topic), pairs in sorted(topics_of.items()):
            ways = sorted({t for _, t in pairs if t})
            hz = max((self._rates.get(k, (0.0, 0))[0] for k, _ in pairs), default=0.0)
            label = " ".join([topic, *ways[:1]]) + _rate_text(hz, " ")
            if len(label) > LABEL:  # the topic and its rate first: how it is taken is in the detail
                label = topic + _rate_text(hz, " ")
            edges.append(GraphEdge(pub, sub, label))
            self._graph_edge_keys.append(pairs)
        links = sorted(tuple(sorted(link)) for link in network.links() if link <= set(network.routers))
        return nodes, edges, keys, groups, links

    def _ros_parts(self) -> tuple[dict[str, list[str]], dict[str, list[str]]]:
        """Of the ROS publishers of camera topics that are no programs of ours (``Discovery.ros_publishers``): per
        session (zid) its ROS nodes and its topics, sorted."""
        nodes: dict[str, set[str]] = {}
        topics_: dict[str, set[str]] = {}
        for t in self.discovery.ros_publishers:
            nodes.setdefault(t.zid, set()).add(t.fully_qualified_node)
            topics_.setdefault(t.zid, set()).add(t.topic)
        return ({z: sorted(n) for z, n in nodes.items()}, {z: sorted(t) for z, t in topics_.items()})

    def _show_graph(self):
        nodes, edges, self._graph_keys, groups, links = self._graph_parts()
        self.query_one("#graph", GraphView).show(nodes, edges, groups, links)
        self._show_graph_detail()

    def on_graph_view_selected(self, event: GraphView.Selected):
        self._show_graph_detail()

    def on_graph_view_hovered(self, event: GraphView.Hovered):
        self._graph_hovered = event.what
        self._show_graph_detail()

    def _show_graph_detail(self):
        """What the mouse is over, else the selected session: its reports, what it sends and what it takes."""
        if self._ended():  # a selection's event that arrives while the app ends (its widgets are gone)
            return
        graph = self.query_one("#graph", GraphView)
        widget = self.query_one("#graph-detail", Static)
        labels = self._labels()
        keys = getattr(self, "_graph_keys", {})
        what = self._graph_hovered or (("node", graph.selected) if graph.selected else None)
        if what is None:
            widget.update(GRAPH_HELP)
            return
        text = Text()
        if what[0] == "edges":
            for k in sorted(what[1]):
                if k >= len(graph.edges):
                    continue
                e = graph.edges[k]
                text.append(f"{labels.get(e.source, _short(e.source))} -> {labels.get(e.target, _short(e.target))}\n",
                            style="bold yellow")
                edge_keys = getattr(self, "_graph_edge_keys", [])
                text.append("".join(f"  {key}" + (f"  [{taken}]" if taken else "")
                                    + _rate_text(self._rates.get(key, (0.0, 0))[0], "  ") + "\n"
                                    for key, taken in (edge_keys[k] if k < len(edge_keys) else [])))
            widget.update(text)
            return
        zid = what[1]
        node = next((n for n in graph.nodes if n.id == zid), None)
        if node is None:
            widget.update(GRAPH_HELP)
            return
        network = self.discovery.network
        n = network.nodes.get(zid) if network is not None else None
        program = network.program(zid) if network is not None else None
        text.append(f"{labels.get(zid, node.title)}  ({program.program if program else 'program –'}, "
                    f"{n.whatami if n else '–'} {_short(zid)})\n", style="bold magenta")
        with self._reports_lock:
            reports = dict(self._reports)
        for (camera, name), (_, status, sender) in sorted(reports.items()):
            if sender == zid:  # its own rate, one per node: in the detail, not on its topics' lines
                text.append(f"  reports {name} on {topics.diagnostics(camera)}: {diagnostics.level_name(status.level)} "
                            f"{status.message}\n")
        for (pub, sub), pairs in sorted(keys.items()):
            if pub == zid:
                text.append(f"  sends to {labels.get(sub, _short(sub))}: ", style="green")
                text.append(", ".join(k + (f" [{t}]" if t else "") for k, t in pairs) + "\n")
        for (pub, sub), pairs in sorted(keys.items()):
            if sub == zid:
                text.append(f"  takes from {labels.get(pub, _short(pub))}: ", style="cyan")
                text.append(", ".join(k + (f" [{t}]" if t else "") for k, t in pairs) + "\n")
        widget.update(text)

    # ----------------------------------------------------------------------------------------------- rerun

    def _show_rerun_streams(self):
        """The tree of what was found, and the marked entries that were not (gone, or a preset's not running); kept
        as it is (cursor, folds) while nothing in it changed."""
        found = rerun_entries(self.discovery.streams, self.discovery.network, self.discovery.ros_cameras)
        now = time.time()
        for e in found:
            self._rerun_seen[e.id] = now
        self._rerun_entries = found
        self._looking = False
        self._show_rerun_header()
        self._show_rerun_tree()

    def _rerun_notes(self) -> dict[str, str]:
        """Entry id -> its note: where the running xblick rerun draws it (its drawn_over), else what the records tell."""
        drawn = self.discovery.drawn_over if self._rerun_key is not None else None
        notes = {}
        for e in self._rerun_entries:
            images = (drawn or {}).get(e.key)
            notes[e.id] = (f"drawn over {', '.join(i[len(e.camera) + 1:] for i in images)}" if images else e.note)
        return notes

    def _show_rerun_tree(self):
        listed = {e.id: e for e in self._rerun_entries}
        missing = [rerun_entry(*mark.split("\t")) for mark in sorted(self._rerun_marks) if mark not in listed]
        entries = list(listed.values()) + missing
        notes = self._rerun_notes()
        gone = {}
        for e in missing:
            seen = self._rerun_seen.get(e.id)
            gone[e.id] = (f"gone since {time.strftime('%H:%M:%S', time.localtime(seen))}" if seen is not None
                          else "not found")
        shown = (tuple(sorted(listed)), tuple(sorted(gone.items())), tuple(sorted(notes.items())),
                 tuple(sorted(self._rerun_marks)))
        if shown == self._rerun_shown:
            return
        tree = self.query_one("#rerun-streams", RerunTree)
        groups = rerun_groups(entries)
        structure = [(camera, node, e.id) for camera, nodes in groups.items() for node, es in nodes.items()
                     for e in es]
        depth = tree.guide_depth
        width = max((len(e.label) + (depth if e.node else 0) for e in entries), default=0)
        if self._rerun_shown is None or [s[2] for s in structure] != list(self._rerun_leaves):
            cursor = tree.cursor_node.data if tree.cursor_node is not None else None
            tree.clear()
            self._rerun_leaves = {}
            for camera, nodes in groups.items():
                branch = tree.root.add(camera, expand=camera not in self._rerun_folded, data=f"\0{camera}")
                for node, es in nodes.items():
                    at = branch
                    if node:
                        path = f"{camera}/{node}"
                        at = branch.add(node, expand=path not in self._rerun_folded, data=f"\0{path}")
                    for e in es:
                        self._rerun_leaves[e.id] = at.add_leaf("", data=e.id)
            if cursor in self._rerun_leaves or (cursor is None and self._rerun_leaves):
                # on the first entry from the start, so that Space marks it
                # after the refresh: the tree places its new nodes on lines only then
                tree.call_after_refresh(tree.move_cursor, self._rerun_leaves.get(cursor)
                                        or next(iter(self._rerun_leaves.values())))
        by_id = {e.id: e for e in entries}
        for entry_id, leaf in self._rerun_leaves.items():
            e = by_id[entry_id]
            leaf.set_label(entry_text(e, entry_id in self._rerun_marks, notes.get(entry_id, ""),
                                      width - (depth if e.node else 0), gone.get(entry_id, "")))
        self._rerun_shown = shown
        self._update_command()

    def _show_rerun_header(self):
        """Above the tree: what was found, when, how often; whether marks apply to a running xblick rerun."""
        if self._ended():
            return
        header = Text()
        header.append(f"Found on the network: {rerun_counts(self._rerun_entries)}\n", style="bold")
        if self._looking or self._discovered_at is None:
            header.append("looking...", style="yellow")
        else:
            every = max(self.discover_every, self.discovery.took_s)
            header.append(f"updated {time.time() - self._discovered_at:.0f} s ago, updates every {every:.0f} s "
                          "(r: now)", style="dim")
        if self.discovery.network is not None and not self.discovery.network.routers:
            header.append("\nno router answered: bodies, IMU, diagnostics, transforms and poses are not listed",
                          style="yellow")
        if self._rerun is not None and self._rerun.poll() is None:
            header.append("\nmarks apply at once to the running xblick rerun" if self._rerun_live else
                          "\nthe running xblick rerun shows everything found: marks apply at its next start",
                          style="magenta")
        self.query_one("#rerun-found", Static).update(header)
        options = Text("Options of xblick rerun\n", style="bold")
        options.append("fixed while it runs: they apply at its start" if self._rerun is not None
                       and self._rerun.poll() is None else "set these before starting it", style="dim")
        self.query_one("#rerun-options", Static).update(options)

    def on_tree_node_selected(self, event: Tree.NodeSelected):
        """Space, Enter or a click on an entry: marked or unmarked (a branch folds by itself)."""
        data = event.node.data
        if not isinstance(data, str) or data.startswith("\0"):
            return
        entries = self._rerun_entries + [rerun_entry(*m.split("\t")) for m in self._rerun_marks
                                         if m not in {e.id for e in self._rerun_entries}]
        entry = next(e for e in entries if e.id == data)
        self._rerun_marks = toggle_mark(self._rerun_marks, entry, entries)
        self._show_rerun_tree()
        self._send_selection()

    def on_tree_node_collapsed(self, event: Tree.NodeCollapsed):
        if isinstance(event.node.data, str) and event.node.data.startswith("\0"):
            self._rerun_folded.add(event.node.data[1:])

    def on_tree_node_expanded(self, event: Tree.NodeExpanded):
        if isinstance(event.node.data, str) and event.node.data.startswith("\0"):
            self._rerun_folded.discard(event.node.data[1:])

    def selection(self) -> RerunSelection:
        """What the Rerun tab says now."""
        chosen, no_transforms = selected_of(self._rerun_marks)

        def number(widget_id: str) -> float | None:
            text = self.query_one(widget_id, Input).value.strip()
            return float(text) if text else None

        return RerunSelection(**chosen, no_transforms=no_transforms,
                              prefix=self.query_one("#rerun-prefix", Input).value.strip(),
                              stale_after=number("#rerun-stale-after"), cloud_rate=number("#rerun-cloud-rate"),
                              viewer=self.query_one("#rerun-viewer", Select).value,
                              viewer_value=self.query_one("#rerun-viewer-value", Input).value.strip(),
                              extra=self.query_one("#rerun-extra", Input).value.strip(), connection=self.connection)

    def apply(self, selection: RerunSelection):
        """Shows a selection (a preset) in the Rerun tab; while xblick rerun takes its selection live, it gets it."""
        self._rerun_marks = marks_of(selection)
        if self._rerun is None or self._rerun.poll() is not None:  # the options apply at the start
            self.query_one("#rerun-prefix", Input).value = selection.prefix
            self.query_one("#rerun-stale-after", Input).value = ("" if selection.stale_after is None
                                                                 else f"{selection.stale_after:g}")
            self.query_one("#rerun-cloud-rate", Input).value = ("" if selection.cloud_rate is None
                                                                else f"{selection.cloud_rate:g}")
            self.query_one("#rerun-viewer", Select).value = selection.viewer
            self.query_one("#rerun-viewer-value", Input).value = selection.viewer_value
            self.query_one("#rerun-extra", Input).value = selection.extra
        if selection.connection != self.connection:
            self.notify(f"the preset's connection: {selection.connection.describe()}")
            self._open(selection.connection)
        self._show_rerun_tree()
        self._update_command()
        self._send_selection()

    def _rerun_drawn_over(self, session: zenoh.Session) -> dict[str, list[str]] | None:
        """In the discovery thread: the running xblick rerun's drawn_over (a query; None while it does not answer)."""
        from .rerun_bridge import DRAWN_OVER

        key = self._rerun_key
        if key is None:
            return None
        try:
            value, = parameters.ParameterClient(session, key, timeout=0.5).get([DRAWN_OVER])
        except (TimeoutError, RuntimeError):
            return None  # starting or ended: the notes of the records meanwhile
        drawn = {}
        for line in value.value() or []:
            result, _, images = line.partition(": ")
            drawn[result] = [i for i in images.split(", ") if i]
        return drawn

    def _send_selection(self):
        """While xblick rerun takes its selection live: the marks to its parameters, in a thread (queries)."""
        if self._rerun_key is None or not self._rerun_live or self.session is None:
            return
        if self._rerun is None or self._rerun.poll() is not None:
            return
        self.run_worker(functools.partial(self._send_selection_thread, self.session, self._rerun_key), thread=True,
                        group="rerun-set")

    def _send_selection_thread(self, session: zenoh.Session, key: str):
        from .rerun_bridge import SELECTION_OPTIONS

        with self._rerun_set_lock:  # the newest marks, also when two of these threads overtook each other
            chosen, no_transforms = selected_of(self._rerun_marks)
            values = {name: (no_transforms if kind == "no_transforms" else chosen[kind])
                      for name, kind in SELECTION_OPTIONS.items()}
            try:
                result = parameters.ParameterClient(session, key, timeout=5.0).set_atomically(values)
                problem = None if result.successful else result.reason
            except (TimeoutError, RuntimeError) as error:
                problem = f"{type(error).__name__}: {error}"
        with contextlib.suppress(RuntimeError):  # the app has ended
            if problem:
                self.call_from_thread(self.notify, f"xblick rerun did not take the selection: {problem}",
                                      severity="error", timeout=10)
            else:
                self.call_from_thread(self._rerun_line, "selection set: " + (", ".join(
                    k for kind in RERUN_KINDS for k in chosen[kind]) or "nothing"), "dim")

    def _update_command(self):
        widget = self.query_one("#rerun-command", Static)
        try:
            args = self.selection().check()
        except (ValueError, OSError) as error:
            widget.update(Text(f"xblick rerun: {error}", style="bold red"))
            return
        widget.update(Text("xblick rerun " + shlex.join(args)))

    def on_input_changed(self, event: Input.Changed):
        if event.input.id and event.input.id.startswith("rerun-"):
            self._update_command()

    def on_select_changed(self, event: Select.Changed):
        if event.select.id == "rerun-viewer":
            self._update_command()

    def on_button_pressed(self, event: Button.Pressed):
        handlers = {"rerun-launch": self.launch, "rerun-refresh": self.action_refresh, "rerun-stop": self.stop_rerun,
                    "preset-save": self._save_preset, "preset-load": self._load_preset,
                    "preset-delete": self._delete_preset}
        if event.button.id in handlers:
            handlers[event.button.id]()

    def _load_presets(self) -> dict[str, RerunSelection]:
        try:
            presets = self.presets.load()
        except ValueError as error:
            self.notify(str(error), severity="error", timeout=10)
            presets = {}
        self.query_one("#preset-select", Select).set_options([(n, n) for n in sorted(presets)])
        return presets

    def _chosen_preset(self) -> str | None:
        value = self.query_one("#preset-select", Select).value
        if value is Select.BLANK:
            self.notify("choose a preset first", severity="error")
            return None
        return value

    def _save_preset(self):
        name = self.query_one("#preset-name", Input).value.strip()
        try:
            self.presets.save(name, self.selection())
        except (ValueError, OSError) as error:
            self.notify(f"not saved: {error}", severity="error", timeout=10)
            return
        self._load_presets()
        self.query_one("#preset-select", Select).value = name
        self.notify(f"preset {name!r} saved in {self.presets.path}")

    def _load_preset(self):
        name = self._chosen_preset()
        if name is None:
            return
        presets = self._load_presets()
        if name not in presets:
            self.notify(f"no preset {name!r} in {self.presets.path}", severity="error")
            return
        self.query_one("#preset-select", Select).value = name
        self.query_one("#preset-name", Input).value = name
        self.apply(presets[name])

    def _delete_preset(self):
        name = self._chosen_preset()
        if name is None:
            return
        try:
            self.presets.delete(name)
        except (KeyError, ValueError, OSError) as error:
            self.notify(str(error), severity="error")
            return
        self._load_presets()
        self.notify(f"preset {name!r} deleted")

    def launch(self):
        """Starts ``xblick rerun`` with the tab's selection as a child process; its output goes to the log."""
        if self._rerun is not None and self._rerun.poll() is None:
            self.notify("xblick rerun runs already: stop it first", severity="error")
            return
        if importlib.util.find_spec("rerun") is None:
            from .extras import missing_extra

            self.notify(missing_extra("xblick rerun needs the Rerun SDK", "rerun"), severity="error", timeout=10)
            return
        try:
            selection = self.selection()
            command = selection.command()
            key = selection.node_key()
        except (ValueError, OSError) as error:
            self.notify(f"xblick rerun: {error}", severity="error", timeout=10)
            return
        self._rerun_line("$ xblick " + shlex.join(command[3:]), "bold")
        # started with a selection it takes a changed one at once; without one it shows everything found
        self._rerun_key, self._rerun_live = key, any(selection.selected().values())
        # a session of its own (no Ctrl-C of the terminal); stopped by PID with SIGTERM, its Ctrl-C
        self._rerun = subprocess.Popen(command, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                                       stderr=subprocess.STDOUT, text=True, bufsize=1, start_new_session=True,
                                       env={**os.environ, "PYTHONUNBUFFERED": "1"})
        self.query_one("#rerun-stop", Button).disabled = False
        self._fix_rerun_options(True)
        self.run_worker(functools.partial(self._pump, self._rerun), thread=True, group="rerun")

    RERUN_OPTIONS: ClassVar = ("#rerun-prefix", "#rerun-stale-after", "#rerun-cloud-rate", "#rerun-viewer",
                               "#rerun-viewer-value", "#rerun-extra", "#rerun-launch")

    def _fix_rerun_options(self, running: bool):
        """The options on the right apply at the start: fixed while xblick rerun runs; the marks stay changeable."""
        for widget_id in self.RERUN_OPTIONS:
            self.query_one(widget_id).disabled = running
        self._show_rerun_header()

    def _rerun_line(self, line: str, style: str = ""):
        self.rerun_output.append(line)
        self.query_one("#rerun-log", RichLog).write(Text(line, style=style))

    def _pump(self, process: subprocess.Popen):
        for line in process.stdout:
            try:
                self.call_from_thread(self._rerun_line, line.rstrip("\n"))
            except RuntimeError:  # the app has ended
                return
        code = process.wait()
        with contextlib.suppress(RuntimeError):
            self.call_from_thread(self._rerun_ended, process, code)

    def _rerun_ended(self, process: subprocess.Popen, code: int):
        self._rerun_line(f"xblick rerun ended, exit code {code}", "green" if code == 0 else "bold red")
        if process is self._rerun:
            self.query_one("#rerun-stop", Button).disabled = True
            self._rerun_key, self._rerun_live = None, False
            self._fix_rerun_options(False)
            self._rerun_shown = None  # the notes of the records again
            self._show_rerun_tree()

    def stop_rerun(self, timeout: float = 10.0):
        """SIGTERM to the child (its cleanup sends what Rerun buffers), SIGKILL after ``timeout``."""
        process = self._rerun
        if process is None or process.poll() is not None:
            return
        process.send_signal(signal.SIGTERM)
        try:
            process.wait(timeout)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()

    def on_unmount(self):
        for pane in self.query(ParametersPane):  # (gone already when the app ends)
            pane.close()
        self.stop_rerun()
        with self._open_lock:  # a session still opening is closed by its thread (_open_thread)
            self._unmounted = True
            handed, self._handed = list(self._handed.values()), {}
        session = self._close_session()
        for s in ([session] if session is not None else []) + handed:
            zsession.close(s)


def _topic_name(key: str, name: str) -> str:
    """A key as the graph names its topic: its stream (``color`` of ``.../color/image_raw/shm/<host>``), below the
    publisher's name relative to it, else the stream's or key's last two tokens."""
    base = tui_nodes.stream_base(key)
    if name != tui_nodes.DASH and base.startswith(name + "/"):
        return base[len(name) + 1:]
    return "/".join(base.split("/")[-2:])


def _rate_text(hz: float, before: str = "") -> str:
    """``12.4/s`` (messages per second as this interface receives them), empty without a measured rate."""
    if not hz:
        return ""
    return f"{before}{hz:.1f}/s" if hz < 100 else f"{before}{hz:.0f}/s"


def _plural(n: int, word: str) -> str:
    return f"{n} {word}{'' if n == 1 else 's'}"


def _ms(value: float) -> str:
    """Milliseconds: whole from 10 ms, else to 0.1 ms."""
    return f"{value:.0f}" if abs(value) >= 10 else f"{value:.1f}"


def _ms_pair(stats: tuple[float, float] | None) -> str:
    """``median / p95`` ms, empty without values."""
    return f"{_ms(stats[0])} / {_ms(stats[1])}" if stats else ""


def _count(uses: list[survey.Use]) -> str:
    """``shm×2, tcp-compressed×1``: the uses by how they take it."""
    counts = collections.Counter(u.summary for u in uses)
    return ", ".join(f"{summary}×{n}" for summary, n in sorted(counts.items()))


def _sender(zid: str | None, labels: dict[str, str]) -> str:
    if zid is None:
        return "unknown (before 0.3.29)"
    return f"{labels.get(zid, 'gone')} ({_short(zid)})"


def _status_line(status: dict) -> str:
    """A ``xblick/.../status`` record in a line (a record and play node's, the MPS daemon's)."""
    if status.get("kind") == "mps":
        if "servers_error" in status:
            return f"{status['pipe_dir']}: {status['servers_error']}"
        clients = [c for s in status.get("servers", []) for c in s["clients"]]
        gpus = ", ".join(g["name"] for g in status.get("gpus", [])) or status.get("gpus_error", "")
        names = ", ".join(sorted({str(c["command"] or c["pid"]) for c in clients}))
        return (f"MPS at {status['pipe_dir']}, {len(clients)} clients" + (f" ({names})" if names else "")
                + f", CUDA_VISIBLE_DEVICES {status.get('cuda_visible_devices')}, {gpus}")
    if status.get("state") == "recording":
        return f"{status.get('file')}, {status.get('messages')} messages, {status.get('dropped')} dropped"
    if status.get("play"):
        from .recording import position_text

        return f"{position_text(status['play'])}, {status['play'].get('sent')} messages"
    return str(status.get("state", ""))


def _sync(table: DataTable, rows: dict, ordered: bool = False):
    """The table's rows made ``rows`` (row key -> cells), sorted by key (``ordered``: in the dict's order), without
    clearing it (the cursor stays)."""
    present = [key.value for key in table.rows]
    wanted = list(rows) if ordered else sorted(rows)
    if (present != wanted) if ordered else (set(present) != set(rows)):
        cursor = table.cursor_row
        table.clear()
        for key in wanted:
            table.add_row(*rows[key], key=key)
        if rows:
            table.move_cursor(row=min(cursor, len(rows) - 1))
        return
    columns = list(table.columns)
    for key, cells in rows.items():
        for column, value in zip(columns, cells):
            if str(table.get_cell(key, column)) != str(value):
                table.update_cell(key, column, value, update_width=True)


def run(connection: Connection | None = None) -> int:
    """The interface in this terminal; ends with q. Without a terminal it refuses (a command is wanted)."""
    if not (sys.stdin.isatty() and sys.stdout.isatty()):
        raise SystemExit("xblick: without a command it opens the terminal interface, which needs a terminal; "
                         "give a command (xblick -h)")
    # The terminal's own size, also after a resize: Textual asks shutil.get_terminal_size, which takes COLUMNS and
    # LINES first when they are set, and would keep the interface at that size whatever the terminal is. They are set
    # where nobody meant them for us: readline (Python before 3.13, e.g. a test harness) puts 80 x 24 into the
    # environment of every child process.
    for name in ("COLUMNS", "LINES"):
        os.environ.pop(name, None)
    XblickApp(connection).run()
    return 0
