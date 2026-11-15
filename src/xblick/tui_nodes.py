"""The terminal interface's Nodes tab, without widgets: every row a node (a camera, a processing node, a tool, the
interface itself), grouped by router.

A group's head is three lines in the colour of the interface's table headers: the router (id, hostname, program,
listen address, its distance from the interface's router), its links (every router it is linked to directly: routers
form a mesh, so a list) and the column names, repeated in every group. The groups follow the routers' distance in
hops from the interface's router (the shortest path over their links); the last group holds the nodes no router knows
(linked directly). Columns: name (the camera or node name one types elsewhere, else ``–``), program (``–`` for one
that declares nothing: another Zenoh program, or Durchblick before 0.5.9; ``@host`` when it runs on another host than
its router), kind, reads, publishes (stream names, relative to the node's name where they are below it; a count where
the names do not fit).

    rows, widths = node_rows(network, own, origins, width=120)   # row key -> Row (cells cut to the widths)
    detail = node_detail(network, zid, ...)                           # a Rich Text
"""

from __future__ import annotations

import dataclasses
import time

from rich.style import Style
from rich.text import Text

from . import declaration, survey, topics
from ._core import protocol as P

DASH = "–"
COLUMNS = ("name", "program", "kind", "reads", "publishes")
INDENT = "  "  # nodes and column names below their router's lines
GAP = 2  # spaces between columns (the table has no padding of its own: the head's colour is one band)
MIN = {"name": 8, "program": 8, "kind": 6, "reads": 10, "publishes": 10}
PARAMETER_SERVICES = (P.LIST_PARAMETERS, P.DESCRIBE_PARAMETERS, P.GET_PARAMETERS, P.GET_PARAMETER_TYPES,
                      P.SET_PARAMETERS, P.SET_PARAMETERS_ATOMICALLY)
LEFT_OUT = (topics.DIAGNOSTICS, P.PARAMETER_EVENTS)  # every node's shared keys: in the detail, not in the columns


@dataclasses.dataclass
class Row:
    """A line of the tab: ``kind`` router, links, columns (a group's head), node; ``zid`` the router's or node's."""

    kind: str
    zid: str | None
    cells: tuple[Text, ...]


def short(zid: str) -> str:
    return zid[:8]


def stream_base(key: str) -> str:
    """The stream a representation key belongs to (``test/camera/color`` of ``.../image_raw/shm/<host>``,
    ``.../camera_info``, ``.../info``); any other key unchanged."""
    place = topics.ring_place(key)
    key = place[0] if place is not None else key
    for end, _ in survey.REPRESENTATIONS:
        if key.endswith("/" + end):
            return key[: -len(end) - 1]
    for end in (topics.CAMERA_INFO, topics.INFO):
        if key.endswith("/" + end):
            return key[: -len(end) - 1]
    return key


def shown_keys(keys, name: str = "") -> list[str]:
    """Keys as the tab names them: by stream (``stream_base``), below ``name`` relative to it by its first token
    (``color``, ``overlay``, ``detections_2d``), ``everything`` for ``**``; the shared keys (``LEFT_OUT``) and the
    declaration's left out. Sorted, each once."""
    out = set()
    for key in keys:
        if key == "**":
            out.add("everything")
            continue
        if key.startswith("@") or declaration.zid_of(key) or key.rsplit("/", 1)[-1] in LEFT_OUT:
            continue
        item = stream_base(key)
        if name and item.startswith(name + "/"):
            item = item[len(name) + 1:].split("/", 1)[0]
        out.add(item)
    return sorted(out)


def fit(items: list[str], width: int, what: str = "topics") -> str:
    """``items`` joined by ``, ``, or their count when that does not fit ``width``; ``–`` for none."""
    if not items:
        return DASH
    text = ", ".join(items)
    if len(text) <= width:
        return text
    count = f"{len(items)} {what if len(items) > 1 else what.rstrip('s')}"
    return count if len(count) <= width else cut(count, width)


def cut(text: str, width: int) -> str:
    return text if len(text) <= width else (text[: max(width - 1, 0)] + "…" if width > 0 else "")


def name_of(network: survey.Network, zid: str) -> str:
    """The node's name: what it declared (its cameras' or its own), else what its keys share (``test/camera/yolox``),
    else ``–``."""
    program = network.program(zid)
    if program is not None and program.names:
        return program.name
    node = network.nodes.get(zid)
    if node is None or node.whatami == "router":
        return DASH
    label = node.label
    if label.startswith("subscribes ") or label.endswith(" keys") or label == "nothing declared":
        return DASH
    return label


def program_of(network: survey.Network, zid: str, router_host: str = "", own_host: str = "") -> str:
    """The program, with ``@host`` when it runs on another host than its router (or, its router's host unknown, than
    this interface); ``–`` for one that declares nothing."""
    program = network.program(zid)
    if program is None:
        return DASH
    other = router_host or own_host
    return program.program + (f" @{program.host}" if program.host and other and program.host != other else "")


def relation(hops: int | None) -> str:
    if hops is None:
        return "no link to this interface's router"
    if hops == 0:
        return "this interface's router"
    return f"{hops} hop{'s' if hops > 1 else ''} away"


def node_rows(network: survey.Network, own: str, origins, width: int,
              own_host: str = "") -> tuple[dict[str, Row], list[int]]:
    """The tab's rows in their order (row key -> Row), the cells cut to fit ``width`` columns, and the columns'
    widths (every cell is as wide as its column: a head's colour is one band)."""
    hops = network.hops(origins)
    groups = network.groups(origins)
    lines: list[tuple[str, str, str | None, list[str] | str]] = []  # (row key, kind, zid, cells or a whole line)
    for router_zid, members in groups:
        if router_zid is None:
            lines.append(("norouter", "router", None, "without a router (linked directly)"))
        else:
            router = network.routers[router_zid]
            program = router.program.program if router.program is not None else DASH
            listen = router.locators[0] if router.locators else DASH
            lines.append((f"router\t{router_zid}", "router", router_zid,
                          f"router {short(router_zid)}  {router.host or DASH}  {program}  {listen}  "
                          f"{relation(hops.get(router_zid))}"))
            linked = ", ".join(f"{short(z)} ({network.routers[z].host or DASH})" if z in network.routers else
                               f"{short(z)} (did not answer)"
                               for z in sorted(router.links, key=lambda z: (hops.get(z, len(hops) + 1), z)))
            lines.append((f"links\t{router_zid}", "links", router_zid, f"links: {linked or 'none'}"))
        lines.append((f"columns\t{router_zid}", "columns", router_zid, list(COLUMNS)))
        router_host = network.routers[router_zid].host if router_zid in network.routers else ""
        def order(zid: str) -> tuple:  # by name, the unnamed last, by program
            program = network.program(zid)
            return (name_of(network, zid) == DASH, name_of(network, zid), program.program if program else DASH, zid)

        for zid in sorted(members, key=order):
            node = network.nodes.get(zid)
            name = name_of(network, zid)
            local = "" if name == DASH else name
            kind = node.whatami if node is not None and node.whatami else "peer"
            reads = shown_keys(node.subscribes, local) if node is not None else []
            publishes = shown_keys(node.publishes, local) if node is not None else []
            program = program_of(network, zid, router_host, own_host)
            if zid == own:
                program += " (this interface)"
            lines.append((f"node\t{zid}", "node", zid, [name, program, kind, reads, publishes]))
    widths = _widths([cells for *_, cells in lines if isinstance(cells, list)], width)
    rows = {}
    for key, kind, zid, cells in lines:
        if isinstance(cells, str):
            rows[key] = Row(kind, zid, _spread(" " + cells, widths))
            continue
        texts = []
        for n, (column, value) in enumerate(zip(COLUMNS, cells)):
            room = widths[n] - GAP - (len(INDENT) if n == 0 else 0)
            text = fit(value, room) if isinstance(value, list) else cut(value, room)
            texts.append(Text((INDENT if n == 0 else "") + text.ljust(room) + " " * GAP))
        rows[key] = Row(kind, zid, tuple(texts))
    return rows, widths


def _widths(cells: list[list], width: int) -> list[int]:
    """Each column's width (its text, the indent, the gap): as wide as its longest cell; when ``width`` is too small
    the reads and publishes first, the wider of them first (to a count), then the program and the name; the last
    column takes what is left, so that a head's band fills the width."""
    def natural(n: int, value) -> int:
        text = ", ".join(value) if isinstance(value, list) else value
        return len(text or DASH)

    want = [max([len(c) for c in COLUMNS[n:n + 1]] + [natural(n, row[n]) for row in cells]) + GAP
            + (len(INDENT) if n == 0 else 0) for n in range(len(COLUMNS))]
    floor = [MIN[c] + GAP + (len(INDENT) if n == 0 else 0) for n, c in enumerate(COLUMNS)]
    while sum(want) > width and (want[3] > floor[3] or want[4] > floor[4]):  # the wider of reads, publishes
        n = 3 if (want[3] - floor[3], 3) > (want[4] - floor[4], 4) else 4
        want[n] -= 1
    for n in (1, 0):  # program, name
        over = sum(want) - width
        if over <= 0:
            break
        want[n] = max(floor[n], want[n] - over)
    want[-1] += max(width - sum(want), 0)
    return want


def _spread(line: str, widths: list[int]) -> tuple[Text, ...]:
    """A whole line (a router's, its links) cut into the columns' cells, so that it reads as one line."""
    total = sum(widths)
    line = cut(line, total).ljust(total)
    out, at = [], 0
    for w in widths:
        out.append(Text(line[at:at + w]))
        at += w
    return tuple(out)


def styled(row: Row, head: Style) -> tuple[Text, ...]:
    """The row's cells, a group's head in ``head`` (the table headers' colour)."""
    if row.kind == "node":
        return row.cells
    return tuple(Text(c.plain, style=head) for c in row.cells)


def _clock(ns: int | None) -> str:
    if ns is None:
        return DASH
    return time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(ns / 1e9))


def parameter_services(queryables) -> tuple[dict[str, int], list[str]]:
    """(node -> its number of parameter services, the other queryables) of a session's queryables."""
    services: dict[str, int] = {}
    other = []
    for key in sorted(queryables):
        node, _, service = key.rpartition("/")
        if service in PARAMETER_SERVICES and node:
            services[node] = services.get(node, 0) + 1
        elif not declaration.zid_of(key):
            other.append(key)
    return services, other


def program_lines(program: declaration.Program | None) -> Text:
    """What a program declared: program and versions, host, process id, start time, command line."""
    text = Text()
    if program is None:
        text.append("program –: it declares nothing (another Zenoh program, or Durchblick before 0.5.9)\n",
                    style="yellow")
        return text
    text.append(f"program {program.program} {program.version or DASH}  (Durchblick {program.xblick or DASH}, "
                f"{program.language or DASH})\n")
    text.append(f"host {program.host or DASH}  process {program.pid if program.pid is not None else DASH}  "
                f"started {_clock(program.started_ns)}\n")
    text.append("command: " + (" ".join(program.command) or DASH) + "\n")
    return text


def node_detail(network: survey.Network, zid: str, readers: dict[str, list[str]], names: dict[str, str]) -> Text:
    """The detail of a node's row: its name, what it declared, kind, Zenoh id, router, addresses, and every key it
    reads, publishes (with who reads each, ``readers``: key -> zids) and answers (the parameter services counted);
    ``names``: zid -> the name to call another session by."""
    node = network.nodes.get(zid)
    text = Text()
    text.append(f"{name_of(network, zid)}\n", style="bold")
    text.append_text(program_lines(network.program(zid)))
    if node is None:
        text.append(f"kind {DASH}  zid {zid}\nno router knows it: it answered its declaration directly, its keys are "
                    "unknown (they are in the routers' admin space only)\n")
        return text
    routers = ", ".join(f"{short(r)} ({network.routers[r].host or DASH})" if r in network.routers else short(r)
                        for r in sorted(node.routers)) or DASH
    text.append(f"kind {node.whatami or DASH}  zid {zid}  router {routers}\n")
    text.append("addresses (as its routers see its links; the addresses a peer listens on are not in the admin "
                "space): " + (", ".join(sorted(node.addresses)) or DASH) + "\n")
    if node.publishes:
        text.append("publishes:\n", style="bold")
        for key in sorted(node.publishes):
            # a router in the admin space subscribes for the sessions behind it, which stand here themselves
            who = sorted({names.get(z, short(z)) for z in readers.get(key, ()) if z not in network.routers})
            text.append(f"  {key}" + (f"  -> {', '.join(who)}" if who else "") + "\n")
    if node.subscribes:
        text.append("reads:\n", style="bold")
        text.append("".join(f"  {k}\n" for k in sorted(node.subscribes)))
    services, other = parameter_services(node.queryables)
    if services or other:
        text.append("answers:\n", style="bold")
        text.append("".join(f"  {k}\n" for k in other))
        text.append("".join(f"  {n} parameter services ({node_name})\n" for node_name, n in sorted(services.items())))
    tokens = [t for t in sorted(node.tokens) if not declaration.zid_of(t)]
    if tokens:
        text.append("liveliness tokens:\n", style="bold")
        text.append("".join(f"  {k}\n" for k in tokens))
    return text


def router_detail(network: survey.Network, zid: str, origins) -> Text:
    """A router's detail: id, host, program, Zenoh version, locators, its links with their addresses, its sessions."""
    router = network.routers[zid]
    text = Text()
    text.append(f"router {zid}  {router.host or DASH}  {relation(network.hops(origins).get(zid))}\n", style="bold")
    text.append(f"listens on {', '.join(router.locators) or DASH}; {router.sessions} sessions; Zenoh "
                f"{router.version.split(' ', 1)[0] or DASH}\n")
    text.append_text(program_lines(router.program) if router.program is not None else
                     Text("program –: not xblick router (zenohd or another), so no record of its program\n"))
    text.append("links:\n", style="bold")
    if not router.links:
        text.append("  none\n")
    for other, addresses in sorted(router.links.items()):
        host = network.routers[other].host if other in network.routers else "did not answer"
        text.append(f"  {short(other)} ({host or DASH}): {'; '.join(addresses) or DASH}\n")
    return text
