"""The terminal interface's drawn graph (tui_graph.py), its key tree and short names (tui.py), and the endpoints
people type (session.endpoint): no network, only the functions."""

import pytest

from xblick import tui
from xblick.session import endpoint
from xblick.tui_graph import LINES, GraphEdge, GraphGroup, GraphNode, GraphView, layout


@pytest.mark.parametrize("typed, wanted", [
    ("robot-pc", "tcp/robot-pc:7447"), ("robot-pc:7448", "tcp/robot-pc:7448"),
    ("tcp/robot-pc", "tcp/robot-pc:7447"), ("tcp/192.0.2.10:7447", "tcp/192.0.2.10:7447"),
    ("udp/192.0.2.10", "udp/192.0.2.10:7447"), ("[::1]", "tcp/[::1]:7447"), ("tcp/[::1]:7448", "tcp/[::1]:7448"),
    ("unixsock-stream/tmp/z.sock", "unixsock-stream/tmp/z.sock"), (" robot-pc ", "tcp/robot-pc:7447"),
    ("tcp/h#iface=eth0", "tcp/h:7447#iface=eth0"), ("127.0.0.1:0", "tcp/127.0.0.1:0")])
def test_endpoints_take_tcp_and_the_default_port(typed, wanted):
    assert endpoint(typed) == wanted


@pytest.mark.parametrize("typed", ["", "::1", "tcp/host:", "tcp/"])
def test_endpoints_that_say_too_little_are_refused(typed):
    with pytest.raises(ValueError):
        endpoint(typed)


def test_a_connection_is_shown_with_its_endpoints_as_zenoh_takes_them():
    c = tui.Connection(connect=["robot-pc"], mode="client")
    assert c.connect == ["tcp/robot-pc:7447"] and c.describe() == "--zenoh-mode client --connect tcp/robot-pc:7447"


HOST = "0c9a1f6e-5b0d-4a52-9d3e-7f1b2c3d4e5f"  # a host id, as the rings' keys carry it
KEYS = ["robot/camera/color/camera_info", "robot/camera/color/image_raw/compressed",
        f"robot/camera/color/image_raw/shm/{HOST}", f"robot/camera/depth/image_raw/shm/{HOST}",
        "robot/camera/diagnostics", "robot/camera/sam3/instances", "robot/joint_states",
        "robot/camera/color/image_raw", "sim/clock"]


def test_the_keys_as_a_tree():
    camera, sam3 = frozenset({"k"}), frozenset({"s"})
    cells = {k: (sam3 if "sam3" in k or k.endswith("diagnostics") else camera, frozenset()) for k in KEYS}
    cells["robot/joint_states"] = (frozenset({"b"}), frozenset())
    rows = {r.path: r for r in tui.topic_tree(KEYS, cells=cells)}
    labels = [r.label for r in rows.values()]
    assert labels[:3] == ["▾ robot/", "├─ ▾ camera/", "│  ├─ ▾ color/"]
    # a chain of single children: one row; a ring's place (its host's id) by its first 8 characters
    assert "│  ├─ depth/image_raw/shm/0c9a1f6e" in labels
    assert rows["robot/camera/depth/image_raw/shm/0c9a1f6e"].key == KEYS[3]
    assert "sim/clock" in labels
    # a key that is also a prefix: one row, the key's
    assert rows["robot/camera/color/image_raw/"].key == "robot/camera/color/image_raw"
    # the publisher once: on color/ (every key below is the camera's), not on its keys, not on camera/ (mixed)
    assert rows["robot/camera/color/"].show_pub and not rows["robot/camera/color/camera_info"].show_pub
    assert not rows["robot/camera/"].show_pub and rows["robot/camera/sam3/instances"].show_pub
    folded = [r.label for r in tui.topic_tree(KEYS, {"robot/camera/color/"})]
    assert "│  ├─ ▸ color/  (4 keys)" in folded and not any("camera_info" in label for label in folded)


def test_short_names_drop_what_another_session_names_already():
    names = tui.short_names({"a": "robot/camera", "b": "robot/camera/sam3", "r": "robot",
                             "t": "robot/camera/tapnext/tracks", "x": "xblick/recorder/status",
                             "v": "subscribes robot/camera", "c": "robot2/camera/sam3",
                             "d": "robot2/camera"})
    assert names == {"a": "camera", "b": "robot/camera/sam3", "r": "robot", "t": "tapnext/tracks",
                     "x": "xblick/recorder/status", "v": "subscribes camera",
                     "c": "robot2/camera/sam3", "d": "robot2/camera"}  # two sam3: both whole


NODES = [GraphNode("k", "camera", ["peer 1", "OK 30.0 frames/s"]), GraphNode("s", "sam3", ["peer 2"]),
         GraphNode("f", "foundationpose", ["peer 3"]), GraphNode("t", "tapnext", ["peer 4"]),
         GraphNode("v", "viewer", ["client 5"]), GraphNode("lonely", "relay", ["peer 6"])]
EDGES = [GraphEdge("k", "s", "shm"), GraphEdge("k", "f", "cuda"), GraphEdge("s", "f", "instances"),
         GraphEdge("k", "v", "tcp-compressed"), GraphEdge("f", "v", "tf"), GraphEdge("s", "t", "instances"),
         GraphEdge("t", "s", "tracks")]


def test_the_layout_puts_takers_right_of_what_they_take():
    d = layout(NODES, EDGES)
    x = {n: d.boxes[n].x for n in d.boxes}
    assert x["k"] < x["s"] < x["f"] < x["v"] and set(d.boxes) == {n.id for n in NODES}
    # one cycle (s and t take each other's data): one edge drawn against the flow
    assert len(d.reversed) == 1 and EDGES[next(iter(d.reversed))].source in ("s", "t")
    for a in d.boxes.values():  # boxes do not overlap
        for b in d.boxes.values():
            assert a is b or not a.overlaps(b)


def test_every_edge_has_its_arrow_and_label_and_no_line_runs_into_a_box():
    d = layout(NODES, EDGES)
    arrows = {owner for (ch, kind, owner) in d.glyphs.values() if kind == "arrow"}
    assert arrows == set(range(len(EDGES)))
    assert sum(ch == "◀" for ch, kind, _ in d.glyphs.values() if kind == "arrow") == 1
    text = GraphView()
    text.nodes, text.edges, text.drawing = NODES, EDGES, d
    plain = text.render().plain
    for label in ("shm▶", "cuda▶", "tcp-compressed▶", "tf▶"):
        assert label in plain.replace("─", ""), label
    for (x, y) in d.bits:
        assert not any(r.contains(x, y) for r in d.boxes.values()), (x, y)
    # every line cell is a box-drawing character of its directions
    assert all(LINES.get(bits) for bits in d.bits.values())


def test_hovering_an_edge_and_moving_the_selection():
    view = GraphView()
    view.nodes, view.edges, view.drawing = NODES, EDGES, layout(NODES, EDGES)
    arrow = next(cell for cell, (ch, kind, owner) in view.drawing.glyphs.items() if kind == "arrow" and owner == 0)
    assert view._at(*arrow) == ("edges", frozenset({0}))
    box = view.drawing.boxes["s"]
    assert view._at(box.x + 1, box.y + 1) == ("node", "s")
    view.selected = "k"
    view.action_move(1, 0)  # right: the nearest box in the next column
    assert view.selected == "s"
    view.action_move(-1, 0)
    assert view.selected == "k"


def many(n: int) -> tuple[list[GraphNode], list[GraphEdge], list[GraphGroup]]:
    """Two routers' groups: a camera on each, ``n`` nodes in all that take a camera's frames, one of them across."""
    nodes = [GraphNode("cam-a", "a/camera", ["xblick cam"]), GraphNode("cam-b", "b/camera", ["xblick cam"])]
    edges = []
    groups = [GraphGroup("ra", "router aaaaaaaa  pc-a", ["cam-a"]),
              GraphGroup("rb", "router bbbbbbbb  pc-b", ["cam-b"])]
    for k in range(n - 2):
        side = "ab"[k % 2]
        nodes.append(GraphNode(f"n{k}", f"{side}/camera/node{k}", [f"xblick-node{k}"]))
        edges.append(GraphEdge(f"cam-{side}", f"n{k}", "shm"))
        groups[k % 2].members.append(f"n{k}")
    edges.append(GraphEdge("cam-a", "n1", "tcp-compressed"))  # across the routers
    edges += [GraphEdge(f"n{k}", f"n{k + 2}", "detections") for k in range(0, 6, 2)]  # a chain: four columns
    return nodes, edges, groups


@pytest.mark.parametrize("n", [20, 40])
def test_many_nodes_on_two_routers_stand_in_their_bands(n):
    """Every box in its group's band below the group's line, no two boxes on each other, the link on the left."""
    nodes, edges, groups = many(n)
    d = layout(nodes, edges, groups, [("ra", "rb")])
    assert len(d.boxes) == n and d.group_rows["ra"] < d.group_rows["rb"]
    for g in groups:
        nxt = min((y for other, y in d.group_rows.items() if y > d.group_rows[g.id]), default=d.height)
        for m in g.members:
            box = d.boxes[m]
            assert d.group_rows[g.id] < box.y and box.bottom <= nxt, (m, box, d.group_rows)
    regions = list(d.boxes.values())
    assert not any(a.overlaps(b) for i, a in enumerate(regions) for b in regions[i + 1:])
    assert all(r.x >= 2 for r in regions)  # the link's lane on the left
    assert d.glyphs[(0, d.group_rows["ra"])][1] == "link" and d.glyphs[(0, d.group_rows["rb"])][1] == "link"
    drawn = GraphView()
    drawn.show(nodes, edges, groups, [("ra", "rb")])
    text = drawn.render().plain
    assert "┄ router aaaaaaaa  pc-a" in text and "┄ router bbbbbbbb  pc-b" in text


def test_the_view_moves_and_returns_to_the_whole_graph():
    """Too large for the terminal: shift and the arrow keys move the view, 0 returns to its start; the arrow keys
    still choose boxes."""
    from textual.app import App
    from textual.containers import VerticalScroll

    nodes, edges, groups = many(40)

    class One(App):
        CSS = "#s { height: 1fr; overflow-x: auto; }"

        def compose(self):
            with VerticalScroll(id="s"):
                yield GraphView(id="g")

        def on_mount(self):
            self.query_one(GraphView).show(nodes, edges, groups, [("ra", "rb")])
            self.query_one(GraphView).focus()

    async def main():
        app = One()
        async with app.run_test(size=(80, 24)) as pilot:
            await pilot.pause(0.2)
            scroll, graph = app.query_one("#s"), app.query_one(GraphView)
            assert graph.drawing.height > 24 and graph.drawing.width > 80
            await pilot.press("shift+down", "shift+down", "shift+right")
            await pilot.pause(0.2)
            assert scroll.scroll_y == 2 * GraphView.PAN[1] and scroll.scroll_x == GraphView.PAN[0]
            await pilot.press("0")
            await pilot.pause(0.2)
            assert (scroll.scroll_x, scroll.scroll_y) == (0, 0)
            first = graph.selected
            await pilot.press("down")
            assert graph.selected != first

    import asyncio

    asyncio.run(main())
