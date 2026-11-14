"""The list of pick.py, which xblick cam and xblick view choose with: moving, expanding (only the active branch),
collapsing, the child last visited, Esc back to the top level, marks; the keys split from what the terminal sends;
the terminal's mode restored after an exception. The commands' tests (test_cam.py, test_view.py) drive it in a
pseudo-terminal."""

import os
import pty
import termios

import pytest

from xblick import pick
from xblick.pick import Item, Tree


def tree() -> Tree:
    """a (b (c, d), e), f (g): b's chosen child is d, as a caller sets the recommended one."""
    b = Item("b", [Item("c"), Item("d")], chosen=1)
    return Tree([Item("a", [b, Item("e")]), Item("f", [Item("g")])])


def shown(t: Tree) -> list[str]:
    return [("  " * depth) + item.label + ("*" if item is t.cursor else "") for depth, item in t.visible()]


def test_moving_expanding_and_collapsing():
    t = tree()
    assert shown(t) == ["a*", "f"]
    t.up()
    assert t.cursor.label == "a"  # no wrap at the top
    t.right()  # expands a
    assert shown(t) == ["a*", "  b", "  e", "f"]
    t.right()  # into a's child: b, the first
    t.right()
    t.right()  # into b's chosen child: d
    assert shown(t) == ["a", "  b", "    c", "    d*", "  e", "f"]
    t.down()
    t.down()
    assert t.cursor.label == "f"
    t.down()
    assert t.cursor.label == "f"  # no wrap at the bottom
    t.right()  # expanding f collapses a: only the active branch is open
    assert shown(t) == ["a", "f*", "  g"]
    t.up()
    t.right()  # a again: b still expanded inside it
    assert shown(t) == ["a*", "  b", "    c", "    d", "  e", "f"]
    t.right()
    assert t.cursor.label == "e"  # the child of a last visited
    for _ in range(3):
        t.up()
    assert t.cursor.label == "b"
    t.left()  # collapses b
    assert shown(t) == ["a", "  b*", "  e", "f"]
    t.left()  # to its parent
    assert t.cursor.label == "a"
    t.left()
    assert shown(t) == ["a*", "f"]


def test_esc_goes_back_to_the_top_level_then_leaves():
    t = tree()
    t.right()
    t.right()
    t.right()
    t.right()
    assert t.cursor.label == "d"
    assert t.back() and shown(t) == ["a*", "f"]  # a collapsed, the cursor on it
    assert not t.back()  # at the top level: leave
    t.right()
    assert shown(t) == ["a*", "  b", "    c", "    d", "  e", "f"]  # the selections are kept


def test_marks():
    streams = [Item("color", markable=True), Item("depth", markable=True)]
    t = Tree([Item("cam", streams), Item("other", [Item("x", markable=True)])])
    t.toggle_mark()  # on a group: all below it
    assert [i.label for i in t.marked()] == ["color", "depth"]
    t.toggle_mark()
    assert t.marked() == []
    t.right()
    t.right()
    t.down()
    t.toggle_mark()
    assert [i.label for i in t.marked()] == ["depth"]
    t.up()
    t.up()
    t.toggle_mark()  # one of two was marked: now both
    assert [i.label for i in t.marked()] == ["color", "depth"]


def test_an_empty_list_is_refused():
    with pytest.raises(ValueError):
        Tree([])


def test_keys_split_from_what_the_terminal_sends():
    assert pick.split_keys(b"\x1b[B\x1b[B\r") == ["down", "down", "enter"]
    assert pick.split_keys(b"\x1bOA\x1b[C\x1b[D ") == ["up", "right", "left", "space"]
    assert pick.split_keys(b"\x1b") == ["esc"]
    assert pick.split_keys(b"\x1b[1;5Cq") == ["", "quit"]  # Ctrl-Right is no key here
    assert pick.split_keys(b"jkhl\n") == ["down", "up", "left", "right", "enter"]


def test_the_terminal_mode_is_restored_after_an_exception():
    class Broken:
        encoding = "utf-8"

        def write(self, text):
            raise OSError("the terminal went away")

        def flush(self):
            pass

        def isatty(self):
            return True

        def fileno(self):
            return 1

    master, slave = pty.openpty()
    try:
        before = termios.tcgetattr(slave)
        with pytest.raises(OSError, match="went away"):
            pick.run(tree(), title="t", keys="k", status=lambda t: "", enter=lambda t: None, fd=slave, out=Broken())
        assert termios.tcgetattr(slave) == before
    finally:
        os.close(master)
        os.close(slave)
