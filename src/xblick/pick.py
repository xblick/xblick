"""An expandable list on the terminal to choose from: ``xblick cam`` without a source (cameras, their pixel formats,
sizes and rates) and ``xblick view`` without a stream (the streams by namespace and camera, then the viewers) use it.

The model (:class:`Item`, :class:`Tree`) holds the items and the cursor and knows no terminal, so tests drive it
directly. :func:`run` draws it with Rich (``rich.tree.Tree`` in a ``rich.live.Live``) and reads the keys itself in
cbreak mode (keys one by one, no echo, Ctrl-C stays a signal):

    Up, Down (k, j)      the previous or next visible item
    Right (l)            expands the item, or, when it is expanded, moves to its child last visited
    Left (h)             collapses the item, or moves to its parent
    Enter                the caller's ``enter``: a result ends the list, None keeps it open
    Space                marks or unmarks a markable item; on a group, all markable items below it
    Esc                  back to the item's top-level item, collapsed; at the top level it leaves
    q, Ctrl-C, SIGTERM   leave (SIGTERM as KeyboardInterrupt, cli.end_on_signals)

Expanding an item collapses its siblings, so only the active branch is open. Each item remembers the child the
cursor was on last (``chosen``); the caller sets it first, for example to the recommended child. The terminal's
mode is restored and the list erased on every way out, an exception too.
"""

from __future__ import annotations

import os
import select
import termios
import tty
from dataclasses import dataclass, field
from typing import Any, Callable


@dataclass(eq=False)
class Item:
    """One line of the list. ``note`` follows the label (a mark such as ``[recommended]``, or why the item is not
    ``enabled``); a disabled item is drawn grey and the caller refuses Enter on it. ``chosen``: the index of the
    child the cursor goes to on Right."""

    label: str
    children: list[Item] = field(default_factory=list)
    value: Any = None
    note: str = ""
    enabled: bool = True
    markable: bool = False
    marked: bool = False
    expanded: bool = False
    chosen: int = 0
    parent: Item | None = field(default=None, repr=False)


class Tree:
    """The items and the cursor (module docstring). ``message``: one line for the user, cleared by the next move."""

    def __init__(self, roots: list[Item]):
        if not roots:
            raise ValueError("a list without items")
        self.roots = roots
        self._link(roots, None)
        self.cursor = roots[0]
        self.message = ""

    def _link(self, items: list[Item], parent: Item | None):
        for item in items:
            item.parent = parent
            self._link(item.children, item)

    def siblings(self, item: Item) -> list[Item]:
        return item.parent.children if item.parent is not None else self.roots

    def path(self, item: Item | None = None) -> list[Item]:
        """The items from the top level down to ``item`` (default: the cursor)."""
        item = item or self.cursor
        path = []
        while item is not None:
            path.append(item)
            item = item.parent
        return path[::-1]

    def visible(self) -> list[tuple[int, Item]]:
        """(depth, item) of every item shown, top to bottom."""
        shown = []

        def walk(items, depth):
            for item in items:
                shown.append((depth, item))
                if item.expanded:
                    walk(item.children, depth + 1)

        walk(self.roots, 0)
        return shown

    def _go(self, item: Item):
        self.cursor = item
        if item.parent is not None:
            item.parent.chosen = item.parent.children.index(item)
        self.message = ""

    def up(self):
        items = [i for _, i in self.visible()]
        self._go(items[max(items.index(self.cursor) - 1, 0)])

    def down(self):
        items = [i for _, i in self.visible()]
        self._go(items[min(items.index(self.cursor) + 1, len(items) - 1)])

    def expand(self, item: Item):
        for sibling in self.siblings(item):
            sibling.expanded = False
        item.expanded = True

    def right(self):
        item = self.cursor
        if not item.children:
            self.message = ""
            return
        if not item.expanded:
            self.expand(item)
            self.message = ""
        else:
            self._go(item.children[min(item.chosen, len(item.children) - 1)])

    def left(self):
        item = self.cursor
        if item.expanded:
            item.expanded = False
            self.message = ""
        elif item.parent is not None:
            self._go(item.parent)

    def back(self) -> bool:
        """Esc: to the cursor's top-level item, collapsed (True); False when the cursor is at the top level."""
        if self.cursor.parent is None:
            return False
        top = self.path()[0]
        top.expanded = False
        self._go(top)
        return True

    def toggle_mark(self):
        """Space: marks or unmarks the cursor's item; on a group every markable item below it (all marked when one
        was not, else all unmarked)."""
        item = self.cursor
        if item.markable:
            item.marked = not item.marked
            return
        below = [i for i in self._below(item) if i.markable]
        if below:
            mark = not all(i.marked for i in below)
            for i in below:
                i.marked = mark

    def _below(self, item: Item) -> list[Item]:
        found = []
        for child in item.children:
            found.append(child)
            found.extend(self._below(child))
        return found

    def marked(self) -> list[Item]:
        """The marked items in the list's order."""
        found = []
        for root in self.roots:
            found.extend(i for i in [root, *self._below(root)] if i.marked)
        return found


# ------------------------------------------------------------------------------------------------- the terminal

KEYS = {b"\x1b[A": "up", b"\x1bOA": "up", b"k": "up", b"\x1b[B": "down", b"\x1bOB": "down", b"j": "down",
        b"\x1b[C": "right", b"\x1bOC": "right", b"l": "right", b"\x1b[D": "left", b"\x1bOD": "left", b"h": "left",
        b"\r": "enter", b"\n": "enter", b" ": "space", b"\x1b": "esc", b"q": "quit", b"Q": "quit", b"\x03": "quit"}


def split_keys(data: bytes) -> list[str]:
    """The keys in what one read from the terminal returned: an arrow's escape sequence is one key; Esc alone is
    ``esc``; an unknown byte or sequence is ``""``."""
    keys = []
    while data:
        if data[:2] == b"\x1b[" and len(data) >= 3:  # a control sequence: parameters, then one final byte
            n = 2
            while n < len(data) - 1 and data[n] in b"0123456789;":  # e.g. ESC [ 1 ; 5 C
                n += 1
            token, data = data[:n + 1], data[n + 1:]
        elif data[:2] == b"\x1bO" and len(data) >= 3:
            token, data = data[:3], data[3:]
        else:
            token, data = data[:1], data[1:]
        keys.append(KEYS.get(token, ""))
    return keys


def read_keys(fd: int) -> list[str]:
    """The keys typed: one read; Esc alone is followed by nothing within 50 ms. The end of the input is ``quit``."""
    data = os.read(fd, 64)
    if not data:
        return ["quit"]
    if data == b"\x1b" and select.select([fd], [], [], 0.05)[0]:
        data += os.read(fd, 64)
    return split_keys(data)


def _symbols(out) -> dict[str, str]:
    """The cursor and the expand marks, in ASCII on a terminal that is not UTF-8."""
    if (getattr(out, "encoding", "") or "").lower().replace("-", "") == "utf8":
        return {"cursor": "❯", "open": "▾", "closed": "▸"}
    return {"cursor": ">", "open": "-", "closed": "+"}


def render(tree: Tree, title: str, status: str, keys: str, symbols: dict[str, str]):
    """The list as a Rich renderable: the items as a tree below the title, the status line, the message and the
    keys."""
    from rich.console import Group
    from rich.text import Text
    from rich.tree import Tree as RichTree

    def label(item: Item) -> Text:
        here = item is tree.cursor
        text = Text(f"{symbols['cursor'] if here else ' '} ", style="bold" if here else "")
        if item.children:
            text.append((symbols["open"] if item.expanded else symbols["closed"]) + " ")
        if item.markable:
            text.append("[x] " if item.marked else "[ ] ")
        text.append(item.label, style=("reverse " if here else "") + ("" if item.enabled else "dim"))
        if item.note:
            text.append("  " + item.note, style="green" if item.note == "[recommended]" else "dim")
        return text

    root = RichTree(Text(title), guide_style="dim")

    def add(node, items):
        for item in items:
            branch = node.add(label(item))
            if item.expanded:
                add(branch, item.children)

    add(root, tree.roots)
    parts = [root, Text(""), Text(status)]
    if tree.message:
        parts.append(Text(tree.message, style="yellow"))
    parts.append(Text(keys, style="dim"))
    return Group(*parts)


def run(tree: Tree, *, title: str, keys: str, status: Callable[[Tree], str], enter: Callable[[Tree], Any],
        fd: int, out) -> Any | None:
    """The list on the terminal (input ``fd``, output ``out``, both a terminal) until ``enter`` returns a result,
    which it returns; None for Esc at the top level, q, Ctrl-C, SIGTERM (KeyboardInterrupt) or the end of the input.
    ``status``: the line under the list, what Enter would do now. ``enter`` may set ``tree.message`` and return None
    to keep the list open."""
    from rich.console import Console
    from rich.live import Live

    console = Console(file=out, force_terminal=True, highlight=False, soft_wrap=False)
    symbols = _symbols(out)
    saved = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)  # keys one by one, no echo; Ctrl-C stays a signal
        draw = lambda: render(tree, title, status(tree), keys, symbols)  # noqa: E731
        with Live(draw(), console=console, auto_refresh=False, transient=True, redirect_stdout=False,
                  redirect_stderr=False) as live:
            while True:
                for key in read_keys(fd):
                    if key == "quit" or (key == "esc" and not tree.back()):
                        return None
                    if key == "enter":
                        result = enter(tree)
                        if result is not None:
                            return result
                    elif key in ("up", "down", "right", "left"):
                        getattr(tree, key)()
                    elif key == "space":
                        tree.toggle_mark()
                live.update(draw(), refresh=True)
    except KeyboardInterrupt:
        return None
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, saved)
