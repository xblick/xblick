"""SIGTERM and SIGINT while an ``xblick`` command starts (the package's imports, its arguments): held from the first
line of ``xblick/__init__.py`` until ``cli.main`` takes them (``end_on_signals``). Only standard modules, so that the
handler comes before ``_core``, numpy and the camera code. Without it SIGTERM kept its default action there (the
command was killed when its start took long) and SIGINT stayed ignored in a background job of a script."""

from __future__ import annotations

import signal
import sys
from pathlib import Path

SCRIPTS = ("xblick",)  # pyproject.toml's console scripts
MODULE = "xblick.cli"  # python -m xblick.cli
INTERFACE_OPTIONS = ("--connect", "--listen", "--zenoh-mode", "--config")  # without a command: the interface's

held: list[int] = []  # the signals that came while the command started (hold)


def relay_command(argv: list[str]) -> bool:
    """``xblick relay``: its process is replaced by the relay program, which keeps the signals as started."""
    return bool(argv) and argv[0] == "relay"


def interface_command(argv: list[str]) -> bool:
    """No command: the terminal interface (tui.py), whose signals are Textual's."""
    return not argv or argv[0].split("=")[0] in INTERFACE_OPTIONS


def _module_of(orig_argv: list[str]) -> str | None:
    """The module of ``python [options] -m <module> ...``, None for a script or ``-c``. Short options may be combined
    as Python takes them (``-Im xblick.cli``, ``-Bum``, ``-IWerror -m``): in a cluster ``m`` ends the options (the
    rest of the cluster, else the next word, is the module), ``c`` ends them for a command, and ``W`` or ``X`` take
    the rest of the cluster, else the next word, as their value."""
    words = iter(orig_argv[1:])
    for word in words:
        if word == "-" or not word.startswith("-"):
            return None
        if word.startswith("--"):
            if word == "--check-hash-based-pycs":  # a long option with a value of its own
                next(words, None)
            continue
        letters = word[1:]
        for i, letter in enumerate(letters):
            rest = letters[i + 1:]
            if letter == "m":
                return rest or next(words, None)
            if letter == "c":
                return None
            if letter in ("W", "X"):
                if not rest:
                    next(words, None)
                break
    return None


def command_argv() -> list[str] | None:
    """The arguments of the ``xblick`` command this process is (the console script or ``python -m xblick.cli``),
    None when it is another program that imports the package."""
    if sys.argv and Path(sys.argv[0]).name in SCRIPTS:
        return sys.argv[1:]
    if _module_of(getattr(sys, "orig_argv", [])) == MODULE:
        return sys.argv[1:]  # while the package is imported for -m, sys.argv[0] is "-m"
    return None


def hold():
    """Installs the handler that notes SIGTERM and SIGINT (``held``), when this process is an ``xblick`` command
    other than ``xblick relay`` and the terminal interface; nothing for a program that imports the package."""
    argv = command_argv()
    if argv is None or relay_command(argv) or interface_command(argv):
        return

    def note(signum, frame):
        held.append(signum)

    signal.signal(signal.SIGTERM, note)
    signal.signal(signal.SIGINT, note)
