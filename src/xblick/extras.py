"""The message of a command whose optional package is missing: which extra brings it, and the command that installs
the extra for the way xblick was installed (docs/getting-started/install.md).

How xblick was installed is known from files of its environment (``sys.prefix``), and from nothing else:

- a uv tool: ``uv-receipt.toml``, in uv's directory of the tool ``xblick``;
- a pipx application: ``pipx_metadata.json``, in pipx's directory of the application ``xblick``;
- a checkout: xblick's ``direct_url.json`` says editable (``uv sync``, ``pip install -e``).

Otherwise (pip, a venv, conda, a uv project) the files do not tell, and the message lists the command of each of
these ways. A uv tool and a pipx application are always found, so the list leaves them out. A uv tool or a pipx
application keeps only the extras of its latest install: its command names the extras installed now as well (those
whose packages are all installed) and the missing one. A checkout gets its commands for uv sync, pip and uv add.
"""

from __future__ import annotations

import importlib.metadata
import json
import re
import sys
from pathlib import Path
from urllib.parse import unquote, urlparse

DISTRIBUTION = "xblick"
LOOK_UP = "look up"  # missing_extra's kind: install_kind() tells it


def _normalized(name: str) -> str:
    return re.sub(r"[-_.]+", "-", name).lower()


def installed_extras(distribution: str = DISTRIBUTION) -> list[str]:
    """The extras of ``distribution`` whose packages are all installed in this environment, sorted."""
    dist = importlib.metadata.distribution(distribution)
    present = {_normalized(d.metadata["Name"]) for d in importlib.metadata.distributions() if d.metadata["Name"]}
    needs: dict[str, set[str]] = {}
    for requirement in dist.requires or []:
        spec, _, marker = requirement.partition(";")
        extra = re.search(r"""extra\s*==\s*["']([^"']+)["']""", marker)
        if extra:
            name = re.match(r"[A-Za-z0-9._-]+", spec.strip()).group(0)
            needs.setdefault(extra.group(1), set()).add(_normalized(name))
    return sorted(extra for extra, names in needs.items() if names <= present)


def install_kind(prefix: Path | None = None, distribution: str = DISTRIBUTION) -> str | None:
    """``"uv tool"``, ``"pipx"`` or ``"checkout"`` when the environment's files say so (module docstring), else
    None."""
    prefix = Path(sys.prefix if prefix is None else prefix)
    if prefix.name == distribution and (prefix / "uv-receipt.toml").is_file():
        return "uv tool"
    if prefix.name == distribution and (prefix / "pipx_metadata.json").is_file():
        return "pipx"
    if _checkout(distribution) is not None:
        return "checkout"
    return None


def _checkout(distribution: str) -> Path | None:
    """The directory of an editable install of ``distribution``, else None."""
    try:
        text = importlib.metadata.distribution(distribution).read_text("direct_url.json")
    except importlib.metadata.PackageNotFoundError:
        return None
    if not text:
        return None
    direct = json.loads(text)
    if not direct.get("dir_info", {}).get("editable"):
        return None
    url = urlparse(direct["url"])
    return Path(unquote(url.path)) if url.scheme == "file" else None


def _spec(extras) -> str:
    return f'"{DISTRIBUTION}[{",".join(extras)}]"'


def missing_extra(needs: str, extra: str, kind: str | None = LOOK_UP, extras: list[str] | None = None,
                  checkout: Path | None = None) -> str:
    """``needs`` (``"xblick rerun needs the Rerun SDK"``) with the extra that brings it and the command that installs
    it: one command for the install found (``kind``, :func:`install_kind`; None: not known), else one line per way
    of installing. ``kind``, ``extras`` (those installed now, :func:`installed_extras`) and ``checkout`` (the editable install's
    directory) are looked up when not given."""
    kind = install_kind() if kind == LOOK_UP else kind
    text = f"{needs}: the extra {extra}, which is not installed."
    if kind in ("uv tool", "pipx"):
        now = installed_extras() if extras is None else extras
        both = sorted(set(now) | {extra})
        command = (f"uv tool install {_spec(both)}" if kind == "uv tool" else
                   f"pipx install --force {_spec(both)}")
        kept = (f" It keeps the extras installed now ({', '.join(now)})." if now else "")
        return (f"{text} xblick is a {'uv tool' if kind == 'uv tool' else 'pipx application'}: install it with\n"
                f"  {command}\nAn install replaces the extras of the previous one.{kept}")
    if kind == "checkout":
        now = installed_extras() if extras is None else extras
        both = sorted(set(now) | {extra})
        where = _checkout(DISTRIBUTION) if checkout is None else checkout
        return (f"{text} xblick is installed from the checkout {where}. Install the extra the way you installed it:\n"
                f"  the checkout's environment: uv sync {' '.join(f'--extra {e}' for e in both)}   (uv sync removes "
                f"the extras it is not given)\n"
                f"  pip:                        pip install -e \"{where}[{extra}]\"\n"
                f"  a uv project:               uv add {_spec([extra])}")
    return (f"{text} Install it the way you installed xblick:\n"
            f"  pip, venv, conda: pip install {_spec([extra])}\n"
            f"  a venv of uv:     uv pip install {_spec([extra])}\n"
            f"  a uv project:     uv add {_spec([extra])}")
