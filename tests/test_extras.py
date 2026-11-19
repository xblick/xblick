"""The message of a command without its optional package (extras.py): how the install is found (a uv tool, a pipx
application, a checkout, else not known), the extras installed now read from the package's metadata, and the command
the message gives for each; the commands that use it (xblick rerun, record and play, play of videos, cam, relay)."""

import builtins
import json

import pytest

from xblick import extras


class FakeDistribution:
    def __init__(self, name, requires=(), direct_url=None):
        self.metadata = {"Name": name}
        self.requires = list(requires)
        self._direct_url = direct_url

    def read_text(self, name):
        return json.dumps(self._direct_url) if name == "direct_url.json" and self._direct_url else None


XBLICK_REQUIRES = ["numpy>=1.24", 'rerun-sdk<0.40,>=0.39; extra == "rerun"', 'mcap<2,>=1.6; extra == "record"',
                   'av>=14; extra == "video"', 'av>=14; extra == "cam"', 'xblick-relay<0.5,>=0.4.0; extra == "relay"']


@pytest.fixture
def environment(monkeypatch):
    """An environment of the given distributions (name -> FakeDistribution), xblick's metadata as in pyproject."""
    def install(*names, direct_url=None):
        dists = {"xblick": FakeDistribution("xblick", XBLICK_REQUIRES, direct_url)}
        dists.update({n: FakeDistribution(n) for n in names})

        def distribution(name):
            if name not in dists:
                raise extras.importlib.metadata.PackageNotFoundError(name)
            return dists[name]

        monkeypatch.setattr(extras.importlib.metadata, "distribution", distribution)
        monkeypatch.setattr(extras.importlib.metadata, "distributions", lambda: list(dists.values()))
    return install


def test_the_extras_installed_now_from_the_metadata(environment):
    environment("numpy")
    assert extras.installed_extras() == []
    environment("numpy", "mcap", "rerun_sdk")  # names compared normalised: rerun_sdk is rerun-sdk
    assert extras.installed_extras() == ["record", "rerun"]
    environment("av")  # one package, two extras
    assert extras.installed_extras() == ["cam", "video"]


def test_how_the_install_is_found(environment, tmp_path):
    environment()
    tool = tmp_path / "tools" / "xblick"
    tool.mkdir(parents=True)
    (tool / "uv-receipt.toml").write_text('[tool]\nrequirements = [{ name = "xblick" }]\n')
    assert extras.install_kind(tool) == "uv tool"
    app = tmp_path / "venvs" / "xblick"
    app.mkdir(parents=True)
    (app / "pipx_metadata.json").write_text("{}")
    assert extras.install_kind(app) == "pipx"
    other = tmp_path / "tools" / "xblick-sam3"  # the tool of a node: not the tool xblick
    other.mkdir()
    (other / "uv-receipt.toml").write_text("")
    assert extras.install_kind(other) is None
    assert extras.install_kind(tmp_path / "venv") is None  # a venv, pip or uv: the files do not tell
    environment(direct_url={"url": "file:///home/me/xblick", "dir_info": {"editable": True}})
    assert extras.install_kind(tmp_path / "venv") == "checkout"
    environment(direct_url={"url": "file:///home/me/xblick-0.5.15.whl", "archive_info": {}})
    assert extras.install_kind(tmp_path / "venv") is None


def test_the_message_for_each_install(environment, tmp_path):
    needs = "xblick rerun needs the Rerun SDK"
    text = extras.missing_extra(needs, "rerun", kind="uv tool", extras=["record", "video"])
    assert text.startswith("xblick rerun needs the Rerun SDK: the extra rerun, which is not installed. xblick is a "
                           "uv tool: install it with\n")
    assert '\n  uv tool install "xblick[record,rerun,video]"\n' in text
    assert "replaces the extras of the previous one. It keeps the extras installed now (record, video)." in text
    assert "pip" not in text
    text = extras.missing_extra(needs, "rerun", kind="uv tool", extras=[])
    assert 'uv tool install "xblick[rerun]"' in text and "keeps" not in text
    text = extras.missing_extra(needs, "rerun", kind="pipx", extras=["record"])
    assert 'pipx install --force "xblick[record,rerun]"' in text and "pipx application" in text
    text = extras.missing_extra(needs, "rerun", kind="checkout", extras=["record"], checkout=tmp_path)
    assert f"from the checkout {tmp_path}" in text and "uv sync --extra record --extra rerun" in text
    assert f'pip install -e "{tmp_path}[rerun]"' in text and 'uv add "xblick[rerun]"' in text
    text = extras.missing_extra(needs, "rerun", kind=None)  # not known: one line per way
    assert text.splitlines()[1:] == ['  pip, venv, conda: pip install "xblick[rerun]"',
                                     '  a venv of uv:     uv pip install "xblick[rerun]"',
                                     '  a uv project:     uv add "xblick[rerun]"']
    environment("numpy")  # looked up when not given: a uv tool of xblick without extras
    tool = tmp_path / "xblick"
    tool.mkdir()
    (tool / "uv-receipt.toml").write_text("")
    with pytest.MonkeyPatch.context() as patch:
        patch.setattr(extras.sys, "prefix", str(tool))
        assert 'uv tool install "xblick[record]"' in extras.missing_extra("x", "record")


def without(monkeypatch, module):
    real_import = builtins.__import__

    def no_module(name, *args, **kwargs):
        if name == module or name.startswith(module + "."):
            raise ModuleNotFoundError(f"No module named {module!r}", name=module)
        return real_import(name, *args, **kwargs)

    monkeypatch.setattr(builtins, "__import__", no_module)


@pytest.mark.parametrize("module, call, said", [
    ("rerun", "xblick.rerun_bridge:import_rerun", "xblick rerun needs the Rerun SDK: the extra rerun"),
    ("mcap", "xblick.recording:import_mcap", "xblick record and play need the mcap package: the extra record"),
    ("av", "xblick.video:import_av", "playing videos needs PyAV: the extra video"),
    ("av", "xblick.cam:import_av", "xblick cam needs PyAV: the extra cam"),
])
def test_the_commands_name_the_extra(monkeypatch, module, call, said):
    import importlib

    path, _, name = call.partition(":")
    function = getattr(importlib.import_module(path), name)
    monkeypatch.setattr(extras, "install_kind", lambda *a, **k: "uv tool")
    monkeypatch.setattr(extras, "installed_extras", lambda *a, **k: [])
    without(monkeypatch, module)
    with pytest.raises((SystemExit, ImportError)) as raised:
        function()
    text = str(raised.value)
    assert text.startswith(said + ", which is not installed.") and f'uv tool install "xblick[{said.split()[-1]}]"' in text


def test_relay_names_the_extra(monkeypatch):
    from xblick import cli

    monkeypatch.setattr(cli.importlib.util, "find_spec", lambda name: None)
    monkeypatch.setattr(cli.shutil, "which", lambda name: None)
    monkeypatch.setattr(extras, "install_kind", lambda *a, **k: None)
    with pytest.raises(FileNotFoundError) as raised:
        cli.relay_program()
    text = str(raised.value)
    assert "the extra relay, which is not installed" in text and 'pip install "xblick[relay]"' in text
    assert text.endswith("put its bin directory on PATH.")
