"""Every program declares itself (declaration.py, protocol.hpp kSessions): a command, a minimal node and a C++ tool,
each a process of its own behind a router; the record's fields; the token and the record gone when the program ends,
also after SIGKILL; a session that does not declare (a program before 0.5.9) has neither; the router's record in its
metadata; and what a declaration costs a session's start."""

import json
import os
import signal
import socket
import subprocess
import sys
import textwrap
import threading
import time
import weakref
from pathlib import Path

import pytest
import zenoh
from test_camera import free_port, wait_until

from xblick import _core, declaration, router, survey
from xblick.session import close, open_session, replies, timed_out

BUILD = os.environ.get("XBLICK_CPP_BUILD")
XBLICK = [sys.executable, "-m", "xblick.cli"]
ENV = {**os.environ, "PYTHONUNBUFFERED": "1"}

NODE = '''
import argparse, time
from xblick import node, session
parser = argparse.ArgumentParser()
session.add_arguments(parser)
parser.add_argument("--camera", required=True)
args = parser.parse_args()
node.start("mini-node")
z = session.open_from_args(args)
report = node.Reporter(z, args.camera, "mini", "mini-node")
print("ready", flush=True)
try:
    while True:
        time.sleep(0.1)
except KeyboardInterrupt:
    pass
finally:
    report.close()
    z.close()
'''


@pytest.fixture
def network():
    """A router (xblick router: admin space on, its record in its metadata) and a session that looks."""
    port = free_port()
    r = router.open_router([f"tcp/127.0.0.1:{port}"])
    z = open_session(connect=[f"tcp/127.0.0.1:{port}"], listen=["127.0.0.1:0"], declare=False)
    yield port, z
    close(z)
    close(r)


def start(command: list[str]) -> subprocess.Popen:
    return subprocess.Popen(command, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                            text=True, env=ENV)


def stop(process: subprocess.Popen, sig=signal.SIGTERM):
    if process.poll() is None:
        process.send_signal(sig)
    try:
        process.communicate(timeout=10)
    except subprocess.TimeoutExpired:
        process.kill()
        process.communicate()


def program_of(z, pid: int, timeout: float = 15.0) -> declaration.Program:
    """The record of the program with this process id, once it answered."""
    found = {}

    def answered():
        programs, errors = declaration.query(z, 0.5)
        assert not errors, errors
        found.update({p.pid: p for p in programs.values()})
        return pid in found

    wait_until(answered, timeout=timeout)
    return found[pid]


def gone_after(z, zid: str, timeout: float = 15.0) -> float:
    """Seconds until neither the token nor the record of ``zid`` is there any more."""
    start_ = time.monotonic()
    wait_until(lambda: zid not in declaration.alive(z, 0.3) and zid not in declaration.query(z, 0.3)[0],
               timeout=timeout)
    return time.monotonic() - start_


def check_common(p: declaration.Program, process: subprocess.Popen):
    assert p.pid == process.pid and p.host == socket.gethostname() and p.xblick == _core.__version__
    assert abs(p.started_ns - time.time_ns()) < 120e9  # started just now (its start, from /proc)
    assert p.command and Path(p.command[0]).name.startswith(("python", "xblick-cpp")), p.command


def test_a_command_declares_itself_and_is_gone_when_it_ends(network):
    port, z = network
    camera = start([*XBLICK, "test-camera", "--namespace", "dt", "--camera", "cam", "--fps", "5",
                    "--connect", f"tcp/127.0.0.1:{port}"])
    try:
        p = program_of(z, camera.pid)
        check_common(p, camera)
        assert p.program == "xblick test-camera" and p.version == _core.__version__ and p.language == "python"
        assert p.names == ["dt/cam"]  # its camera (its publisher's base: dt/cam/color)
        assert p.command[1:4] == ["-m", "xblick.cli", "test-camera"] and f"tcp/127.0.0.1:{port}" in p.command
        assert p.zid in declaration.alive(z)  # its token
    finally:
        stop(camera)
    assert gone_after(z, p.zid) < 3.0


def test_a_node_declares_its_name_and_a_killed_one_disappears(network, tmp_path):
    port, z = network
    script = tmp_path / "mini-node.py"
    script.write_text(textwrap.dedent(NODE))
    node = start([sys.executable, str(script), "--camera", "dt/cam", "--connect", f"tcp/127.0.0.1:{port}"])
    try:
        assert node.stdout.readline().strip().endswith("ready")
        wait_until(lambda: program_of(z, node.pid).names == ["dt/cam/mini"], timeout=10)
        p = program_of(z, node.pid)
        check_common(p, node)
        # the program is the name it was started by; no installed distribution names it: no version
        assert p.program == "mini-node.py" and p.version is None and p.command[1] == str(script)
    finally:
        node.kill()  # SIGKILL: no cleanup; its links close with the process
        node.communicate(timeout=10)
    seconds = gone_after(z, p.zid)
    print(f"declaration gone {seconds:.2f} s after SIGKILL")
    assert seconds < 3.0


@pytest.mark.skipif(not BUILD, reason="XBLICK_CPP_BUILD (the C++ build directory) is not set")
def test_a_cpp_tool_declares_itself(network):
    port, z = network
    tool = Path(BUILD) / "xblick-cpp-camera"
    camera = start([str(tool), "--connect", f"tcp/127.0.0.1:{port}", "--base", "dt/cpp/color", "--size", "64", "48",
                    "--fps", "5", "--seconds", "60"])
    try:
        p = program_of(z, camera.pid)
        check_common(p, camera)
        assert (p.program, p.version, p.language, p.names) == ("xblick-cpp-camera", _core.__version__, "c++",
                                                               ["dt/cpp"])
        assert p.command[0] == str(tool) and "--base" in p.command
    finally:
        stop(camera)
    assert gone_after(z, p.zid) < 3.0


def test_a_session_that_does_not_declare_has_no_record(network):
    port, z = network
    with open_session(connect=[f"tcp/127.0.0.1:{port}"], listen=["127.0.0.1:0"], declare=False) as old, \
            open_session(connect=[f"tcp/127.0.0.1:{port}"], listen=["127.0.0.1:0"], program="new") as new:
        wait_until(lambda: str(new.info.zid()) in declaration.query(z, 0.5)[0], timeout=10)
        programs = declaration.query(z, 0.5)[0]
        assert str(old.info.zid()) not in programs and str(old.info.zid()) not in declaration.alive(z)
        assert declaration.of(old) is None and not declaration.add_name(old, "x/y")
        assert programs[str(new.info.zid())].program == "new"
        # the declaration's keys do not name a session (Node.label)
        assert survey.query_network(z, 1.0).nodes[str(new.info.zid())].label == "nothing declared"


def test_the_router_carries_its_record_in_its_metadata(network):
    port, z = network
    [record] = [json.loads(r.ok.payload.to_bytes()) for r in replies(z, "@/*/router", 1.0) if r.ok is not None]
    p = declaration.Program.from_record(record["metadata"], record["zid"])
    assert p.program == "xblick router" and p.version == _core.__version__ and p.names == []
    assert p.pid == os.getpid() and p.host == socket.gethostname() and "zid" not in record["metadata"]


def test_a_record_of_another_kind_or_version_is_refused():
    record = declaration.program_record("x", None)
    assert declaration.Program.from_record(record, "z").program == "x"
    with pytest.raises(ValueError, match="version 2"):
        declaration.Program.from_record({**record, "v": 2})
    with pytest.raises(ValueError, match="not a program record"):
        declaration.Program.from_record({"v": 4, "kind": "image"})
    assert declaration.zid_of("xblick/sessions/abc/program") == "abc"
    assert declaration.zid_of("xblick/sessions/a/b/program") is None and declaration.zid_of("x/program") is None
    assert "PATH" not in json.dumps(record)  # nothing of the environment


def test_the_cost_of_a_declaration(capsys):
    """A session's start with and without its declaration (median of 9, after a first one), and the record's size:
    printed for the release notes, and bounded."""
    port = free_port()
    with open_session(listen=[f"tcp/127.0.0.1:{port}"], local_router=False, declare=False):
        def opened(declare: bool) -> float:
            t = time.perf_counter()
            s = open_session(connect=[f"tcp/127.0.0.1:{port}"], declare=declare, program="cost")
            took = time.perf_counter() - t
            close(s)
            return took

        opened(True)
        without = sorted(opened(False) for _ in range(9))[4]
        with_ = sorted(opened(True) for _ in range(9))[4]
    size = len(json.dumps(declaration.program_record("xblick-yolox", "0.3.0"), separators=(",", ":")))
    with capsys.disabled():
        print(f"\nsession start: {without * 1e3:.1f} ms without, {with_ * 1e3:.1f} ms with the declaration; "
              f"record about {size} bytes")
    assert with_ - without < 0.05


def test_zenoh_query_of_the_declaration_is_answered_at_once(network):
    """The receiver rule: a reply and the query dropped at once (no "Timeout" reply on a short timeout)."""
    port, z = network
    with open_session(connect=[f"tcp/127.0.0.1:{port}"], listen=["127.0.0.1:0"], program="quick") as s:
        key = declaration.key(str(s.info.zid()))
        wait_until(lambda: declaration.query(z, 0.3)[0], timeout=10)
        answers = replies(z, key, 1.0)
        assert len([r for r in answers if r.ok is not None]) == 1 and not any(timed_out(r) for r in answers)
        assert str(answers[0].ok.encoding).startswith(str(zenoh.Encoding.APPLICATION_JSON))


def test_a_declared_session_dropped_without_close_is_released(network):
    """A Session dropped without close() goes with its last reference, as in 0.5.15: its declaration's thread has
    ended and its token is gone soon after. The first fix of the exit hang (0.5.16, not released) kept every declared
    Session until the program ended (CI: a C++ interop test that drops its Session saw the thread still there)."""
    port, z = network
    s = open_session(connect=[f"tcp/127.0.0.1:{port}"], listen=["127.0.0.1:0"], program="dropped")
    zid = str(s.info.zid())
    wait_until(lambda: zid in declaration.alive(z, 0.3), timeout=10)
    thread = f"declaration {zid[:8]}"
    assert [t for t in threading.enumerate() if t.name == thread]
    dropped = weakref.ref(s)
    del s
    assert dropped() is None
    assert not [t for t in threading.enumerate() if t.name == thread]  # ended while the Session went (_drop)
    assert gone_after(z, zid, timeout=10) < 3.0


def test_a_program_that_ends_without_closing_its_session_ends():
    """An exception before close(): the process ends (no thread of zenoh-python's waited for: the declaration's
    queries are answered by a daemon thread of its own)."""
    script = ("from xblick.session import open_session\ns = open_session(listen=['127.0.0.1:0'], local_router=False)\n"
              "raise KeyboardInterrupt\n")
    out = subprocess.run([sys.executable, "-c", script], capture_output=True, text=True, timeout=30)
    assert out.returncode != 0 and "KeyboardInterrupt" in out.stderr
