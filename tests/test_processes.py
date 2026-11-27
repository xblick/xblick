"""The commands as separate processes behind ``xblick router``, in the default (peer) mode, as a user starts them:
a publisher, then ``list``, ``hz`` (network and the move to the shm ring), ``param`` (liveliness and a query), a
client-mode session and ``xblick view``; a script that ends without closing what it opened, and one whose close at exit
stalls; the default port without any option (skipped when 7447 is taken); and the long-running commands, a node
(``xblick.node.start``) and the C++ tools ending on SIGTERM and on SIGINT as background jobs of a script (a
non-interactive shell starts those with SIGINT ignored).

0.5.3 found nothing here: a peer that only connected did not listen, and a Zenoh 1.10 router does not route between
peers (they reach each other directly), while the earlier router tests used sessions that listened."""

import argparse
import os
import shlex
import shutil
import signal
import socket
import subprocess
import sys
import time
import urllib.request

import pytest
from test_camera import free_port
from test_view import read_frame

try:
    import av

    LAVFI = "lavfi" in av.formats_available
except ImportError:
    LAVFI = False

XBLICK = [sys.executable, "-m", "xblick.cli"]
# XBLICK_ROS=0: these check the programs' own lines and keys; their part in ROS 2 is tests/test_ros_session.py's
ENV = {**os.environ, "PYTHONUNBUFFERED": "1", "XBLICK_ROS": "0"}


def start(*args, env=ENV) -> subprocess.Popen:
    return subprocess.Popen([*XBLICK, *args], stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT, text=True, env=env)


def run(*args, timeout=60) -> subprocess.CompletedProcess:
    return subprocess.run([*XBLICK, *args], stdin=subprocess.DEVNULL, capture_output=True, text=True,
                          timeout=timeout, env=ENV)


def first_line(process: subprocess.Popen, prefix: str) -> str:
    line = process.stdout.readline()
    assert line.startswith(prefix), line + (process.stdout.read() if process.poll() is not None else "")
    return line


def end(process: subprocess.Popen, sig=signal.SIGTERM) -> int:
    if process.poll() is None:
        process.send_signal(sig)
    try:
        process.communicate(timeout=10)
    except subprocess.TimeoutExpired:
        process.kill()
        process.communicate()
        raise AssertionError(f"{process.args[3:5]} did not end on {sig!r}")
    return process.returncode


@pytest.fixture
def router():
    """``xblick router`` on a free loopback port, and its endpoint."""
    port = free_port()
    endpoint = f"tcp/127.0.0.1:{port}"
    process = start("router", "--listen", endpoint)
    line = first_line(process, f"xblick router: listening on {endpoint}; ")
    assert ("other PCs cannot connect (it listens on a loopback address only; --listen tcp/0.0.0.0:"
            f"{port} opens it to them)") in line
    yield endpoint
    assert end(process) == 0


@pytest.fixture
def camera(router):
    """``xblick test-camera`` behind the router, in peer mode with only --connect, publishing test/camera/color."""
    process = start("test-camera", "--connect", router, "--size", "160", "120")
    first_line(process, "xblick test-camera:")
    time.sleep(1.0)
    yield router
    assert end(process) == 0


def test_peers_find_each_other_through_the_router(camera):
    r = camera
    out = run("list", "--connect", r)
    assert out.returncode == 0 and out.stdout.startswith("test/camera/color "), out.stdout + out.stderr
    out = run("list", "--connect", r, "--zenoh-mode", "client")  # a client as well
    assert out.returncode == 0 and out.stdout.startswith("test/camera/color "), out.stdout + out.stderr
    out = run("list", "--traffic", "--connect", r)
    assert out.returncode == 0 and "test/camera/color/image_raw " in out.stdout, out.stdout + out.stderr
    out = run("hz", "test/camera/color", "--transport", "raw", "--seconds", "3", "--connect", r)  # over the network
    assert out.stdout.startswith("transport: raw") and " Hz " in out.stdout and "no frames" not in out.stdout, \
        out.stdout + out.stderr
    out = run("hz", "test/camera/color", "--seconds", "3", "--connect", r)  # auto: to the ring of this host
    assert out.stdout.startswith("transport: shm") and " Hz " in out.stdout and "no frames" not in out.stdout, \
        out.stdout + out.stderr


@pytest.mark.skipif(not LAVFI, reason="PyAV without lavfi (xblick[cam])")
def test_a_node_answers_queries_through_the_router(router):
    node = start("cam", "testsrc=size=160x120:rate=15,realtime", "--format", "lavfi", "--stream", "t/lav/color",
                 "--connect", router)
    try:
        first_line(node, "xblick cam:")
        time.sleep(1.0)
        out = run("param", "--nodes", "--connect", router)  # liveliness tokens
        assert out.returncode == 0 and "t/lav" in out.stdout.split(), out.stdout + out.stderr
        out = run("param", "--node", "t/lav", "--get", "--connect", router)  # a query and its reply
        assert out.returncode == 0 and out.stdout.strip(), out.stdout + out.stderr
    finally:
        assert end(node) == 0


def test_view_serves_frames_through_the_router(camera):
    port = free_port()
    env = {k: v for k, v in ENV.items() if k not in ("DISPLAY", "WAYLAND_DISPLAY")}  # headless: no player
    view = start("view", "test/camera/color", "--http", f"127.0.0.1:{port}", "--connect", camera, env=env)
    try:
        lines = [view.stdout.readline() for _ in range(6)]
        assert lines[0].startswith(f"xblick view: serving 1 stream(s) on http://127.0.0.1:{port}/"), "".join(lines)
        assert lines[4].startswith("xblick view: this system has no display"), "".join(lines)
        with urllib.request.urlopen(f"http://127.0.0.1:{port}/", timeout=10) as response:
            assert '<img src="/test/camera/color.mjpg"' in response.read().decode()
        with urllib.request.urlopen(f"http://127.0.0.1:{port}/test/camera/color.mjpg", timeout=10) as response:
            frames = [read_frame(response) for _ in range(5)]
        assert all(f.startswith(b"\xff\xd8") for f in frames) and len(set(frames)) > 1
    finally:
        assert end(view) == 0


READER = """
import sys

from xblick import node, session, topics
from xblick.camera import ImageSubscriber

node.start("xblick-test-reader")
camera = topics.check_name(sys.argv[2])
with session.open_session(connect=[sys.argv[1]]) as s:
    color = ImageSubscriber(s, f"{camera}/color", transport="auto")
    try:
        while color.latest(timeout=1.0) is None:
            pass
        print(f"xblick-test-reader: a frame of {camera}/color", flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        color.close()
"""


def test_a_node_given_a_stream_as_its_camera_ends_with_one_line(camera):
    """A node started with the stream that xblick test-camera prints (test/camera/color) instead of its camera reads
    test/camera/color/color, which nobody publishes: it ends with one line that names the camera, exit code 1. With
    the camera it reads its frames. With a camera that nothing publishes yet it waits, and says once what may be
    wrong."""
    reader = [sys.executable, "-c", READER, camera]
    started = time.monotonic()
    done = subprocess.run([*reader, "test/camera/color"], capture_output=True, text=True, timeout=60, env=ENV)
    took = time.monotonic() - started
    assert done.returncode == 1 and done.stderr.splitlines() == [
        "test/camera/color/color: no publisher answers it, and test/camera/color is a stream: the camera is "
        "test/camera. A node takes the camera: --camera test/camera"], done.stdout + done.stderr
    assert took < 15.0, took
    done = subprocess.run([*reader, "test/camera"], capture_output=True, text=True, timeout=60, env=ENV)
    assert done.returncode == 0 and "xblick-test-reader: a frame of test/camera/color" in done.stdout, \
        done.stdout + done.stderr
    waiting = subprocess.Popen([*reader, "not/started"], stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                               stderr=subprocess.STDOUT, text=True, env=ENV)
    try:
        time.sleep(7.0)
        assert waiting.poll() is None
    finally:
        waiting.send_signal(signal.SIGTERM)
        out, _ = waiting.communicate(timeout=20)
    assert waiting.returncode == 0, out
    hints = [line for line in out.splitlines() if "no publisher answers within 5 s" in line]
    assert len(hints) == 1 and "a camera that has not started yet, or a wrong name" in hints[0], out


UNCLOSED = """
import sys

import numpy as np

from xblick import session
from xblick.camera import CameraPublisher, ImageSubscriber

s = session.open_session(connect=[sys.argv[1]])
color = ImageSubscriber(s, "test/camera/color", transport="auto")
while color.latest(timeout=1.0) is None:
    pass
own = CameraPublisher(s, "unclosed/camera/color", encoding="mono8", frame_id="unclosed")
own.publish(np.zeros((4, 4), np.uint8))
print("xblick-test-unclosed: a frame", flush=True)
"""


def test_a_script_that_does_not_close_ends(camera):
    """A script that leaves its session, an auto subscriber and a publisher open ends at once, without a word on
    stderr: zenoh's callback threads are not daemons and ended only with their session, so it waited for good; the
    subscriber's chooser kept its ring reader, which nanobind then reported as leaked."""
    script = subprocess.Popen([sys.executable, "-c", UNCLOSED, camera], stdin=subprocess.DEVNULL,
                              stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, env=ENV)
    try:
        assert script.stdout.readline() == "xblick-test-unclosed: a frame\n"
        printed = time.monotonic()
        out, err = script.communicate(timeout=20)
        took = time.monotonic() - printed
    finally:
        if script.poll() is None:
            script.kill()
            script.communicate()
    assert script.returncode == 0 and not err, out + err
    assert took < 3.0, took  # well within a second on a PC that is not busy


STALLED = """
import threading

from xblick import session


class Stalled:
    def close(self):  # as a Zenoh close that never returns
        threading.Event().wait()


stalled = Stalled()
session.EXIT_CLOSE_S = 0.5
session.close_at_exit(stalled, "the stalled thing")
print("xblick-test-stalled: ends", flush=True)
"""


def test_a_close_at_exit_that_stalls_ends_the_program():
    """The close at the end of a program has a limit (session.EXIT_CLOSE_S): after it the program ends with exit code
    1 and one line on stderr that names what did not close. Before 0.5.17 a stalled close kept it for good."""
    script = subprocess.Popen([sys.executable, "-c", STALLED], stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                              stderr=subprocess.PIPE, text=True, env=ENV)
    try:
        assert script.stdout.readline() == "xblick-test-stalled: ends\n"
        printed = time.monotonic()
        out, err = script.communicate(timeout=20)
        took = time.monotonic() - printed
    finally:
        if script.poll() is None:
            script.kill()
            script.communicate()
    assert script.returncode == 1, (script.returncode, out + err)
    assert err.splitlines() == [("-c: the stalled thing did not close within 0.5 s at the end of the program: ended "
                                 "without waiting for it")], err
    assert 0.5 <= took < 3.0, took


def port_free(port: int) -> bool:
    """Nothing listens on the port (a connection attempt is refused); a bind test would also fail on connections
    of earlier tests that are still in TIME_WAIT."""
    with socket.socket() as s:
        return s.connect_ex(("127.0.0.1", port)) != 0


@pytest.mark.skipif(not port_free(7447), reason="port 7447 is taken (a router runs on this PC)")
def test_the_default_port_without_any_option():
    router = start("router")
    try:
        first_line(router, "xblick router: listening on tcp/0.0.0.0:7447; programs on this PC find it without options")
        camera = start("test-camera", "--size", "160", "120")
        try:
            first_line(camera, "xblick test-camera:")
            time.sleep(1.0)
            out = run("list")
            assert out.returncode == 0 and out.stdout.startswith("test/camera/color "), out.stdout + out.stderr
            out = run("hz", "test/camera/color", "--seconds", "2")
            assert " Hz " in out.stdout and "no frames" not in out.stdout, out.stdout + out.stderr
        finally:
            assert end(camera) == 0
    finally:
        assert end(router) == 0


CPP_BUILD = os.environ.get("XBLICK_CPP_BUILD")

NODE = """
import sys
import time

from xblick import node, session

node.start("xblick-test-node")  # SIGINT and SIGTERM: KeyboardInterrupt, also as a background job of a script
s = session.open_session(connect=[sys.argv[1]])
try:
    while True:
        time.sleep(0.1)
except KeyboardInterrupt:
    pass
finally:
    s.close()
"""


def commands(router: str) -> dict:
    """The programs' command lines (``XBLICK``: an ``xblick`` command)."""
    return {
        "router": [*XBLICK, "router", "--listen", f"tcp/127.0.0.1:{free_port()}"],
        "test-camera": [*XBLICK, "test-camera", "--connect", router, "--namespace", "sig"],
        "view": [*XBLICK, "view", "test/camera/color", "--no-player", "--http", f"127.0.0.1:{free_port()}",
                 "--connect", router],
        "cam": [*XBLICK, "cam", "testsrc=size=160x120:rate=15,realtime", "--format", "lavfi", "--stream",
                "sig/lav/color", "--connect", router],
        "node": [sys.executable, "-c", NODE, router],
        "cpp-camera": [os.path.join(CPP_BUILD or "", "xblick-cpp-camera"), "--connect", router, "--base",
                       "sig/cpp/color", "--size", "64", "48", "--fps", "15", "--seconds", "60"],
        "cpp-probe": [os.path.join(CPP_BUILD or "", "xblick-cpp-probe"), "--connect", router, "--base",
                      "test/camera/color", "--transport", "raw", "--frames", "100000", "--timeout", "60"],
    }


# the exit status a script sees: the probe has no cleanup and keeps the default action (killed by the signal)
EXIT = {("cpp-probe", "INT"): 128 + signal.SIGINT, ("cpp-probe", "TERM"): 128 + signal.SIGTERM}


@pytest.mark.skipif(shutil.which("bash") is None, reason="needs bash")
@pytest.mark.parametrize("name", ["router", "test-camera", "view", "cam", "node", "cpp-camera", "cpp-probe"])
@pytest.mark.parametrize("sig", ["INT", "TERM"])
def test_a_background_job_of_a_script_ends_on_sigint_and_sigterm(camera, name, sig):
    if name == "cam" and not LAVFI:
        pytest.skip("PyAV without lavfi (xblick[cam])")
    if name.startswith("cpp-") and not CPP_BUILD:
        pytest.skip("XBLICK_CPP_BUILD (the C++ build directory) is not set")
    argv = shlex.join(commands(camera)[name])
    script = ("sleep 30 &\ngrep '^SigIgn' /proc/$!/status\nkill $!\n"  # a background job: SIGINT ignored
              f"{argv} > /dev/null 2>&1 &\npid=$!\nsleep 4\nkill -{sig} $pid\n"
              # at most 5 s; a job that does not end is killed, so that no test leaves it running
              "for i in $(seq 50); do kill -0 $pid 2>/dev/null || break; sleep 0.1; done\n"
              "if kill -0 $pid 2>/dev/null; then kill -KILL $pid; wait $pid; echo exit=hung; "
              "else wait $pid; echo exit=$?; fi\n")
    started = time.monotonic()
    out = subprocess.run(["bash", "-c", script], capture_output=True, text=True, timeout=60, env=ENV)
    ignored = int(out.stdout.split()[1], 16)
    assert ignored & (1 << (signal.SIGINT - 1)), out.stdout  # the shell did start background jobs so
    assert out.stdout.strip().endswith(f"exit={EXIT.get((name, sig), 0)}"), out.stdout + out.stderr
    assert time.monotonic() - started < 4 + 5  # ended within some seconds of the signal


HELD = """
import signal
import sys

sys.argv, sys.orig_argv = {argv!r}, {orig_argv!r}  # as the console script or python -m starts it
import xblick

handler = signal.getsignal(signal.SIGTERM)
print(handler.__name__ if callable(handler) else handler.name, flush=True)
order = list(sys.modules)
print(all(order.index("xblick._signals") < order.index(m) for m in ("xblick._core", "numpy", "xblick.camera")))
"""

PY = sys.executable


@pytest.mark.parametrize("argv, orig_argv, handler", [
    (["/venv/bin/xblick", "test-camera"], [PY, "/venv/bin/xblick", "test-camera"], "note"),
    (["/venv/bin/xblick", "list"], [PY, "/venv/bin/xblick", "list"], "note"),
    (["-m", "hz", "/T"], [PY, "-X", "importtime", "-m", "xblick.cli", "hz", "/T"], "note"),
    (["-m", "router"], [PY, "-mxblick.cli", "router"], "note"),
    (["/venv/bin/xblick", "relay"], [PY, "/venv/bin/xblick", "relay"], "SIG_DFL"),
    (["/venv/bin/xblick"], [PY, "/venv/bin/xblick"], "SIG_DFL"),
    (["/venv/bin/xblick", "--connect", "pc"], [PY, "/venv/bin/xblick", "--connect", "pc"], "SIG_DFL"),
    (["-m", "relay"], [PY, "-m", "xblick.cli", "relay"], "SIG_DFL"),
    (["-m", "test-camera"], [PY, "-m", "tool.main", "test-camera"], "SIG_DFL"),
    (["a-program", "test-camera"], [PY, "a-program", "test-camera"], "SIG_DFL"),
    (["-c", "test-camera"], [PY, "-c", "import xblick", "test-camera"], "SIG_DFL")])
def test_signals_are_held_from_the_first_line_of_the_package(argv, orig_argv, handler):
    """The package (xblick/__init__.py, _signals.hold) takes SIGTERM and SIGINT before its other imports (_core,
    numpy, the camera code) when the process is an xblick command, the console script or python -m xblick.cli; not
    for xblick relay (the relay keeps the signals as started), the terminal interface (Textual's), or an import by
    another program, also not by one started with -m."""
    done = subprocess.run([sys.executable, "-c", HELD.format(argv=argv, orig_argv=orig_argv)],
                          stdin=subprocess.DEVNULL, capture_output=True, text=True, timeout=60, env=ENV)
    assert done.returncode == 0 and done.stdout == f"{handler}\nTrue\n" and not done.stderr, done.stdout + done.stderr


STARTING = """
import os
import signal
import sys

sys.argv = ["xblick", *sys.argv[1:]]  # as the console script starts it: xblick <command> ...
import xblick.cli as cli

os.kill(os.getpid(), getattr(signal, sys.argv.pop(1)))  # while it starts: before main()
sys.exit(cli.main())
"""


@pytest.mark.parametrize("command, sig, status", [
    (["test-camera"], "SIGTERM", 0), (["test-camera"], "SIGINT", 0), (["router"], "SIGTERM", 0),
    (["view", "test/camera/color"], "SIGINT", 0),
    (["list"], "SIGTERM", -signal.SIGTERM), (["list"], "SIGINT", -signal.SIGINT),
    (["hz", "/image_raw"], "SIGTERM", -signal.SIGTERM), (["param", "--nodes"], "SIGINT", -signal.SIGINT)])
def test_a_signal_while_a_command_starts_ends_it_before_it_begins(command, sig, status):
    """A signal held while the command started (_signals.held): main() ends it before it begins. A long-running
    command with exit code 0, as on a later Ctrl-C or SIGTERM; a one-shot command (cli.ONE_SHOT) by the signal
    itself, as an interrupted one (130 for SIGINT, 143 for SIGTERM in a shell). Nothing printed in either case."""
    done = subprocess.run([sys.executable, "-c", STARTING, sig, *command], stdin=subprocess.DEVNULL,
                          capture_output=True, text=True, timeout=60, env=ENV)
    assert done.returncode == status and not done.stdout and not done.stderr, (done.returncode, done.stdout,
                                                                               done.stderr)


INTERRUPTED = """
import sys

import xblick.cli as cli


def interrupted(*args, **kwargs):
    raise KeyboardInterrupt  # Ctrl-C or SIGTERM after main() took the signals, before the command's own handler


name = sys.argv[1]
if name == "router":
    cli.router.serve = interrupted
else:
    setattr(cli, {"test-camera": "_test_camera", "view": "_view", "cam": "_cam", "list": "_list"}[name], interrupted)
sys.exit(cli.main(sys.argv[1:]))
"""


@pytest.mark.parametrize("command, status", [
    (["router"], 0), (["view", "test/camera/color"], 0), (["cam", "/dev/video0"], 0), (["test-camera"], 0),
    (["list"], -signal.SIGINT)])
def test_a_keyboard_interrupt_before_the_command_takes_it_ends_a_long_running_command_with_0(command, status):
    """main() ends a long-running command with exit code 0 on a KeyboardInterrupt that comes before the command's
    own handler (end_on_signals has turned SIGTERM into one); before, router, view and cam ended by SIGINT then. A
    one-shot command keeps the interrupted end: the traceback, and Python ends by SIGINT."""
    done = subprocess.run([sys.executable, "-c", INTERRUPTED, *command], stdin=subprocess.DEVNULL,
                          capture_output=True, text=True, timeout=60, env=ENV)
    assert done.returncode == status, (done.returncode, done.stdout, done.stderr)
    assert ("KeyboardInterrupt" in done.stderr) == (status != 0), done.stderr


def test_every_command_is_one_shot_or_long_running():
    """cli.ONE_SHOT names commands of the parser, and every other command but relay serves until it is stopped: a new
    command needs its place in that set."""
    from xblick import cli

    parser = cli.build_parser()
    names = set(next(a for a in parser._actions if isinstance(a, argparse._SubParsersAction)).choices)
    assert cli.ONE_SHOT <= names
    assert names - cli.ONE_SHOT - {"relay"} == {"test-camera", "cam", "router", "view", "diagnostics", "rerun", "mps",
                                                 "record", "play"}


def caught(pid: int) -> int:
    """The signals the process has a handler for (SigCgt in /proc/<pid>/status); 0 once it has ended."""
    try:
        with open(f"/proc/{pid}/status") as status:
            return next(int(line.split()[1], 16) for line in status if line.startswith("SigCgt:"))
    except (FileNotFoundError, ProcessLookupError):
        return 0


@pytest.mark.skipif(not os.path.exists("/proc/self/status"), reason="needs /proc")
def test_a_sigterm_as_early_as_the_command_takes_it_ends_it_with_0(router):
    """SIGTERM at the first moment ``xblick test-camera`` has a handler for it, which the package installs before its
    other imports (_signals.hold): the command ends with exit code 0 and no traceback, as on a later SIGTERM. Before,
    the handler came after the imports and the parsing, and a SIGTERM before it, under load, killed the command
    (exit 143)."""
    process = subprocess.Popen(commands(router)["test-camera"], stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                               stderr=subprocess.STDOUT, text=True, env=ENV)
    try:
        deadline = time.monotonic() + 30.0
        while not caught(process.pid) & (1 << (signal.SIGTERM - 1)):
            assert process.poll() is None and time.monotonic() < deadline, "no handler for SIGTERM"
            time.sleep(0.001)
        process.send_signal(signal.SIGTERM)
        out, _ = process.communicate(timeout=20)
    finally:
        if process.poll() is None:
            process.kill()
            process.communicate()
    assert process.returncode == 0 and "Traceback" not in out, (process.returncode, out)


@pytest.mark.skipif(not CPP_BUILD, reason="XBLICK_CPP_BUILD (the C++ build directory) is not set")
def test_a_cpp_peer_behind_the_router(router):
    """The C++ session rule is Python's: a peer with --connect only listens on a free loopback port."""
    camera = subprocess.Popen([os.path.join(CPP_BUILD, "xblick-cpp-camera"), "--connect", router, "--base",
                               "t/cppr/color", "--size", "64", "48", "--fps", "30", "--seconds", "30"],
                              stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        time.sleep(1.5)
        out = run("list", "--connect", router)
        assert out.returncode == 0 and out.stdout.startswith("t/cppr/color "), out.stdout + out.stderr
        out = run("hz", "t/cppr/color", "--transport", "raw", "--seconds", "2", "--connect", router)
        assert " Hz " in out.stdout and "no frames" not in out.stdout, out.stdout + out.stderr
    finally:
        assert end(camera) == 0
