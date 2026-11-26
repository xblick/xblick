"""The commands as separate processes behind ``xblick router``, in the default (peer) mode, as a user starts them:
a publisher, then ``list``, ``hz`` (network and the move to the shm ring), ``param`` (liveliness and a query), a
client-mode session and ``xblick view``; a script that ends without closing what it opened, and one whose close at exit
stalls; the default port without any option (skipped when 7447 is taken); and the long-running commands, a node
(``xblick.node.start``) and the C++ tools ending on SIGTERM and on SIGINT as background jobs of a script (a
non-interactive shell starts those with SIGINT ignored).

0.5.3 found nothing here: a peer that only connected did not listen, and a Zenoh 1.10 router does not route between
peers (they reach each other directly), while the earlier router tests used sessions that listened."""

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
ENV = {**os.environ, "PYTHONUNBUFFERED": "1"}


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


LEFT_OPEN = """
import atexit
import importlib.util
import os
import sys
import threading
import time

import numpy as np
import zenoh

from xblick import session
from xblick.camera import CountedPublisher, ImageSubscriber, StaticTransformSubscriber

kind, router = sys.argv[1], sys.argv[2]


@atexit.register
def left():  # atexit runs after the close at exit: the threads still alive then
    own = (threading.main_thread().name, session.EXIT_WATCHDOG, "own query")  # the watchdog may have ended already
    print("left: " + ", ".join(sorted(t.name for t in threading.enumerate() if t.name not in own)), flush=True)


s = session.open_session(connect=[router])
if kind == "history":  # its thread reads the shm ring
    from xblick.history import FrameHistory

    history = FrameHistory(ImageSubscriber(s, "test/camera/color", transport="shm"), 4)
    end = time.monotonic() + 20
    while history.newest() is None and time.monotonic() < end:
        time.sleep(0.05)
    assert history.newest() is not None
elif kind == "static transforms":  # its thread asks for 10 s, nobody answers
    transforms = StaticTransformSubscriber(s, "nobody/camera", lambda message: None)
elif kind == "query":  # a query in a daemon thread, open at the end: its reader is inside Zenoh
    held = s.declare_queryable("held/query")  # never read: the query stays open
    threading.Thread(target=lambda: session.replies(s, "held/query", 5.0), name="own query", daemon=True).start()
elif kind == "parameters":  # the services' thread
    from xblick.parameters import Parameters

    parameters = Parameters(s, "exit", "xblick_test")
    parameters.declare("value", 1.0)
elif kind == "overlay":  # its thread draws and publishes
    from xblick.overlay import Overlay

    overlay = Overlay(s, "exit/camera", "node", draw=lambda rgb, results: rgb)
    reader = ImageSubscriber(s, "exit/camera/node/overlay", transport="raw")
    for _ in range(50):
        overlay.put(time.time_ns(), "exit", np.zeros((120, 160, 3), np.uint8), None)
        time.sleep(0.01)
elif kind == "commands":  # its thread runs the callback, which publishes
    from xblick.controller import CommandPublisher, CommandSubscriber

    echo = CountedPublisher(s, "exit/echo", zenoh.Encoding.APPLICATION_OCTET_STREAM)
    commands = CommandSubscriber(s, "exit", "arms", lambda command: [echo.put(b"x" * 1000) for _ in range(20)])
    sender = CommandPublisher(s, "exit", "arms", "base")
    for i in range(300):
        sender.send(joints={"a": float(i)})
elif kind == "recorder":  # its writer, discovery and status threads, its parameters
    from xblick.recording import Recorder

    recorder = Recorder(s, os.environ["XBLICK_TEST_DIR"], look_every=0.2)
    recorder.record()
time.sleep(0.3)
print("xblick-test-left-open: ends", flush=True)
"""
LEFT_OPEN_KINDS = ["history", "static transforms", "query", "parameters", "overlay", "commands"]
FAULTS = ("FATAL", "terminate called", "Fatal Python error", "Traceback", "Exception in thread", "nanobind: leaked")
try:
    import mcap  # noqa: F401 - the extra xblick[record]

    LEFT_OPEN_KINDS.append("recorder")
except ImportError:
    pass


def test_threads_of_what_a_script_left_open_end_before_python_finalises(camera, tmp_path):
    """Per kind of thread: a script that leaves its session and the thing that runs the thread open ends with exit
    code 0, and no thread of xblick but the exit watchdog is left when Python finalises. Before 0.5.18 these threads
    were not waited for; one inside a Zenoh call or a ring read at that moment aborted the process now and then
    ("FATAL: exception not rethrown", "terminate called without an active exception"), a history's left a nanobind
    leak report, and the transforms' asking thread a traceback once its session was closed. stderr is checked for
    these faults only: zenoh's own runtime threads panic now and then at the very end of a process ("static_init ...
    AccessError"), with exit code 0 and with or without xblick's threads."""
    failed = []
    for kind in LEFT_OPEN_KINDS:
        done = subprocess.run([sys.executable, "-c", LEFT_OPEN, kind, camera], stdin=subprocess.DEVNULL,
                              capture_output=True, text=True, timeout=60, check=False,
                              env={**ENV, "XBLICK_TEST_DIR": str(tmp_path)})
        left = [line for line in done.stdout.splitlines() if line.startswith("left: ")]
        if (done.returncode != 0 or "xblick-test-left-open: ends" not in done.stdout
                or left != ["left: "] or any(fault in done.stderr for fault in FAULTS)):
            failed.append(f"{kind}: exit code {done.returncode}\n{done.stdout}{done.stderr}")
    assert not failed, "\n".join(failed)


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
