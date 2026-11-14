"""xblick cam without a source (cam.select_source): the tree of this PC's cameras, their pixel formats, sizes and
rates in a pseudo-terminal, with stand-in cameras and a stand-in LiveCamera that prints what it was given. Moving,
expanding, Esc at each level, Enter at each level (what is not chosen comes from the ranking), the command printed
and that the same command typed later starts the same, items that cannot be started, Ctrl-C and SIGTERM with the
terminal restored, no terminal. PyAV is not needed: the stand-in replaces what needs it."""

import fcntl
import os
import pty
import re
import select
import shlex
import signal
import struct
import subprocess
import sys
import termios
import time

import pytest
from test_camera import free_port

ANSI = re.compile(r"\x1b\[[0-9;?]*[A-Za-z]|\x1b[()][A-Za-z0-9]|\x1b[=>78]")
UP, DOWN, RIGHT, LEFT, ENTER, ESC = b"\x1b[A", b"\x1b[B", b"\x1b[C", b"\x1b[D", b"\r", b"\x1b"


class Terminal:
    """A program in a pseudo-terminal (wide, so that no line wraps): its output without escape sequences, keys
    typed, its end."""

    def __init__(self, argv, title: str, env=None):
        env = {k: v for k, v in (os.environ if env is None else env).items() if k not in ("COLUMNS", "LINES")}
        env["TERM"] = "xterm-256color"  # as in a terminal emulator; Rich takes "dumb" for 80 columns
        self.master, self.slave = pty.openpty()
        fcntl.ioctl(self.slave, termios.TIOCSWINSZ, struct.pack("HHHH", 60, 300, 0, 0))
        self.before = termios.tcgetattr(self.slave)
        self.process = subprocess.Popen(argv, stdin=self.slave, stdout=self.slave, stderr=self.slave, close_fds=True,
                                        env=env, preexec_fn=self._controlling)
        self.raw = b""
        self.title = title

    @staticmethod
    def _controlling():
        """In the child: a session of its own with the pseudo-terminal as its controlling terminal, so that Ctrl-C
        typed there is SIGINT to it, as in a shell."""
        os.setsid()
        fcntl.ioctl(0, termios.TIOCSCTTY, 0)

    def _read(self, timeout: float):
        if select.select([self.master], [], [], timeout)[0]:
            try:
                self.raw += os.read(self.master, 65536)
            except OSError:  # the program ended and closed its side
                time.sleep(timeout)

    @property
    def text(self) -> str:
        return ANSI.sub("", self.raw.decode(errors="replace")).replace("\r", "")

    def frame(self) -> str:
        """The list as last drawn: from the last title on."""
        return self.text[self.text.rindex(self.title):]

    def wait(self, check, timeout=30.0):
        end = time.monotonic() + timeout
        while not check():
            assert time.monotonic() < end and self.process.poll() is None, self.text[-3000:]
            self._read(0.05)

    def shown(self, timeout=30.0):
        self.wait(lambda: self.title in self.text, timeout)

    def keys(self, data: bytes, until: str | None = None):
        """Types ``data``; with ``until``, waits until the list as drawn last contains it."""
        os.write(self.master, data)
        if until is not None:
            self.wait(lambda: self.title in self.text and until in self.frame())

    def end(self, timeout=20.0) -> tuple[int, str, bool]:
        """(exit code, output, whether the terminal's mode is as before)."""
        end = time.monotonic() + timeout
        while self.process.poll() is None and time.monotonic() < end:
            self._read(0.05)
        self.process.wait(timeout=5)
        for _ in range(5):
            self._read(0.05)
        return self.process.returncode, self.text, termios.tcgetattr(self.slave) == self.before

    def close(self):
        if self.process.poll() is None:
            self.process.kill()
            self.process.wait()
        os.close(self.master)
        os.close(self.slave)


CAMERAS = """
import sys
from fractions import Fraction as F
from xblick import cam, cli, v4l2
from xblick.v4l2 import Camera, Format, Size, name_format

def fmt(code, description, *sizes):
    f = name_format(code, description, 1 if code in ("MJPG", "H264", "HEVC") else 0)
    f.sizes = [Size(w, h, [F(r) for r in rates], "discrete" if rates else "") for w, h, *rates in sizes]
    f.sizes_from = "discrete" if sizes else ""
    return f

listed = [v4l2.Device("/dev/video0", "Azure Kinect 4K Camera", True), v4l2.Device("/dev/video1", "meta", False),
          v4l2.Device("/dev/video2", "C922 Pro Stream Webcam", True), v4l2.Device("/dev/video4", "Locked", None),
          v4l2.Device("/dev/video6", "Grabber", True)]
v4l2.devices = lambda: listed
v4l2.cameras = lambda devices=None, by_id=None: [
    Camera("/dev/video0", "Azure Kinect 4K Camera", [fmt("NV12", "Y/UV 4:2:0", (1280, 720, 30, 15)),
                                                     fmt("MJPG", "Motion-JPEG", (3840, 2160, 30), (1920, 1080, 30))]),
    Camera("/dev/video2", "C922 Pro Stream Webcam",
           [fmt("YUYV", "YUYV 4:2:2", (640, 480, 30, 15), (1920, 1080, 5)),
            fmt("MJPG", "Motion-JPEG", (1920, 1080, 30, 15), (1280, 720, 60, 30), (640, 480, 30)),
            fmt("HEVC", "HEVC")],
           by_id=["usb-046d_C922_Pro_Stream_Webcam_A1B2C3D4-video-index0"]),
    Camera("/dev/video4", "Locked", why="no permission to open it (add your user to the group video)"),
    Camera("/dev/video6", "Grabber", []),
]
cam.import_av = lambda: None
class Started:
    def __init__(self, session, source, base, **options):
        mode = {k: options[k] for k in ("size", "fps", "pixel_format")}
        print(f"STARTED {source} {base} {mode}", flush=True)
    def run(self):
        pass
cam.LiveCamera = Started
sys.exit(cli.main(["cam", *sys.argv[1:]]))
"""

TITLE = "Select camera / configuration:"


@pytest.fixture
def script(tmp_path):
    path = tmp_path / "cameras.py"
    path.write_text(CAMERAS)
    return path


@pytest.fixture
def terminal(script):
    """A function that starts ``xblick cam <options>`` with the stand-ins in a pseudo-terminal."""
    made = []

    def start(*options):
        port = free_port()
        made.append(Terminal([sys.executable, str(script), *options, "--listen", f"tcp/127.0.0.1:{port}"], TITLE))
        made[-1].port = port
        made[-1].shown()
        return made[-1]

    yield start
    for t in made:
        t.close()


def started(out: str) -> str:
    return next(line for line in out.splitlines() if line.startswith("STARTED"))


def test_the_list_and_enter_on_a_camera_starts_its_recommended_mode(terminal):
    t = terminal("--stream", "t/sel/color")
    frame = t.frame()
    for line in ("▸ /dev/video0  Azure Kinect 4K Camera", "/dev/video2  C922 Pro Stream Webcam  (by-id: "
                 "usb-046d_C922_Pro_Stream_Webcam_A1B2C3D4-video-index0)",
                 "/dev/video4  Locked  (no permission to open it (add your user to the group video))",
                 "/dev/video6  Grabber  (reports no formats: Enter starts its own mode)"):
        assert line in frame, frame
    assert "/dev/video1" not in frame  # the metadata node is not offered
    assert "❯ ▸ /dev/video0" in frame  # the cursor
    port = t.port
    assert (f"Enter starts: xblick cam /dev/video0 --pixel-format mjpeg --size 1920x1080 --fps 30 --stream t/sel/color "
            f"--listen tcp/127.0.0.1:{port}") in frame
    t.keys(DOWN, until="Enter starts: xblick cam /dev/video2 --pixel-format mjpeg --size 1920x1080 --fps 30")
    t.keys(ENTER)
    code, out, restored = t.end()
    assert code == 0 and restored, out
    command = (f"xblick cam /dev/video2 --pixel-format mjpeg --size 1920x1080 --fps 30 --stream t/sel/color --listen "
               f"tcp/127.0.0.1:{port}")
    assert (f"xblick cam: starting /dev/video2 with mjpeg 1920x1080 at 30 frames/s; the same without the list:\n"
            f"{command}\n") in out, out
    assert started(out) == ("STARTED /dev/video2 t/sel/color {'size': (1920, 1080), 'fps': 30.0, "
                            "'pixel_format': 'mjpeg'}")


def test_the_printed_command_starts_the_same_without_the_list(terminal, script):
    t = terminal()
    t.keys(DOWN + RIGHT * 4, until="❯ ▸ 1920x1080")  # into mjpeg, on its recommended size
    t.keys(DOWN + RIGHT + RIGHT, until="❯ 60 fps")  # 1280x720, its recommended rate
    t.keys(ENTER)
    code, out, restored = t.end()
    assert code == 0 and restored, out
    lines = out.splitlines()
    command = lines[lines.index(next(x for x in lines if "the same without the list" in x)) + 1]
    assert command == (f"xblick cam /dev/video2 --pixel-format mjpeg --size 1280x720 --fps 60 --listen "
                       f"tcp/127.0.0.1:{t.port}")
    again = subprocess.run([sys.executable, str(script), *shlex.split(command)[2:]], stdin=subprocess.DEVNULL,
                           capture_output=True, text=True, timeout=60)
    assert again.returncode == 0, again.stderr
    assert "Select camera" not in again.stdout and started(again.stdout) == started(out)
    assert started(out) == "STARTED /dev/video2 None {'size': (1280, 720), 'fps': 60.0, 'pixel_format': 'mjpeg'}"


@pytest.mark.parametrize("keys, expected", [
    # Enter on a pixel format: its recommended size and rate (YUYV: 640x480 at 30 beats 1920x1080 at 5)
    (DOWN + RIGHT + RIGHT + UP, "--pixel-format yuyv422 --size 640x480 --fps 30"),
    # Enter on a size: its highest rate
    (DOWN + RIGHT + RIGHT + UP + RIGHT + RIGHT + DOWN, "--pixel-format yuyv422 --size 1920x1080 --fps 5"),
    # Enter on a rate: exactly that
    (DOWN + RIGHT + RIGHT + RIGHT + RIGHT + RIGHT + RIGHT + DOWN, "--pixel-format mjpeg --size 1920x1080 --fps 15"),
    # a camera that reports no formats: its own mode, no options
    (DOWN * 3, ""),
])
def test_enter_at_each_level(terminal, keys, expected):
    t = terminal()
    t.keys(keys, until="Enter starts: xblick cam /dev/video" + ("6" if not expected else "2 " + expected))
    t.keys(ENTER)
    code, out, restored = t.end()
    assert code == 0 and restored, out
    source = "/dev/video6" if not expected else "/dev/video2"
    assert f"xblick cam {source} {expected} --listen".replace("  ", " ") in out, out


def test_only_the_active_branch_is_open_and_choices_are_kept(terminal):
    t = terminal()
    t.keys(RIGHT, until="▾ /dev/video0")  # expands the Kinect: mjpeg recommended
    assert "mjpeg  [recommended]" in t.frame() and "nv12" in t.frame()
    t.keys(DOWN * 3 + RIGHT, until="▾ /dev/video2")  # expanding the webcam collapses the Kinect
    frame = t.frame()
    assert "▸ /dev/video0" in frame and "nv12" not in frame
    assert "HEVC  HEVC  (not supported: FFmpeg's V4L2 input does not read it)" in frame
    t.keys(RIGHT + RIGHT + DOWN + DOWN, until="❯ ▸ 1280x720")  # mjpeg (recommended), on 1280x720
    t.keys(LEFT + LEFT, until="❯ ▸ mjpeg")  # to mjpeg, collapsed
    t.keys(RIGHT + RIGHT, until="❯ ▸ 1280x720")  # back in: the size last visited
    t.keys(b"q")
    code, out, restored = t.end()
    assert code == 1 and restored and out.rstrip().endswith("xblick cam: no camera chosen"), out


def test_esc_goes_back_to_the_cameras_then_leaves(terminal):
    t = terminal()
    t.keys(DOWN + RIGHT * 6, until="❯ 30 fps")  # down to a rate
    t.keys(ESC, until="❯ ▸ /dev/video2")  # back at the camera, collapsed
    assert "mjpeg  [recommended]" not in t.frame()  # its formats hidden
    t.keys(RIGHT, until="▾ mjpeg")  # its branch as it was left
    t.keys(ESC)  # at the top level: leaves
    code, out, restored = t.end()
    assert code == 1 and restored and "STARTED" not in out and out.rstrip().endswith("no camera chosen"), out


def test_what_cannot_be_started(terminal):
    t = terminal()
    t.keys(DOWN * 2, until="/dev/video4  Locked: cannot be started (no permission")
    t.keys(ENTER, until="/dev/video4  Locked cannot be started")
    t.keys(UP + RIGHT + RIGHT + DOWN, until="HEVC  HEVC: cannot be started (not supported")  # the list stays
    t.keys(ENTER, until="HEVC  HEVC cannot be started (not supported")
    t.keys(RIGHT, until="HEVC  HEVC: cannot be started")  # not expandable: still on it
    t.keys(b"q")
    code, out, restored = t.end()
    assert code == 1 and restored and "STARTED" not in out, out


@pytest.mark.parametrize("how", ["ctrl-c", "sigterm"])
def test_ctrl_c_and_sigterm_restore_the_terminal(terminal, how):
    t = terminal()
    t.keys(RIGHT, until="▾ /dev/video0")
    if how == "ctrl-c":
        t.keys(b"\x03")  # cbreak keeps Ctrl-C a signal: SIGINT
    else:
        t.process.send_signal(signal.SIGTERM)
    code, out, restored = t.end()
    assert code == 1 and restored and "STARTED" not in out and out.rstrip().endswith("no camera chosen"), out
    assert "\x1b[?25h" in t.raw.decode(errors="replace")  # the cursor shown again


def test_without_a_terminal_the_source_must_be_given(script):
    out = subprocess.run([sys.executable, str(script), "--stream", "t/sel/color"], stdin=subprocess.DEVNULL,
                         capture_output=True, text=True, timeout=60)
    assert out.returncode == 1 and "STARTED" not in out.stdout and out.stdout == ""
    assert out.stderr == ("xblick cam: no source given, and no terminal to choose one in: give the source, one of "
                          "/dev/video0  Azure Kinect 4K Camera; /dev/video2  C922 Pro Stream Webcam; /dev/video4  "
                          "Locked; /dev/video6  Grabber\n")


@pytest.mark.parametrize("options", [("--size", "640x480"), ("--fps", "30"), ("--pixel-format", "mjpeg"),
                                     ("--format", "v4l2")])
def test_a_mode_without_a_source_is_refused(script, options):
    out = subprocess.run([sys.executable, str(script), *options], stdin=subprocess.DEVNULL, capture_output=True,
                         text=True, timeout=60)
    assert out.returncode == 1 and out.stderr == (f"xblick cam: {options[0]} without a source: the list chooses the "
                                                  "mode; give the source as well, or leave them out\n"), out.stderr


def test_an_invalid_stream_is_refused_before_the_list(script):
    out = subprocess.run([sys.executable, str(script), "--stream", "color"], stdin=subprocess.DEVNULL,
                         capture_output=True, text=True, timeout=60)
    assert out.returncode == 1 and "xblick cam: stream 'color': <camera>/<stream>" in out.stderr, out.stderr
