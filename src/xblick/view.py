"""``xblick view``: streams as MJPEG over HTTP, shown in a browser or VLC, from Python's standard library.

    xblick view test/camera/color                 # serves it and opens the grid page in the browser
    xblick view a/color b/depth                   # both side by side on that page
    xblick view test/camera/color --vlc           # ... in VLC instead, one window per stream
    xblick view test/camera/color --no-player     # serves it and prints the addresses only
    xblick view                                   # choose the streams and the viewer in a list

The command subscribes to each named stream's compressed transport and serves its JPEG frames unchanged, one URL per
stream (``/<stream>.mjpg``, ``multipart/x-mixed-replace``) and a page with a grid of all of them (``/``).
Depth (16-bit, RVL or PNG) is decoded, coloured by ``overlay.depth_colours`` (a fixed range: red at 300 mm through
green to blue at 5000 mm, no depth black) and encoded as JPEG; other frames that are no JPEG are encoded as JPEG.
Point clouds and streams without compressed transport are refused.

By default it opens the grid page in the system's browser. ``--vlc`` starts VLC instead (the program ``vlc`` on
PATH; without it the command ends with an error), one window per stream; ``--no-player`` starts nothing. On a system
without a display (Linux: neither DISPLAY nor WAYLAND_DISPLAY set) it starts no VLC and no browser: it prints the
addresses and how to open them from another PC (``--http``, or ``ssh -L PORT:127.0.0.1:PORT <this PC>``), and keeps
serving.

**VLC** plays such a stream only in time with a rate hint (``--mjpeg-fps``): without it, it lags behind whatever its
cache. This is the one place where a measured value sets an option: the rate of a stream is the inverse of the median
interval between its last RATE_FRAMES frames as they arrive here (at least RATE_MIN_FRAMES frames, waited for at
most RATE_WAIT_S seconds), and VLC gets ``--mjpeg-fps`` = that rate times MJPEG_FPS_FACTOR, rounded up (35 for 30
frames/s), with ``--network-caching=20 --no-audio``. A stream without enough frames in that time gets no VLC window,
with one line that says so. A stream whose rate changes a lot plays badly in VLC; the browser has no such hint.
The start lines give, per stream, its measured rate and the command to open it in VLC by hand.

``xblick view`` without a stream on a terminal shows the image streams it finds as a tree by namespace and camera
(pick.py): Space marks streams, Enter opens the marked ones (or the one under the cursor) in the browser, Right
offers the viewers (browser, VLC, addresses only). It prints the command that starts the same without the list.

While it serves on a terminal, the key v opens the VLC windows that are not open (any more) and b the browser view,
without Enter, also with --no-player; on a headless system they start nothing. Without a terminal nothing reads
stdin.

Each client gets the newest frame whenever it can take one: a slow client skips frames and holds up neither the
subscriber nor the other clients. It listens on 127.0.0.1 by default (this PC only); ``--http 0.0.0.0:PORT`` opens
it to the network WITHOUT A PASSWORD: anyone who can reach the address can watch.
"""

from __future__ import annotations

import collections
import html
import http.server
import io
import math
import os
import shlex
import shutil
import socket
import subprocess
import sys
import threading
import time
import webbrowser

from . import pick, topics
from .camera import find_streams, wait_for_info
from .codecs import JPEG, decompress, parse_format
from .msg import CompressedImage
from .session import why_nothing
from .overlay import DEPTH_FAR_MM, DEPTH_NEAR_MM, depth_colours

PORT = 8090  # the default port (fixed)
LISTEN = f"127.0.0.1:{PORT}"
BOUNDARY = "xblickframe"
VLC_CACHE_MS = 20  # VLC's --network-caching when xblick view starts it (with the rate hint, kept up in tests)
RATE_FRAMES = 31  # the rate of a stream from the intervals between its last frames (module docstring)
RATE_MIN_FRAMES = 6
RATE_WAIT_S = 5.0
MJPEG_FPS_FACTOR = 1.15  # VLC's --mjpeg-fps a little above the rate: 35 for 30 frames/s
CLIENT_TIMEOUT_S = 10.0  # a client that takes no frame for this long is dropped
JPEG_QUALITY = 85
# depth colours for frames xblick view encodes itself: red at 300 mm through green to blue at 5000 mm (fixed)
DEPTH_RANGE = (DEPTH_NEAR_MM, DEPTH_FAR_MM)


class ViewError(RuntimeError):
    """One line for the user: a stream that cannot be shown, a port that is taken, VLC that is missing."""


def parse_listen(text: str) -> tuple[str, int]:
    host, sep, port = text.rpartition(":")
    if not sep or not host or not port.isdigit():
        raise ViewError(f"--http {text!r}: ADDRESS:PORT, e.g. 0.0.0.0:{PORT}")
    return host.strip("[]"), int(port)


class _Feed:
    """The newest JPEG of one stream and its number; clients wait for a newer one."""

    def __init__(self, base: str):
        self.base = base
        self.jpeg: bytes | None = None
        self.number = 0
        self.cond = threading.Condition()
        self.error: BaseException | None = None
        self.arrivals: collections.deque[float] = collections.deque(maxlen=RATE_FRAMES)  # time.monotonic()

    def put(self, jpeg: bytes):
        with self.cond:
            self.jpeg, self.number = jpeg, self.number + 1
            self.arrivals.append(time.monotonic())
            self.cond.notify_all()

    def next_after(self, number: int, timeout: float) -> tuple[int, bytes | None]:
        with self.cond:
            self.cond.wait_for(lambda: self.number > number, timeout)
            return self.number, self.jpeg

    def rate(self) -> float | None:
        with self.cond:
            return rate_of(list(self.arrivals))


def rate_of(arrivals: list[float]) -> float | None:
    """Frames per second from arrival times (seconds, ascending): the inverse of the median interval; None for fewer
    than RATE_MIN_FRAMES times (module docstring)."""
    if len(arrivals) < RATE_MIN_FRAMES:
        return None
    intervals = sorted(b - a for a, b in zip(arrivals, arrivals[1:]))
    n = len(intervals)
    median = intervals[n // 2] if n % 2 else (intervals[n // 2 - 1] + intervals[n // 2]) / 2
    return 1.0 / median if median > 0 else None


def mjpeg_fps(rate: float) -> int:
    """VLC's --mjpeg-fps for a stream of ``rate`` frames/s: MJPEG_FPS_FACTOR times it, rounded up."""
    return math.ceil(round(rate * MJPEG_FPS_FACTOR, 6))


def to_jpeg(message: CompressedImage, depth_range: tuple[float, float]) -> bytes:
    """A compressed frame as JPEG: a JPEG as it is (the same bytes), else decoded (depth coloured) and encoded."""
    if parse_format(message.format)[1] == JPEG:
        return bytes(message.data)
    from PIL import Image

    image, encoding = decompress(message)
    if encoding in ("16UC1", "mono16"):
        image = depth_colours(image, *depth_range)
    elif encoding in ("bgr8", "bgra8"):
        image = image[..., 2::-1]
    elif encoding == "rgba8":
        image = image[..., :3]
    elif encoding not in ("rgb8", "mono8", "8UC1"):
        raise ViewError(f"frames of {encoding} cannot be shown")
    out = io.BytesIO()
    Image.fromarray(image.copy()).save(out, "JPEG", quality=JPEG_QUALITY)
    return out.getvalue()


class View:
    """The streams ``bases`` served as MJPEG on ``listen`` (host, port) until :meth:`close`. ViewError for a stream
    that does not answer, has no compressed transport or is a point cloud, and for a port that is taken."""

    def __init__(self, session, bases: list[str], listen: tuple[str, int] = ("127.0.0.1", PORT),
                 depth_range: tuple[float, float] = DEPTH_RANGE, seconds: float = 2.0):
        if not bases:
            raise ViewError("name the streams to show, e.g. xblick view test/camera/color")
        self.session, self.depth_range = session, depth_range
        self.feeds: dict[str, _Feed] = {}
        self._subscribers = []
        self.clients = 0
        self._clients_lock = threading.Lock()
        self._stop = threading.Event()
        for base in bases:
            self._check(topics.check_name(base), seconds)
        for base in bases:
            feed = self.feeds[base] = _Feed(base)
            self._subscribers.append(session.declare_subscriber(topics.compressed(base),
                                                                lambda s, f=feed: self._receive(f, s)))
        try:
            self.server = self._serve(listen)
        except ViewError:
            for subscriber in self._subscribers:
                subscriber.undeclare()
            raise
        self.host, self.port = listen[0], self.server.server_address[1]
        threading.Thread(target=self.server.serve_forever, name="xblick view", daemon=True).start()

    def _check(self, base: str, seconds: float):
        records = wait_for_info(self.session, base, seconds)
        if not records:
            found = [b for b, r in find_streams(self.session, "", seconds) if r.get("kind") == "image"]
            why = "" if found else why_nothing(self.session)
            raise ViewError(f"{base}: no publisher answers within {seconds:g} s; image streams found: "
                            + (", ".join(found) or "none") + (f" ({why})" if why else ""))
        record = records[0]
        if record.get("kind") == "points":
            raise ViewError(f"{base}: a point cloud stream; xblick view shows images only (xblick rerun shows clouds)")
        if "compressed" not in record.get("transports", ()):
            raise ViewError(f"{base}: no compressed transport (its encoding {record.get('encoding')} has no codec), so "
                            "it cannot be served")

    def _receive(self, feed: _Feed, sample):
        try:
            feed.put(to_jpeg(CompressedImage.deserialize(sample.payload.to_bytes()), self.depth_range))
        except Exception as error:  # noqa: BLE001 - shown by the command's loop (check)
            feed.error = error

    def check(self):
        for feed in self.feeds.values():
            if feed.error is not None:
                raise ViewError(f"{feed.base}: {type(feed.error).__name__}: {feed.error}")

    def rates(self, bases: list[str], wait: float = RATE_WAIT_S) -> dict[str, float | None]:
        """The rate of each of ``bases`` (rate_of), waiting up to ``wait`` seconds for those without one yet."""
        end = time.monotonic() + wait
        while any(self.feeds[b].rate() is None for b in bases) and time.monotonic() < end:
            time.sleep(0.05)
        return {b: self.feeds[b].rate() for b in bases}

    def url(self, base: str | None = None) -> str:
        host = "127.0.0.1" if self.host in ("0.0.0.0", "") else self.host
        return f"http://{host}:{self.port}/" + (f"{base}.mjpg" if base else "")

    def index(self) -> bytes:
        cells = "\n".join(f'<figure><img src="/{html.escape(b)}.mjpg" alt="{html.escape(b)}">'
                          f"<figcaption>{html.escape(b)}</figcaption></figure>" for b in self.feeds)
        return ("<!doctype html><html><head><meta charset='utf-8'><title>xblick view</title><style>"
                "body{margin:0;background:#111;color:#ddd;font:14px sans-serif}"
                "main{display:grid;gap:8px;padding:8px;grid-template-columns:repeat(auto-fit,minmax(320px,1fr))}"
                "figure{margin:0}img{width:100%;height:auto;background:#000}figcaption{padding:4px 0}"
                f"</style></head><body><main>\n{cells}\n</main></body></html>\n").encode()

    def _serve(self, listen: tuple[str, int]) -> http.server.ThreadingHTTPServer:
        view = self

        class Handler(http.server.BaseHTTPRequestHandler):
            timeout = CLIENT_TIMEOUT_S  # a client that takes nothing for this long is dropped

            def log_message(self, *args):
                pass

            def do_GET(self):
                path = self.path.split("?", 1)[0]
                if path in ("/", "/index.html"):
                    body = view.index()
                    self.send_response(200)
                    self.send_header("Content-Type", "text/html; charset=utf-8")
                    self.send_header("Content-Length", str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                    return
                base = path.lstrip("/").removesuffix(".mjpg")
                if not path.endswith(".mjpg") or base not in view.feeds:
                    self.send_error(404, "no such stream: " + ", ".join(f"/{b}.mjpg" for b in view.feeds))
                    return
                view._stream(self, view.feeds[base])

        class Server(http.server.ThreadingHTTPServer):
            address_family = socket.AF_INET6 if ":" in listen[0] else socket.AF_INET
            allow_reuse_address = True  # a restart right after clients left (TIME_WAIT); a listening port stays an error
            allow_reuse_port = False
            daemon_threads = True

        try:
            server = Server(listen, Handler)
        except OSError as error:
            raise ViewError(f"cannot listen on {listen[0]}:{listen[1]} ({error.strerror or error}): the port is taken "
                            "or the address is not this PC's; give another one with --http") from None
        return server

    def _stream(self, handler, feed: _Feed):
        """One client: the newest frame whenever it can take one (frames in between are skipped, for it alone)."""
        with self._clients_lock:
            self.clients += 1
        try:
            handler.send_response(200)
            handler.send_header("Content-Type", f"multipart/x-mixed-replace; boundary={BOUNDARY}")
            handler.send_header("Cache-Control", "no-cache")
            handler.end_headers()
            number = 0
            while not self._stop.is_set():
                number, jpeg = feed.next_after(number, 0.5)
                if jpeg is None or number == 0:
                    continue
                handler.wfile.write(f"--{BOUNDARY}\r\nContent-Type: image/jpeg\r\nContent-Length: {len(jpeg)}"
                                    "\r\n\r\n".encode() + jpeg + b"\r\n")
                handler.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, TimeoutError, OSError):
            pass  # the client went
        finally:
            with self._clients_lock:
                self.clients -= 1

    def close(self):
        self._stop.set()
        for feed in self.feeds.values():
            with feed.cond:
                feed.cond.notify_all()
        for subscriber in self._subscribers:
            subscriber.undeclare()
        self.server.shutdown()
        self.server.server_close()


def vlc_arguments(url: str, rate: float) -> list[str]:
    """VLC's options for a stream of ``rate`` frames/s at ``url`` (module docstring), without the program."""
    return [f"--mjpeg-fps={mjpeg_fps(rate)}", f"--network-caching={VLC_CACHE_MS}", "--no-audio", url]


def start_vlc(vlc: str, url: str, rate: float) -> subprocess.Popen:
    """One VLC window (``vlc`` the program) for a stream of ``rate`` frames/s."""
    return subprocess.Popen([vlc, *vlc_arguments(url, rate)], stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
                            stderr=subprocess.DEVNULL)


def headless() -> bool:
    """No display: on Linux and the other Unix systems neither DISPLAY (X11) nor WAYLAND_DISPLAY (Wayland) is set,
    the variables every graphical program there needs (a fixed rule, not a probe). macOS and Windows always have one.
    On a headless system xblick view starts no VLC and no browser: Python's webbrowser would start a text browser
    (lynx, w3m) in this terminal."""
    return (sys.platform not in ("darwin", "win32") and not os.environ.get("DISPLAY")
            and not os.environ.get("WAYLAND_DISPLAY"))


def remote_hint(view: "View") -> str:
    """How to open the addresses from another PC."""
    host = socket.gethostname()
    if view.host in ("127.0.0.1", "localhost", "::1"):
        return (f"open them from another PC: start with --http 0.0.0.0:{view.port} (no password), or forward the port "
                f"with ssh -L {view.port}:127.0.0.1:{view.port} {host} and open http://127.0.0.1:{view.port}/ there")
    return f"open them from another PC as http://{host}:{view.port}/"


def open_browser(url: str, prog: str):
    """The grid page in the system's browser (not called on a headless system), or one line when none opens."""
    if not webbrowser.open(url):
        print(f"{prog}: no browser could be opened; open {url} yourself", flush=True)


NO_DISPLAY = "this system has no display (neither DISPLAY nor WAYLAND_DISPLAY is set)"


class _Players:
    """The VLC windows, one per stream; ``open_vlc`` starts those that are not open (any more)."""

    def __init__(self, view: View, prog: str):
        self.view, self.prog = view, prog
        self.processes: dict[str, subprocess.Popen] = {}
        self.closed_said = False

    def open_vlc(self, asked: bool):
        """The windows that are not open, each with its stream's rate hint (measured now, View.rates); ``asked``: by
        the key v (then one line also when nothing is to start)."""
        if headless():
            print(f"{self.prog}: {NO_DISPLAY}: nothing started", flush=True)
            return
        vlc = shutil.which("vlc")
        if vlc is None:
            print(f"{self.prog}: {NO_VLC}", flush=True)
            return
        missing = [b for b in self.view.feeds if b not in self.processes or self.processes[b].poll() is not None]
        if not missing:
            if asked:
                print(f"{self.prog}: the VLC windows are open", flush=True)
            return
        started = 0
        for base, rate in self.view.rates(missing).items():
            by_hand(self.view, base, rate, self.prog)
            if rate is not None:
                self.processes[base] = start_vlc(vlc, self.view.url(base), rate)
                started += 1
        if started:
            print(f"{self.prog}: started VLC ({started} window(s))", flush=True)
        self.closed_said = False

    def all_closed(self) -> bool:
        return bool(self.processes) and all(p.poll() is not None for p in self.processes.values())

    def close(self):
        for process in self.processes.values():
            if process.poll() is None:
                process.terminate()
        for process in self.processes.values():
            try:
                process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                process.kill()


NO_VLC = "VLC is not installed (no vlc on PATH)"


def by_hand(view: View, base: str, rate: float | None, prog: str):
    """The line with a stream's measured rate and the command that opens it in VLC by hand, or why there is none."""
    if rate is None:
        print(f"{prog}: {base}: its rate is unknown (fewer than {RATE_MIN_FRAMES} frames within {RATE_WAIT_S:g} s); "
              "VLC needs it for --mjpeg-fps, so no VLC window is started for it", flush=True)
        return
    print(f"{prog}: {base} at {rate:.1f} frames/s, by hand: "
          + shlex.join(["vlc", *vlc_arguments(view.url(base), rate)]), flush=True)


def browser(view: View, prog: str):
    if headless():
        print(f"{prog}: {NO_DISPLAY}: nothing started", flush=True)
    else:
        open_browser(view.url(), prog)


class _Keys:
    """Single keys from the terminal on stdin (cbreak: no Enter, no echo, Ctrl-C stays a signal); the terminal's mode
    restored by ``close``. Without a terminal on stdin nothing reads it."""

    def __init__(self):
        self.fd = sys.stdin.fileno() if sys.stdin is not None and sys.stdin.isatty() else None
        self.saved = None
        if self.fd is not None:
            import termios
            import tty

            self.saved = termios.tcgetattr(self.fd)
            tty.setcbreak(self.fd)

    def get(self, timeout: float) -> str:
        """A key pressed within ``timeout`` seconds, else ""; without a terminal it only waits."""
        if self.fd is None:
            time.sleep(timeout)
            return ""
        import select

        if not select.select([self.fd], [], [], timeout)[0]:
            return ""
        return os.read(self.fd, 1).decode(errors="replace").lower()

    def close(self):
        if self.saved is not None:
            import termios

            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.saved)
            self.saved = None


KEYS = "press v for VLC, b for the browser view, Ctrl-C to end"


def run(session, bases: list[str], listen: str = LISTEN, player: str = "browser",
        depth_range: tuple[float, float] = DEPTH_RANGE, seconds: float = 2.0, prog: str = "xblick view") -> int:
    """The command: serve until Ctrl-C or SIGTERM (KeyboardInterrupt); the exit code. ``player``: "browser" (the
    default: the grid page in the system's browser), "vlc" (one window per stream; an error without ``vlc`` on PATH)
    or "none" (serve only). On a terminal the keys v (VLC windows that are not open) and b (the browser view) work in
    every case."""
    if player not in ("vlc", "browser", "none"):
        raise ValueError(f"player {player!r}: browser, vlc or none")
    if player == "vlc" and not headless() and shutil.which("vlc") is None:
        print(f"{prog}: --vlc: {NO_VLC}; install it, or leave out --vlc for the browser view", file=sys.stderr,
              flush=True)
        return 1
    try:
        view = View(session, bases, parse_listen(listen), depth_range, seconds)
    except ViewError as error:
        print(f"{prog}: {error}", file=sys.stderr, flush=True)
        return 1
    players = _Players(view, prog)
    keys = None
    try:
        local = view.host in ("127.0.0.1", "localhost", "::1")
        print(f"{prog}: serving {len(bases)} stream(s) on http://{view.host}:{view.port}/ "
              + ("(this PC only; --http 0.0.0.0:PORT opens it to the network)" if local else
                 "(open to the network: anyone who can reach this address can watch, there is no password)"),
              flush=True)
        for base in bases:
            print(f"{prog}: {base}: {view.url(base)}", flush=True)
        print(f"{prog}: all of them: {view.url()}", flush=True)
        if player == "vlc" and not headless():
            players.open_vlc(asked=False)  # measures the rates, prints the lines by hand
        else:
            if player == "browser" and not headless():
                open_browser(view.url(), prog)
            for base, rate in view.rates(bases).items():
                by_hand(view, base, rate, prog)
            if player != "none" and headless():
                print(f"{prog}: {NO_DISPLAY}, so no player is started; {remote_hint(view)}", flush=True)
        keys = _Keys()
        print(f"{prog}: serving; " + (KEYS if keys.fd is not None else "Ctrl-C to stop"), flush=True)
        while True:
            key = keys.get(0.5)
            view.check()
            if key == "v":
                players.open_vlc(asked=True)
            elif key == "b":
                browser(view, prog)
            if players.all_closed() and not players.closed_said:
                players.closed_said = True
                print(f"{prog}: the VLC windows were closed; still serving. "
                      + (KEYS[0].upper() + KEYS[1:] + "." if keys.fd is not None else
                         f"At {view.url()} until Ctrl-C."), flush=True)
    except KeyboardInterrupt:
        return 0
    except ViewError as error:
        print(f"{prog}: {error}", file=sys.stderr, flush=True)
        return 1
    finally:
        if keys is not None:
            keys.close()
        players.close()
        view.close()


# ------------------------------------------------------------------------------- choosing streams without a name

LIST_TITLE = "Select streams / viewer:"
LIST_KEYS = ("\u2191\u2193 Navigate   \u2192 Viewers   \u2190 Back   Space Mark   Enter Open   Esc Back/Exit   "
             "q Quit")
VIEWERS = (("browser", "browser"), ("VLC", "vlc"), ("addresses only", "none"))  # label, player
PLAYER_OPTIONS = {"browser": [], "vlc": ["--vlc"], "none": ["--no-player"]}


def showable(found: list[tuple[str, dict]]) -> list[str]:
    """The streams of find_streams that xblick view can show: images with a compressed transport."""
    return [b for b, r in found if r.get("kind") == "image" and "compressed" in r.get("transports", ())]


def stream_items(bases: list[str], default: str = "browser") -> list[pick.Item]:
    """The list's items: namespace, camera, then each stream (the rest of its name, markable) with the viewers as its
    children, ``default`` marked and first under the cursor. A viewer that cannot be carried out here is disabled,
    its note says why."""
    unavailable = {}
    if headless():
        unavailable = {"browser": NO_DISPLAY, "vlc": NO_DISPLAY}
    elif shutil.which("vlc") is None:
        unavailable = {"vlc": NO_VLC}
    roots: list[pick.Item] = []

    def group(items: list[pick.Item], label: str) -> pick.Item:
        for item in items:
            if item.label == label and item.value is None:
                return item
        items.append(pick.Item(label))
        return items[-1]

    for base in bases:
        parts = base.split("/")
        items = roots
        for part in parts[:min(2, len(parts) - 1)]:
            items = group(items, part).children
        viewers = [pick.Item(label, value=player, enabled=player not in unavailable,
                             note=f"(unavailable: {unavailable[player]})" if player in unavailable else
                             "(default)" if player == default else "")
                   for label, player in VIEWERS]
        items.append(pick.Item("/".join(parts[min(2, len(parts) - 1):]), viewers, value=base, markable=True,
                               chosen=[p for _, p in VIEWERS].index(default)))
    return roots


def choose_streams(session, seconds: float, command, default: str = "browser", stdin=None,
                   stdout=None) -> tuple[list[str], str]:
    """The streams and the viewer of ``xblick view`` without a stream, chosen in a list on the terminal (module
    docstring); Enter on a stream opens it with ``default``. ``command(bases, player)``: the complete command, shown
    in the status line. ViewError when no stream is found (with the reason the session knows), when stdin or stdout
    is no terminal, or when nothing was chosen."""
    stdin, stdout = stdin or sys.stdin, stdout or sys.stdout
    bases = showable(find_streams(session, "", seconds))
    if not bases:
        why = why_nothing(session)
        raise ViewError(f"no image streams found within {seconds:g} s" + (f" ({why})" if why else ""))
    if not (stdin.isatty() and stdout.isatty()):
        raise ViewError("no streams given, and no terminal to choose them in: give the streams, of these: "
                        + ", ".join(bases))
    tree = pick.Tree(stream_items(bases, default))

    def resolve(tree: pick.Tree):
        """(bases, player) that Enter starts now, or a line why it starts nothing."""
        item = tree.cursor
        marked = [i.value for i in tree.marked()]
        if item.parent is not None and item.parent.markable:  # a viewer
            if not item.enabled:
                return f"{item.label} {item.note}"
            return marked or [item.parent.value], item.value
        if item.markable:
            return marked or [item.value], default
        if marked:
            return marked, default
        return f"{item.label}: Space marks its streams, Right opens it"

    def status(tree: pick.Tree) -> str:
        chosen = resolve(tree)
        return chosen if isinstance(chosen, str) else "Enter opens: " + shlex.join(command(*chosen))

    def enter(tree: pick.Tree):
        chosen = resolve(tree)
        if isinstance(chosen, str):
            tree.message = chosen
            return None
        return chosen

    chosen = pick.run(tree, title=LIST_TITLE, keys=LIST_KEYS, status=status, enter=enter, fd=stdin.fileno(),
                      out=stdout)
    if chosen is None:
        raise ViewError("no stream chosen")
    return chosen
