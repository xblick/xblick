# Streams in a browser or VLC

`xblick view` shows camera streams in a browser or in VLC. It serves them as MJPEG over HTTP, so any MJPEG player can
show them. The server is Python's standard library; the package needs nothing more.

```bash
xblick view test/camera/color
```

```text
xblick view: serving 1 stream(s) on http://127.0.0.1:8090/ (this PC only; --http 0.0.0.0:PORT opens it to the network)
xblick view: test/camera/color: http://127.0.0.1:8090/test/camera/color.mjpg
xblick view: all of them: http://127.0.0.1:8090/
xblick view: test/camera/color at 15.0 frames/s, by hand: vlc --mjpeg-fps=18 --network-caching=20 --no-audio http://127.0.0.1:8090/test/camera/color.mjpg
xblick view: serving; press v for VLC, b for the browser view, Ctrl-C to end
```

## Which player starts

| Call | What happens |
| --- | --- |
| `xblick view STREAM ...` | the page with all streams side by side, in the system's browser |
| `xblick view STREAM ... --browser` | the same |
| `xblick view STREAM ... --vlc` | VLC, one window per stream. Without VLC the command ends with an error. |
| `xblick view STREAM ... --no-player` | no player; the addresses only |
| `xblick view` | a list of the streams found, to choose the streams and the viewer (below) |
| any of these on a system without a display | no player; the addresses and how to open them from another PC |

In every case the command keeps serving until Ctrl-C.

**Keys.** While the command serves on a terminal, two keys work without Enter, also with `--no-player`:

| Key | Action |
| --- | --- |
| `v` | open the VLC windows that are not open (any more), each with its stream's rate measured again |
| `b` | open the browser view |
| Ctrl-C | end the command |

On a system without a display both keys print one line and start nothing. Without a terminal (stdin a pipe or a
file) there are no keys, and the command reads nothing from stdin. When you close all VLC windows, the command says
so in one line and keeps serving. Press `v` to open them again.

```bash
xblick view test/camera/color                                 # the browser
xblick view test/camera/color test/camera/depth               # both streams side by side in the browser
xblick view test/camera/color --vlc                           # VLC
xblick view cam/video0/color --connect tcp/192.0.2.10:7447 --zenoh-mode client   # a stream of another PC
```

Put the streams before `--connect`, because `--connect` accepts several endpoints. To watch the streams of another PC
from a PC without a router, connect as a client. The router of the other PC then routes the frames. As a peer, the
command would listen on loopback only, and the programs there could not reach it. It says so at the start.

**The browser** is the system's browser, as Python's `webbrowser` module finds it. The environment variable `BROWSER`
chooses another one.

**No display**: on Linux, a system on which neither `DISPLAY` (X11) nor `WAYLAND_DISPLAY` (Wayland) is set has no
display. Every graphical program needs one of the two. On such a system the command starts neither VLC nor a browser.
Python would otherwise start a text browser such as `lynx` in the terminal. macOS and Windows always count as having
a display.

## VLC and the rate hint

VLC is the program `vlc` on `PATH`. Install it yourself, for example with `sudo apt install vlc`. With several
streams, VLC opens one window per stream.

VLC plays an MJPEG stream over HTTP in time only with a hint of its rate. Without the option `--mjpeg-fps`, VLC 3.0
fell behind a 30 Hz stream whatever its cache. With `--mjpeg-fps=35 --network-caching=20 --no-audio` it kept up with
the browser. `xblick view` therefore measures the rate of each stream before it starts VLC. This is the one place
where a measured value sets an option:

1. The command keeps the arrival times of the last 31 frames of each stream.
2. The rate is the inverse of the median interval between them. It needs at least 6 frames and waits for them for at
   most 5 s.
3. VLC gets `--mjpeg-fps` at 1.15 times the rate, rounded up: 35 for 30 frames/s, 18 for 15 frames/s.
4. VLC also gets `--network-caching=20` (a cache of 20 ms) and `--no-audio`.

A stream that delivers fewer than 6 frames within 5 s gets no VLC window, and the command says so in one line. The
start lines give each stream's measured rate and the command that opens it in VLC by hand, so that you can start VLC
on another PC as well. A stream whose rate changes a lot plays badly in VLC, because the hint fits one rate only. The
browser needs no hint.

## Choose streams in a list

Without streams on a terminal, `xblick view` lists the image streams that it finds, as a tree by namespace and
camera. Each stream has the viewers below it. The list is the same as that of [`xblick cam`](cam.md).

```text
Select streams / viewer:
├── ▾ cam
│   └── ▾ video2
│       ├── ❯ ▾ [x] color
│       │   ├── browser  (default)
│       │   ├── VLC
│       │   └── addresses only
│       └── ▸ [x] yolox/overlay
└── ▸ test

Enter opens: xblick view cam/video2/color cam/video2/yolox/overlay
```

| Key | Action |
| --- | --- |
| Up, Down | move to the previous or the next line |
| Space | mark or unmark the stream. On a namespace or a camera, it marks all of its streams. |
| Enter | open the marked streams together, or the stream under the cursor if none is marked |
| Right | expand the line. On a stream, it shows the viewers for the marked streams, and Enter on one starts it. |
| Left | collapse the line, or move to the line it belongs to |
| Esc | go back to the top level, or leave there |
| `q`, Ctrl-C | leave without starting anything, with exit code 1 |

Enter on a stream opens the browser, or the viewer of `--vlc` or `--no-player` if you gave one. Several marked streams
open side by side on one browser page. In VLC each stream gets its own window. The line under the list shows the
command that Enter would start. Before the streams open, `xblick view` prints that command:

```text
xblick view: starting cam/video2/color, cam/video2/yolox/overlay in the browser; the same without the list:
xblick view cam/video2/color cam/video2/yolox/overlay
```

The same command typed later opens the same view without the list. It also contains the other options that you gave,
such as `--connect`. A viewer that cannot start here is shown as unavailable with the reason: without a display, the
browser and VLC; without `vlc` on `PATH`, VLC. "addresses only" always works.

The list holds the streams that answer within `--seconds` (2 s) and that have a compressed transport. When it finds
none, the command ends with one line and exit code 1. The line names the cause where the session knows it, for
example that no router was reached. Without a terminal it shows no list. It ends with a line that names the streams
found, and you give the streams.

## What it shows

| Stream | Served as |
| --- | --- |
| JPEG on the compressed transport (colour, 8-bit grey) | the same bytes, unchanged |
| depth and 16-bit grey (RVL or PNG) | colours, encoded as JPEG (quality 85) |
| other PNG frames | encoded as JPEG (quality 85) |
| a stream without compressed transport (for example float disparity) | refused with one line |
| a point cloud | refused with one line; `xblick rerun` shows clouds |

The depth colours have a fixed range: red at 300 mm, through yellow and green, to blue at 5000 mm. Nearer is red,
farther is blue, and no depth (0) is black. The same distance has the same colour in every frame. It is the map of
`xblick.overlay.depth_colours`.

A stream that no publisher answers for within `--seconds` (2 s) is an error. The line lists the image streams that
were found.

## Addresses

| Address | Content |
| --- | --- |
| `http://127.0.0.1:8090/` | a page with every stream and its name, in a grid; no scripts and nothing from other servers |
| `http://127.0.0.1:8090/<stream>.mjpg` | one stream, `multipart/x-mixed-replace` |

The port is 8090. A port that another program uses is an error; give another one with `--http 127.0.0.1:PORT`.

## Watching from another PC

By default the server listens on 127.0.0.1, so only this PC can watch. Two ways open it to another PC:

- Forward the port with SSH and keep the default. On the other PC:

  ```bash
  ssh -L 8090:127.0.0.1:8090 robot-pc     # then open http://127.0.0.1:8090/ there
  ```

- Start with `--http 0.0.0.0:8090`. **There is no password: anyone who can reach the address can watch the
  streams.** The start line says so.

On a system without a display, the command prints both ways with its own port and host name.

## Slow clients

Each client gets the newest frame whenever it can take one. A slow client skips frames. It holds up neither the
subscriber nor the other clients. A client that takes no frame for 10 s is dropped, and a client that disconnects is
cleaned up at once. Ctrl-C ends the command, also while clients watch. VLC windows that the command started end with
it.

## Status and limits

- The values of the rate hint come from one measurement with VLC 3.0.23 and a 30 Hz stream.
- The example output above was written for this page. Its rate and hint follow the rules above.

The [command line reference](../reference/cli.md#xblick-view-streams-) lists all options.
