# Live cameras

`xblick cam` publishes a live camera that FFmpeg can open. This covers a webcam or a capture card as a V4L2 device
and an IP camera by its RTSP or HTTP URL. The stream has the same topics as the stream of a camera node, so every
node and tool works on it.

The command needs the extra `xblick[cam]`. It contains PyAV with FFmpeg inside its wheels, so no system FFmpeg is
needed. To play a video file as a camera, use [`xblick play`](video.md).

## Quick start

```bash
pip install "xblick[cam]"
xblick cam /dev/video0        # untested: no V4L2 device was at hand
```

The webcam is now the stream `cam/video0/color`. Once the first frame is in, the command prints the stream and the
camera that a node takes, each on a line of its own:

```
xblick cam: publishing the stream cam/video0/color
xblick cam: start a node with --camera cam/video0 (the camera, not the stream)
```

Start any node with that `--camera`. Nodes take the camera explicitly. A node that is given the stream instead ends
with one line that names the camera. When the session reached no Zenoh router
and no other session, the command prints a hint once at the start: `nobody can receive this stream yet`, and how
to start the router. It publishes anyway.

```bash
# an IP camera, asked for 1280 x 720 at 25 frames per second, as door/camera/color (untested: no IP camera)
xblick cam rtsp://192.0.2.20/stream1 --stream door/camera/color --size 1280x720 --fps 25

# FFmpeg's test pattern as a camera, without any device: cam/camera/color
xblick cam "testsrc=size=640x480:rate=30,realtime" --format lavfi
```

## Stream names

Without `--stream`, the stream is named after the device node: `/dev/video0` becomes `cam/video0/color`. A URL or a
source with `--format` has no device node and becomes `cam/camera/color`. A grey camera publishes on `.../mono`
instead of `.../color`. `xblick cam` names the stream at the first frame, when it knows the format, and writes the
name to its log. The camera, for example `cam/video0`, is the node of its parameters and diagnostics. `--stream`
replaces the default.

FFmpeg's filter `realtime` makes the test pattern deliver at its rate, as a camera does. Without it, the filter
graph delivers frames as fast as it can.

## Choose a camera and its mode

Leave out the source to choose one of the cameras of this PC and its mode in a list:

```bash
xblick cam
```

The list is a tree with four levels: camera, pixel format, size and rate. This example was drawn with stand-in
cameras in a test:

```text
Select camera / configuration:
├── ▸ /dev/video0  Azure Kinect 4K Camera
└── ▾ /dev/video2  C922 Pro Stream Webcam  (by-id: usb-046d_C922_Pro_Stream_Webcam_A1B2C3D4-video-index0)
    ├── ▾ mjpeg  [recommended]
    │   ├── ▾ 1920x1080  [recommended]
    │   │   ├── ❯ 30 fps  [recommended]
    │   │   └── 15 fps
    │   ├── ▸ 1280x720
    │   └── ▸ 640x480
    └── ▸ yuyv422

Enter starts: xblick cam /dev/video2 --pixel-format mjpeg --size 1920x1080 --fps 30
↑↓ Navigate   → Expand   ← Collapse   Enter Stream   Esc Back/Exit   q Quit
```

| Key | Action |
| --- | --- |
| Up, Down | move to the previous or the next line |
| Right | expand the line, or move into it when it is expanded |
| Left | collapse the line, or move to the line it belongs to |
| Enter | start the stream in the mode of the line |
| Esc | go back to the cameras, or leave when the cursor is on a camera |
| `q`, Ctrl-C | leave without starting anything, with exit code 1 |

Expanding a line collapses the other lines of its level, so only one branch is open. When you move into a line
again, the cursor goes to the entry you left there. At first that is the recommended entry. The line under the
tree shows the command that Enter would start.

Enter works on every level. The ranking in the next section fills in what you have not chosen:

| Enter on | Starts |
| --- | --- |
| a camera | its recommended mode |
| a pixel format | the recommended size in that format, at its highest rate |
| a size | that size at its highest rate |
| a rate | exactly that format, size and rate |

Before the stream starts, `xblick cam` prints the command that starts the same mode without the list:

```text
xblick cam: starting /dev/video2 with mjpeg 1920x1080 at 30 frames/s; the same without the list:
xblick cam /dev/video2 --pixel-format mjpeg --size 1920x1080 --fps 30
```

The command also contains the other options that you gave, such as `--stream`. The camera then starts as with a
given source. `xblick cam` checks the delivered mode as described in
[Size, rate and pixel format](#size-rate-and-pixel-format). `--format`, `--pixel-format`, `--size` and `--fps`
without a source are refused, because the list chooses the mode.

### The recommended mode

`xblick cam` ranks the modes that the camera advertises. It measures nothing and knows no camera models. These
rules apply in this order:

1. Only pixel formats that FFmpeg's V4L2 input reads count.
2. Modes with at least 29.97 frames/s win, if there are any. The NTSC rate 30000/1001 counts as 30. Without such a
   mode, all modes count.
3. Among those, the size with the most pixels up to 1920 x 1080 wins. If every size is larger, the smallest one
   wins. A smaller size never wins for a higher rate.
4. At that size, the highest rate wins. So 60 frames/s beats 30 frames/s only at the same size.
5. Then the pixel format decides: `mjpeg`, then `h264`, then `hevc`, then other compressed formats, then
   uncompressed formats. Every MJPEG frame decodes on its own. H.264 saves bandwidth, but its decoder adds delay.
6. Then the order of the device decides: the format that it lists first, then the size and the rate it lists first.

A camera with `h264` 1920x1080 at 30, `mjpeg` 1920x1080 at 30, `mjpeg` 1280x720 at 60 and `yuyv422` 1920x1080 at 5
frames/s gets `mjpeg` 1920x1080 at 30. A camera with 3840x2160 at 30 and 1920x1080 at 15 frames/s gets 3840x2160,
because rule 2 comes before rule 3.

### What the list shows

- The list leaves out metadata nodes. A webcam usually has two device nodes, one for video and one for metadata.
  The kernel tells the two apart in the capabilities of each node (`VIDIOC_QUERYCAP`).
- `xblick cam` reads the modes with the ioctls `VIDIOC_ENUM_FMT`, `VIDIOC_ENUM_FRAMESIZES` and
  `VIDIOC_ENUM_FRAMEINTERVALS`. These ioctls only read, so no setting of the camera changes before the stream
  starts.
- The pixel formats have the names that `--pixel-format` takes, which are FFmpeg's names. A format that FFmpeg's
  V4L2 input does not read is grey, marked `(not supported)`, and cannot be started. Two codes of V4L2 can share one
  name: `yuv420p` is `YU12` and `YV12`. `--pixel-format yuv420p` asks FFmpeg for the first code, so the second code
  counts as not supported.
- A device can give its sizes or rates as a range instead of a list. The list then offers the common values inside
  the range and its two ends. The common sizes are 3840x2160, 2560x1440, 1920x1080, 1600x1200, 1280x960, 1280x720,
  1024x768, 800x600, 640x480 and 320x240. The common rates are 120, 90, 60, 50, 30, 25, 24, 20, 15, 10 and 5
  frames/s.
- Rates are exact fractions, as the device states them. The command gives a rate in the decimal form that `--fps`
  takes, such as `--fps 29.97` for 30000/1001. The driver chooses the nearest rate it has.
- A camera that you may not open shows the reason on its line and cannot be expanded. Add your user to the group
  `video` to open it. A device that another program holds shows `busy` only if it refuses to be opened, because
  `xblick cam` does not try to start it.
- A camera that reports no formats can still be started in its own mode. Its line says so.
- A camera with a stable name in `/dev/v4l/by-id/` shows it on its line. The command uses the device node, such as
  `/dev/video2`, because the default stream name is made from it. A name in `/dev/v4l/by-id/` contains characters
  that a stream name cannot have, so such a source needs `--stream`. The device node's number can change when you
  plug the camera in again. The name in `/dev/v4l/by-id/` stays.

`xblick cam` checks a given `--stream` before it shows the list. Without a terminal, for example in a script or a
service, it shows no list. It ends with an error that names the cameras, and you give the source.

## Sources

| Source | How FFmpeg opens it |
| --- | --- |
| `/dev/video0`, any path below `/dev` | FFmpeg's V4L2 input (`video4linux2`) |
| `rtsp://`, `rtsps://`, `http://`, `https://`, `rtmp://`, `udp://`, `tcp://`, `srt://` | FFmpeg's protocol of the URL. FFmpeg takes the format from the stream. |
| anything with `--format FORMAT` | that FFmpeg input format, for example `lavfi` for a filter graph |

`xblick cam` refuses a file. `xblick play` plays files. It also refuses a device path that does not exist and names
the V4L2 devices of the PC.

## Size, rate and pixel format

Without options, the camera delivers its own mode. With options, `xblick cam` asks FFmpeg for a mode.

| Option | FFmpeg's option | Meaning |
| --- | --- | --- |
| `--size WxH` | `video_size` | the frame size |
| `--fps FPS` | `framerate` | the frame rate |
| `--pixel-format FORMAT` | `input_format` | V4L2 only: the format of the device, for example `mjpeg` or `yuyv422` |

A device may change the mode that it was asked for, and a URL or a filter graph ignores these options. `xblick cam`
therefore compares the first frame and the stream with what you asked for. It ends at a difference and names both:

```
xblick cam: testsrc=size=320x240:rate=30,realtime: asked for 640x480, the camera delivers 320x240 rgb24 at 30 frames/s
```

The rate of the stream is the rate that FFmpeg states for it. A camera that states no rate cannot be checked, so
`xblick cam` refuses `--fps` for it. To see the modes of a V4L2 device, run `xblick cam` without a source, or `v4l2-ctl
--list-formats-ext -d /dev/video0` from the package v4l-utils (untested here).

A colour camera becomes an `rgb8` stream, a grey camera `mono8`, and a 16-bit grey camera `16UC1`. The stream
offers `image_raw` with the transports `shm` and `compressed` (JPEG, or RVL for `16UC1`), its `camera_info`, an info
record with the source `cam`, and diagnostics.

## Camera info

Give the calibration of the camera with `--camera-info FILE`, a ROS calibration file as `camera_calibration` writes
it. Its size must be the delivered size, and `xblick cam` refuses another.

Without `--camera-info`, `xblick cam` assumes a pinhole camera, as `xblick play` does for a video. Its horizontal
field of view is 60 degrees, the principal point is the centre of the image, and the distortion is zero.
`--hfov DEG` or `--vfov DEG` sets another field of view. The command writes `camera_info ASSUMED` to its log. A node
that measures in 3D gives wrong results with an assumed calibration.

The frame id is `--frame-id`, by default the name of the stream with `_` in place of `/`.

## Stamps

The stamp of a frame is the time at which its data arrived from FFmpeg on this PC, read from the wall clock. The
camera exposed the frame earlier. The difference is the latency of the camera and its transport, which `xblick cam`
does not know.

Two such cameras therefore do not pair exactly. Their stamps come from two independent arrivals, and their latencies
differ. A stereo or RGB-D node pairs their frames only within a maximum offset. A pair then can hold frames that
were exposed several milliseconds apart. [Pairing streams](pairing-streams.md) explains the maximum offset, and
[timestamps](../concepts/timestamps.md) explains what a stamp can be compared with.

## When the camera stops

Without `--reconnect`, `xblick cam` ends with an error and the exit code 1 in these cases:

| Case | Message ends with |
| --- | --- |
| the camera delivers no frame for `--timeout` seconds (default 5), for example after it was unplugged | `no frame for 5 s (waiting for a frame; the camera stopped, or raise --timeout)` |
| the stream ends | `the stream ended` |
| FFmpeg reports an error while reading | `the camera stopped delivering (<FFmpeg's error>)` |
| the camera changes its frame size | `the camera changed its frame size from <old> to <new>` |

With `--reconnect [SECONDS]`, the command opens the camera again in these cases, every SECONDS (default 2) until
it delivers. It also tries the first opening again, because a robot or a PC may start before its IP camera. With
`--reconnect`, a V4L2 device that does not exist yet is waited for as well.

```bash
xblick cam rtsp://192.0.2.20/stream1 --reconnect 5        # untested: no IP camera was at hand
```

- Each loss is one log line, for example `lost (the stream ended); trying again every 5 s`. Each return is one log
  line, for example `back after 7.3 s`.
- While the camera is away, the diagnostics status `cam` has the level WARN and the message `camera away for 7 s
  (<why>), trying again every 5 s`. The value `losses` counts the losses.
- The camera must come back with the mode of its first frame: the same size and the same format. The stream's
  subscribers and its ring were set up for that mode, so another mode ends the command with an error that names
  both.
- A mode that differs from `--size`, `--fps` or `--pixel-format` ends the command also with `--reconnect`.
- `xblick cam` opens the camera again only after the previous opening has ended, because a V4L2 device cannot be
  opened twice. For a URL, FFmpeg ends a hung connection or read after `--timeout` seconds. A V4L2 device that hangs
  without an error keeps the command waiting until it returns.

Ctrl-C or SIGTERM ends the command within a second, also while FFmpeg waits for the camera and while the command
waits to open it again. The ring of the stream is removed from `/dev/shm`.

## MJPEG cameras

Many USB cameras deliver MJPEG: each frame is a JPEG file. By default, `xblick cam` decodes the frame and encodes
the decoded image again for the transport `compressed`. With `--mjpeg passthrough`, the transport `compressed`
carries the camera's own JPEG unchanged. The rings still get the decoded frame. This saves the second encoding and
its loss of quality.

```bash
xblick cam /dev/video0 --pixel-format mjpeg --mjpeg passthrough      # untested: no camera was at hand
```

- Many cameras leave the Huffman tables out of their frames, as the USB video class allows. Such a frame uses the
  standard tables of the JPEG standard (ITU-T T.81, Annex K.3). `xblick cam` inserts these tables before the
  frame's scan, as FFmpeg's bitstream filter `mjpeg2jpeg` does. It changes nothing else of the frame.
- A camera that does not deliver MJPEG ends the command with an error. A frame that is no JPEG ends it too, because
  the command never switches to encoding by itself.
- The stream keeps its encoding, for example `rgb8`. Each message's format names JPEG, as for an encoded frame.

`passthrough` is not the default, because it was not tested with a real camera. These receivers were tested with
4:2:2 frames that a test camera sent without Huffman tables:

| Receiver | Result |
| --- | --- |
| Python subscriber (Pillow, libjpeg-turbo) | decodes |
| C++ subscriber (libjpeg) | decodes, the same pixels as Pillow in 10 of 10 frames |
| xblick-relay with `--cpu` | decodes, the same pixels as Pillow |
| xblick-relay with nvJPEG | decodes; pixels differ from Pillow's by a mean of about 1 level, as its chroma upsampling differs |

Frames with restart markers were not tested.

## FFmpeg options

`--option KEY=VALUE` passes an input option to FFmpeg. Repeat it for more options. Use it for what `xblick cam` has
no option of its own for. These three options are useful for an RTSP camera. They exist in the FFmpeg of PyAV's
wheel (PyAV 19.0.2, FFmpeg's libavformat 63).

| Option | Meaning |
| --- | --- |
| `rtsp_transport=tcp` | RTSP over TCP, for a network that loses UDP packets |
| `timeout=5000000` | the RTSP socket's timeout, in microseconds |
| `fflags=nobuffer` | less buffering in FFmpeg, for a lower latency |

```bash
xblick cam rtsp://192.0.2.20/stream1 --option rtsp_transport=tcp --option fflags=nobuffer   # untested on a camera
```

`xblick cam` refuses an option that FFmpeg does not have where it applies options for this source. FFmpeg itself
would ignore such an option without a word. The command looks in FFmpeg's generic input options, in the options of
the input format (`--format`, or the URL's scheme such as `rtsp`) and in the options of the URL's protocol (such as
`http`). For an HTTP URL, FFmpeg detects the format from the data, so an option of that format needs `--format`.
The options `video_size`, `framerate` and `input_format` are given with `--size`, `--fps` and `--pixel-format`.

## Parameters and diagnostics

The options of `xblick cam` are read-only [parameters](../reference/parameters.md) of the node `<camera>`, for
example `cam/video0`. They apply at the next start. `--param NAME:=VALUE` and `--params-file FILE` set them at the
start, as for every node.

```bash
xblick param --node cam/video0
```

`<camera>/diagnostics` contains the status `cam` once per second.

| Value | Meaning |
| --- | --- |
| `frames/s` | the frames published in the last second |
| `mode` | the delivered mode, for example `640x480 mjpeg at 30 frames/s` |
| `decode ms` | the mean time from the arrival of a frame's data to the decoded image |
| `losses` | how often the camera was lost (with `--reconnect`) |

## Status and limits

- Tested on 2026-11-13 with FFmpeg's `lavfi` test pattern and with an HTTP URL that served a Matroska file with
  MJPEG frames. No V4L2 device, RTSP camera or other IP camera was tested.
- The list of cameras was tested in a pseudo-terminal with stand-in cameras. Reading the capabilities and the modes
  was tested with sample answers of the ioctls. Neither was tested on a PC with a real camera.
- The list does not read the formats of a multi-planar capture node. Such a camera can be started in its own mode.
  Whether FFmpeg's V4L2 input reads such a node was not tested.
- Whether FFmpeg reaches the second code of a shared name, such as `YV12`, when the device lacks the first one was
  not tested. The list therefore offers only the first.
- A list taller than the terminal is cut at the bottom.
- The command decodes the JPEG frames of an MJPEG camera and encodes them again for the transport `compressed`.
  It does not pass the camera's JPEG frames on unchanged.
- The command publishes the first video stream of a source.
- `--reconnect` was tested with an HTTP server that ended its stream, refused connections and came back, and with a
  file of another size after the return. It was not tested with an unplugged USB camera.
- `--mjpeg passthrough` was tested with a test camera over HTTP (FFmpeg's `mjpeg` format, 4:2:2 frames without
  Huffman tables) and the receivers in the table above. No real MJPEG camera was tested.
- `--option` refuses options of a protocol below another one (the `tcp` under `rtsp`) and FFmpeg's generic URL
  options such as `rw_timeout`, because FFmpeg's public API does not tell whether they exist.
