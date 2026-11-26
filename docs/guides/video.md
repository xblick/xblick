# Videos as camera streams

`xblick play` turns a video file or an image sequence into a camera stream. The stream has the same topics as
the stream of a camera node, so every node and tool works on it. You can therefore try a node without a camera
and without a recording.

Videos need the extra `xblick[video]`. It contains PyAV with FFmpeg inside its wheels, so no system FFmpeg is
needed.

## Quick start

```bash
xblick play clip.mp4 --camera video/clip --loop
```

The video is now the stream `video/clip/color`. Start any node on the camera `video/clip`.

```bash
# a stereo pair, with frames paired exactly (see "A stereo pair from two videos")
xblick play left.mp4 right.mp4 --stream cam/stereo/left --stream cam/stereo/right \
    --camera-info left.yaml --camera-info right.yaml --loop

# colour and depth from two files, the depth 3 ms later
xblick play color.mp4 depth.mkv --stream cam/color --stream cam/depth --offset-ms 0 --offset-ms 3

# an image sequence
xblick play 'frames/*.png' --camera video/clip --fps 10
```

## What counts as a video

`play` recognises an MCAP recording by its content. Every other file is a video:

- a file that FFmpeg reads, such as mp4, mkv, avi or webm,
- a directory of images, sorted by name,
- a glob such as `'frames/*.png'`.

`play` plays recordings one after another and videos at the same time. It refuses a mix of both.

| Source | Encoding of the stream |
| --- | --- |
| a colour video | `rgb8` |
| a grey video | `mono8` |
| a 16-bit grey video, for example FFV1 `gray16le` in Matroska | `16UC1`, which is depth in mm. The compressed transport uses RVL. |
| an image sequence | by the mode of the images: `I;16` gives `16UC1`, `L` gives `mono8`, all others `rgb8` |

An RGB-D camera can therefore be two files, one for colour and one for depth.

`play` shows every frame upright, as a photo viewer does. It turns and mirrors each image as its EXIF orientation
says, so a portrait photo of a phone becomes a portrait stream. It applies a video's display matrix in the same way.
`play` refuses a display matrix that turns by an angle other than a multiple of 90 degrees or that scales. The
stream's size, `--size` and `camera_info` refer to the upright frame. The first image of a sequence sets the
stream's size, and `play` resizes every later image to it.

Each video stream offers `image_raw` with the transports `shm` and `compressed`, its `camera_info`, an info record
with the source `video`, and diagnostics. With `--cuda-device`, `play` uploads each decoded frame once and also
offers a CUDA ring.

## Options

**The rule.** Give each option of a video once per video, in the order of the files. Give it once to apply it to
all videos, or leave it out. `play` refuses every other count with one line before it starts. Names, frame ids and
poses belong to one video, so give them once per video or not at all.

| Option | Meaning | Once for all |
| --- | --- | --- |
| `--stream BASE` | the name of the stream, for example `cam/stereo/left`. Required. | no |
| `--camera CAM` | in place of `--stream`: the stream `CAM/color` | no |
| `--frame-id ID` | the frame id. The default is the name of the stream with `_` in place of `/`. With `--pose` it names the body frame. | no |
| `--pose X Y Z YAW PITCH ROLL` | the fixed pose of the video's camera in the room, as [`xblick cam --pose`](cam.md#a-fixed-pose-in-the-room) | no |
| `--camera-info FILE` | a ROS calibration file, as `camera_calibration` writes it. Its size must be the size that the video plays at. | yes |
| `--hfov DEG`, `--vfov DEG` | without `--camera-info`: a pinhole camera with this field of view | yes |
| `--offset-ms MS` | moves the stamps and the send times of the stream, forwards or backwards. Use it for videos that are not cut in sync. | yes |
| `--size WxH`, `--scale F` | scale the frames | yes |
| `--fps FPS` | take the frame times from this rate. Required for image sequences. | yes |

Three videos, each with its own camera, calibration and pose, all 40 ms later than the wall clock:

```bash
xblick play door.mp4 desk.mp4 shelf.mp4 \
    --camera room/door --camera room/desk --camera room/shelf \
    --camera-info door.yaml --camera-info desk.yaml --camera-info shelf.yaml \
    --pose 0 0 2.2 0 30 0 --pose 1.5 0.5 1.2 180 10 0 --pose 3 -1 1.8 90 20 0 \
    --offset-ms 40
```

`--start S` and `--end S` select the part of the playback, in seconds of the videos. They hold for all videos.

**Calibration.** `play` reads the calibration file with the same reader as `xblick cam` and `xblick test-camera`. The
reader takes the keys `image_width`, `image_height`, `camera_name`, `camera_matrix`, `distortion_model`,
`distortion_coefficients`, `rectification_matrix` and `projection_matrix`. Each matrix has `rows`, `cols` and
`data`, and the list in `data` may span several lines. The reader refuses any other key and a matrix of the wrong
size. The `camera_info` of the stream carries every field as the file has it. `play` never scales a calibration. It
refuses a file whose size differs from the size of the video after `--size` or `--scale`.

**No calibration.** Without `--camera-info`, `--hfov` and `--vfov`, `play` assumes a pinhole camera with a
horizontal field of view of 60 degrees, the principal point in the centre of the image and no distortion. Its
start line then says `no calibration given: camera_info from an assumed horizontal field of view of 60 degrees`. A
node that measures in 3D gives wrong results with an assumed calibration.

**A pose.** `play` publishes the poses of a camera's videos once on `<camera>/tf_static`, in one message, and answers
it to every subscriber that starts later. The images of a video with a pose are in its optical frame
`<frame id>_optical`.

**Shared with recordings.** `--rate`, `--loop`, `--exit`, `--name`, `--report`, `--dir`, `--cuda-device` and the
connection options keep their meaning. The options `--key`, `--prefix`, `--clock` and every `--pace` other than
`wall` work only with recordings, and `play` refuses them for videos. `xblick recorder play` sends the options of
videos to a running play node. There, relative paths of files and calibration files start at the node's directory.

## Stamps

All videos of one playback use one clock. Each video has one thread for decoding and one for sending.

```
stamp = start of the playback (wall time) + (time of the frame in its video - --start) / --rate + --offset-ms
```

`play` sends each frame at its stamp. This has the following consequences.

- **Exact pairs.** Frames with equal time and equal offset receive the same stamp to the nanosecond in every
  stream. Stereo and RGB-D nodes pair them exactly, with a maximum offset of 0. With an `--offset-ms`, a node
  pairs them within its own maximum offset.
- **Different rates.** Each video keeps its frame rate. A colour video with 20 frames per second and a depth video
  with 10 pair at every second colour frame. Frames of equal time in their videos have equal stamps, to the
  nanosecond.
- **No dropped frames.** A frame that cannot go out at its stamp goes out late. The diagnostics count it.
- **Different lengths.** A shorter video sends no frame after its end. Subscribers keep its last frame.
- **Loops.** All streams start again together when the longest video has ended. The stamps move forward by the
  length of the longest video, so they always grow.
- **Pauses.** A pause stops the clock. After it, the stamps continue from the wall time of the resume.

[Timestamps](../concepts/timestamps.md) explains what the stamps can be compared with.

## Control a playback

The play node offers its controls as [parameters](../reference/parameters.md) of the node `xblick/<name>`. The
default name is `xblick/recorder`. They work for videos and recordings alike.

| Parameter | Meaning |
| --- | --- |
| `rate` | a multiple of the pace of the files. The next frame is due at the new rate, counted from the last frame. |
| `paused` | `true` pauses and `false` resumes. Reading it returns the state. |
| `position_s` | the position in seconds of the files. Setting it seeks. A video continues from the key frame before the position and decodes up to the frame. A recording continues from that log time. 0 starts again. After the end, a set plays the files again. |
| `loop` | start again at the end. You can switch it during the playback. |
| `duration_s`, `file` | read-only: the length of the files and the file that plays |

```bash
xblick param --node xblick/recorder --set paused:=true
```

After a seek, the stamps continue to grow, as after a loop. The frame that you seek to arrives one frame period
after the last frame sent. The node reports the values that the playback changes itself, such as the position, in
parameter events about once per second and at the end.

The terminal interface drives a playback through its Parameters tab and shows the position in its Nodes tab. The
commands `xblick recorder pause|resume|stop|status --name <name>` also work. Stepping with `--pace step` exists
only for recordings.

## Diagnostics

Each camera's `<camera>/diagnostics` contains the status `play`.

| Value | Meaning |
| --- | --- |
| frames per second, decode time, position | per stream |
| late frames | in total and in the last second. The status is WARN when frames went out late in the last second. |
| send skew | the spread between the streams of their mean difference "send time minus stamp", in ms. A stereo pair on one PC has about 0.05 ms. |

## Which nodes work on a video

State of the node READMEs on 2026-11-05.

| Node | Needs | On a plain colour video |
| --- | --- | --- |
| xblick-sam3 | colour with `camera_info` | yes (see the results below) |
| xblick-insightface | colour | yes |
| xblick-vjepa2 | colour | yes |
| xblick-tapnext | colour, depth, and an xblick-sam3 on the stream | with `--no-depth` (2D only), and with SAM 3 or `--no-segmenter` (points by hand) |
| xblick-boxer | colour, depth (`--no-depth` guesses distances), and the pose of the camera | with `--pose`. Without it the node skips every frame ("no pose"). |
| xblick-gem-x | colour and depth, paired | no. Yes with a 16-bit depth video beside it. |
| xblick-foundationpose, xblick-nemo, xblick-any6d, xblick-recgen, xblick-sam3d-objects | colour and depth | no. Yes with a depth video. |
| xblick-fast-foundationstereo | a rectified stereo pair `<camera>/left` and `<camera>/right` | from two videos with their calibration files (see the results below) |
| xblick-ivt | a rectified stereo pair as above | from two videos with their calibration files |
| xblick-mapping (cuVSLAM, nvblox) | stereo or RGB-D. nvblox also needs the pose of the camera. | no |

## A stereo pair from two videos

A stereo node reads the streams `<camera>/left` and `<camera>/right` with their `camera_info`. It needs rectified
images. Fast-FoundationStereo takes the focal length from `K[0]` of the left `camera_info` and the baseline from `P`
of the right one, where `P[3] = -f b`. This is the stereo convention of ROS, and `camera_calibration` writes the
files of a rectified pair in this form. The stereo nodes read no transform between the two cameras.

1. Export the two rectified videos with the same rate and length, and their two calibration files.
2. Play them as the two streams of one camera, each with its file:

   ```bash
   xblick play left.mp4 right.mp4 --stream cam/stereo/left --stream cam/stereo/right \
       --camera-info left.yaml --camera-info right.yaml --loop
   ```

3. Start the stereo node on the camera `cam/stereo`.

Frames of equal time get equal stamps, so the node pairs them exactly. A node that also needs the pose of the pair in
the room reads it from `tf_static`. Give each video its pose. The right camera sits the baseline b to the right of
the left one, which is -b along the body frame's y axis. For b = 0.12 m and a pair 1.2 m high:

```bash
xblick play left.mp4 right.mp4 --stream cam/stereo/left --stream cam/stereo/right \
    --camera-info left.yaml --camera-info right.yaml \
    --pose 0 0 1.2 0 0 0 --pose 0 -0.12 1.2 0 0 0 --loop
```

The two poses must agree with the baseline in the right `P`. `play` does not compare them.

## A complete example

Use one terminal for each step.

```bash
# 1. a router on this PC, if none is running
zenohd --no-multicast-scouting -l tcp/127.0.0.1:7447

# 2. the video as video/clip/color, looped
xblick play ~/clip.mp4 --camera video/clip --loop

# 3. nodes on the video, each from its own checkout and on its own GPU
cd xblick-sam3 && CUDA_VISIBLE_DEVICES=0 .venv/bin/xblick-sam3 --camera video/clip --words person cup
cd xblick-insightface && CUDA_VISIBLE_DEVICES=1 .venv/bin/xblick-insightface --camera video/clip

# 4. look at the streams, the reports of the nodes and the position of the playback
xblick
xblick recorder pause         # and: resume
```

Without a router, start the playback with `--listen tcp/127.0.0.1:<port>`. Give every other program `--connect
tcp/127.0.0.1:<port>`. Each of them also listens on a free loopback port, and the programs find each other through the
playback. Connect the terminal interface with `c` to the same address.

## Results

On 2026-11-05 on a server with a private router and two GPUs.

| Input | Node | Result |
| --- | --- | --- |
| NVIDIA's GEM-X demo video (1280 x 720, 30 frames per second, 7.2 s, looped) | xblick-sam3 with `--words person` | The colour stream had 30.0 Hz and an age at send of 0.6 ms. The instances of SAM 3 had 30.0 Hz and an age at send of 29.5 ms, which is its 29 ms per frame. |
| A stereo pair exported from a simulation recording (1280 x 720, 22 frames per second, 40 s) with calibration files | xblick-fast-foundationstereo with `--max-width 640` | The node paired every pair at 22.0 pairs per second. The depth had an age at send of 14.1 ms. The send skew of the two streams was 0.04 ms. |
