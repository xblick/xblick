# Viewer

`xblick rerun` shows camera streams and the results of the nodes in [Rerun](https://rerun.io), an open-source
viewer for multimodal data. It places everything on the capture time of the frames.

```bash
pip install "xblick[rerun]"
xblick rerun
```

Without arguments, the command shows every stream, label image and detection topic that it finds, and the bodies,
IMU and diagnostics of their cameras. It looks again every 5 s, so a node that starts later appears too. It starts a
viewer on this machine and sends its own layout: a 3D view and a 2D view per stream.

## Choose what it shows

Name the streams as arguments and the results with options. The command then shows only what you named.

```bash
xblick rerun cam/azurekinect/color --bodies cam/azurekinect/bodies --segmentation cam/azurekinect/sam3/instances
```

| Option | Shows |
| --- | --- |
| `--points BASE` | a point cloud stream |
| `--detections KEY` | a node's 2D or 3D boxes, or its classification |
| `--segmentation KEY` | a node's label image, drawn over the image of its frame |
| `--bodies KEY` | body tracking results, `<camera>/bodies` or `<camera>/<node>/bodies` |
| `--trajectory KEY` | poses in the world as a path |
| `--imu KEY`, `--diagnostics KEY` | a camera's `<camera>/imu` or `<camera>/diagnostics` |

The transforms of a camera (`tf_static`, `tf`) come with its streams and results, because they place them in 3D.
`--no-transforms <camera>` leaves them out. The terminal interface builds such a command line from the entries that
you mark in its [Rerun tab](tui.md#rerun).

## Common calls

```bash
xblick rerun --connect sim-host --zenoh-mode client    # everything from another host
xblick rerun --prefix test/camera               # the streams and results of one camera
xblick rerun --serve                                   # a viewer connects with: rerun --connect <printed URL>
xblick rerun --connect-viewer rerun+http://laptop:9876/proxy  # to a running viewer
xblick rerun test/camera/color --save head.rrd         # write a recording file
```

Put the streams before `--connect`, because `--connect` accepts several endpoints. On a PC without a router of its
own, connect to another host as a client, as in the first line. The
[command line reference](../reference/cli.md#xblick-rerun-streams-) lists all options.

## What it draws

| Data | Drawn as |
| --- | --- |
| Colour images (JPEG, PNG) | the image, and the calibration as a pinhole camera |
| Depth (`16UC1` in mm) | a depth image, and a point cloud in 3D |
| 16-bit grey images (IR) | an image |
| Point clouds | coloured points in 3D, at most `--cloud-rate` (5) per second |
| Tracks (clouds with `u, v`, `object_id`, `visible`) | points in 3D and over the image of their frame, one colour per object |
| Bodies | skeletons in 3D and in the images of the camera |
| 3D detections (object poses, boxes) | boxes with class, score and tracking id, coloured by class, in 3D and in the images |
| 2D detections | rectangles in the image of their frame's stream |
| Label images (instances) | one colour per id over the image |
| IMU, diagnostics | time series, a text log |

A camera's `tf_static` places its streams. Each stream sits at the pose of its frame, and the bodies sit with them.
A camera whose pose arrives on `<camera>/tf` moves through the world with its streams, points, bodies and boxes.
A fixed camera's pose in the world on its `tf_static` (`xblick cam --pose`) places its streams in the world.
The 3D view then has z up.

`--trajectory <key>` draws the poses of any `TFMessage` topic as a path, with axes at the newest pose. Two
trajectories compare two pose sources:

```bash
xblick rerun --trajectory <camera>/vslam/tf --trajectory <camera>/tf
```

## Change the selection while it runs

The selection is a set of parameters of the node `xblick/rerun`. Each parameter has the name of its option:
`streams`, `points`, `detections`, `segmentation`, `trajectory`, `bodies`, `imu`, `diagnostics` and
`no_transforms`. A change applies at once. The bridge subscribes to a new topic and ends the subscription to a
removed one. The viewer keeps what it already received.

```bash
xblick param --node xblick/rerun --set bodies:='[cam/azurekinect/bodies]'
```

`drawn_over` names each result that is drawn over images, and over which images. The Rerun tab of the terminal
interface sets the parameters when you mark an entry while the viewer runs. A second bridge needs its own name
(`--name`). A bridge that was started without a selection shows everything that it finds, and its parameters are
read-only.

The bridge sends the viewer its layout once, 2 s after the start (`--layout-after`). It builds the layout from the
data that arrived by then, so it knows which results belong over which image. A topic that you add later appears in
the viewer's list of data and not in the layout. The viewer therefore never rearranges what you arranged.

## How results are found

Node results announce themselves in the same way as streams. A `DetectionPublisher` answers `<key>/info` with the
kind `detections`. The info record of a label image says `labels`. A publisher sets this with
`CameraPublisher(..., labels=True)`. For a node that does not announce its results, name the topics with
`--detections <key>` and `--segmentation <key>`. Without a selection, the command takes the bodies, IMU and
diagnostics of every camera that it shows.

## Rates and ages

For every topic, the viewer plots three time series on the capture timeline.

| Entity | Content |
| --- | --- |
| `rates/<topic>` | samples per second that arrive here, over the last second |
| `ages/<topic>` | the age at send in ms |
| `ages_received/<topic>` | the age at receive in ms. Across PCs it needs synchronised clocks. |

`ages_now` shows the newest values as a table:

| topic | rate Hz | age at send ms | age at receive ms | dropped/s |
| --- | --: | --: | --: | --: |
| test/camera/color | 30.0 | 0.6 | 1.9 | 0 |

"dropped/s" counts the samples that the publisher sent and that did not arrive. The viewer finds them as gaps in
the sequence numbers. For a stream that the viewer limits itself, such as point clouds, the rate is the limited
rate. `--no-ages` removes all of these. [Timestamps](../concepts/timestamps.md#ages) defines the ages.

## What you should know

- **The viewer keeps publishers busy.** Rerun cannot tell which streams you look at, so `xblick rerun`
  subscribes to every stream of its selection. A publisher that renders only for subscribers renders all that time.
  Remove a stream from the selection to end it.
- **A result over another frame's image needs the camera's transforms.** Without them, the bridge draws bodies and
  3D boxes only over the image of their own frame, and it places every stream at the camera's origin in 3D.
- **A point cloud from another host arrives compressed only where its publisher has rays**, as the point cloud node has. Other clouds arrive whole and are large. See [point clouds](point-clouds.md).
- **A publisher must be reachable.** Zenoh peers do not route data for each other. A publisher to which the viewer
  has no direct connection must listen on an endpoint of its own.
- **Rerun sends anonymous usage data** unless you turn it off once per machine with `rerun analytics disable`.
- **The viewer binary.** `--spawn` starts the `rerun` program that `rerun-sdk` installed beside this Python
  (`<venv>/bin/rerun`). Otherwise it uses a `rerun` on the `PATH`. If it finds neither, it prints where it looked.
  You therefore do not need an activated virtual environment.
