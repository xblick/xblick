# Command line

The command `xblick` inspects, records and plays streams.
Without a subcommand it opens the [terminal interface](../guides/tui.md). `xblick <command> --help` prints every
option.

## Connection options

Every command accepts the same options for its Zenoh session. [Zenoh's deployment
page](https://zenoh.io/docs/getting-started/deployment/) explains the modes.

| Option | Meaning |
| --- | --- |
| none | Connect to the router of this PC at `127.0.0.1:7447`. |
| `--connect <endpoint>` | Connect to another router or program. The option takes a Zenoh [endpoint](https://docs.rs/zenoh/1.10.1/zenoh/config/struct.EndPoint.html), such as `tcp/10.0.0.5:7447`. A host alone means `tcp/<host>:7447`, and `host:port` means `tcp/<host>:<port>`. |
| `--listen <endpoint>` | Listen on this endpoint. A peer without it listens on a free loopback port. |
| `--zenoh-mode peer\|client` | `peer` is the default. Use `client` for a program on another PC that only connects to a router. |
| `--config FILE` | A [Zenoh JSON5 configuration](https://zenoh.io/docs/manual/configuration/). It replaces the other options. |

A router does not route data between peers. Peers reach each other directly, so a Durchblick peer always listens.
Without `--listen` it listens on a free loopback port. A program on another PC that only connects to a router
therefore uses `--zenoh-mode client`.

| Setup | Options |
| --- | --- |
| a router on every PC (the standard) | none. The routers connect with `xblick router --connect tcp/<other PC>:7447`. |
| a visiting program, for example a viewer on a laptop | `--connect tcp/<other PC>:7447 --zenoh-mode client` |
| a peer on a PC without a router, on purpose | `--connect tcp/<other PC>:7447 --listen tcp/0.0.0.0:0`. Every program on the network can then connect to it. |
| two programs without a router | `--listen tcp/127.0.0.1:<port>` on one, `--connect tcp/127.0.0.1:<port>` on the other |

A peer that connects only to other PCs and listens on loopback only prints one line at the start: the programs there
usually cannot reach it. The line names the three ways out with the given endpoints. The messages for an empty
result (`no stream answered`, `nothing received`, `no publisher answers`) then name the same cause. The C++ programs
`xblick-cpp-camera`, `xblick-cpp-probe` and `xblick-points` print the same line and accept `--zenoh-mode`.
[Sessions and routers](../concepts/sessions.md) explains the background.

## ROS 2 options

Every command and every node that opens its session with `session.open_from_args` accepts these options. Without
`--ros` no command sends anything for ROS 2. [Durchblick and ROS 2](../integrations/ros2.md) explains what takes part.

| Option | Meaning |
| --- | --- |
| `--ros` | Take part in ROS 2 (rmw_zenoh, ROS 2 Jazzy) as well. The program is a ROS node, and its streams, results, transforms and parameters also appear on ROS's keys. Our keys stay as they are. |
| `--ros-domain ID` | The ROS domain id. The default is `ROS_DOMAIN_ID`, else 0. Only with `--ros`. |
| `--ros-node NAME` | The program's ROS node name. The default is the program's name with every character other than a letter, digit or `_` written as `_`, such as `xblick_cam`. Only with `--ros`. |

A node also declares the three options as read-only parameters `ros`, `ros_domain` and `ros_node`. With `--ros` a
program prints one line at the start with its ROS node and domain.

A client whose `--connect` endpoints do not answer ends with one line and exit code 1. The line names the endpoints
and says that a client needs its router there. In our tests Zenoh gave up within 10 s. Another error at the start of a
session, such as a `--listen` port that is taken, also ends with one line. Nodes that open their session with
`session.open_from_args` and the C++ programs do the same.

## Router

### `xblick router`

Runs a Zenoh router from the Zenoh library in the package, until Ctrl-C or SIGTERM. Every PC needs one router; this
one replaces `zenohd`. Nothing starts it on its own. It has no plugins (REST, storage) and reads no Zenoh
configuration file. A port that is taken ends it with one line: a router may run there already. On a loopback
address (`--listen tcp/127.0.0.1:PORT`) the start line says that other PCs cannot connect.

| Option | Meaning |
| --- | --- |
| `--listen ENDPOINT ...` | where it listens. The default is `tcp/0.0.0.0:7447`, where every program looks for the router of its PC. |
| `--connect ENDPOINT ...` | the routers of other PCs (`zenohd`'s `-e`) |

## Inspect

### `xblick list [prefix]`

Lists every stream with its encoding or fields, its transports, its host and its Durchblick version. The command
queries the info records and subscribes to nothing.

| Option | Meaning |
| --- | --- |
| `prefix` | only keys below this prefix, for example `test` |
| `--seconds S` | how long to look for streams |
| `--traffic` | subscribe to every topic and show rates and sizes |

### `xblick hz <stream>`

Subscribes to an image or point cloud stream. Prints the rate, the age at send and at receive, and the missing
frames.

| Option | Meaning |
| --- | --- |
| `--transport auto\|compressed\|raw\|shm\|ros` | the transport to measure. `ros` (with `--ros`) reads the ROS 2 topic `/<stream>/image_raw` of a ROS camera. |
| `--seconds S` | stop after this time |

### `xblick snapshot <stream> <file>`

Saves one frame as an image file. PNG keeps 16-bit depth. Options: `--transport`, `--timeout`.

### `xblick diagnostics [prefix]`

Prints the state of every program that publishes for a camera, once per second.

| Option | Meaning |
| --- | --- |
| `prefix` | only cameras below this prefix |
| `--values` | every value of a status. The default prints level and message. |
| `--seconds S` | stop after this time |

### `xblick rerun [streams ...]`

Shows streams and node results in the Rerun viewer. It shows its selection: the streams given as arguments and the
topics that the options below name. Without a selection it shows everything that it finds. Needs the extra
`xblick[rerun]`. See the [viewer guide](../guides/viewer.md).

| Option | Meaning |
| --- | --- |
| `streams` | the stream bases to show |
| `--points BASE` | a point cloud stream to show. `--cloud-rate` limits its rate. |
| `--detections KEY`, `--segmentation KEY`, `--trajectory KEY` | a node's detections, label image or poses to draw |
| `--bodies KEY` | body tracking results, `<camera>/bodies` or `<camera>/<node>/bodies` |
| `--imu KEY`, `--diagnostics KEY` | a camera's `<camera>/imu` or `<camera>/diagnostics` |
| `--no-transforms CAMERA` | leave out the camera's `tf_static` and `tf`. Otherwise they come with its streams and results. |
| `--prefix P`, `--seconds S` | without a selection: where and how long to look for streams |
| `--layout-after S` | send the viewer its layout once, S seconds after the start (default 2) |
| `--name NAME` | the node name of its parameters, `xblick/<name>` (default `rerun`). The selection is live there. |
| `--no-ages` | leave out the plots of rates and ages |
| `--stale-after S` | when a stream counts as stale |
| `--spawn` | start a viewer on this machine (the default) |
| `--connect-viewer URL` | send to a viewer that already runs, for example `rerun+http://laptop:9876/proxy` |
| `--serve [PORT]` | let viewers connect to this process (default port 9876). `--web [PORT]` also serves the web viewer. |
| `--save FILE` | write an `.rrd` recording |
| `--report S` | seconds between the status lines |

### `xblick view [streams ...]`

Shows streams in a browser (all streams side by side, the default) or in VLC (one window per stream). It serves them
as MJPEG over HTTP on 127.0.0.1:8090. VLC gets a rate hint measured from the frames. Without streams, on a terminal,
it lists the streams found to choose from and prints the command that starts the same without the list. On a system
without a display (Linux: neither `DISPLAY` nor `WAYLAND_DISPLAY` set) it starts no player and prints how to open the
addresses from another PC. On a terminal, the key `v` opens the VLC windows that are not open (any more) and `b` the
browser view. See the [view guide](../guides/view.md).

| Option | Meaning |
| --- | --- |
| `streams` | the stream bases to show. Without them, choose in a list. |
| `--browser` | open the page with all streams in the system's browser (the default) |
| `--vlc` | start VLC instead, one window per stream, with `--mjpeg-fps` a little above the measured rate. Without VLC: an error. |
| `--no-player` | start no player; print the addresses only |
| `--http ADDRESS:PORT` | where the HTTP server listens. The default is `127.0.0.1:8090`, this PC only. `0.0.0.0:8090` opens it to the network **without a password**. |
| `--seconds S` | how long to wait for each stream's publisher (default 2 s) |

## Cameras

### `xblick cam [source]`

Publishes a live camera that FFmpeg opens: a V4L2 device such as `/dev/video0`, or an RTSP or HTTP URL. Needs the
extra `xblick[cam]`. See the [guide](../guides/cam.md).

| Option | Meaning |
| --- | --- |
| `source` | `/dev/videoN`, a URL, or with `--format` another FFmpeg input. Without it, choose a V4L2 camera of this PC and its mode in a tree (camera, pixel format, size, rate). `xblick cam` prints the command that starts the same without the list. |
| `--stream BASE` | the stream, `<camera>/<stream>`. The default is `cam/<device node>/color`, for example `cam/video0/color`, or `cam/camera/color` for a URL or a `--format` source. A grey camera gets `/mono` in place of `/color`. |
| `--format FORMAT` | FFmpeg's input format, for example `lavfi`. The default is `video4linux2` for `/dev/...` and the protocol for a URL. |
| `--size WxH`, `--fps FPS`, `--pixel-format FORMAT` | the mode to ask the camera for. The camera must deliver it. `--pixel-format` is for V4L2 only. |
| `--camera-info FILE` | a ROS calibration file of the delivered size |
| `--hfov DEG`, `--vfov DEG` | without `--camera-info`: the field of view of the pinhole camera. With none of the three: an assumed horizontal field of view of 60 degrees, said in the start line. |
| `--frame-id ID` | the frame id. The default is the stream with `_` in place of `/`. With `--pose` it names the body frame, and the images are in `<ID>_optical`. |
| `--pose X Y Z YAW PITCH ROLL` | the camera's fixed pose in the room: its body frame (x forward, y left, z up) in `world`, in metres and degrees. A positive pitch looks down. It is published once on `<camera>/tf_static` with the step to the optical frame, and answered to later subscribers. Example, a webcam on a monitor 1.5 m high: `--pose 0 0 1.5 0 10 0`. |
| `--timeout S` | end with an error when no frame comes for this long (default 5). With `--reconnect`: open the camera again. |
| `--reconnect [SECONDS]` | open a camera that stops or cannot be opened again every SECONDS (default 2), also at the start. It must come back with its first mode. |
| `--mjpeg reencode\|passthrough` | an MJPEG camera's frames on the transport `compressed`: the decoded frame encoded again (default), or the camera's own JPEG with the standard Huffman tables inserted where a frame has none |
| `--option KEY=VALUE` | an FFmpeg input option, for example `rtsp_transport=tcp` (repeat for more). An option FFmpeg does not have for this source is refused. |
| `--cuda-device N` | the frames also in a CUDA ring on GPU `N`, besides the host ring. Without it: the host ring only, and no CUDA library is loaded. A GPU that cannot be used ends the command with one line and exit code 1. |
| `--param`, `--params-file`, `--dump-params`, `--params-read-only` | the options as parameters of the node `<camera>` |

## Test

### `xblick test-camera`

Publishes a moving test pattern on `test/camera/color`.

| Option | Default |
| --- | --- |
| `--namespace`, `--camera`, `--stream` | `test`, `camera`, `color` |
| `--size W H` | 640 480 |
| `--fps` | 15 |
| `--camera-info FILE` | none: a ROS calibration file of `--size`, read as by `xblick cam` |
| `--hfov`, `--vfov` | without `--camera-info`: the field of view of the pinhole camera. With none of the three: an assumed vertical field of view of 60 degrees, said in the start line. |
| `--frame-id` | `TestCamera`: the frame id of the images. With `--pose` the body frame's, and the images are in `<ID>_optical`. |
| `--pose X Y Z YAW PITCH ROLL` | none: the camera's fixed pose in the room, as `xblick cam --pose` |
| `--cuda-device N` | none: the frames also in a CUDA ring on GPU `N`, as `xblick cam --cuda-device` |

### `xblick-cpp-camera`, `xblick-cpp-probe`

The C++ counterparts: a test pattern publisher and a prober that prints JSON. `--jpeg-frames` makes the camera
pass JPEG frames through. `--points` tests point cloud streams. Both programs are part of the C++ build.

## Parameters

### `xblick param`

Reads and changes the parameters of a node. See [parameters](parameters.md).

| Option | Meaning |
| --- | --- |
| `--nodes` | list the nodes that offer parameters |
| `--node NAMESPACE/NAME` | the node to talk to |
| `names` | only these parameters. The default is all. |
| `--list`, `--get` | print names and values (the default) |
| `--describe` | also print type, range, constraints and read-only |
| `--set NAME:=VALUE` | change a parameter. Repeat the option for several. Values are YAML scalars. |
| `--atomically` | with `--set`: change all or none |
| `--dump FILE` | write the parameters as a ROS params file |
| `--timeout S` | how long to wait for the node |
| `--via-ros` | with `--ros`: the parameters of the ROS node `/<--node>` through ROS 2's keys, such as `--node talker`. A set carries no node and no deadline, because ROS has neither. |

## Record and play

See the [recording guide](../guides/recording.md) and the [video guide](../guides/video.md). Both commands need the
extra `xblick[record]`. Videos need `xblick[video]`.

### `xblick record`

| Option | Meaning |
| --- | --- |
| `--dir DIR` | where the files go. `${VAR}` and `~` are expanded. |
| `--name NAME` | the name of the recorder, used for its files and its control topics |
| `--file FILE` | one file, relative to `--dir`, without splitting |
| `--paused` | start idle and wait for a command |
| `--namespace NS`, `--prefix P` | record only streams and detections below this prefix |
| `--key KEY_EXPR` | record exactly these key expressions. Repeat the option for several. |
| `--points` | record point clouds too. They are large and can be computed again from depth. |
| `--split-minutes M` | start a new file after this time |
| `--compression none\|lz4\|zstd` | the MCAP compression |
| `--blocking` | for the keys of blocking publishers: block, and check that no sample is lost |
| `--log-time receive\|header` | the log time of a message: its arrival, or its own stamp |
| `--report S` | seconds between the status lines |

### `xblick recorder`

Sends commands to a running recorder. `xblick recorder --help` lists them.

### `xblick play <files ...>`

Plays MCAP files of `xblick record` one after another. Files that are not MCAP are played as videos or image
sequences, one stream each, on one clock.

| Option | Meaning |
| --- | --- |
| `--loop` | start again at the end |
| `--rate R` | a multiple of the recorded pace |
| `--pace wall\|step\|clock` | what drives the playback. `--clock KEY` names the clock for `--pace clock`. |
| `--prefix P` | publish below this prefix. The default is the recorded names. |
| `--key KEY_EXPR` | only topics that match this key expression |
| `--cuda-device N` | also offer the frames in a CUDA ring on this GPU |
| `--dir DIR`, `--name NAME` | the directory and the name of the node, for its control topics |
| `--exit` | end when the playback has ended |
| `--report S` | seconds between the status lines |

Options for videos. Give each once per video, in the order of the files, or once for all videos, or not at all.
`--stream`, `--camera`, `--frame-id` and `--pose` are only once per video. `play` refuses another count with one
line. See [videos](../guides/video.md#options).

| Option | Meaning |
| --- | --- |
| `--stream BASE` | the stream of a video, for example `cam/stereo/left` |
| `--camera CAMERA` | in place of `--stream`: the stream `<camera>/color` |
| `--camera-info FILE` | the ROS calibration file of a video, of the size it plays at, published as the file has it |
| `--hfov DEG`, `--vfov DEG` | without `--camera-info`: a pinhole camera with this field of view. With none of the three: an assumed horizontal field of view of 60 degrees, said in the start line. |
| `--frame-id ID` | the frame id of a video. With `--pose` the body frame's, and the images are in `<ID>_optical`. |
| `--pose X Y Z YAW PITCH ROLL` | the fixed pose of a video's camera, as `xblick cam --pose`, on `<camera>/tf_static` |
| `--offset-ms MS` | move the stamps and send times of a video by this time |
| `--size WxH`, `--scale S` | scale the frames |
| `--fps FPS` | take the frame times from this rate. The default is the times in the file. |
| `--start S`, `--end S` | play only this part of the videos (for all videos) |

## GPU sharing

### `xblick mps`

Runs NVIDIA's MPS control daemon for the nodes on the GPU of this PC. See
[sharing a GPU](../guides/sharing-a-gpu.md).

| Option | Meaning |
| --- | --- |
| `--pipe-dir DIR` | the directory of the daemon. Clients set `CUDA_MPS_PIPE_DIRECTORY` to it. |
| `--log-dir DIR` | the log of the daemon. The default is `<pipe-dir>/log`. |
| `--name NAME` | its name on Zenoh, `xblick/mps/<name>`. The default is the host name. |

## Relay

### `xblick relay [arguments]`

Starts the program `xblick-relay` with the arguments as they stand. The relay brings the streams that nodes on
this PC read from other PCs into the rings of this PC. See [relays](../concepts/transports.md#relays).
`xblick relay --help` prints the relay's options.

```bash
pip install "xblick[relay]"
xblick relay --stream test/camera/color
```

The command looks for the program in this order and nowhere else.

| Order | Place | Comes from |
| --- | --- | --- |
| 1 | the installed package `xblick-relay` | `pip install "xblick[relay]"` |
| 2 | `xblick-relay` on `PATH` | a CMake build of [xblick-relay](https://github.com/xblick/xblick-relay) |

If neither exists, the command fails and names both ways to install the relay. If the package is installed without
its program, the command fails as well. It does not try `PATH` then.

## Rings

### `xblick clean [rings ...]`

Lists the rings on this PC (`/dev/shm/xblick-*`) and tells which ones a killed publisher left behind. It removes
nothing unless you give `--remove`. See [rings left behind](../concepts/rings.md#rings-left-behind).

| Option | Meaning |
| --- | --- |
| `rings` | only these rings, for example `xblick-0123456789abcdef`. The default is every ring in `/dev/shm`. |
| `--remove` | remove the rings whose creator is gone and that no process maps |
| `--force` | with `--remove` and named rings: also remove a named ring without a creator record that no process maps, and a named ring whose creator is gone but that a reader still maps |

The listing has one line per ring with these columns.

| Column | Content |
| --- | --- |
| `RING` | the name in `/dev/shm` |
| `KIND` | `host` for a host ring, `cuda` for the control block of a CUDA ring |
| `SIZE` | the size of the file |
| `CREATOR` | the process id of the creator and its program name, or `(ended)` |
| `LAST FRAME` | how long ago the creator wrote its last frame |
| `ACTION` | `would remove`, `removed` or `keep` |
| `STATE` | `creator living`, `creator gone`, `unknown`, `lock held`, `another user's` or `cannot judge`, with the processes that map the ring |

The command never removes a ring of another user, a ring whose creator lives, or a ring it cannot judge.

## Point clouds

### `xblick-points`

The point cloud node, a C++ program with CUDA. See the [point cloud guide](../guides/point-clouds.md).

```bash
xblick-points --depth test/camera/depth --color test/camera/color
```
