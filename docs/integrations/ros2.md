# Durchblick and ROS 2

Durchblick uses the message types and conventions of ROS 2, and it needs no ROS installation. Every program also
takes part in ROS 2 on `rmw_zenoh_cpp` (Jazzy and Lyrical), unless you turn it off with `--no-ros`. This page states
what is compatible, what the part in ROS 2 adds, how you try it with ROS 2's own tools, and how ROS 2 nodes on DDS
read the topics through a bridge.

## What is identical

| Aspect | State |
| --- | --- |
| Message definitions and bytes on the wire | identical. Tests compare every message type with an independent CDR implementation ([protocol](../concepts/protocol.md)). |
| Topic layout of a camera stream | identical to `image_transport`: `image_raw`, `image_raw/compressed`, `camera_info` |
| Transforms, diagnostics, clock | the types and roles of `/tf`, `/tf_static`, `/diagnostics` and `/clock` |
| Parameters | the messages and services of `rcl_interfaces`, with the same bytes ([parameters](../reference/parameters.md)) |
| Calibration files | the YAML format of `camera_calibration`. `CameraInfo.read_file` reads it and `write_file` writes it. |
| Recordings | MCAP files with ROS 2 schemas, which ROS 2 tools open. `ros2 bag play` of a recording feeds tf2 from its `/tf_static`, and `xblick play` plays bags of `ros2 bag record` ([recording](../guides/recording.md)). |

Durchblick's own keys differ from the keys of `rmw_zenoh`. A ROS 2 node therefore sees nothing of a program that
is started with `--no-ros`.

## Take part in ROS 2

Every command and every node takes part in ROS 2. The program sends on our keys as before and also on the keys of
`rmw_zenoh_cpp`. It needs `rmw_zenoh_cpp` 0.2.13 (ROS 2 Jazzy) or 0.10.6 or newer (ROS 2 Lyrical) on the ROS side.
A publisher sends the ROS form only while a ROS subscriber is there. Until then the part in ROS 2 costs the ROS
node's announcements and nothing else.

The keys and the attachment are `rmw_zenoh_cpp`'s format, which ROS 2 itself does not define. Nodes on another RMW,
such as one on DDS, do not use Zenoh and see nothing of it. For them see
[ROS 2 on DDS through a bridge](#ros-2-on-dds-through-a-bridge). Humble's `rmw_zenoh_cpp` puts no type hash into its
keys, so a Humble node never matches ours. Jazzy's releases 0.2.0 to 0.2.3 used another attachment
([ROS 2 by default](../project/ros-roadmap.md#which-versions-use-these-33-bytes)).

| Option | Meaning |
| --- | --- |
| `--no-ros` | take no part in ROS 2. The program sends nothing on ROS's keys and is no ROS node. |
| `--ros` | accepted for older command lines. It does nothing. |
| `--ros-domain ID` | the ROS domain id. The default is `ROS_DOMAIN_ID`, else 0. |
| `--ros-node NAME` | the ROS node, as `NAME` or `/namespace/NAME`. The default is described in [One node per program](#one-node-per-program). |
| `--ros-params-writable` | ROS clients may set the program's parameters. Without it they read them, and their sets are refused. A ROS set is unchecked and uncounted: see [Parameters and services](#parameters-and-services). |
| `--ros-clock KEY` | the program's clock key goes to ROS's `/clock`: see [The clock](#the-clock). |

The environment variable `XBLICK_ROS` turns the part off for a whole shell. `XBLICK_ROS=0` acts as `--no-ros`, and
`XBLICK_ROS=1` or no variable leaves it on. Any other value ends the program with one line. While the part is off,
`--ros-domain`, `--ros-node`, `--ros-params-writable` and `--ros-clock` end the program with one line.

The options work in every `xblick` command and in every node that opens its session with `session.open_from_args`.
`session.open_session` takes part as `XBLICK_ROS` says, and `ros_options=False` turns it off. In C++ the options
come with `Options::add_session_options()` and in the C++ programs `xblick-cpp-camera`, `xblick-cpp-probe` and
`xblick-points` ([C++ and ROS 2](#c-and-ros-2)). A node also shows them as read-only parameters. With `--no-ros` the
program sends nothing for ROS. A test checks this on the wire.

A key becomes a ROS topic by one rule: the key with a leading `/`. `test/camera/color/image_raw` is
`/test/camera/color/image_raw`. ROS allows letters, digits and `_` in each part of a name, and a part must not start
with a digit. Our names follow the same rule. A key that breaks it, such as one of an older recording, stays on our
keys only. It gives one warning line, and the program goes on.

### One node per program

A program is one ROS node. The node carries the program's publishers, its subscribers and the six parameter services
of its parameters. The program prints the node's name and its domain in one line at the start.

The node's name follows the first rule that applies:

1. `--ros-node`, when you give it.
2. The name of the program's parameters, such as `/test/camera/yolox` for the YOLOX node on `test/camera`. The node
   takes this name when the program declares its parameters, and the program prints a second line then.
3. The program's name in the namespace of its `--camera`, such as `/test/camera/xblick_test_camera`. A program
   without `--camera` gets the root namespace, such as `/xblick_list`.

Every character other than a letter, a digit or `_` becomes `_` in these names. A second set of parameters in one
program is a ROS node of its own, of that set's name. A program without parameters serves the six services as
well. They list no parameter and refuse every set, so `ros2 param list` answers at once.

A program warns with one line when another node of the same full name is in the ROS graph, at the start or later.
ROS's tools show both nodes and reach only one of them. Give each program a name of its own with `--ros-node`.

### What ROS 2 receives

The fixed transforms on `/tf_static` are the
exception to the rule that a publisher sends only to a ROS subscriber. A publisher puts them into its cache whether or
not a subscriber is there. Our rings and compressed forms
keep working between our programs.

| Our key | ROS topic | ROS type |
| --- | --- | --- |
| `<stream>/image_raw` | `/<stream>/image_raw` | `sensor_msgs/msg/Image` |
| `<stream>/camera_info` | `/<stream>/camera_info`, one per frame with the image's header | `sensor_msgs/msg/CameraInfo` |
| `<stream>/image_raw/compressed` with JPEG or PNG | `/<stream>/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` |
| `<stream>/cloud` and its compressed and ring forms | `/<stream>/cloud`, always the plain cloud | `sensor_msgs/msg/PointCloud2` |
| `<camera>/<node>/<name>` | `/<camera>/<node>/<name>` | `vision_msgs/msg/Detection2DArray`, `Detection3DArray`, `Classification` |
| `<camera>/diagnostics` | `/<camera>/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` |
| `<camera>/tf`, `<robot>/tf`, `<world>/tf` | `/tf` | `tf2_msgs/msg/TFMessage` |
| `<camera>/tf_static` | `/tf_static`, transient local | `tf2_msgs/msg/TFMessage` |
| a robot's `joint_states`, `odom`, `wrench/<sensor>`, a world's `clock`, a controller's `record` and `status` | `/<key>` | their ROS types |
| the clock key of `--ros-clock` | `/clock` | `rosgraph_msgs/msg/Clock` |
| a node's parameter events | `/parameter_events` | `rcl_interfaces/msg/ParameterEvent` |

Some keys have no ROS type, and ROS does not get them. These are the ring descriptors, the RVL depth form, the
ray-table clouds and JSON keys. Each such key gives one warning line at the start.

A key that tells a robot what to do never goes to ROS, whatever its type. Every key with a part `command` counts as
such a key: a controller's `<robot>/controller/<instance>/command`, and a simulated robot's `<robot>/command/joints`
and `<robot>/command/tf`. This holds for the live publishers and for `xblick play`. Each such key gives one warning
line.

### What a program reads from ROS 2

A reader needs a session in ROS 2 to read a ROS topic, which every session is by default. It reads one in three
cases: you name a ROS topic, you ask for the transport `ros`, or the transport `auto` finds only a ROS publisher.

| Reader | ROS topic |
| --- | --- |
| `ImageSubscriber(session, "/T")`, `xblick hz /T`, `xblick snapshot /T` | an image topic `/T` of any name, such as `/image_raw` or `/resize/image`, with `camera_info=True` also its camera_info. See [Read a ROS camera](#read-a-ros-camera). |
| `PointCloudSubscriber(session, "/T")` | a point cloud topic `/T` of any name |
| `ImageSubscriber(..., transport="ros")`, `xblick hz --transport ros`, `xblick snapshot --transport ros` | `/<stream>/image_raw`, with `camera_info=True` also `/<stream>/camera_info` |
| `ImageSubscriber(..., transport="auto")` and every node and command that reads with `auto` | `/<stream>/image_raw` when only a ROS node publishes the stream. See [Streams that only ROS publishes](#streams-that-only-ros-publishes). |
| `PointCloudSubscriber(..., transport="ros")`, and with `auto` as above | `/<stream>/cloud` |
| C++: `ImageSubscriber` and `PointCloudSubscriber` with the same base, transport and camera_info, `xblick-cpp-probe --base /T` | as in Python, by the rows above. `SubscriberOptions::camera_info_topic` names another camera_info topic. |
| C++: `StaticTransformSubscriber(..., ros = true)`, `xblick-cpp-probe --tf-static --via-ros` | `/tf_static`, also of publishers that started before it |
| `DetectionSubscriber(..., ros_type=Detection3DArray)` | `/<key>` of that type |
| `TransformBuffer(..., ros=True)`, `TransformSubscriber(..., ros=True)`, `StaticTransformSubscriber(..., ros=True)` | `/tf` and `/tf_static` |
| `ParameterClient(..., ros=True)`, `xblick param --via-ros` | the parameter services of a ROS node |

A ROS sample carries no attachment of ours. Its frames therefore have no `seq`, `source` or publish time.

### Read a ROS camera

A name with a leading `/` is a ROS topic, as `ros2 topic list` prints it. A name without one is a stream of ours.

```python
images = db.ImageSubscriber(session, "/resize/image", transport="auto", camera_info=True)
```

```bash
xblick hz /image_raw
xblick snapshot /resize/image frame.png
```

The camera_info follows ROS's convention: the topic `camera_info` in the namespace of the image topic. `/image_raw`
has `/camera_info`, and `/resize/image` has `/resize/camera_info`. Name another topic with
`camera_info="/cam/calibration"`. Compressed frames follow image_transport's convention, the topic
`<image topic>/compressed`, and so do the frames of its `compressedDepth` plugin, `<image topic>/compressedDepth`.

In C++, `ImageSubscriber(session, "/resize/image", {.camera_info = true})` reads the same topics.
`SubscriberOptions::camera_info_topic` names another camera_info topic. The compressed topic's transport is
`Transport::RosCompressed`, the compressedDepth topic's `Transport::RosCompressedDepth`.

| `transport` | Image topic `/T` |
| --- | --- |
| `auto` | `/T`. Else `/T/compressed`, else `/T/compressedDepth`: the first that the publishers offer. |
| `ros` or `raw` | `/T` |
| `compressed` | `/T/compressed` |

The `transport` of a subscriber that reads `/T/compressedDepth` says `ros_compressed_depth`. No transport asks for it
by name: `auto` takes it when it is the only form.

A compressedDepth frame has the format `<encoding>; compressedDepth png` or `<encoding>; compressedDepth rvl`.
`<encoding>; compressedDepth` alone, as older versions of the plugin write it, is PNG. Its data is ROS's 12-byte
header, then the PNG or the RVL bytes. A `16UC1` or `mono16` image is decoded as it is. A `32FC1` image comes as
quantised inverse depth, and the reader gives back the depth in metres, NaN where the plugin had no depth. Another
codec or encoding is an error that names the format.

A reader never picks a topic in place of the one you named. After 5 s without a frame it looks at ROS's graph, and
`latest()` raises an error in two cases:

- The topic's publishers send another type. For `/T/compressed` the error names `/T` and the transport `compressed`,
  for `/T/compressedDepth` it names `/T` and `auto`.
- No topic has the name, but topics of the kind lie below it. `xblick hz /rgbd` then says `no ROS topic of this
  name, and these are below it: /rgbd/color/image_raw, /rgbd/depth/image_raw; name one of them`.

When frames come but no camera_info, the reader logs one line after 5 s with the CameraInfo topics in the graph.
In C++, `latest()` throws these errors, `ProtocolError` and `std::invalid_argument`.

### Streams that only ROS publishes

A subscriber with the transport `auto` waits for a publisher of its stream. When no program of ours answers, a ROS
node that publishes the stream's ROS topic counts as well. The subscriber then reads `/<stream>/image_raw`, else
`/<stream>/image_raw/compressed`, else `/<stream>/image_raw/compressedDepth`, with `camera_info` beside it. A cloud's
topic is `/<stream>/cloud`. Its `transport` then says `ros`, `ros_compressed` or `ros_compressed_depth`. A publisher of
ours has priority: once one answers, the subscriber moves to it.

The ROS copies that our programs publish never count. A ROS publisher belongs to a program of ours when its session
declares a program ([protocol](../concepts/protocol.md)). So a camera of ours is read in its own forms, as before.
Each session's ROS graph learns which sessions declare a program from their declaration tokens, which it follows as
it follows ROS's publisher tokens. A program declares itself before it takes part in ROS, so its declaration arrives
first, also under heavy load.

`xblick hz usb_cam` therefore reads a ROS camera on `/usb_cam/image_raw`. A node with `--camera lab/cam` reads
`/lab/cam/color/image_raw` when only a ROS node publishes it. `RGBDInputs` pairs a ROS camera's `/<camera>/color/image_raw` and
`/<camera>/depth/image_raw` ([pairing](../guides/pairing-streams.md)). `xblick list` shows the ROS topics of the types
Image, CompressedImage, PointCloud2 and CameraInfo after our streams, with `ros` in the transports column and the
publishing nodes. It leaves out the ROS copies of our streams.

### Show and record a ROS camera

The tools take a camera that exists only as ROS topics by the name of its image topic, as a subscriber does:

- an Image topic `/T`, such as `/usb_cam/image_raw` or `/resize/image`;
- `/T` when only `/T/compressed` or only `/T/compressedDepth` is published;
- a PointCloud2 topic, such as `/velodyne_points`, as a point cloud stream.

`xblick rerun /resize/image` shows the topic, and `--points /velodyne_points` a cloud. Without a selection,
`xblick rerun` shows such cameras beside our streams and marks them `(ROS)` in its lines. Each frame is decoded and
drawn at the entity of its topic, with the camera_info beside it as its pinhole. `16UC1` shows as depth in
millimetres, `32FC1` as depth in metres. A ROS camera has no transforms in the viewer, and no bodies, IMU or
diagnostics: ROS keeps its transforms on its global `/tf` and `/tf_static`.

`xblick record` records such cameras under their ROS topic names, each channel as `ros2 bag record` writes it. Of an
image topic `/T` it records the first form its publishers offer of `/T/compressed`, `/T/compressedDepth` and `/T`,
and the camera_info beside it. With `--points` it records the PointCloud2 topics too. `xblick play` plays these
channels as it plays a rosbag2: a camera on image_transport's names, `/<base>/image_raw`, its compressed or its
compressedDepth form with `/<base>/camera_info`, becomes the stream `<base>` of ours. A compressedDepth form is
decoded and compressed again as a live camera's frames are. Other topics play on their keys and their ROS topics.

The terminal interface lists such cameras in its Rerun tab below their topics' namespace, marked `(ROS)`, without
transforms. Its Graph titles the session of a ROS node that publishes them by the node's name, marked `(ROS)`, and
lists its camera topics in the box.

### Read an Isaac ROS node's output

An Isaac ROS node on `rmw_zenoh_cpp` reaches our programs through the same router. Its output topics keep the node's
own names. Read them as ROS topics, with no remap. A `ResizeNode` of `isaac_ros_image_proc` that reads our test
camera publishes `/resize/image` and `/resize/camera_info`:

```bash
ros2 component load /container isaac_ros_image_proc nvidia::isaac_ros::image_proc::ResizeNode -n resize \
  -r image:=/test/camera/color/image_raw -r camera_info:=/test/camera/color/camera_info \
  -p output_width:=320 -p output_height:=240
xblick hz /resize/image
```

```python
resized = db.ImageSubscriber(session, "/resize/image", transport="auto", camera_info=True)
```

The frames arrive as plain CDR, with the input's stamp and frame id and the scaled camera_info. `ros2 topic list`
shows the names of other nodes' outputs.

A Lyrical publisher sends images, compressed images and clouds on keys of its own for each kind of subscriber. Our
readers of these types announce themselves as subscribers that take plain CDR. A Lyrical publisher then sends them the
usual bytes on the topic's key with the suffix `/_buf_cpu`, and our readers read that key as well. A Jazzy publisher
sends on the topic's key, which our readers read as before
([protocol](../concepts/protocol.md#ros-2-on-rmw_zenoh)). Our publishers send on the topic's key as well. One case
differs: while a ROS reader takes the frames of a `CameraPublisher` with `cuda_device` as CUDA buffers, that publisher
sends on the buffer keys only. Our readers then get the bytes on `/_buf_cpu`
([GPU frames for ROS readers](#gpu-frames-for-ros-readers)).

## Several cameras and tf

All transforms go to ROS's global `/tf` and `/tf_static`, as tf2 and rviz expect. tf2 keeps one parent per frame, so
every camera and every robot frame needs a name of its own. `xblick test-camera`, `xblick cam` and `xblick video`
name a stream's frame after the stream by default, with `/` written as `_`. The test camera's stream
`test/camera/color` has the frame `test_camera_color`, and with `--pose` also `test_camera_color_optical`. Give
another name with `--frame-id`. Our keys `<camera>/tf` and `<camera>/tf_static` stay as they are.

A program checks its transforms in two ways and writes one error line per frame:

- It gives one frame two parents itself.
- Another program sends one of its fixed frames on `/tf_static` with another parent or another transform. The
  program reads `/tf_static` for this from its first fixed transform on. A program without fixed transforms reads
  nothing. Both programs write the line.

The transforms still go out, so tf2 shows the clash too. A clash on `/tf` between two programs is not checked.

A camera publishes its fixed transforms once. On ROS's `/tf_static` they stay in the cache of its publisher, so a ROS
subscriber that starts later gets them. Our reader with `ros=True` gets the transforms of ROS publishers that started
before it in the same way.

## The clock

A ROS node with `use_sim_time` reads ROS's global `/clock`. Our worlds publish their clock on `<world>/clock`, which
ROS sees as `/<world>/clock`. Start the program that publishes the clock with `--ros-clock <world>/clock`, such as
`--ros-clock sim/clock`. That key then goes to `/clock` instead of `/<world>/clock`, and ROS nodes need no remap. The
key must be of type `rosgraph_msgs/msg/Clock`, else the program stops with one line. A program that has published no
such key 1 s after its start writes one warning line and goes on, because a simulation may publish its clock later.
The option names one key and chooses nothing by itself. Give it to one program per domain, because ROS has one `/clock` per domain.

## Parameters and services

A node's parameters are served by its ROS node ([One node per program](#one-node-per-program)), with the six
parameter services of ROS 2. `ros2 param
list`, `get` and `describe` work on it. Its ROS services refuse every set with a reason, unless the node was started
with `--ros-params-writable`.

A set is a command. Our own callers send it on our keys with the node and the deadline in the attachment, and the
node checks both ([protocol](../concepts/protocol.md#a-commands-node-and-deadline)). Our callers also count the nodes
of the name first and send nothing when more than one answers. A ROS client does none of this. With
`--ros-params-writable` a ROS client's set is therefore unchecked and uncounted:

- The node executes it without a node check and without a deadline.
- Every node of that name gets it and executes it. Two nodes of one name both change, and the ROS client shows one
  success.

Read-only access (`--params-read-only`), read-only parameters, types, ranges and choices still apply. A ROS set whose
node blocks in its writer holds the parameters' lock, for our own calls too. Each round, a node answers at most four
calls per ROS service, so a ROS client that floods it does not starve our own calls (Python; C++ answers each call
at once, [C++ and ROS 2](#c-and-ros-2)). While sets through ROS are off, a ROS client's describe shows every parameter
read-only, so that a ROS tool offers no field that every set refuses.

`ParameterClient(..., ros=True)` and `xblick param --via-ros --node <ROS node>` read and set the parameters of
any ROS node. A set goes out only after exactly one node answered a list_parameters read. This check is best effort.
The set then goes out once and carries no deadline.

Our other services are offered to ROS only where a ROS service type fits exactly. None does today.

| Service | ROS |
| --- | --- |
| parameters (`rcl_interfaces/srv/*`) | offered |
| the recorder's control (`xblick/<name>/control`, text or JSON) | not offered, no ROS type |
| a controller's `torque_step` (`xblick_msgs/srv/TorqueStep`) | not offered, no ROS type |
| a controller manager's services | served by a robot's bridge, not by this package |

## One router for both

A Zenoh router serves our programs and ROS nodes alike. Both default to port 7447, so run one router per PC: either
`xblick router` or ROS's `rmw_zenohd`. Our programs find `rmw_zenohd` there without options, and ROS nodes find
`xblick router` with their default configuration. Both ways were tested on another port: our tests run against
`rmw_zenohd`, and ROS's `talker` and `listener` talked through `xblick router`.

## Try it with ROS 2's tools

These commands need a shell with ROS 2 Jazzy or Lyrical (such as `source /opt/ros/jazzy/setup.bash`) and
`RMW_IMPLEMENTATION=rmw_zenoh_cpp`. Keep `ROS_DOMAIN_ID` the same in both shells, or pass `--ros-domain`.

On Lyrical, add `--no-daemon` to `ros2 topic echo` and `ros2 topic info -v`. With the daemon, Lyrical's `ros2`
command stops there with `unknown tag 'rclpy.topic_endpoint_info.TopicEndpointInfo'`, also between ROS nodes alone.
`ros2 topic echo` with an explicit QoS option, such as `--qos-durability`, works with the daemon. `ros2 topic hz` has
no `--no-daemon` option and works without it. `ros2 node list`, `ros2 topic list` and `ros2 param` work either way.

1. Start one router: `xblick router`, or `ros2 run rmw_zenoh_cpp rmw_zenohd`.
2. Start a test camera with a fixed pose:

   ```bash
   xblick test-camera --pose 0 0 1.5 0 10 0
   ```

3. Look at it from ROS 2:

   ```bash
   ros2 node list                                   # /test/camera/xblick_test_camera
   ros2 topic list                                  # /test/camera/color/image_raw, .../camera_info, /tf_static, ...
   ros2 topic hz /test/camera/color/image_raw
   ros2 topic echo /test/camera/color/camera_info --once          # Lyrical: add --no-daemon
   ros2 topic echo /tf_static --qos-durability transient_local --once
   ros2 run tf2_ros tf2_echo world test_camera_color_optical
   rviz2                                            # Image display on /test/camera/color/image_raw, TF display
   ```

4. Look at a node's parameters. The record and play node has some, and it needs `xblick[record]`. It records
   every stream until you stop it with Ctrl-C:

   ```bash
   xblick record --dir /tmp/recordings
   ros2 param list /xblick/recorder
   ros2 param get /xblick/recorder rate
   ros2 param set /xblick/recorder rate 2.0         # refused: sets through ROS 2 are off on this node
   ```

   A node of another repository works the same way under its parameters' name, such as `/test/camera/yolox` for
   `xblick-yolox --camera test/camera`. Start it with `--ros-params-writable` to allow ROS's sets.

5. Read a ROS node's parameters from Durchblick:

   ```bash
   ros2 run demo_nodes_cpp parameter_blackboard
   xblick param --via-ros --node parameter_blackboard
   xblick param --via-ros --node parameter_blackboard --set use_sim_time:=true
   ```

6. Read a ROS camera from Durchblick:

   ```bash
   ros2 run image_tools cam2image --ros-args -p burger_mode:=true   # publishes /image
   xblick hz /image
   xblick list                                      # /image  Image  ros  node /cam2image
   ```

The test camera's frames are `world`, `test_camera_color` (`--frame-id`) and `test_camera_color_optical`, the
images' frame. `ros2 topic list` and `ros2 node list` show the names of other nodes.

## C++ and ROS 2

The C++ API takes part in ROS 2 as the Python API does, with the same keys, tokens, QoS and attachment
([C++](../reference/cpp-api.md#ros-2)). A test compares both sides byte for byte. A session in ROS 2 (the default of
`SessionOptions::ros`) mirrors `CameraPublisher`, `PointCloudPublisher`, `TransformPublisher`,
`StaticTransformPublisher`, `RobotStatePublisher`, `DiagnosticsPublisher`, `ImuPublisher` and `MessagePublisher` to
ROS, and `Parameters` serves the six parameter services on the program's node with the same rules as in Python. A publisher that offers only
its rings, such as a relay's, offers nothing to ROS.

Three differences remain:

- `rmw_zenoh` keeps `/tf_static` for late subscribers with zenoh-ext's advanced publisher. zenoh-c offers it only with
  its unstable API, which its standalone release lacks. The C++ publisher answers late subscribers with its own
  cache in the same format, without sequence numbers. A ROS subscriber that starts later may therefore get the
  message twice. tf2 takes it as the same transform.
- C++ answers each ROS call at once in Zenoh's callback thread; it has no rounds and no cap per round. A ROS set whose
  writer blocks therefore also blocks that Zenoh thread, and with it the other traffic that thread delivers. A set
  refused because sets through ROS are off is refused before its parameters are decoded.
- The C++ API offers no `ParameterClient` for a ROS node yet.

## ROS 2 on DDS through a bridge

ROS 2 nodes on a DDS middleware (`rmw_cyclonedds_cpp`, `rmw_fastrtps_cpp`) read Durchblick's topics through
`zenoh-bridge-ros2dds` of Eclipse Zenoh. The bridge runs next to the ROS nodes. It connects to a Durchblick router as
a Zenoh client and writes each Zenoh sample unchanged to DDS. Durchblick's payloads are already ROS 2 messages in CDR,
so the ROS nodes read them as they are. Durchblick's programs need no option for this. Their part in ROS 2 on
`rmw_zenoh` uses other keys, which the bridge does not carry.

You need `zenoh-bridge-ros2dds` of the Zenoh version that Durchblick uses (1.10). The bridge needs no ROS
installation.

What works through the bridge:

- Topics with a ROS 2 message reach ROS nodes on Cyclone DDS and on Fast DDS. Examples are `image_raw`,
  `image_raw/compressed`, `camera_info`, `cloud`, `<camera>/tf` and `<camera>/diagnostics`.
- A ROS subscriber with the durability "transient local" gets `<camera>/tf_static` also when it starts after the
  camera. Durchblick answers the bridge's request for the latched message.
- `ros2 service call` reaches the parameter services of a Durchblick node, when the ROS side uses Cyclone DDS. The
  bridge offers services to Cyclone DDS only.

### Run the bridge

The steps show one PC on which Durchblick and the ROS nodes run, with ROS discovery limited to that PC.

1. Start the router and a camera, here the test camera with a fixed pose:

   ```bash
   xblick router
   xblick test-camera --pose 0 0 1.5 0 10 0
   ```

2. Start the bridge in client mode. Give it the ROS side's domain id and distribution:

   ```bash
   CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface address="127.0.0.1"/></Interfaces></General></Domain></CycloneDDS>' \
   ROS_DISTRO=jazzy zenoh-bridge-ros2dds client -e tcp/127.0.0.1:7447 --no-multicast-scouting \
     -d 0 --ros-automatic-discovery-range LOCALHOST
   ```

   `CYCLONEDDS_URI` binds the bridge's DDS to the loopback interface. With `LOCALHOST`, the ROS nodes talk over
   loopback. The bridge's DDS chooses a network interface of its own, and without this setting the two sides do not
   find each other. Leave the setting out when the ROS nodes run on other PCs.

3. Read a topic on the ROS side. Name the message type:

   ```bash
   export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
   ros2 topic echo --once /test/camera/color/camera_info sensor_msgs/msg/CameraInfo
   ros2 topic echo --once --qos-durability transient_local /test/camera/tf_static tf2_msgs/msg/TFMessage
   ```

   `ros2 topic list` shows a Durchblick topic only while a ROS subscriber of it exists. The bridge creates the DDS
   writer of a topic when a ROS node subscribes, with the type that the subscriber names. For the same reason,
   `ros2 topic echo` and `ros2 topic hz` need the type on the command line.

4. Call a parameter service of a node, for example of `cam/azurekinect`:

   ```bash
   ros2 service call /cam/azurekinect/list_parameters rcl_interfaces/srv/ListParameters "{}"
   ```

### Limits of the bridge

These limits hold for ROS 2 nodes behind `zenoh-bridge-ros2dds`:

- A camera's transforms arrive on `/<camera>/tf` and `/<camera>/tf_static`. ROS's tf2 reads `/tf` and `/tf_static`.
  The bridge's option `-n /<camera>` maps the camera's keys to the ROS root, which serves one camera per bridge.
- `ros2 param` does not find Durchblick's nodes, because the ROS graph has no node of their names. Use
  `ros2 service call` for their parameter services.
- The bridge carries data from Durchblick to ROS only. Durchblick's subscribers do not read what ROS nodes publish
  through the bridge. Such samples carry no Durchblick encoding, and no publisher answers their stream's info query.
- ROS subscribers receive every frame as a message over Zenoh and DDS. Shared memory and the CUDA ring serve
  Durchblick's own subscribers only.
- `ros2 bag record` of named topics records nothing. The recorder waits until the topics appear in the ROS graph.
  The bridge creates a topic only for a reader that names its type, so the topics never appear. Record on the direct
  route through `rmw_zenoh` instead.

## GPU frames for ROS readers

A ROS 2 Lyrical subscriber can accept image data as a CUDA buffer. Such a subscriber, an Isaac ROS node for
example, receives a frame that a publisher of ours has on the GPU as a CUDA buffer. The frame then stays in GPU
memory. Every other ROS reader gets the bytes as before. Our own readers are not affected.

This uses ROS 2's CUDA buffer backend `cuda_buffer_backend` of `ros2/rosidl_buffer_backends`. The backend is
experimental upstream, and its wire format carries no version. Its released package 0.1.2 and its current source
already differ in how the reader fetches a block, and our publisher serves both forms. A later change of the backend
can break the link until Durchblick follows it.

A publisher of ours with `cuda_device` sends a frame as a CUDA buffer while all of these hold for the reader:

1. The reader runs on `rmw_zenoh_cpp` and has the backend's plugin `ros-lyrical-cuda-buffer-backend`. Isaac ROS's
   packages do not install this plugin.
2. The reader accepts the backend, for example with `acceptable_buffer_backends = "any"`.
3. The reader runs on the same PC and as the same Linux user as the publisher.
4. The publisher reaches the reader's `/dev/shm`. It reads the backend's registry there and writes each block's
   record there. By default that is its own `/dev/shm`, which a container shares with `--ipc host`. A container
   with a `/dev/shm` of its own works too: set `XBLICK_ROS_CUDA_SHM` on the publisher's side to that directory as
   the publisher sees it, such as `/proc/<pid>/root/dev/shm` for a process `<pid>` of the container.
5. The reader shares the publisher's process ids, its network namespace and its socket directory (`/tmp` by
   default) at the same path. A container needs `--pid host`, `--network host` and `-v /tmp:/tmp`. The release
   0.1.2 fetches a block through a socket file in that directory. The current source connects to a socket of the
   same name in the network namespace.
6. The reader uses the publisher's GPU. A reader that sees one GPU is compared by that GPU. A reader that sees
   several GPUs needs the publisher's `CUDA_VISIBLE_DEVICES` and `CUDA_DEVICE_ORDER`, because the backend compares
   CUDA ordinals only.
7. The reader keeps the last few messages of its topic (`KEEP_LAST`). A reader that keeps all messages gets the bytes.
8. No reader of the topic is older than Lyrical. A Jazzy reader reads only the topic's key, and then all readers of
   that topic get the bytes there.

A reader that accepts the backend and misses one of these conditions gets the bytes. The publisher prints one line
that names the reason.

The publisher keeps the CUDA ring as it is. While at least one such reader is there, it copies each frame on the GPU
into a block of a small pool of its own and sends the block's descriptor to the reader. The reader maps the block in
place. The publisher allocates nothing before the first such reader appears, and it removes the blocks and their
files when it closes.

The publisher never waits for a reader. It rewrites a block only when no reader holds the block. It also waits until
as many frames as the readers' queues hold have gone out since the block's last frame, so that no queued message
loses its block. A block that a reader holds longer than 2 s counts as free again, because a reader that crashed never
gives it back. When every block is held,
the frame does not go to these readers, and the publisher counts it. For one frame size the pool has at most the
readers' queue depth plus five blocks, and never more than 64. A 1080p `rgb8` block takes 6 MiB of GPU memory.

| Setting | Default | Meaning |
| --- | --- | --- |
| `XBLICK_ROS_CUDA` | `1` | `0` sends the bytes to every ROS reader |
| `XBLICK_ROS_CUDA_SOCKETS` | `/tmp` | the directory of the sockets that hand out the blocks |
| `XBLICK_ROS_CUDA_SHM` | `/dev/shm` | the readers' `/dev/shm` as the publisher sees it |

The Python and the C++ `CameraPublisher` follow these rules alike, and the C++ core holds them for both. In C++,
`CameraPublisher::ros_gpu_stats()` gives the counters. [Frames on the GPU](../concepts/gpu-frames.md#ros-2-readers)
explains the mechanism.

## Wire format

The [protocol](../concepts/protocol.md#ros-2-on-rmw_zenoh) page lists the keys, the attachment, the tokens and the
type hashes of the part in ROS 2.

## Status and limits

- The type hashes of `vision_msgs` are those of `ros-jazzy-vision-msgs` 4.1.1. They equal the hashes of
  `vision_msgs` 4.2.0 in a Lyrical installation.
- A Jazzy subscriber gets no images, compressed images or clouds from a Lyrical publisher. Lyrical sends these
  types on keys that Jazzy does not read. This holds between ROS nodes as well, so a program of ours cannot change
  it. Lyrical subscribers get these types from Jazzy publishers and from ours.
- Each `xblick` command is a ROS node while it runs, such as `/xblick_hz`. Two such commands at once share a name
  and each warns once.
- A subscriber of every key (`**`), such as the traffic meter of the terminal interface, also subscribes to the ROS
  copies. The publishers then send them as well. The terminal interface shows and counts our keys only.
- The C++ tool `xblick-cpp-camera` keeps its fixed frame `TestCamera`. It serves the tests, which compare its bytes.
- A ROS camera that `xblick record` recorded plays as a stream of ours only when its topics follow image_transport's
  names (`/<base>/image_raw` and `/<base>/camera_info`). An image topic of another name, such as `/resize/image`,
  plays on its key and its ROS topic, which the tools then take for the ROS copy of `xblick play`.
- The terminal interface's Graph marks a ROS node's session only when the routers' admin space knows the session.
- A ROS 2 reader gets GPU frames as CUDA buffers only from a `CameraPublisher` with `cuda_device`, only on
  `rmw_zenoh`, and only on the publisher's PC and GPU ([GPU frames for ROS readers](#gpu-frames-for-ros-readers)).
  Point clouds and frames from ROS publishers into our readers go as bytes. [Alternatives](../project/alternatives.md)
  describes the buffer backends.
- The CUDA buffer backend keeps a registry in `/dev/shm`, which the first Linux user's ROS node creates. A ROS node
  of a second user with the backend then fails to start on that PC. This is a limit of the backend.
- [ROS 2 by default](../project/ros-roadmap.md) collects the facts behind the part in ROS 2 and the options that
  would change our own keys.
