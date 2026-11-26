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
| Recordings | MCAP files with ROS 2 schemas, which ROS 2 tools open ([recording](../guides/recording.md)) |

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

A reader reads a ROS topic only when you ask for it. It then needs a session in ROS 2, which every session is by
default.

| Reader | ROS topic |
| --- | --- |
| `ImageSubscriber(..., transport="ros")`, `xblick hz --transport ros`, `xblick snapshot --transport ros` | `/<stream>/image_raw`, with `camera_info=True` also `/<stream>/camera_info` |
| `PointCloudSubscriber(..., transport="ros")` | `/<stream>/cloud` |
| C++: `Transport::Ros` of `ImageSubscriber` (with `camera_info`) and `PointCloudSubscriber`, `xblick-cpp-probe --transport ros` | the same topics |
| C++: `StaticTransformSubscriber(..., ros = true)`, `xblick-cpp-probe --tf-static --via-ros` | `/tf_static`, also of publishers that started before it |
| `DetectionSubscriber(..., ros_type=Detection3DArray)` | `/<key>` of that type |
| `TransformBuffer(..., ros=True)`, `TransformSubscriber(..., ros=True)`, `StaticTransformSubscriber(..., ros=True)` | `/tf` and `/tf_static` |
| `ParameterClient(..., ros=True)`, `xblick param --via-ros` | the parameter services of a ROS node |

A ROS sample carries no attachment of ours. Its frames therefore have no `seq`, `source` or publish time.

A Lyrical publisher sends images, compressed images and clouds on keys of its own for each kind of subscriber. Our
readers of these types announce themselves as subscribers that take plain CDR. A Lyrical publisher then sends them the
usual bytes on the topic's key with the suffix `/_buf_cpu`, and our readers read that key as well. A Jazzy publisher
and our own publishers send on the topic's key, which our readers read as before
([protocol](../concepts/protocol.md#ros-2-on-rmw_zenoh)).

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
key must be of type `rosgraph_msgs/msg/Clock`, else the program stops with one line. The option names one key and
chooses nothing by itself. Give it to one program per domain, because ROS has one `/clock` per domain.

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
`RMW_IMPLEMENTATION=rmw_zenoh_cpp`. Keep `ROS_DOMAIN_ID` the same in both shells, or pass `--ros-domain`. On Lyrical,
add `--no-daemon` to each `ros2 node`, `ros2 topic` and `ros2 param` command. The daemon of Lyrical's `ros2` command
fails on `ros2 topic echo` and `ros2 topic info -v` with `unknown tag 'rclpy.topic_endpoint_info.TopicEndpointInfo'`,
also between ROS nodes alone.

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
   ros2 topic echo /test/camera/color/camera_info --once
   ros2 topic echo /tf_static --qos-durability transient_local --once
   ros2 run tf2_ros tf2_echo world test_camera_color_optical
   rviz2                                            # Image display on /test/camera/color/image_raw, TF display
   ```

4. Look at a node's parameters, such as those of a node started with `xblick-yolox --camera test/camera`. The node
   is installed with its own repository:

   ```bash
   ros2 param list /test/camera/yolox
   ros2 param get /test/camera/yolox threshold
   ros2 param set /test/camera/yolox threshold 0.4
   ```

5. Read a ROS node's parameters from Durchblick:

   ```bash
   ros2 run demo_nodes_cpp parameter_blackboard
   xblick param --via-ros --node parameter_blackboard
   xblick param --via-ros --node parameter_blackboard --set use_sim_time:=true
   ```

The test camera's frames are `world`, `test_camera_color` (`--frame-id`) and `test_camera_color_optical`, the
images' frame. The names in step 4 depend on the node. `ros2 topic list` and `ros2 node list` show the real ones.

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
- To make the streams visible to ROS 2 nodes that read rings in place, ROS 2 Lyrical's buffer backends
  (`rosidl::Buffer`) would be needed. [Alternatives](../project/alternatives.md) describes these projects.
- [ROS 2 by default](../project/ros-roadmap.md) collects the facts behind the part in ROS 2 and the options that
  would change our own keys.
