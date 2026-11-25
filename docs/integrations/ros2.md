# Durchblick and ROS 2

Durchblick uses the message types and conventions of ROS 2, and it needs no ROS installation. With the option `--ros`
a program also takes part in ROS 2 on `rmw_zenoh`. This page states what is compatible, what `--ros` adds and how
you try it with ROS 2's own tools.

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
is started without `--ros`.

## Take part in ROS 2

Start a command or a node with `--ros`. The program then sends on our keys as before and also on the keys of
`rmw_zenoh_cpp`. It needs `rmw_zenoh_cpp` 0.2.13 (ROS 2 Jazzy) or 0.10.6 or newer (ROS 2 Lyrical) on the ROS side.
In ROS 2 it is a node of its own.

| Option | Meaning |
| --- | --- |
| `--ros` | take part in ROS 2 as well |
| `--ros-domain ID` | the ROS domain id. The default is `ROS_DOMAIN_ID`, else 0. |
| `--ros-node NAME` | the ROS node name. The default is the program's name with every other character written as `_`, such as `xblick_test_camera`. |
| `--ros-params-writable` | ROS clients may set the program's parameters. Without it they read them, and their sets are refused. A ROS set is unchecked and uncounted: see [Parameters and services](#parameters-and-services). |

The options work in every `xblick` command and in every node that opens its session with `session.open_from_args`.
In C++ they come with `Options::add_session_options()` and in the C++ programs `xblick-cpp-camera`, `xblick-cpp-probe`
and `xblick-points` ([C++ and ROS 2](#c-and-ros-2)).
A node also shows them as read-only parameters. Without `--ros` the program sends nothing for ROS. A test checks this
on the wire.

A key becomes a ROS topic by one rule: the key with a leading `/`. `test/camera/color/image_raw` is
`/test/camera/color/image_raw`. ROS allows letters, digits and `_` in each part of a name, and a part must not start
with a digit. A key that breaks this rule stops the program with one line when it starts with `--ros`.

### What ROS 2 receives

A publisher sends the ROS form only while a ROS subscriber is there. The fixed transforms on `/tf_static` are the
exception: a publisher puts them into its cache whether or not a subscriber is there. Our rings and compressed forms
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
| a node's parameter events | `/parameter_events` | `rcl_interfaces/msg/ParameterEvent` |

Some keys have no ROS type, and ROS does not get them. These are the ring descriptors, the RVL depth form, the
ray-table clouds and JSON keys. Each such key gives one warning line at the start.

A key that tells a robot what to do never goes to ROS, whatever its type. Every key with a part `command` counts as
such a key: a controller's `<robot>/controller/<instance>/command`, and a simulated robot's `<robot>/command/joints`
and `<robot>/command/tf`. This holds for the live publishers and for `xblick play`. Each such key gives one warning
line.

### What a program reads from ROS 2

A reader reads a ROS topic only when you ask for it. It then needs a session with `--ros`.

| Reader | ROS topic |
| --- | --- |
| `ImageSubscriber(..., transport="ros")`, `xblick hz --transport ros`, `xblick snapshot --transport ros` | `/<stream>/image_raw`, with `camera_info=True` also `/<stream>/camera_info` |
| `PointCloudSubscriber(..., transport="ros")` | `/<stream>/cloud` |
| C++: `Transport::Ros` of `ImageSubscriber` (with `camera_info`) and `PointCloudSubscriber`, `xblick-cpp-probe --transport ros` | the same topics |
| C++: `StaticTransformSubscriber(..., ros = true)`, `xblick-cpp-probe --tf-static --ros` | `/tf_static`, also of publishers that started before it |
| `DetectionSubscriber(..., ros_type=Detection3DArray)` | `/<key>` of that type |
| `TransformBuffer(..., ros=True)`, `TransformSubscriber(..., ros=True)`, `StaticTransformSubscriber(..., ros=True)` | `/tf` and `/tf_static` |
| `ParameterClient(..., ros=True)`, `xblick param --ros --via-ros` | the parameter services of a ROS node |

A ROS sample carries no attachment of ours. Its frames therefore have no `seq`, `source` or publish time.

A Lyrical publisher sends images, compressed images and clouds on keys of its own for each kind of subscriber. Our
readers of these types announce themselves as subscribers that take plain CDR. A Lyrical publisher then sends them the
usual bytes on the topic's key with the suffix `/_buf_cpu`, and our readers read that key as well. A Jazzy publisher
and our own publishers send on the topic's key, which our readers read as before
([protocol](../concepts/protocol.md#ros-2-on-rmw_zenoh)).

## Several cameras and tf

With `--ros` all transforms go to ROS's global `/tf` and `/tf_static`, as tf2 and rviz expect. tf2 keeps one parent
per frame. Give every camera and every robot frame names of its own, with `--frame-id` or the camera's name. A program
that gives one frame two parents writes one error line. A clash between two programs shows in tf2 itself. Our keys
`<camera>/tf` and `<camera>/tf_static` stay as they are.

A camera publishes its fixed transforms once. On ROS's `/tf_static` they stay in the cache of its publisher, so a ROS
subscriber that starts later gets them. Our reader with `ros=True` gets the transforms of ROS publishers that started
before it in the same way.

## Parameters and services

A node with `--ros` is also the ROS node `/<namespace>/<name>` with the six parameter services of ROS 2. `ros2 param
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

`ParameterClient(..., ros=True)` and `xblick param --ros --via-ros --node <ROS node>` read and set the parameters of
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
`RMW_IMPLEMENTATION=rmw_zenoh_cpp`. Keep `ROS_DOMAIN_ID` the same in both shells, or pass `--ros-domain`. Steps 2 and
5 and `tf2_echo` were run, the ROS programs started directly. The other `ros2` commands, `rviz2` and step 4 are
untested.

1. Start one router: `xblick router`, or `ros2 run rmw_zenoh_cpp rmw_zenohd`.
2. Start a test camera with a fixed pose:

   ```bash
   xblick test-camera --ros --pose 0 0 1.5 0 10 0
   ```

3. Look at it from ROS 2:

   ```bash
   ros2 node list                                   # /xblick_test_camera
   ros2 topic list                                  # /test/camera/color/image_raw, .../camera_info, /tf_static, ...
   ros2 topic hz /test/camera/color/image_raw
   ros2 topic echo /test/camera/color/camera_info --once
   ros2 topic echo /tf_static --qos-durability transient_local --once
   ros2 run tf2_ros tf2_echo world TestCamera_optical
   rviz2                                            # Image display on /test/camera/color/image_raw, TF display
   ```

4. Look at a node's parameters, for a node started with `--ros`, such as `xblick-yolox --camera test/camera --ros`:

   ```bash
   ros2 param list /test/camera/yolox
   ros2 param get /test/camera/yolox threshold
   ros2 param set /test/camera/yolox threshold 0.4
   ```

5. Read a ROS node's parameters from Durchblick:

   ```bash
   ros2 run demo_nodes_cpp parameter_blackboard
   xblick param --ros --via-ros --node parameter_blackboard
   xblick param --ros --via-ros --node parameter_blackboard --set use_sim_time:=true
   ```

The test camera's frames are `world`, `TestCamera` (`--frame-id`) and `TestCamera_optical`, the images' frame. The
names in step 4 depend on the node. `ros2 topic list` and `ros2 node list` show the real ones.

## C++ and ROS 2

The C++ API takes part in ROS 2 as the Python API does, with the same keys, tokens, QoS and attachment
([C++](../reference/cpp-api.md#ros-2)). A test compares both sides byte for byte. A session opened with
`SessionOptions::ros` mirrors `CameraPublisher`, `PointCloudPublisher`, `TransformPublisher`,
`StaticTransformPublisher`, `RobotStatePublisher`, `DiagnosticsPublisher`, `ImuPublisher` and `MessagePublisher` to
ROS, and `Parameters` serves the six parameter services with the same rules as in Python. A publisher that offers only
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

## Wire format

The [protocol](../concepts/protocol.md#ros-2-on-rmw_zenoh) page lists the keys, the attachment, the tokens and the
type hashes that `--ros` uses.

## Status and limits

- Tested with ROS 2 Jazzy and `rmw_zenoh_cpp` 0.2.13 on one PC: ROS 2's `listener` and `talker`,
  `static_transform_publisher`, `tf2_echo`, `parameter_blackboard` and an `rclcpp` node built for the tests. The
  `ros2` command line, `rviz2` and two PCs are not tested.
- The C++ programs were tested against ROS 2's own programs as well: `rclcpp` reads `xblick-cpp-camera`'s streams,
  `tf2_echo` its `/tf_static`, `rclcpp`'s parameter client its parameters, and `xblick-cpp-probe` reads an `rclcpp`
  node's image.
- The type hashes of `vision_msgs` come from our own implementation of ROS's hash over the message definitions. It
  gives ROS's hashes for all 252 message and service types of a Jazzy installation. They equal the hashes of
  `vision_msgs` 4.2.0 in a Lyrical installation. A Jazzy installation with `vision_msgs` was not at hand.
- A Jazzy subscriber gets no images, compressed images or clouds from a Lyrical publisher. Lyrical sends these
  types on keys that Jazzy does not read. This holds between ROS nodes as well, so a program with `--ros` cannot
  change it. Lyrical subscribers get these types from Jazzy publishers and from ours.
- A program is one ROS node, and each node's parameters are one more. A vision node therefore shows as two ROS
  nodes, such as `/xblick_yolox` and `/test/camera/yolox`.
- `/clock` is not mapped: a world's clock is `/<world>/clock`. A ROS node that uses simulated time needs a remap.
- To make the streams visible to ROS 2 nodes that read rings in place, ROS 2 Lyrical's buffer backends
  (`rosidl::Buffer`) would be needed. [Alternatives](../project/alternatives.md) describes these projects.
