# C++

The C++ API by example. It has the same topics, messages and behaviour as the [Python API](python-api.md).

The rings are implemented once, in C++ (`xblick::core`). The stream API is `xblick::xblick`, built on
zenoh-cpp. The library opens the CUDA driver (`libcuda.so.1`) at run time. It therefore loads on a host without a
GPU, and only the CUDA ring needs one.

## Use it in CMake

```cmake
find_package(xblick 0.5 REQUIRED)        # finds zenohc and zenohcxx 1.10, JPEG and PNG itself
target_link_libraries(my_program PRIVATE xblick::xblick)
```

| Target | Content |
| --- | --- |
| `xblick::xblick` | the stream API |
| `xblick::core` | the rings alone |
| `xblick::points` | the point cloud node as a library. Needs `find_package(xblick 0.5 REQUIRED COMPONENTS points)` and a Durchblick built with `XBLICK_POINTS`. |

## Publish a camera stream

```cpp
#include <xblick/camera.hpp>

zenoh::Session session = xblick::open_session({});          // the router of this PC
xblick::CameraPublisher camera(session, "cam/azurekinect/color",
                                   {.encoding = "rgb8", .frame_id = "AzureKinectCamera", .source = "robot"});
if (camera.subscribed())
    camera.publish(xblick::HostImage{pixels, height, width}, capture_ns);
```

`publish` also accepts a `DeviceImage` for a frame in GPU memory.

Set `SessionOptions::program` to the program's name (`Options::session_options` sets it). `open_session` then prints
one line on stderr, `<program>: ...`, when the options make a peer that connects only to other PCs and listens on
loopback only, as the Python commands do ([sessions](../concepts/sessions.md#which-mode-when)).

`open_session` declares the program on the network, as Python does
([protocol](../concepts/protocol.md#every-program-declares-itself)). The record takes `SessionOptions::program`
(empty: the name the program was started by) and `SessionOptions::version` (`Options::version` sets it; empty: unknown).
`CameraPublisher` and `PointCloudPublisher` add their camera's name. `xblick::add_name(session, name)` adds another.
`SessionOptions::declare = false` opens a session that declares nothing.

## Pass a camera's JPEG through

A camera that delivers JPEG needs no decoding for subscribers that want JPEG.

```cpp
if (camera.needs_pixels())
    camera.publish(decode(jpeg), capture_ns, jpeg);     // somebody wants pixels: decode once
else
    camera.publish_jpeg(jpeg, capture_ns);              // only JPEG subscribers
```

## Subscribe

```cpp
xblick::ImageSubscriber images(session, "test/camera/color", {.transport = xblick::Transport::Auto});
if (auto frame = images.latest(true, std::chrono::milliseconds(100)))
    use(frame->data(), frame->height, frame->width, frame->step, frame->encoding);
```

`SubscriberOptions` has the same fields as the arguments in Python: `transport`, `local`, `cuda_device`, `max_hz`
and `keep_recent`. `transport_reason()` states which transport the subscriber chose and why.
`frame->age_at_send_ns()` returns the age at send.

A partner stream pairs its samples with the frames of a primary stream, as Python's `PartnerHistory` does
([pairing streams](../guides/pairing-streams.md)). `xblick/pairing.hpp` has the rule, in `xblick::core`.

```cpp
xblick::ImageSubscriber depth(session, "test/camera/depth", {.keep_recent = 8});
// max_offset_ns: kPairMaxOffsetNs (20 ms, as Python's default); 0 for equal stamps; std::nullopt for half the
// partner's measured period
if (auto stamp = xblick::partner_of(depth.recent_stamps(), color_frame.stamp_ns(), xblick::kPairMaxOffsetNs))
    if (auto frame = depth.at(*stamp, std::chrono::nanoseconds(0)))
        use(color_frame, *frame);
```

`partner_of` returns the stamp of the nearest kept sample within the offset, once no nearer sample can come. It
returns nothing while a nearer one may still come. Call it again when the next sample arrives. `partner_period_ns`
is the measured period.

## Transforms, IMU and diagnostics

```cpp
// the fixed poses between the camera's frames (p_parent = R p_child + t), on test/camera/tf_static
xblick::StaticTransformPublisher tf(session, "test/camera",
    {{{{now, "DepthCamera"}, "ColorCamera", xblick::Transform::from_matrix(rotation, {0.032, 0.002, 0.004})}}});

xblick::ImuPublisher imu(session, "test/camera");
if (imu.subscribed()) imu.publish(sample);              // an xblick::Imu

xblick::DiagnosticsPublisher diagnostics(session, "test/camera");
diagnostics.publish({{now, ""}, {{xblick::DiagnosticStatus::kOk, "my_camera", "30.0 captures/s", serial, {}}}});
```

A program publishes one diagnostics status per publisher, about once per second.

## Point clouds

A publisher lends you a slot of the CUDA ring. Your kernel writes the cloud directly into the slot, and readers
take it from there.

```cpp
#include <xblick/points.hpp>

xblick::PointCloudPublisher points(session, "test/camera/points",
                                       {.layout = xblick::xyzrgb_layout(), .frame_id = "AzureKinectDepthCamera",
                                        .cuda_device = 0});
if (points.subscribed()) {
    const std::uint64_t slot = points.loan(height, width);
    my_kernel<<<grid, block, 0, stream>>>(reinterpret_cast<float4*>(slot), ...);
    points.commit(reinterpret_cast<std::uint64_t>(stream), false, capture_ns);
}

xblick::PointCloudSubscriber clouds(session, "test/camera/points", {.local = xblick::Transport::Cuda});
if (auto cloud = clouds.latest(true, std::chrono::milliseconds(100)))
    use(cloud->device_ptr, cloud->height, cloud->width, cloud->point_step);     // data() for a host cloud
```

`commit` takes the CUDA stream of your kernel, so the publisher waits for exactly that work.

## Rectify an image

`xblick/rectify.hpp` computes the map of OpenCV's `initUndistortRectifyMap` from a CameraInfo, with its R and its
P: the image without lens distortion, turned into the camera of P. For the two cameras of a stereo pair calibrated
with `cv::stereoRectify`, the rectified images share their rows.

```cpp
#include <xblick/rectify.hpp>

const xblick::RectifyMap map = xblick::rectify_map(xblick::rectify_camera(info));              // once per calibration
const xblick::PixelLayout layout = xblick::rectify_layout(frame.encoding);                       // rgb8, mono16, ...
std::vector<std::uint8_t> out(std::size_t(frame.height) * frame.width * layout.pixel_bytes());
xblick::remap(map, layout, frame.data(), frame.step, out.data());
publisher.publish(xblick::HostImage{out.data(), frame.height, frame.width}, frame.stamp_ns());
```

`remap` interpolates 8-bit images bilinearly and takes the nearest pixel of 16-bit images, so that a depth is never
blended. A pixel whose point lies outside the camera's image is 0. `map.x` and `map.y` are what `nppiRemap` and
`cv::remap` take, for a remap on the GPU. `rectified_info(info)` is the CameraInfo of the rectified image: K is P's
left 3 x 3, the distortion is zero, R is the identity and P is unchanged. The right camera of a pair keeps P[3] = -f b.

`rectify_map` and `remap` are in `xblick::core` and need no messages. `rectify_camera` and `rectified_info` take a
CameraInfo and are in `xblick::xblick`. Python has the same as `xblick.rectify`.

## Queries and commands

`camera.hpp` has the two kinds of query that a program sends to other nodes. `xblick::ask` sends a read, such as an info
record or a node's parameter values. It asks again until a first reply comes or the timeout passed. `xblick::ask_once`
sends a command, such as a parameter set. It waits until the session knows a queryable for the key and then sends the
command once to each queryable of the key. It never sends the command again. Zenoh's default query target delivers a
query to every queryable of its key, as long as none of them is declared complete. No Durchblick queryable is declared
complete. `xblick::ParameterClient` sends a set only after a read had exactly one answer and ended before its timeout.
This check is best effort. A second node that drops the read or starts after it still gets the set. So does a second
node that a session just opened has not linked to yet. The set then names the node that answered the read and carries
a deadline. A node on `xblick::Parameters` of 0.5.13 or later refuses a set meant for another node, or one past its
deadline ([Node and deadline](python-api.md#node-and-deadline)).

| Exception (base) | Sent? | Meaning |
| --- | --- | --- |
| `xblick::NotReached` (`std::runtime_error`) | not sent | no queryable became known within the timeout. You may try again. |
| `xblick::ParameterError` "not sent: ..." (`std::runtime_error`) | not sent | the read before the set showed no node or several nodes, or it was still open at its timeout |
| `xblick::ParameterError` with the node's reason | sent | the node refused it, or a read got no answer |
| `xblick::Expired` (`std::runtime_error`) | sent once | the named node refused it, because its deadline had passed. That node did not execute it. |
| `xblick::NoAnswer` (`std::runtime_error`) | sent once | no answer within the timeout. It may have been executed. A receiver that does not check the deadline may still execute it. It was not sent again. |
| `xblick::PartlyAnswered` (`NoAnswer`) | sent once | one node answered, and the query was still open at its timeout: another node may still execute it |
| `xblick::SeveralExecuted` (`NoAnswer`) | sent once | several nodes answered: each that answered with success executed it |

All of them derive from `std::runtime_error`. `NotReached`, `Expired` and `NoAnswer` do not derive from
`ParameterError`: a caller that catches only `ParameterError` must catch them as well. `ParameterClient::last_checked()`
is true when the node of the last set confirmed that it checked node and deadline.

A receiver of commands outside this library, such as a robot's controller manager, checks node and deadline with
`xblick::CommandQuery`. It creates one from the query and its session's Zenoh id, inside the code that answers every
exception with an error reply. It calls `check_node()` when the command arrives, before anything else, and
`check_deadline()` just before the command takes effect. A refusal is a `CommandRefused`, which `xblick::refuse(query, refused)` sends back. Every reply,
also the one to the read before the command, carries `xblick::answer_attachment(zid, checked)`. A sender of its own
uses `xblick::target_of` on the read's answer and the overload of `ask_once` with a `CommandTarget`. Python has the
same functions in `xblick.session`.

## ROS 2

A session takes part in ROS 2 by default ([Durchblick and ROS 2](../integrations/ros2.md)). `SessionOptions::ros`
starts as `ros::from_environment()`: a part, unless `XBLICK_ROS` is `0`. `std::nullopt` turns it off.
`Options::add_session_options()` adds `--no-ros`, `--ros` (does nothing), `--ros-domain`, `--ros-node`,
`--ros-params-writable` and `--ros-clock` with the rules of Python, and `session_options()` fills in `ros`:

```cpp
xblick::Options options("xblick-mycam", "my camera");
options.add_session_options();  // also --no-ros, --ros-domain, --ros-node, --ros-params-writable, --ros-clock
options.parse(argc, argv);
zenoh::Session session = xblick::open_session(options.session_options());
// or without Options: .ros = xblick::ros::Options{.domain = 0, .node = "my_camera"}, or .ros = std::nullopt
```

The session is then a ROS node, and its publishers and parameters offer themselves to ROS while a ROS subscriber or
client is there. Its first `Parameters` serve their services on that node, and a default node name becomes theirs.
`ImageSubscriber` and `PointCloudSubscriber` read a stream's ROS topic with `Transport::Ros`, and a ROS topic of any
name with a base that starts with `/`. `Transport::Auto` takes a stream's ROS topic when only a ROS node of another
program publishes it ([Read a ROS camera](../integrations/ros2.md#read-a-ros-camera)).
`SubscriberOptions::camera_info_topic` names a ROS camera_info topic, `Transport::RosCompressed` is
image_transport's compressed topic and `Transport::RosCompressedDepth` its compressedDepth topic, whose frames
`decompress()` reads (`compressed_depth()` parses the format). A `CameraPublisher` with `cuda_device` gives ROS readers that accept CUDA
buffers its GPU frames as such (`ros::GpuLink`), and `CameraPublisher::ros_gpu_stats()` gives its counters
([GPU frames for ROS readers](../integrations/ros2.md#gpu-frames-for-ros-readers)). An `ImageSubscriber` with
`cuda_device` takes the CUDA buffers of ROS publishers on its GPU as frames in device memory (`Frame::device_ptr`), and
`ImageSubscriber::ros_gpu_stats()` gives its counters
([GPU frames from ROS publishers](../integrations/ros2.md#gpu-frames-from-ros-publishers)). `ros::Subscriber` with a
`cuda_device` and `ros_cuda::Receiver` are the parts below it.
`xblick::ros::of(session)` is the session's part in ROS 2, null for a session without one. `xblick/ros.hpp` also has
the keys, names, tokens, QoS strings and the attachment of `rmw_zenoh`, and `ros::Publisher`, `ros::Subscriber` and
`ros::Service` for topics and services of your own.

## Messages

`xblick/msg.hpp` contains the message types with the names of ROS. Tests compare their bytes with the ROS
definitions (vision_msgs 4.1.1, in `tests/data/vision_msgs`).

## Headers

| Header | Content |
| --- | --- |
| `camera.hpp` | `CameraPublisher`, `ImageSubscriber`, `StaticTransformPublisher`, `StaticTransformSubscriber`, `TransformPublisher`, `ImuPublisher`, `DiagnosticsPublisher`, `RobotStatePublisher` |
| `points.hpp`, `point_cloud_node.hpp` | point cloud streams, the point cloud node |
| `msg.hpp`, `image.hpp` | messages, encodings and codecs |
| `parameters.hpp` | parameters of a node ([parameters](parameters.md)) |
| `options.hpp` | `Options`: a program's command-line options, parsed and offered as its parameters ([a C++ program's options](parameters.md#a-c-programs-options-as-parameters)) |
| `pairing.hpp` | `partner_of`, `partner_period_ns`: a partner stream's sample of a primary frame, by stamp |
| `paths.hpp` | `cache_dir`, `data_dir`, `config_dir`, `check_node_name`: where a node keeps its files ([Where files are](files.md)), in `xblick::core` |
| `protocol.hpp` | topic names, message types and versions |
| `ros.hpp`, `ros_gid.hpp` | ROS 2 on `rmw_zenoh`: keys, tokens, attachment, the session's part ([ROS 2](#ros-2)); the gid in `xblick::core` |
| `ring.hpp`, `cuda_ring.hpp` | the rings |
