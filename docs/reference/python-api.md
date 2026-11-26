# Python

The Python API by example. Every example assumes `import xblick as db` and an open session.

Python and C++ share one implementation where it matters. The rings are C++ (`xblick::core`), and Python
reaches them through bindings (`xblick._core`). The stream API exists in both languages with the same topics
and messages. `tests/test_cpp_interop.py` runs the two languages against each other with every transport in both
directions. The [C++ reference](cpp-api.md) shows the same API in C++.

## Open a session

```python
session = db.open_session()                         # the router of this PC
session = db.open_session(connect=["other-pc"])     # the router of another PC
```

A script takes the connection options from its command line:

```python
from xblick import session as db_session
db_session.add_arguments(parser)                    # --connect, --listen, --zenoh-mode, --config, --ros, ...
session = db_session.open_from_args(args)
```

[Sessions and routers](../concepts/sessions.md) explains the options.

A session takes part in ROS 2 by default and then has `session.ros`. Its publishers also send on ROS's keys, and
readers take ROS topics where you ask for them. `open_session(ros_options=False)`, `--no-ros` and `XBLICK_ROS=0` turn
it off. [Durchblick and ROS 2](../integrations/ros2.md) lists both.

```python
session = db_session.open_session(ros_options=db_session.RosOptions(domain=0, node="my_program"))
images = db.ImageSubscriber(session, "ros/camera", transport="ros")    # /ros/camera/image_raw of a ROS node
client = ParameterClient(session, "talker", ros=True)                  # the ROS node /talker's parameters
```

Each session declares its program on the network: its name, version, host, command line, process id and start time
([protocol](../concepts/protocol.md#every-program-declares-itself)). The name is the one the program was started by,
or `program=` of `open_session` (`prog=` of `open_from_args`). A camera publisher and `node.Reporter` add their
camera or node name to it. Other names, and what the network declares:

```python
from xblick import declaration
declaration.add_name(session, "test/camera/mynode")      # one more name of this program
programs, problems = declaration.query(session)          # zid -> Program of every program that declares itself
session = db.open_session(declare=False)                 # a session that declares nothing
```

## Publish a camera stream

```python
info = db.CameraInfo.pinhole(640, 480, fx=554.3, fy=554.3, cx=319.5, cy=239.5, frame_id="AzureKinectCamera")
camera = db.CameraPublisher(session, "cam/azurekinect/color", encoding="rgb8", frame_id="AzureKinectCamera",
                            camera_info=info, source="sim")
if camera.subscribed:
    camera.publish(rgb_array, stamp_ns=capture_time_ns)
```

`publish()` accepts a NumPy array or a device array. `camera.subscribed` is false while nobody reads the stream.
You can set `camera_info` and `frame_id` of a publisher at any time.

`publish_compressed(message, image=None)` publishes a frame that is compressed already, such as a camera's own JPEG
or a recording's `image_raw/compressed`. Compressed subscribers get it unchanged. Raw, shm and cuda subscribers get
the frame decoded once, or `image` when you pass the decoded frame yourself.

To use a real calibration, read a ROS calibration file. This is the YAML format of `camera_calibration`. The file
contains no frame, so you set it. Any other format raises `ValueError`.

```python
info = db.CameraInfo.read_file("azurekinect.yaml")
info.header.frame_id = "AzureKinectCamera"
```

## Publish a device array

```python
camera.publish(tensor, stamp_ns=capture_time_ns, stream=torch.cuda.current_stream())
```

Name the CUDA stream on which you computed the array. Without it, `publish()` waits for all GPU work of the
process. [Frames on the GPU](../concepts/gpu-frames.md) explains the reason. A publisher offers a CUDA ring when
you create it with `cuda_device=...`.

## Subscribe

```python
images = db.ImageSubscriber(session, "test/camera/color", transport="auto")
frame = images.latest(timeout=1.0)
```

`latest()` returns the newest frame, or `None` after the timeout.

With `transport="auto"` the subscriber waits for a publisher of its stream. After 5 s without one it logs a single
line: the camera may not have started yet, or the name may be wrong. When nobody publishes the stream but the name
before its last part answers as a stream, `latest()` raises `camera.StreamGivenAsCamera`. This happens to a node that
was given a stream as its camera, such as `--camera cam/video2/color`. The exception is a `SystemExit` with one line
that names the camera, so the program ends with exit code 1. The subscriber never reads the camera in its place.

| Attribute of a frame | Content |
| --- | --- |
| `image` | the pixels, H × W × channels |
| `encoding` | for example `rgb8` or `16UC1` |
| `header` | the stamp and the frame id |
| `seq`, `source` | the sequence number and the source from the attachment |
| `age_s`, `age_at_send_ns` | the [ages](../concepts/timestamps.md#ages) |

| Argument of `ImageSubscriber` | Default | Meaning |
| --- | --- | --- |
| `transport` | `"compressed"` | `auto`, `compressed`, `raw`, `shm` or `cuda`. Use `auto`. |
| `local` | `"auto"` | with `auto` on the publisher's host: `auto`, `shm` or `cuda` |
| `remote` | `"compressed"` | with `auto` on another host: `compressed` or `raw` |
| `device` | `None` | your GPU: an ordinal, `"cuda:N"` or a torch device. Without it the subscriber works on the CPU. |
| `copy` | `True` | `False` returns views into the ring |
| `camera_info` | `False` | also receive the calibration |
| `max_hz` | `None` | hand out at most this many frames per second |

## Read without a copy

```python
views = db.ImageSubscriber(session, "test/camera/color", transport="shm", copy=False)
frame = views.latest(timeout=1.0)
result = process(frame.image)
if not frame.is_valid():        # the publisher replaced the slot meanwhile
    result = None
```

The frame is a view into the ring. Check `is_valid()` after your work. [Rings](../concepts/rings.md) explains when
this is worth it.

## Keep frames on the GPU

```python
gpu = db.ImageSubscriber(session, "test/camera/color", transport="cuda")
tensor = torch.as_tensor(gpu.latest(timeout=1.0).image, device="cuda")
```

`xblick.tensors` converts frames for you: `tensors.rgb(frame, "cuda")` returns an (H, W, 3) `uint8` tensor from
any colour encoding, and `tensors.depth_metres(frame, "cuda")` returns depth in metres.

## Point clouds

```python
clouds = db.PointCloudSubscriber(session, "test/camera/points")
cloud = clouds.latest(timeout=1.0)
xyz = np.stack([cloud.points["x"], cloud.points["y"], cloud.points["z"]], -1)   # NaN where there is no depth
rgb = db.points.unpack_rgb(cloud.points["rgb"])
```

`cloud.points` is a structured NumPy array of shape (height, width) with the fields `x`, `y`, `z` and `rgb`.
`cloud.header.frame_id` is the depth camera. On the publisher's host the cloud comes from a ring. On another host
it arrives as a whole `PointCloud2`, which is large. See the [point cloud guide](../guides/point-clouds.md).

```python
gpu_clouds = db.PointCloudSubscriber(session, "test/camera/points", local="cuda")
```

With `local="cuda"` a cloud is an (h, w, 16) byte array on the GPU.

## Publish and read results

A node publishes its results as `vision_msgs` messages, one message per processed image, with the stamp of that
image.

```python
boxes = db.DetectionPublisher(session, "test/camera/boxer/boxes_3d", db.Detection3DArray)
box = db.Detection3D(results=[db.ObjectHypothesisWithPose(db.ObjectHypothesis("mug", 0.82))],
                     bbox=db.BoundingBox3D(db.Pose(db.Point(0.1, -0.2, 1.5), db.Quaternion()),
                                           db.Vector3(0.08, 0.12, 0.10)), id="7")     # metres
boxes.publish(db.Detection3DArray(db.Header(db.Time.from_ns(frame.stamp_ns), "AzureKinectCamera"), [box]))

reader = db.DetectionSubscriber(session, "test/camera/boxer/boxes_3d", print)   # calls print(message)
```

Classes without a place use `Classification`, for example the actions in a video clip. The message carries the
stamp of the newest image of the clip.

```python
actions = db.DetectionPublisher(session, "test/camera/vjepa2/actions", db.Classification)
actions.publish(db.Classification(db.Header(db.Time.from_ns(frame.stamp_ns), "AzureKinectCamera"),
                                  [db.ObjectHypothesis("Opening something", 0.62)]))
```

## Command a robot controller

A robot's controller listens on `<robot>/controller/<instance>/command` and reports below
`<robot>/controller/<instance>`. A simulation can take the controller's place with the same keys, so a client needs
no special case. The [topic reference](topics.md#controller) lists the keys and the messages.

**Send commands.** One `moveit_msgs/RobotState` per command, with pose targets of the controlled frames (for example
a TCP) and joint targets (an arm's posture, a hand's closure):

```python
from xblick.controller import CommandPublisher

commands = CommandPublisher(session, "robot", "arms", root_frame="base_link")
commands.send({"left_tcp": (position, quaternion_wxyz)}, {"left_shoulder_1": 0.1})
```

Send commands continuously at a fixed rate. The controller refuses a command whose stamp is not newer than the last
one it took, so an older command never follows a newer one. `send` stamps with the wall clock unless you pass
`stamp_ns`.

**Read the state.** `ControllerSubscriber` calls your functions in Zenoh's thread:

```python
from xblick.controller import ControllerSubscriber

def on_records(records, fields): ...      # every batch of records: count x len(fields)
def on_status(status): ...                # DiagnosticStatus at the controller's status rate (for example 50 Hz)

controller = ControllerSubscriber(session, "robot", "arms", on_records, on_status=on_status)
```

| Callback | Content |
| --- | --- |
| `on_records(records, fields)` | every recorded control cycle. The names are the controller's parameter `record_fields`. |
| `on_status(status)` | active, level, commands accepted and refused with the last reason |
| `on_diagnostics(status)` | about once a second |
| `on_hands(names, positions)` | the measured positions of the relayed hands |

The subscriber reads the fields when the controller answers, and raises `ConnectionError` if it does not. A batch of
another width, after a controller of the same name was created again, raises at `check()`.

**Manage controllers.** `ControllerManager` follows ROS 2's controller manager. The robot's software, or a bridge
to it, serves it.

```python
from xblick.controller_manager import ControllerManager

manager = ControllerManager(session, "robot")
config = manager.config_file("Impedance", "robot")       # the robot's configuration file, as a dict
config["zenoh"]["key"] = "robot/controller/arms"
manager.create("teleop_arms", "Impedance", config)        # inactive
manager.switch(activate=["teleop_arms"])                    # all or nothing; waits until active
...
manager.switch(deactivate=["teleop_arms"])
manager.unload("teleop_arms")
```

A read (`list`, `states`, `config`, `config_file`) raises `TimeoutError` when nobody answers and `RuntimeError` when
several managers answer. A command (`create`, `switch`, `unload`, and `torque_step` of a controller) goes out once
to each queryable of its key. Zenoh's default query target delivers a query to every queryable of its key, as long
as none of them is declared complete. No Durchblick queryable is declared complete. Before the command, a read of the
same manager or controller must have exactly one answer and must end before its timeout, else the command is not
sent.

This check is best effort. It catches two managers that both answer the read, and one that has not finished the read at
its timeout. A second manager that drops the read, that starts between the read and the command, or that serves a broad
key expression still gets the command. A session that was just opened links to the serving sessions one after the other.
For a short time its read can miss a second manager, which may then get the command too, or reach only a busy one and
refuse with "no node answered in time". For `torque_step` the read counts the controller's parameter servers, not its
torque-step servers. The check can also stop a command that worked before 0.5.2: a manager whose read takes longer than
the timeout gets nothing ("not sent"). This fails safe. A second node that gets the command refuses it when it checks
the command's node ([Node and deadline](#node-and-deadline)).

A command raises:

| Exception (base) | Sent? | Meaning |
| --- | --- | --- |
| `xblick.NotReached` (`TimeoutError`) | not sent | no manager or controller became known within the timeout. You may try again. |
| `TimeoutError` "not sent: no node answered ..." | not sent | nobody answered the read before the command, in time or at all |
| `RuntimeError` "not sent: ..." | not sent | the read showed several nodes, or it was still open at its timeout (a second node, or one node holding its query) |
| `RuntimeError` with the manager's reason | sent | the manager refused it: nothing was done |
| `xblick.Expired` (`RuntimeError`) | sent once | the named node refused it, because its deadline had passed when the node came to execute it. That node did not execute it. |
| `xblick.NoAnswer` (`RuntimeError`, no `TimeoutError`) | sent once | no answer within the timeout. It may have been executed. A receiver that does not check the deadline may still execute it. It was not sent again. |
| `xblick.PartlyAnswered` (`NoAnswer`) | sent once | one node answered, and the query was still open at its timeout: another node may still execute it |
| `xblick.SeveralExecuted` (`NoAnswer`) | sent once | several nodes answered: each that answered with success executed it |

**A receiver replies and drops the query at once.** A node that serves a command or a parameter (its queryable's
callback) replies and then ends the callback, without keeping the query object. Do long work outside the callback. Zenoh
ends a query when every queryable that received it has dropped the query object. A node that keeps the query after its
reply therefore keeps the query open. At the query's timeout, the caller's own session ends the query and adds an error
reply with the text `Timeout`. The caller takes that reply for a second node: the read before a command gives "not
sent", and a command gives `PartlyAnswered`. Durchblick's own parameter servers (Python `Parameters`, C++) and the
record and play node follow this rule. Check a receiver outside this repository, such as a robot's bridge, for it before
you command it with 0.5.2.

### Node and deadline

A command names the node that it is meant for and carries a deadline. A receiver that takes part checks the node when
the command arrives and the deadline just before the command takes effect. Durchblick's parameter servers (Python
`Parameters`, C++ `xblick::Parameters`) and the record and play node take part. This repository has no controller
manager and no torque-step server. They belong to the robot's software, and they check only if its code uses
`xblick.session.CommandQuery` (C++ `xblick::CommandQuery`). This program cannot tell whether a command's receiver
checks until that receiver answers.

**The node.** The read before the command names the session that answered it, in its reply's
[attachment](../concepts/protocol.md#a-commands-node-and-deadline). The command then names that session by its Zenoh
id. A receiver that checks and runs in any other session refuses the command with an error reply when it arrives,
before any check of its own. So among receivers that check, only the named node executes the command. This also holds
when the read missed a second node. A Zenoh id is random, unless a Zenoh configuration sets `id`. Two programs given
the same id count as one node, and both execute the command. Two receivers on one key in one session share the id as
well. A receiver serves the read before a command and the command in one session. A command handler in another
session refuses every command, because the read names the other session.

**The deadline.** The deadline is the moment at which the caller stops waiting for the answer: `timeout` after it
sends the command. A receiver that checks refuses the command when its clock has reached the deadline. It answers
with an error reply, and the caller raises `xblick.Expired`. A timeout above 24 h is refused at the caller, and a
deadline more than 24 h ahead at the receiver.

**The clock.** Each side reads its own `CLOCK_BOOTTIME`, the time since its PC started. The clocks of two PCs need
not agree. The receiver puts its clock reading into its answer to the read. The caller takes the offset between the
clocks as that reading minus its own clock when the read ended. The true offset can only be larger. So the deadline
on the receiver's clock is early by at most the read's round trip. It is late only by the difference of the clocks'
rates over the time from the read to the check. NTP changes a clock's rate by at most 0.05 %, which is 5 ms over 10 s.
This also assumes that no clock jumps between the read and the check. `CLOCK_BOOTTIME` is never set, and it goes on
during a suspend.

**Where the receiver checks.** The deadline check comes as late as possible before the effect. `Parameters` checks
after it holds its lock and has called the parameters' readers, just before the first writer. A set of several
parameters is checked once, so its later writers can run after the deadline. The record and play node checks inside
its locks: `record` before it subscribes, `play` before it ends a playback that runs, `stop` before it ends a
playback, and `pause`, `resume` and `step` before the playback is told. A `play` builds the new playback after the
check, which can take longer than the deadline. The deadline bounds when an effect begins. A writer, a switch that
waits for the controllers' states, or a `record` that waits for its publishers can end after the deadline.

**What the caller may claim.**

| Outcome | Executed? |
| --- | --- |
| a success | executed by the node that answered. `last_checked` of `ParameterClient` and `ControllerManager` is true when that node confirmed the check (`xblick.session.checked`). |
| `xblick.Expired` | not executed by the named node |
| `NoAnswer` after a command with a deadline | unknown. It may still be executed unless its receiver checks the deadline, which the caller cannot tell. A receiver that checks does not begin it after the deadline, which has passed when the caller raises `NoAnswer` (up to the clock-rate error above). |
| a success, and another node refused it as not meant for it | executed by the named node only. The caller logs a warning: one key has a second node. |
| no success, and only refusals as not meant for them | `NoAnswer`: the named node did not answer, and the others did not execute it |

A reply to the read shows only that the read's server takes part. For `torque_step` that server is the
controller's parameter server, which need not be the torque-step server. Only the answer to the command confirms the
check. Check the robot's state before you act on a `NoAnswer`, and before you go near the robot.

**Older programs and receivers that do not take part.** When the read's answer carries no such attachment, its
receiver does not take part, whatever its version. The caller then attaches nothing and behaves as before. Such a
receiver executes a command when it gets it, also after the caller got `NoAnswer`. A receiver that ignores the
attachment of a command executes it as before. An older caller sends no attachment, and a receiver that checks
executes its command without a check. `session.command` without `to` sends no attachment either.

A command takes a few ms when the manager answers at once. At the worst, when Zenoh does not end a query by its
timeout, `switch()` with the defaults takes up to 27 s: up to 5 s for the read (the manager's timeout of 2 s), then
up to 7 s until the manager is known and up to 15 s for its answer (the switch's timeout of 2 + 5 s).

The controller side, for a simulation that takes the controller's place, is `CommandSubscriber` and
`ControllerPublisher` in the same module.

## More modules

| Module | Content | Read |
| --- | --- | --- |
| `xblick.node`, `history`, `tensors`, `rgbd` | helpers for nodes | [Writing nodes](../guides/writing-nodes.md), [Pairing streams](../guides/pairing-streams.md) |
| `xblick.overlay` | a node's drawn results (`--overlay`) and drawing helpers | [Writing nodes](../guides/writing-nodes.md#drawn-results---overlay) |
| `xblick.weights` | a node's weight download at its start, into its cache directory | [Writing nodes](../guides/writing-nodes.md#rules-every-node-keeps), [Where files are](files.md) |
| `xblick.paths` | `cache_dir`, `data_dir`, `config_dir` of a node; `expand_path` for paths on a node's command line | [Where files are](files.md) |
| `xblick.view` | `xblick view`: streams as MJPEG over HTTP (`View`, `to_jpeg`) | [Streams in a browser or VLC](../guides/view.md) |
| `xblick.parameters` | parameters of a node; `declare_options` makes a node's command-line options its parameters | [Parameters](parameters.md) |
| `xblick.poses` | `TransformBuffer.lookup`: a transform between a camera's frames at the stamp of an image, from its `tf_static` and `tf` (`TransformError` if there is none); a robot's poses; a fixed camera's transforms (`pose_from`, `fixed_camera_transforms`); rotations and quaternions | [Writing nodes](../guides/writing-nodes.md), [Live cameras](../guides/cam.md#a-fixed-pose-in-the-room) |
| `xblick.rectify` | `rectify_map(info)`: the rectification map of a CameraInfo, as `cv2.initUndistortRectifyMap` gives it; `rectified_info(info)`: the CameraInfo of the rectified image | [C++](cpp-api.md#rectify-an-image) |
| `xblick.robot`, `sim`, `controller` | robot state, simulated worlds, the controller link | [Topics](topics.md) |
| `xblick.recording`, `video` | recording and playback | [Recording](../guides/recording.md) |
