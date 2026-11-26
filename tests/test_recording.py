import dataclasses
import json
import multiprocessing as mp
import threading
import time
from pathlib import Path

import numpy as np
import pytest
from test_camera import pattern, sessions, stop_child, wait_until  # noqa: F401 - sessions is a fixture
from test_points import free_port

from xblick import _core, recording
from xblick.camera import (CameraPublisher, ImageSubscriber, StaticTransformPublisher, StaticTransformSubscriber,
                           cdr_encoding, find_streams)
from xblick.codecs import PNG
from xblick.gpu import CudaImage
from xblick.session import open_session, replies
from xblick.detections import DetectionPublisher, DetectionSubscriber
from xblick.msg import (BoundingBox3D, CameraInfo, CompressedImage, Detection3D, Detection3DArray, Header,
                        ObjectHypothesis, ObjectHypothesisWithPose, Point, Pose, Quaternion, TFMessage, Time, Transform,
                        TransformStamped, Vector3)
from xblick.msgdefs import MSGDEFS
from xblick.parameters import ParameterClient

mcap_reader = pytest.importorskip("mcap.reader")  # the extra xblick[record]

CAMERA = "robot/camera"


def transform(parent, child, x):
    return TransformStamped(Header(Time.from_ns(1), parent), child,
                            Transform(Vector3(x, 0.0, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0)))


def read(path):
    with open(path, "rb") as stream:
        reader = mcap_reader.make_reader(stream)
        summary = reader.get_summary()
        messages = [(channel.metadata["zenoh_key"], message) for _, channel, message in reader.iter_messages()]
    return summary, messages


class Scene:
    """A camera as a robot has it: colour, 16-bit depth, a float stream without compressed form, fixed transforms,
    its pose, a node's detections and bodies."""

    def __init__(self, session):
        info = CameraInfo.pinhole(64, 48, 50.0, 50.0, 31.5, 23.5, frame_id="cam")
        self.color = CameraPublisher(session, f"{CAMERA}/color", encoding="rgb8", frame_id="cam", camera_info=info)
        self.depth = CameraPublisher(session, f"{CAMERA}/depth", encoding="16UC1", frame_id="cam", camera_info=info,
                                     codec=PNG)
        self.disparity = CameraPublisher(session, f"{CAMERA}/ffs/disparity", encoding="32FC1", frame_id="cam",
                                         codec=None)
        self.static = StaticTransformPublisher(session, CAMERA, [transform("cam", "depth", 0.03)])
        self.boxes = DetectionPublisher(session, f"{CAMERA}/nemo/detections_3d", Detection3DArray)
        self.session = session

    def publish(self, k: int):
        stamp = 1_000_000_000 * (k + 1)
        self.color.publish(pattern(k), stamp_ns=stamp)
        self.depth.publish(np.full((48, 64), 1000 + k, np.uint16), stamp_ns=stamp)
        self.disparity.publish(np.full((48, 64), float(k), np.float32), stamp_ns=stamp)
        box = Detection3D(Header(Time.from_ns(stamp), "cam"), [ObjectHypothesisWithPose(ObjectHypothesis("mug", 0.9))],
                          BoundingBox3D(Pose(Point(0.0, 0.0, 1.0 + k), Quaternion(0.0, 0.0, 0.0, 1.0)),
                                        Vector3(0.1, 0.1, 0.1)), str(k))
        self.boxes.publish(Detection3DArray(Header(Time.from_ns(stamp), "cam"), [box]))
        self.session.put(f"{CAMERA}/tf", bytes(TFMessage([transform("world", "cam", float(k))]).serialize()),
                         encoding=cdr_encoding(TFMessage))
        self.session.put(f"{CAMERA}/gem_x/bodies", json.dumps({"v": 1, "stamp_ns": stamp, "bodies": []}),
                         encoding="application/json")

    def close(self):
        for c in (self.color, self.depth, self.disparity, self.static, self.boxes):
            c.close()


def record(recorder, scene, frames=5):
    recorder.record()
    # the streams found and subscribed (the pattern subscribers are there at once)
    wait_until(lambda: recorder.status()["topics"] >= 12 + 5, timeout=10.0)
    wait_until(lambda: scene.color.subscribed and scene.depth.subscribed)
    time.sleep(0.3)
    for k in range(frames):
        scene.publish(k)
        time.sleep(0.05)
    time.sleep(0.5)
    recorder.stop()


def test_record_and_play(sessions, tmp_path):  # noqa: F811
    a, b = sessions
    scene = Scene(a)
    recorder = recording.Recorder(b, tmp_path, name="test_rec", split=0, look_every=0.5)
    record(recorder, scene)
    (path,) = recorder.files
    assert recorder.status()["state"] == "idle" and recorder.status()["dropped"] == 0
    summary, messages = read(path)
    keys = [k for k, _ in messages]
    for key in (f"{CAMERA}/color/image_raw/compressed", f"{CAMERA}/color/camera_info",
                f"{CAMERA}/depth/image_raw/compressed", f"{CAMERA}/nemo/detections_3d", f"{CAMERA}/tf",
                f"{CAMERA}/gem_x/bodies"):
        assert keys.count(key) == 5, (key, keys.count(key))
    assert keys.count(f"{CAMERA}/tf_static") == 1  # asked for when the file was opened
    assert not [k for k in keys if "disparity" in k or k.endswith(("/shm", "/cuda", "/image_raw"))]
    # schemas: the ROS definitions; channels: the Zenoh key, encoding, info record
    schemas = {s.name: s for s in summary.schemas.values()}
    assert schemas["sensor_msgs/msg/CompressedImage"].encoding == "ros2msg"
    assert schemas["sensor_msgs/msg/CompressedImage"].data == MSGDEFS["sensor_msgs/msg/CompressedImage"].encode()
    channels = {c.topic: c for c in summary.channels.values()}
    color = channels[f"/{CAMERA}/color/image_raw/compressed"]
    assert color.message_encoding == "cdr" and json.loads(color.metadata["xblick_info"])["encoding"] == "rgb8"
    assert channels[f"/{CAMERA}/gem_x/bodies"].message_encoding == "json"
    assert channels[f"/{CAMERA}/gem_x/bodies"].schema_id == 0
    # the camera's tf_static also on ROS's /tf_static, transient local for ros2 bag play; our playback leaves it out
    assert keys.count("tf_static") == 1
    copy = channels["/tf_static"]
    assert copy.metadata[recording.ROS_COPY] == "/tf_static" and "transient_local" in copy.metadata[
        "offered_qos_profiles"]
    copied = [m for k, m in messages if k == "tf_static"][0]
    assert TFMessage.deserialize(copied.data).transforms[0].child_frame_id == "depth"
    messages = [(k, m) for k, m in messages if k != "tf_static"]

    # played back below a prefix: found and read as the camera was (on this host from shared memory, decoded once)
    player = recording.Player(a, [path], prefix="replay", rate=1.0)
    base = f"replay/{CAMERA}/color"
    images = ImageSubscriber(b, base, transport="auto", camera_info=True)
    raw = ImageSubscriber(b, f"replay/{CAMERA}/depth", transport="raw")
    compressed = ImageSubscriber(b, f"replay/{CAMERA}/depth", transport="compressed")
    got_boxes = []
    boxes = DetectionSubscriber(b, f"replay/{CAMERA}/nemo/detections_3d", got_boxes.append)
    time.sleep(0.5)
    thread = threading.Thread(target=player.run)
    thread.start()
    frame = images.latest(timeout=5.0)
    assert frame is not None and frame.header.stamp.to_ns() in [1_000_000_000 * (k + 1) for k in range(5)]
    assert images.describe().startswith("shm") and frame.image.shape == (48, 64, 3)
    wait_until(lambda: images.camera_info is not None)
    assert images.camera_info.k[0] == 50.0  # the recorded camera_info goes with the frames
    depth = raw.latest(timeout=5.0)
    assert depth.encoding == "16UC1" and depth.image[0, 0] in range(1000, 1005)  # PNG: lossless
    passed = compressed.latest(timeout=5.0)
    assert passed.header.frame_id == "cam" and passed.image[0, 0] in range(1000, 1005)
    thread.join()
    found = dict(find_streams(b, "replay", timeout=1.0))
    assert found[base]["transports"] == ["raw", "compressed", "shm"] and found[base]["source"] == ""
    assert found[f"replay/{CAMERA}/nemo/detections_3d"]["kind"] == "detections"
    assert got_boxes and got_boxes[-1].detections[0].bbox.center.position.z == 5.0
    statics = []
    static = StaticTransformSubscriber(b, f"replay/{CAMERA}", statics.append)
    wait_until(lambda: statics)
    assert statics[0].transforms[0].child_frame_id == "depth"
    # zenoh-ext's form (zenoh-bridge-ros2dds for a late transient-local ROS subscriber): once, on the played key
    bridge = [r.ok for r in replies(b, f"replay/{CAMERA}/tf_static/@adv/**?_max=100;_anyke", 1.0) if r.ok is not None]
    assert [str(r.key_expr) for r in bridge] == [f"replay/{CAMERA}/tf_static"]
    assert TFMessage.deserialize(bridge[0].payload.to_bytes()).transforms[0].child_frame_id == "depth"
    assert player.sent == len(messages)
    for c in (static, boxes, raw, compressed, images, player, recorder, scene):
        c.close()


def test_looped_stamps_go_on(sessions, tmp_path):  # noqa: F811
    """Looped, each pass moves the stamps on by the recording's length and a period: frames, transforms and JSON
    samples newer than the pass before; tf_static as it was."""
    a, b = sessions
    scene = Scene(a)
    recorder = recording.Recorder(b, tmp_path, name="loop_rec", split=0, look_every=0.5)
    record(recorder, scene, frames=3)
    (path,) = recorder.files
    player = recording.Player(a, [path], prefix="loop", rate=4.0, loop=True)
    got = {"frames": [], "tf": [], "bodies": []}
    subs = [b.declare_subscriber(f"loop/{CAMERA}/color/image_raw/compressed", lambda s: got["frames"].append(
                CompressedImage.deserialize(s.payload.to_bytes()).header.stamp.to_ns())),
            b.declare_subscriber(f"loop/{CAMERA}/tf", lambda s: got["tf"].append(
                TFMessage.deserialize(s.payload.to_bytes()).transforms[0].header.stamp.to_ns())),
            b.declare_subscriber(f"loop/{CAMERA}/gem_x/bodies", lambda s: got["bodies"].append(
                json.loads(s.payload.to_bytes())["stamp_ns"]))]
    time.sleep(0.5)
    thread = threading.Thread(target=player.run)
    thread.start()
    wait_until(lambda: len(got["frames"]) >= 9 and len(got["tf"]) >= 9 and len(got["bodies"]) >= 9, timeout=10.0)
    player.end()
    thread.join()
    assert player.error is None and player.passes >= 2
    length = player._pass_ns
    first = [1_000_000_000 * (k + 1) for k in range(3)]
    assert got["frames"][:9] == first + [t + length for t in first] + [t + 2 * length for t in first]
    assert got["bodies"][:9] == got["frames"][:9]
    assert got["tf"][:9] == [1, 1, 1, 1 + length, 1 + length, 1 + length, 1 + 2 * length, 1 + 2 * length,
                             1 + 2 * length]
    statics = []
    static = StaticTransformSubscriber(b, f"loop/{CAMERA}", statics.append)
    wait_until(lambda: statics)
    assert statics[0].transforms[0].header.stamp.to_ns() == 1
    for sub in subs:
        sub.undeclare()
    for c in (static, player, recorder, scene):
        c.close()


def test_a_channel_without_known_stamps_is_not_looped(sessions, tmp_path):  # noqa: F811
    """A CDR channel of a type it does not know: refused with loop (its stamps would go back), played without."""
    mcap = recording.import_mcap()
    path = tmp_path / "foreign.mcap"
    with open(path, "wb") as stream:
        writer = mcap.writer.Writer(stream)
        writer.start()
        schema = writer.register_schema("other_msgs/msg/Thing", "ros2msg", b"int32 x")
        channel = writer.register_channel("/thing", "cdr", schema, {"zenoh_key": "thing",
                                                                     "zenoh_encoding": "application/cdr"})
        for k in range(2):
            writer.add_message(channel, log_time=k * 1000, data=b"\0\1\0\0\0\0\0\0", publish_time=k * 1000)
        writer.finish()
    a, _ = sessions
    with pytest.raises(ValueError, match="cannot be looped"):
        recording.Player(a, [path], loop=True)
    recording.Player(a, [path]).close()


def test_compressed_frames_pass_as_recorded(sessions):  # noqa: F811
    """publish_compressed: the bytes of a recorded JPEG go out unchanged; shm readers get it decoded."""
    a, b = sessions
    source = CameraPublisher(a, f"{CAMERA}/src", encoding="rgb8", frame_id="cam")
    tap = ImageSubscriber(b, f"{CAMERA}/src", transport="compressed")
    wait_until(lambda: source.subscribed)
    source.publish(pattern(3), stamp_ns=7)
    got = []
    sub = b.declare_subscriber(f"{CAMERA}/src/image_raw/compressed", lambda smp: got.append(smp.payload.to_bytes()))
    source.publish(pattern(3), stamp_ns=7)
    wait_until(lambda: got)
    message = CompressedImage.deserialize(got[0])
    again = CameraPublisher(a, f"{CAMERA}/again", encoding="rgb8", frame_id="other")
    out = []
    sub2 = b.declare_subscriber(f"{CAMERA}/again/image_raw/compressed", lambda smp: out.append(smp.payload.to_bytes()))
    shm = ImageSubscriber(b, f"{CAMERA}/again", transport="shm")
    wait_until(lambda: again.subscribed)
    time.sleep(0.3)
    again.publish_compressed(message)
    wait_until(lambda: out)
    assert out[0] == got[0]  # as recorded, its header (frame cam, stamp 7) too
    frame = shm.latest(timeout=5.0)
    assert frame.header.stamp.to_ns() == 7 and frame.header.frame_id == "cam" and frame.image.shape == (48, 64, 3)
    with pytest.raises(ValueError, match="without codec"):
        CameraPublisher(a, f"{CAMERA}/raw_only", encoding="rgb8", frame_id="cam", codec=None).publish_compressed(message)
    for c in (sub, sub2):
        c.undeclare()
    for c in (shm, again, tap, source):
        c.close()


def test_one_node_records_and_plays_over_zenoh(sessions, tmp_path):  # noqa: F811
    """The node records, then plays its file by name (relative to its directory), pauses, resumes and stops it;
    recording is refused while it plays; a played stream reads from shared memory as a camera's."""
    a, b = sessions
    scene = Scene(a)
    node = recording.Recorder(b, tmp_path, name="both", split=0, look_every=0.5)
    record(node, scene, frames=20)
    scene.close()
    (path,) = node.files
    images = ImageSubscriber(a, f"again/{CAMERA}/color", transport="shm")
    status = recording.send_command(a, "both", "play", files=[path.name], prefix="again", rate=0.5, loop=True)
    assert status["state"] == "playing" and status["play"]["files"] == [str(path)]
    assert images.latest(timeout=5.0) is not None
    with pytest.raises(RuntimeError, match="refused.*it plays"):
        recording.send_command(a, "both", "record")
    paused = recording.send_command(a, "both", "pause")
    assert paused["state"] == "paused"
    time.sleep(0.6)
    assert recording.query_status(a, "both")["play"]["position_s"] == paused["play"]["position_s"]
    images.latest(timeout=0.0)
    assert images.latest(timeout=0.5) is None  # nothing while paused
    assert recording.send_command(a, "both", "resume")["state"] == "playing"
    assert images.latest(timeout=5.0) is not None
    assert recording.send_command(a, "both", "stop")["state"] == "idle"
    with pytest.raises(RuntimeError, match="nothing is playing"):
        recording.send_command(a, "both", "pause")
    with pytest.raises(RuntimeError, match="no such recording"):
        recording.send_command(a, "both", "play", files=["missing.mcap"])
    with pytest.raises(ValueError, match="files required"):
        recording.parse_command(b'{"command": "play"}')
    # played to its end by itself: idle again
    node.play([path], rate=20.0)
    assert node.wait_played(10.0) and node.state == "idle"
    images.close()
    node.close()


def test_control_over_zenoh(sessions, tmp_path):  # noqa: F811
    """A paused recorder started and stopped on its control topic; each start a file of its own."""
    a, b = sessions
    scene = Scene(a)
    recorder = recording.Recorder(b, tmp_path, name="ctl", split=0, look_every=0.5)
    assert recording.query_status(a, "ctl")["state"] == "idle"
    for _ in range(2):
        status = recording.send_command(a, "ctl", "record")
        assert status["state"] == "recording" and status["file"].endswith(".mcap")
        wait_until(lambda: recorder.status()["topics"] >= 12 + 5, timeout=10.0)
        time.sleep(0.3)
        scene.publish(0)
        time.sleep(0.5)
        status = recording.send_command(a, "ctl", "stop")
        assert status["state"] == "idle"
    assert len(recorder.files) == 2 and recorder.files[0] != recorder.files[1]
    for path in recorder.files:
        assert any(k.endswith("/color/image_raw/compressed") for k, _ in read(path)[1])
    with pytest.raises(ValueError, match="one of record"):
        recording.parse_command(b"start")
    assert recording.parse_command(b'{"command": "stop"}') == ("stop", {})
    recorder.close()
    scene.close()


def test_split(sessions, tmp_path):  # noqa: F811
    a, b = sessions
    scene = Scene(a)
    recorder = recording.Recorder(b, tmp_path, name="split", split=0.6, look_every=0.5)
    record(recorder, scene, frames=20)
    assert len(recorder.files) >= 2
    for path in recorder.files:
        summary, _ = read(path)  # every file finished (summary written)
        assert summary is not None
    recorder.close()
    scene.close()


def _play_on_gpu(port, path, ready, stop):
    """Child (a CUDA ring cannot be opened by the process that made it): the node playing ``path`` with a CUDA ring,
    until ``stop``."""
    session = open_session(listen=[f"127.0.0.1:{port}"])
    node = recording.Recorder(session, Path(path).parent, name="gpu")
    node.play([path], prefix="gpu", rate=0.5, loop=True, cuda_device=0)
    ready.set()
    stop.wait(60)
    node.close()
    session.close()


@pytest.mark.skipif(_core.cuda_device_count() == 0, reason="no CUDA device")
def test_played_streams_in_a_cuda_ring(sessions, tmp_path):  # noqa: F811
    """With cuda_device, a played stream offers a CUDA ring: a node asking for it reads the frames on the GPU."""
    a, b = sessions
    scene = Scene(a)
    node = recording.Recorder(b, tmp_path, name="rec", split=0, look_every=0.5)
    record(node, scene, frames=20)
    scene.close()
    node.close()
    ctx = mp.get_context("spawn")
    port = free_port()
    ready, stop = ctx.Event(), ctx.Event()
    child = ctx.Process(target=_play_on_gpu, args=(port, str(node.files[0]), ready, stop), daemon=True)
    child.start()
    try:
        assert ready.wait(60)
        session = open_session(connect=[f"127.0.0.1:{port}"])
        images = ImageSubscriber(session, f"gpu/{CAMERA}/color", transport="auto", local="cuda")
        frame = None
        end = time.monotonic() + 30
        while frame is None and time.monotonic() < end:
            frame = images.latest(timeout=0.5)
        assert images.transport == "cuda" and isinstance(frame.image, CudaImage) and frame.image.shape == (48, 64, 3)
        images.close()
        session.close()
    finally:
        stop_child(child, stop)


def test_playback_controls_as_parameters(sessions, tmp_path):  # noqa: F811
    """The play node's parameters drive a recording's playback: duration and file read, paused holds it, position_s
    seeks (back to 0: the stamps go on growing), loop at run time, events from the playback's own changes (its end),
    and position_s after the end plays it again."""
    a, b = sessions
    scene = Scene(a)
    recorder = recording.Recorder(b, tmp_path, name="ctl_rec", split=0, look_every=0.5)
    record(recorder, scene, frames=10)
    (path,) = recorder.files
    player_node = recording.Recorder(a, tmp_path, name="ctl_play")
    stamps = []
    sub = b.declare_subscriber(f"{CAMERA}/color/image_raw/compressed", lambda s: stamps.append(
        CompressedImage.deserialize(s.payload.to_bytes()).header.stamp.to_ns()))
    client = ParameterClient(b, "xblick/ctl_play")
    events = []
    client.on_event(lambda e: events.append({p.name: p.value.value() for p in e.changed_parameters}))
    time.sleep(0.5)
    try:
        player_node.play([path], rate=0.5)  # 10 frames, recorded 0.1 s apart: about 2 s
        wait_until(lambda: len(stamps) >= 2, timeout=5.0)
        values = dict(zip(client.list(), [v.value() for v in client.get(client.list())]))
        assert values["file"] == str(path) and 0.85 < values["duration_s"] < 2.0, values
        # paused: nothing goes out; resumed
        assert client.set({"paused": True})[0].successful
        assert client.get(["paused"])[0].value() is True
        time.sleep(0.3)
        held = len(stamps)
        time.sleep(0.5)
        assert len(stamps) == held
        assert client.set({"paused": False})[0].successful
        wait_until(lambda: len(stamps) > held, timeout=5.0)
        # sought back to the start: the frames again, their stamps moved on as a loop moves them (by the log
        # times played; the Scene's stamps are 1 s apart, its log times 0.1 s)
        played = len(stamps)
        assert client.set({"position_s": 0.0})[0].successful
        wait_until(lambda: len(stamps) > played + 1, timeout=5.0)
        again = stamps[played]
        assert again > stamps[0] and (again - stamps[0]) % 1 == 0  # the first frame, later
        assert client.set({"position_s": 99.0})[0].reason.startswith("position 99 s: between 0 and the length")
        # the end: announced, then a position plays it again
        wait_until(lambda: player_node.state == "idle", timeout=10.0)
        wait_until(lambda: any(e.get("file") == "" for e in events), timeout=5.0)
        count = len(stamps)
        assert client.set({"position_s": 0.0})[0].successful
        wait_until(lambda: len(stamps) > count, timeout=5.0)
        assert client.set({"loop": True})[0].successful and player_node.status()["play"]["loop"] is True
    finally:
        sub.undeclare()
        client.close()
        for c in (player_node, recorder, scene):
            c.close()


def write_rosbag(path, frames=5, compressed_too=False):
    """An MCAP file as ros2 bag record writes it (profile ros2, ros2msg schemas, channels on ROS topics without our
    metadata): a colour camera's image_raw and camera_info, a depth camera's compressed frames only, /tf_static, a
    point cloud, and an image topic outside image_transport's names. Frames 100 ms apart, stamped 1 s, 2 s, ..."""
    from xblick.codecs import compress
    from xblick.encodings import numpy_to_image
    from xblick.msg import PointCloud2, PointField

    mcap = recording.import_mcap()
    with open(path, "wb") as stream:
        writer = mcap.writer.Writer(stream)
        writer.start(profile="ros2", library="rosbag2")
        schemas = {name: writer.register_schema(name, "ros2msg", MSGDEFS[name].encode())
                   for name in ("sensor_msgs/msg/Image", "sensor_msgs/msg/CameraInfo",
                                "sensor_msgs/msg/CompressedImage", "tf2_msgs/msg/TFMessage",
                                "sensor_msgs/msg/PointCloud2")}
        topics = {"/rb/cam/color/image_raw": "sensor_msgs/msg/Image",
                  "/rb/cam/color/camera_info": "sensor_msgs/msg/CameraInfo",
                  "/rb/cam/depth/image_raw/compressed": "sensor_msgs/msg/CompressedImage",
                  "/tf_static": "tf2_msgs/msg/TFMessage", "/rb/lidar/cloud": "sensor_msgs/msg/PointCloud2",
                  "/other/image": "sensor_msgs/msg/Image"}
        if compressed_too:
            topics["/rb/cam/color/image_raw/compressed"] = "sensor_msgs/msg/CompressedImage"
        channels = {t: writer.register_channel(t, "cdr", schemas[n], {"offered_qos_profiles": ""})
                    for t, n in topics.items()}
        info = CameraInfo.pinhole(6, 4, 5.0, 5.0, 2.5, 1.5, frame_id="rb_cam")
        fields = [PointField(n, 4 * i, PointField.FLOAT32) for i, n in enumerate("xyz")]
        for k in range(frames):
            stamp = 1_000_000_000 * (k + 1)
            header = Header(Time.from_ns(stamp), "rb_cam")
            image = np.full((4, 6, 3), k, np.uint8)
            payloads = {"/rb/cam/color/image_raw": numpy_to_image(image, "rgb8", header),
                        "/rb/cam/color/camera_info": dataclasses.replace(info, header=header),
                        "/rb/cam/depth/image_raw/compressed": compress(np.full((4, 6), 1000 + k, np.uint16),
                                                                         "16UC1", PNG, header=header),
                        "/rb/lidar/cloud": PointCloud2(Header(Time.from_ns(stamp), "rb_lidar"), 1, 2, fields,
                                                       False, 12, 24, np.full(6, k, "<f4").tobytes(), True),
                        "/other/image": numpy_to_image(image, "rgb8", header)}
            if compressed_too:
                payloads["/rb/cam/color/image_raw/compressed"] = compress(image, "rgb8", header=header)
            if k == 0:
                payloads["/tf_static"] = TFMessage([transform("rb_base", "rb_cam", 0.1)])
            for topic, message in payloads.items():
                log_ns = 100_000_000 * k + 1
                writer.add_message(channels[topic], log_time=log_ns, data=bytes(message.serialize()),
                                   publish_time=log_ns)
        writer.finish()


def test_a_rosbag2_plays_its_cameras_as_ours(sessions, tmp_path):  # noqa: F811
    """A rosbag2 MCAP file: image_transport's topics become streams of ours (raw frames, compressed-only frames), a
    cloud a stream with its info record, every other topic the key live reading maps it to (/a/b -> a/b)."""
    from xblick.points import PointCloudSubscriber

    a, b = sessions
    path = tmp_path / "bag_0.mcap"
    write_rosbag(path)
    player = recording.Player(a, [path])
    color = ImageSubscriber(b, "rb/cam/color", transport="auto", camera_info=True)
    compressed = ImageSubscriber(b, "rb/cam/color", transport="compressed")
    depth = ImageSubscriber(b, "rb/cam/depth", transport="auto")
    clouds = PointCloudSubscriber(b, "rb/lidar")
    other = []
    plain = b.declare_subscriber("other/image", lambda s: other.append(str(s.encoding)))
    time.sleep(0.5)
    thread = threading.Thread(target=player.run)
    thread.start()
    try:
        frame = color.latest(timeout=5.0)
        assert frame is not None and frame.encoding == "rgb8" and frame.header.frame_id == "rb_cam"
        assert color.describe().startswith("shm") and frame.image[0, 0, 0] == frame.stamp_ns // 10**9 - 1
        wait_until(lambda: color.camera_info is not None)
        assert color.camera_info.k[0] == 5.0
        assert compressed.latest(timeout=5.0) is not None  # the playback camera compresses the raw frames
        d = depth.latest(timeout=5.0)
        assert d is not None and d.encoding == "16UC1" and d.image[0, 0] in range(1000, 1005)
        cloud = clouds.latest(timeout=5.0)
        assert cloud is not None and cloud.header.frame_id == "rb_lidar" and cloud.points.shape == (1, 2)
        thread.join()
        assert other and other[0] == "application/cdr;sensor_msgs/msg/Image"
        answers = [r.ok for r in replies(b, "tf_static", 1.0) if r.ok is not None]  # the bag's /tf_static: tf_static
        assert len(answers) == 1
        assert TFMessage.deserialize(answers[0].payload.to_bytes()).transforms[0].child_frame_id == "rb_cam"
        assert player.error is None
    finally:
        plain.undeclare()
        thread.join()
        for c in (color, compressed, depth, clouds, player):
            c.close()


def test_a_rosbag2_with_raw_and_compressed_plays_the_raw_frames(sessions, tmp_path, caplog):  # noqa: F811
    a, _ = sessions
    path = tmp_path / "both.mcap"
    write_rosbag(path, frames=2, compressed_too=True)
    with caplog.at_level("INFO", logger="xblick"):
        player = recording.Player(a, [path])
    try:
        assert "rb/cam/color" in player._raw_streams and "rb/cam/color/image_raw/compressed" not in player._publishers
        assert any("playing image_raw" in r.getMessage() for r in caplog.records)
    finally:
        player.close()


def test_rosbag2_directories_and_the_sqlite3_storage(tmp_path):
    bag = tmp_path / "bag"
    bag.mkdir()
    (bag / "metadata.yaml").write_text("rosbag2_bagfile_information:\n  storage_identifier: mcap\n")
    for name in ("bag_1.mcap", "bag_0.mcap"):
        write_rosbag(bag / name, frames=1)
    assert recording.rosbag_files(bag) == [bag / "bag_0.mcap", bag / "bag_1.mcap"]
    old = tmp_path / "old"
    old.mkdir()
    (old / "metadata.yaml").write_text("rosbag2_bagfile_information:\n  storage_identifier: sqlite3\n")
    (old / "old_0.db3").write_bytes(recording.SQLITE_MAGIC + b"\0" * 84)
    for given in (old, old / "old_0.db3"):
        with pytest.raises(recording.RosbagError, match="sqlite3 storage; xblick play reads rosbag2's MCAP"):
            recording.rosbag_files(given)
    empty = tmp_path / "empty"
    empty.mkdir()
    (empty / "metadata.yaml").write_text("")
    with pytest.raises(recording.RosbagError, match="without MCAP files"):
        recording.rosbag_files(empty)


def test_the_ros_copy_of_tf_static_holds_every_camera_and_is_not_played(sessions, tmp_path):  # noqa: F811
    """Two cameras' tf_static: each message of the /tf_static copy holds the static transforms of both so far (the
    last one all of them, as tf2's static broadcaster sends them). Playback serves each camera's key and its ROS copy
    on /tf_static once, never the file's copy as well."""
    a, b = sessions
    one = StaticTransformPublisher(a, "robot/cam1", [transform("base", "cam1", 0.1)])
    two = StaticTransformPublisher(a, "robot/cam2", [transform("base", "cam2", 0.2)])
    recorder = recording.Recorder(b, tmp_path, name="static_rec", split=0, look_every=0.5)
    recorder.record(keys=["robot/*/tf_static"])
    time.sleep(1.0)
    recorder.stop()
    (path,) = recorder.files
    for c in (one, two):
        c.close()
    _, messages = read(path)
    copies = [TFMessage.deserialize(m.data) for k, m in messages if k == "tf_static"]
    assert sorted(k for k, _ in messages if k != "tf_static") == ["robot/cam1/tf_static", "robot/cam2/tf_static"]
    assert {t.child_frame_id for t in copies[-1].transforms} == {"cam1", "cam2"}
    player = recording.Player(a, [path])
    try:
        assert "tf_static" not in player._publishers and set(player._publishers) == set(player._ros)
        assert sum(p.topic == "/tf_static" for p in player._ros.values()) == 2  # one per camera's key, no third
    finally:
        player.close()
        recorder.close()
