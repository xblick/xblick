"""``--ros`` against ROS 2 Jazzy's own programs on rmw_zenoh_cpp: its router (rmw_zenohd) on a free port, the demo
nodes talker, listener and parameter_blackboard, tf2_ros' static_transform_publisher and tf2_echo, and rosprobe
(tests/ros/rosprobe.cpp, an rclcpp node built here with g++, standing in for ros2 topic and ros2 param, which need
the install's Python). Skipped without /opt/ros/jazzy; the ROS programs run with its libraries, our sessions in this
process without them."""

import os
import re
import shutil
import socket
import subprocess
import time
from pathlib import Path

import numpy as np
import pytest

from xblick import ros, session
from xblick.camera import CameraPublisher, ImageSubscriber, StaticTransformPublisher, StaticTransformSubscriber
from xblick.detections import DetectionPublisher
from xblick.encodings import numpy_to_image
from xblick.msg import (BoundingBox2D, BoundingBox3D, CameraInfo, Detection2D, Detection2DArray, Detection3D,
                        Detection3DArray, Header, ObjectHypothesis, ObjectHypothesisWithPose, Point, Point2D, Pose,
                        Pose2D, PoseWithCovariance, Quaternion, Time, Transform, TransformStamped, Vector3)
from xblick.parameters import ParameterClient, Parameters

JAZZY = Path("/opt/ros/jazzy")
pytestmark = pytest.mark.skipif(not (JAZZY / "lib/rmw_zenoh_cpp/rmw_zenohd").exists(),
                                reason="needs ROS 2 Jazzy with rmw_zenoh_cpp in /opt/ros/jazzy")
DOMAIN = 93
CONFIGS = JAZZY / "share/rmw_zenoh_cpp/config"


def free_port():
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def with_port(path: Path, endpoint: str, port: int) -> str:
    """ROS's default config with ``endpoint`` (its one line outside the comments) on ``port`` of 127.0.0.1."""
    lines = path.read_text().splitlines()
    hits = [i for i, line in enumerate(lines) if endpoint in line and not line.lstrip().startswith("//")]
    assert len(hits) == 1, (path, hits)
    lines[hits[0]] = lines[hits[0]].replace(endpoint, f'"tcp/127.0.0.1:{port}"')
    return "\n".join(lines) + "\n"


@pytest.fixture(scope="module")
def ros_env(tmp_path_factory):
    """ROS's router on a free port; the environment of a ROS program that uses it."""
    tmp = tmp_path_factory.mktemp("ros")
    port = free_port()
    (tmp / "router.json5").write_text(with_port(CONFIGS / "DEFAULT_RMW_ZENOH_ROUTER_CONFIG.json5", '"tcp/[::]:7447"',
                                                port))
    (tmp / "session.json5").write_text(with_port(CONFIGS / "DEFAULT_RMW_ZENOH_SESSION_CONFIG.json5",
                                                 '"tcp/localhost:7447"', port))
    env = {k: v for k, v in os.environ.items() if not k.startswith(("ROS_", "RMW_", "ZENOH_", "AMENT_"))}
    env.update(AMENT_PREFIX_PATH=str(JAZZY), RMW_IMPLEMENTATION="rmw_zenoh_cpp", ROS_DOMAIN_ID=str(DOMAIN),
               ROS_LOG_DIR=str(tmp / "log"), ZENOH_ROUTER_CONFIG_URI=str(tmp / "router.json5"),
               ZENOH_SESSION_CONFIG_URI=str(tmp / "session.json5"),
               LD_LIBRARY_PATH=f"{JAZZY}/lib:{JAZZY}/lib/x86_64-linux-gnu:{JAZZY}/opt/zenoh_cpp_vendor/lib")
    process = subprocess.Popen([str(JAZZY / "lib/rmw_zenoh_cpp/rmw_zenohd")], env=env, stdout=subprocess.DEVNULL,
                               stderr=subprocess.DEVNULL)
    try:
        end = time.monotonic() + 10
        while True:
            try:
                socket.create_connection(("127.0.0.1", port), timeout=0.2).close()
                break
            except OSError:
                if time.monotonic() > end:
                    raise
                time.sleep(0.1)
        yield env, port
    finally:
        process.terminate()
        process.wait(10)


RCLCPP_LIBRARIES = ["rclcpp", "rcl", "rcutils", "rmw", "rcpputils", "rosidl_runtime_c", "rosidl_typesupport_cpp",
                    "rosidl_typesupport_c", "libstatistics_collector", "tracetools",
                    "rcl_interfaces__rosidl_typesupport_cpp", "rcl_interfaces__rosidl_generator_c",
                    "rcl_yaml_param_parser",
                    "ament_index_cpp", "rmw_implementation", "rcl_logging_interface",
                    "builtin_interfaces__rosidl_typesupport_cpp", "rosgraph_msgs__rosidl_typesupport_cpp",
                    "statistics_msgs__rosidl_typesupport_cpp", "rosidl_dynamic_typesupport",
                    "service_msgs__rosidl_typesupport_cpp", "type_description_interfaces__rosidl_typesupport_cpp"]


def build_rclcpp(tmp_path_factory, name: str, libraries=()) -> Path:
    """tests/ros/<name>.cpp built with g++ against /opt/ros/jazzy."""
    if shutil.which("g++") is None:
        pytest.skip(f"needs g++ to build {name}")
    out = tmp_path_factory.mktemp(name) / name
    includes = [f"-I{d}" for d in sorted((JAZZY / "include").iterdir()) if d.is_dir()]
    subprocess.run(["g++", "-std=c++17", "-O1", *includes, str(Path(__file__).parent / f"ros/{name}.cpp"), "-o",
                    str(out), f"-L{JAZZY}/lib", f"-Wl,-rpath,{JAZZY}/lib",
                    *[f"-l{lib}" for lib in [*RCLCPP_LIBRARIES, *libraries]]],
                   check=True, timeout=600, capture_output=True)
    return out


@pytest.fixture(scope="module")
def rosprobe(tmp_path_factory):
    return build_rclcpp(tmp_path_factory, "rosprobe")


@pytest.fixture(scope="module")
def rosvision(tmp_path_factory):
    """tests/ros/rosvision.cpp: typed vision_msgs subscriptions (needs ros-jazzy-vision-msgs)."""
    if not (JAZZY / "include/vision_msgs").is_dir():
        pytest.skip("needs ros-jazzy-vision-msgs")
    return build_rclcpp(tmp_path_factory, "rosvision", [
        f"{pkg}__rosidl_typesupport_cpp" for pkg in ("vision_msgs", "std_msgs", "geometry_msgs")])


def run(env, argv, timeout):
    """A ROS program's output (stdout and stderr), ended after ``timeout`` s by SIGTERM if still running."""
    process = subprocess.Popen(argv, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    try:
        out, _ = process.communicate(timeout=timeout)
    except subprocess.TimeoutExpired:
        process.terminate()
        out, _ = process.communicate(timeout=10)
    return out


@pytest.fixture
def ours(ros_env):
    """Our session with --ros on ROS's router."""
    _, port = ros_env
    with session.open_session(connect=[f"tcp/127.0.0.1:{port}"],
                              ros_options=session.RosOptions(DOMAIN, "xblick_live")) as s:
        yield s


class Background:
    """``step()`` called every ``period`` s in a thread until stopped."""

    def __init__(self, step, period=0.1):
        import threading
        self._stop = threading.Event()

        def loop():
            while not self._stop.wait(period):
                step()
        self._thread = threading.Thread(target=loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._stop.set()
        self._thread.join()


def string_cdr(text: str) -> bytes:
    data = text.encode() + b"\0"
    return b"\x00\x01\x00\x00" + len(data).to_bytes(4, "little") + data


def test_rosnode_listener_hears_our_string(ros_env, ours):
    env, _ = ros_env
    publisher = ours.ros.node.publisher("/chatter", "std_msgs/msg/String", ros.Qos(depth=7))
    n = iter(range(1000))
    sender = Background(lambda: publisher.put(string_cdr(f"from xblick {next(n)}")), 0.2)
    try:
        out = run(env, [str(JAZZY / "lib/demo_nodes_cpp/listener")], 4)
    finally:
        sender.stop()
        publisher.close()
    assert re.search(r"I heard: \[from xblick \d+\]", out), out[-2000:]


def test_rclcpp_reads_our_camera_stream(ros_env, ours, rosprobe):
    env, _ = ros_env
    camera = CameraPublisher(ours, "live/cam/color", encoding="rgb8", frame_id="live_cam",
                             camera_info=CameraInfo(height=48, width=64))
    sender = Background(lambda: camera.publish(np.full((48, 64, 3), 7, np.uint8)), 0.05)
    try:
        image = run(env, [str(rosprobe), "sub", "/live/cam/color/image_raw", "sensor_msgs/msg/Image", "3"], 30)
        info = run(env, [str(rosprobe), "sub", "/live/cam/color/camera_info", "sensor_msgs/msg/CameraInfo", "2"], 30)
        jpeg = run(env, [str(rosprobe), "sub", "/live/cam/color/image_raw/compressed",
                         "sensor_msgs/msg/CompressedImage", "2"], 30)
    finally:
        sender.stop()
        camera.close()
    for out in (image, info, jpeg):
        assert int(re.search(r"TOTAL (\d+)", out)[1]) > 3, out[-2000:]
    # the bytes as ours: CDR header, the frame id
    assert "GOT 1 " in image and b"live_cam".hex() in image


def test_rclcpp_deserialises_our_detections(ros_env, ours, rosvision):
    """Our Detection2DArray and Detection3DArray reach typed rclcpp subscriptions with every field as sent."""
    env, _ = ros_env
    cov = np.zeros(36)
    cov[0] = 0.01
    pose = Pose(Point(0.1, -0.2, 1.5), Quaternion(0.0, 0.0, 0.6, 0.8))
    boxes3d = Detection3DArray(Header(Time(19, 20), "live_cam"), [Detection3D(
        Header(Time(19, 20), "live_cam"), [ObjectHypothesisWithPose(ObjectHypothesis("mug", 0.75),
                                                                    PoseWithCovariance(pose, cov))],
        BoundingBox3D(pose, Vector3(0.08, 0.12, 0.1)), "7")])
    boxes2d = Detection2DArray(Header(Time(21, 22), "live_cam"), [Detection2D(
        Header(Time(21, 22), "live_cam"), [ObjectHypothesisWithPose(ObjectHypothesis("chair", 0.5))],
        BoundingBox2D(Pose2D(Point2D(320.5, 240.25), 0.0), 40.0, 30.0), "3")])
    pub2d = DetectionPublisher(ours, "live/cam/boxer/boxes", Detection2DArray)
    pub3d = DetectionPublisher(ours, "live/cam/boxer/boxes_3d", Detection3DArray)
    sender = Background(lambda: (pub2d.publish(boxes2d), pub3d.publish(boxes3d)), 0.1)
    try:
        got2d = run(env, [str(rosvision), "2d", "/live/cam/boxer/boxes", "3"], 30)
        got3d = run(env, [str(rosvision), "3d", "/live/cam/boxer/boxes_3d", "3"], 30)
    finally:
        sender.stop()
        pub2d.close()
        pub3d.close()
    for out in (got2d, got3d):
        assert int(re.search(r"TOTAL (\d+)", out)[1]) > 3, out[-2000:]
    assert ("frame=live_cam stamp=21.000000022 n=1 [id=3 chair:0.500 box=320.50,240.25,40.00,30.00]"
            in got2d), got2d[-2000:]
    assert ("frame=live_cam stamp=19.000000020 n=1 [id=7 mug:0.750 cov0=0.010 box=0.10,-0.20,1.50 "
            "q=0.00,0.00,0.60,0.80 size=0.08,0.12,0.10]" in got3d), got3d[-2000:]


def test_we_hear_the_talker(ros_env, ours):
    env, _ = ros_env
    heard = []
    subscriber = ours.ros.node.subscriber("/chatter", "std_msgs/msg/String", lambda payload, *_: heard.append(payload))
    try:
        run(env, [str(JAZZY / "lib/demo_nodes_cpp/talker")], 3.5)
    finally:
        subscriber.close()
    assert heard and b"Hello World" in heard[-1]


def test_transport_ros_reads_a_ros_image(ros_env, ours, rosprobe):
    env, _ = ros_env
    picture = np.arange(4 * 6 * 3, dtype=np.uint8).reshape(4, 6, 3)
    payload = bytes(numpy_to_image(picture, "rgb8", Header(Time(5, 0), "ros_cam")).serialize())
    reader = ImageSubscriber(ours, "live/roscam", transport="ros")
    try:
        process = subprocess.Popen([str(rosprobe), "pub", "/live/roscam/image_raw", "sensor_msgs/msg/Image",
                                    payload.hex(), "20"], env=env, stdout=subprocess.DEVNULL,
                                   stderr=subprocess.DEVNULL)
        frame = reader.latest(timeout=10.0)
        process.terminate()
        process.wait(10)
    finally:
        reader.close()
    assert frame is not None and frame.header.frame_id == "ros_cam" and (frame.image == picture).all()


def test_tf_static_to_a_late_ros_listener(ros_env, ours):
    """Our /tf_static, published once before tf2_echo started, reaches it (transient local: from the cache)."""
    env, _ = ros_env
    pose = TransformStamped(Header(Time(1, 0), "live_world"), "live_cam_body", Transform(Vector3(1.0, 2.0, 3.0)))
    static = StaticTransformPublisher(ours, "live/cam", [pose])
    try:
        time.sleep(1.0)
        out = run(env, [str(JAZZY / "lib/tf2_ros/tf2_echo"), "live_world", "live_cam_body"], 5)
    finally:
        static.close()
    assert "Translation: [1.000, 2.000, 3.000]" in out, out[-2000:]


def test_tf_static_of_ros_to_our_late_reader(ros_env, ours):
    """static_transform_publisher's /tf_static, published before our reader started, reaches it."""
    env, _ = ros_env
    got = []
    process = subprocess.Popen([str(JAZZY / "lib/tf2_ros/static_transform_publisher"), "--x", "4", "--frame-id",
                                "ros_world", "--child-frame-id", "ros_thing"], env=env, stdout=subprocess.DEVNULL,
                               stderr=subprocess.DEVNULL)
    try:
        time.sleep(2.0)
        reader = StaticTransformSubscriber(ours, "unused", got.append, ros=True)
        end = time.monotonic() + 5
        while not got and time.monotonic() < end:
            time.sleep(0.05)
        reader.close()
    finally:
        process.terminate()
        process.wait(10)
    transforms = [t for m in got for t in m.transforms]
    assert any(t.child_frame_id == "ros_thing" and t.transform.translation.x == 4.0 for t in transforms), got


def test_ros_parameter_client_on_our_node(ros_env, ours, rosprobe):
    """rclcpp's parameter client on our node: reads; a set refused by default; with --ros-params-writable taken, and an
    out-of-range one refused."""
    env, port = ros_env
    params = Parameters(ours, "boxer", "live/cam")
    params.declare("max_boxes", 10, range=(1, 100))
    params.declare("model", "base", read_only=True)
    try:
        listed = run(env, [str(rosprobe), "params", "/live/cam/boxer", "list"], 30)
        got = run(env, [str(rosprobe), "params", "/live/cam/boxer", "get", "max_boxes"], 30)
        set_off = run(env, [str(rosprobe), "params", "/live/cam/boxer", "set", "max_boxes", "42"], 30)
        value_off = params["max_boxes"]
    finally:
        params.close()
    with session.open_session(connect=[f"tcp/127.0.0.1:{port}"], ros_options=session.RosOptions(
            DOMAIN, "xblick_live_writable", params_writable=True)) as writable:
        params = Parameters(writable, "boxer", "live/cam2")
        params.declare("max_boxes", 10, range=(1, 100))
        try:
            set_ok = run(env, [str(rosprobe), "params", "/live/cam2/boxer", "set", "max_boxes", "42"], 30)
            set_refused = run(env, [str(rosprobe), "params", "/live/cam2/boxer", "set", "max_boxes", "500"], 30)
        finally:
            value_on = params["max_boxes"]
            params.close()
    assert "NAME max_boxes" in listed and "NAME model" in listed, listed[-2000:]
    assert "VALUE max_boxes 10" in got, got[-2000:]
    assert "SET 0 sets through ROS 2 are off" in set_off and value_off == 10, set_off
    assert "SET 1" in set_ok and "SET 0" in set_refused, set_ok + set_refused
    assert value_on == 42


def test_our_parameter_client_on_a_ros_node(ros_env, ours):
    env, _ = ros_env
    process = subprocess.Popen([str(JAZZY / "lib/demo_nodes_cpp/parameter_blackboard")], env=env,
                               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    client = ParameterClient(ours, "parameter_blackboard", timeout=5.0, ros=True)
    try:
        (result,) = client.set({"answer": 42})
        assert result.successful and not client.last_checked
        assert [v.value() for v in client.get(["answer"])] == [42]
        assert "answer" in client.list()
    finally:
        client.close()
        process.terminate()
        process.wait(10)
