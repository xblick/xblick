"""``--ros`` against ROS 2 Lyrical's own programs on rmw_zenoh_cpp 0.10.x, in a docker container: its router
(rmw_zenohd) on a free port of the host's network, image_tools' cam2image (rclcpp) and ``ros2 topic pub`` (rclpy).
Lyrical's publishers send Image, CompressedImage and PointCloud2 on buffer keys (ros.BUFFER_FIELDS), which our
readers must take; the other direction goes on the plain key.

Skipped unless XBLICK_ROS_LYRICAL_IMAGE names a docker image with ROS 2 Lyrical in /opt/ros/lyrical, rmw_zenoh_cpp,
image_tools and ros2cli. tests/ros/lyrical.Dockerfile builds such an image. The C++ case also needs XBLICK_CPP_BUILD (as tests/test_cpp_ros.py)."""

import json
import os
import shutil
import socket
import subprocess
import threading
import time
import uuid

import numpy as np
import pytest

from xblick import ros, session
from xblick.camera import CameraPublisher, ImageSubscriber
from xblick.msg import CameraInfo
from xblick.points import PointCloudSubscriber

IMAGE = os.environ.get("XBLICK_ROS_LYRICAL_IMAGE", "")
pytestmark = pytest.mark.skipif(not IMAGE or shutil.which("docker") is None,
                                reason="XBLICK_ROS_LYRICAL_IMAGE (a docker image with ROS 2 Lyrical and rmw_zenoh_cpp) "
                                       "is not set, or docker is missing")
DOMAIN = 77
LYRICAL = "/opt/ros/lyrical"
CONFIGS = f"{LYRICAL}/share/rmw_zenoh_cpp/config"


def free_port():
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def with_port(text: str, endpoint: str, port: int) -> str:
    """ROS's default config with ``endpoint`` (its one line outside the comments) on ``port`` of 127.0.0.1."""
    lines = text.splitlines()
    hits = [i for i, line in enumerate(lines) if endpoint in line and not line.lstrip().startswith("//")]
    assert len(hits) == 1, hits
    lines[hits[0]] = lines[hits[0]].replace(endpoint, f'"tcp/127.0.0.1:{port}"')
    return "\n".join(lines) + "\n"


class Lyrical:
    """The container: ``start(command)`` runs a ROS program in it (ended with the container), ``run(command,
    timeout)`` one to its end and returns its output."""

    def __init__(self, tmp, port: int):
        self.name = f"xblick-test-lyrical-{uuid.uuid4().hex[:8]}"
        subprocess.run(["docker", "run", "-d", "--rm", "--name", self.name, "--network", "host", "-v", f"{tmp}:/cfg",
                        IMAGE, "sleep", "infinity"], check=True, capture_output=True, timeout=120)
        self.env = (f"source {LYRICAL}/setup.bash && export RMW_IMPLEMENTATION=rmw_zenoh_cpp ROS_DOMAIN_ID={DOMAIN} "
                    "ZENOH_ROUTER_CONFIG_URI=/cfg/router.json5 ZENOH_SESSION_CONFIG_URI=/cfg/session.json5 "
                    "ROS_LOG_DIR=/tmp/roslog && ")
        for name, endpoint in (("ROUTER", '"tcp/[::]:7447"'), ("SESSION", '"tcp/localhost:7447"')):
            text = self.run(f"cat {CONFIGS}/DEFAULT_RMW_ZENOH_{name}_CONFIG.json5", 30, check=True)
            (tmp / f"{name.lower()}.json5").write_text(with_port(text, endpoint, port))

    def run(self, command: str, timeout: float, check: bool = False) -> str:
        done = subprocess.run(["docker", "exec", self.name, "bash", "-c", self.env + f"timeout {timeout} {command}"],
                              capture_output=True, text=True, timeout=timeout + 30)
        if check:
            assert done.returncode == 0, done.stderr
        return done.stdout + done.stderr

    def start(self, command: str):
        subprocess.run(["docker", "exec", "-d", self.name, "bash", "-c", self.env + f"exec {command}"], check=True,
                       timeout=30)

    def close(self):
        subprocess.run(["docker", "rm", "-f", self.name], capture_output=True, timeout=60)


@pytest.fixture(scope="module")
def lyrical(tmp_path_factory):
    tmp = tmp_path_factory.mktemp("lyrical")
    port = free_port()
    box = Lyrical(tmp, port)
    try:
        box.start("ros2 run rmw_zenoh_cpp rmw_zenohd")
        end = time.monotonic() + 20
        while True:
            try:
                socket.create_connection(("127.0.0.1", port), timeout=0.2).close()
                break
            except OSError:
                if time.monotonic() > end:
                    raise
                time.sleep(0.1)
        yield box, port
    finally:
        box.close()


@pytest.fixture
def ours(lyrical):
    _, port = lyrical
    with session.open_session(connect=[f"tcp/127.0.0.1:{port}"],
                              ros_options=session.RosOptions(DOMAIN, "xblick_lyrical")) as s:
        yield s


def test_we_read_cam2image(lyrical, ours):
    """rclcpp's Image publisher: the frames come on <key>/_buf_cpu."""
    box, _ = lyrical
    reader = ImageSubscriber(ours, "lyr/cpp", transport="ros")
    try:
        box.start("ros2 run image_tools cam2image --ros-args -p burger_mode:=true -p show_camera:=false "
                  "-p frequency:=10.0 -r image:=/lyr/cpp/image_raw")
        frame = reader.latest(timeout=20.0)
    finally:
        reader.close()
    assert frame is not None and frame.image.shape == (240, 320, 3)


def test_we_read_rclpy_image_and_cloud(lyrical, ours):
    box, _ = lyrical
    images = ImageSubscriber(ours, "lyr/py", transport="ros")
    clouds = PointCloudSubscriber(ours, "lyr/py", transport="ros")
    try:
        box.start("ros2 topic pub -r 10 /lyr/py/image_raw sensor_msgs/msg/Image "
                  "'{height: 2, width: 3, encoding: mono8, step: 3, data: [1, 2, 3, 4, 5, 6]}'")
        box.start("ros2 topic pub -r 10 /lyr/py/cloud sensor_msgs/msg/PointCloud2 '{height: 1, width: 2, fields: "
                  "[{name: x, offset: 0, datatype: 7, count: 1}], point_step: 4, row_step: 8, "
                  "data: [0, 0, 128, 63, 0, 0, 0, 64], is_dense: true}'")
        frame = images.latest(timeout=20.0)
        cloud = clouds.latest(timeout=20.0)
    finally:
        images.close()
        clouds.close()
    assert frame is not None and frame.image.reshape(-1).tolist() == [1, 2, 3, 4, 5, 6]
    assert cloud is not None and cloud.points["x"].reshape(-1).tolist() == [1.0, 2.0]


def test_lyrical_reads_our_camera(lyrical, ours):
    box, _ = lyrical
    camera = CameraPublisher(ours, "lyr/xb/color", encoding="rgb8", frame_id="lyr_xb",
                             camera_info=CameraInfo(height=48, width=64))
    stop = []

    def send():
        while not stop:
            camera.publish(np.full((48, 64, 3), 7, np.uint8))
            time.sleep(0.05)
    sender = threading.Thread(target=send, daemon=True)
    sender.start()
    try:
        out = box.run("ros2 topic echo --once --field header.frame_id /lyr/xb/color/image_raw "
                      "sensor_msgs/msg/Image", 40)
    finally:
        stop.append(True)
        sender.join()
        camera.close()
    assert "lyr_xb" in out, out[-2000:]


def test_cpp_reads_cam2image(lyrical):
    from test_cpp_interop import BUILD, tool
    if not BUILD:
        pytest.skip("XBLICK_CPP_BUILD (the C++ build directory) is not set")
    box, port = lyrical
    box.start("ros2 run image_tools cam2image --ros-args -p burger_mode:=true -p show_camera:=false "
              "-p frequency:=10.0 -r image:=/lyr/cpp2/image_raw")
    probe = subprocess.run([tool("xblick-cpp-probe"), "--connect", f"127.0.0.1:{port}", "--ros", "--ros-domain",
                            str(DOMAIN), "--base", "lyr/cpp2", "--transport", "ros", "--frames", "2",
                            "--timeout", "20"],
                           capture_output=True, text=True, timeout=60)
    lines = [json.loads(line) for line in probe.stdout.splitlines()]
    assert probe.returncode == 0 and len(lines) == 2, probe.stdout + probe.stderr


def test_lyrical_publishers_say_backends_for_our_buffer_types(lyrical, ours):
    """A Lyrical publisher's token ends in backends exactly for the types that ros.BUFFER_FIELDS marks."""
    box, _ = lyrical
    types = ("sensor_msgs/msg/Image", "sensor_msgs/msg/CompressedImage", "sensor_msgs/msg/PointCloud2",
             "sensor_msgs/msg/CameraInfo", "tf2_msgs/msg/TFMessage")
    for i, type_name in enumerate(types):
        box.start(f"ros2 topic pub -r 1 /lyr/tok/t{i} {type_name} '{{}}'")
    tokens = []
    watch = ros._raw(ours).liveliness().declare_subscriber(
        f"@ros2_lv/{DOMAIN}/**/MP/**", lambda s: tokens.append(str(s.key_expr)), history=True)
    try:
        end = time.monotonic() + 20
        while time.monotonic() < end:
            found = {t.topic: t for t in map(ros.parse_token, list(tokens)) if t.topic.startswith("/lyr/tok/")}
            if len(found) == len(types):
                break
            time.sleep(0.2)
    finally:
        watch.undeclare()
    assert len(found) == len(types), sorted(found)
    for i, type_name in enumerate(types):
        token = found[f"/lyr/tok/t{i}"]
        assert token.type_name == type_name
        assert (token.backends == ros.CPU_BACKENDS) == ros.BUFFER_FIELDS[type_name], token.key()
