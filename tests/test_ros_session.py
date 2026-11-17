"""``--ros`` without a ROS install: the options, that nothing of ROS goes on the wire without them, and that with them
the streams, transforms and parameters appear on rmw_zenoh's keys with its attachment and tokens, read back by our own
ROS readers and clients. tests/test_ros_live.py checks the same against ROS 2's own programs."""

import argparse
import socket
import time

import numpy as np
import pytest

from xblick import ros, session
from xblick.camera import CameraPublisher, ImageSubscriber, StaticTransformPublisher, TransformPublisher
from xblick.detections import DetectionPublisher, DetectionSubscriber
from xblick.msg import CameraInfo, Detection2DArray, Header, Time, Transform, TransformStamped, Vector3
from xblick.parameters import Parameters
from xblick.poses import TransformBuffer

DOMAIN = 91


def free_port():
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def wait_until(condition, timeout=5.0):
    end = time.monotonic() + timeout
    while not condition():
        if time.monotonic() > end:
            raise TimeoutError
        time.sleep(0.02)


def parse(argv):
    parser = argparse.ArgumentParser()
    session.add_arguments(parser)
    return parser.parse_args(argv)


def test_ros_options(monkeypatch):
    monkeypatch.delenv("ROS_DOMAIN_ID", raising=False)
    assert session.ros_options_from_args(parse([]), prog="xblick cam") is None
    assert session.ros_options_from_args(parse(["--ros"]), prog="xblick cam") == session.RosOptions(0, "xblick_cam")
    monkeypatch.setenv("ROS_DOMAIN_ID", "12")
    assert session.ros_options_from_args(parse(["--ros", "--ros-node", "eye"]), prog="x").domain == 12
    assert session.ros_options_from_args(parse(["--ros", "--ros-domain", "3"]), prog="x").domain == 3
    for argv in (["--ros-domain", "3"], ["--ros-node", "eye"], ["--ros", "--ros-node", "a-b"],
                 ["--ros", "--ros-domain", "-1"]):
        with pytest.raises(ValueError):
            session.ros_options_from_args(parse(argv), prog="x")


class Network:
    """Two sessions connected directly (one listens on a free port), the first one ``--ros`` or not, and an observer
    session that sees every key and every ROS token."""

    def __init__(self, with_ros: bool):
        port = free_port()
        self.endpoint = f"tcp/127.0.0.1:{port}"
        options = session.RosOptions(DOMAIN, "xblick_test") if with_ros else None
        self.a = session.open_session(listen=[self.endpoint], ros_options=options)
        self.b = session.open_session(connect=[self.endpoint], ros_options=options and session.RosOptions(
            DOMAIN, "xblick_reader"))
        self.observer = session.open_session(connect=[self.endpoint], declare=False)
        self.keys: dict[str, bytes | None] = {}
        self.tokens: set[str] = set()
        self._sub = self.observer.declare_subscriber(
            "**", lambda s: self.keys.setdefault(str(s.key_expr), s.attachment.to_bytes() if s.attachment else None))
        self._live = self.observer.liveliness().declare_subscriber(
            "@ros2_lv/**", lambda s: self.tokens.add(str(s.key_expr)), history=True)

    def close(self):
        self._sub.undeclare()
        self._live.undeclare()
        for s in (self.b, self.a, self.observer):
            s.close()


def _publish_everything(net: Network):
    """A camera stream with camera_info, detections, a camera's tf and tf_static, a node's parameters, for 1.5 s."""
    info = CameraInfo(height=48, width=64, distortion_model="plumb_bob", d=[0.0] * 5)
    camera = CameraPublisher(net.a, "rostest/cam/color", encoding="rgb8", frame_id="rostest_cam", camera_info=info)
    boxes = DetectionPublisher(net.a, "rostest/cam/boxer/boxes", Detection2DArray)
    pose = TransformStamped(Header(Time(1, 0), "world"), "rostest_cam_body", Transform(Vector3(1, 2, 3)))
    static = StaticTransformPublisher(net.a, "rostest/cam", [pose])
    moving = TransformPublisher(net.a, "rostest/cam")
    params = Parameters(net.a, "boxer", "rostest/cam")
    params.declare("threshold", 0.5, range=(0.0, 1.0))
    try:
        end = time.monotonic() + 1.5
        while time.monotonic() < end:
            stamp = time.time_ns()
            camera.publish(np.zeros((48, 64, 3), np.uint8), stamp)
            boxes.publish(Detection2DArray(Header(Time.from_ns(stamp), "rostest_cam")))
            moving.publish([TransformStamped(Header(Time.from_ns(stamp), "world"), "rostest_cam_moving",
                                             Transform(Vector3(0, 0, 1)))])
            time.sleep(0.05)
    finally:
        for p in (camera, boxes, static, moving, params):
            p.close()


def test_without_ros_nothing_of_ros_goes_on_the_wire():
    net = Network(with_ros=False)
    try:
        time.sleep(0.5)
        _publish_everything(net)
        time.sleep(0.3)
        assert any(k.endswith("/image_raw") for k in net.keys)  # our keys were sent and seen
        assert not [k for k in net.keys if "::dds_::" in k or k[:1].isdigit()]
        assert net.tokens == set()
        assert net.a.ros is None and net.b.ros is None
    finally:
        net.close()


def test_with_ros_our_keys_and_rmw_zenohs_side_by_side():
    net = Network(with_ros=True)
    try:
        time.sleep(0.5)
        _publish_everything(net)
        time.sleep(0.3)
        image = ros.data_key(DOMAIN, "/rostest/cam/color/image_raw", "sensor_msgs/msg/Image",
                             ros.hash_of("sensor_msgs/msg/Image"))
        assert "rostest/cam/color/image_raw" in net.keys  # ours, unchanged
        for key in (image, ros.data_key(DOMAIN, "/rostest/cam/color/camera_info", "sensor_msgs/msg/CameraInfo",
                                        ros.hash_of("sensor_msgs/msg/CameraInfo")),
                    ros.data_key(DOMAIN, "/rostest/cam/boxer/boxes", "vision_msgs/msg/Detection2DArray",
                                 ros.hash_of("vision_msgs/msg/Detection2DArray")),
                    ros.data_key(DOMAIN, "/tf", "tf2_msgs/msg/TFMessage", ros.hash_of("tf2_msgs/msg/TFMessage"))):
            assert key in net.keys, key
            seq, stamp, gid = ros.decode_attachment(net.keys[key])
            assert seq >= 1 and abs(stamp - time.time_ns()) < 10 * 10**9
        tokens = [ros.parse_token(t) for t in net.tokens]
        nodes = {t.fully_qualified_node for t in tokens if t.kind == ros.NODE}
        assert {"/xblick_test", "/xblick_reader", "/rostest/cam/boxer"} <= nodes
        publishers = {(t.topic, t.qos.key()) for t in tokens if t.kind == ros.PUBLISHER}
        assert ("/rostest/cam/color/image_raw", "::,1:,:,:,,") in publishers
        assert ("/tf_static", ":1:,1:,:,:,,") in publishers
        services = {t.topic for t in tokens if t.kind == ros.SERVICE}
        assert {f"/rostest/cam/boxer/{s}" for s in ("get_parameters", "set_parameters", "list_parameters",
                                                    "describe_parameters", "get_parameter_types",
                                                    "set_parameters_atomically")} <= services
    finally:
        net.close()


def test_a_reader_with_ros_reads_the_ros_topics():
    """Our ROS readers against our ROS publishers: transport ros, detections by ROS type, tf_static of a publisher
    that published before the reader started (transient local, from its cache), /tf."""
    net = Network(with_ros=True)
    camera = CameraPublisher(net.a, "rostest/cam/color", encoding="mono8", frame_id="rostest_cam",
                             camera_info=CameraInfo(height=4, width=6))
    pose = TransformStamped(Header(Time(1, 0), "world"), "rostest_cam_body", Transform(Vector3(1, 2, 3)))
    static = StaticTransformPublisher(net.a, "rostest/cam", [pose])
    boxes = DetectionPublisher(net.a, "rostest/cam/boxer/boxes", Detection2DArray)
    moving = TransformPublisher(net.a, "rostest/cam")
    got_boxes = []
    time.sleep(1.0)  # tf_static was published before any reader
    images = ImageSubscriber(net.b, "rostest/cam/color", transport="ros", camera_info=True)
    reader = DetectionSubscriber(net.b, "rostest/cam/boxer/boxes", got_boxes.append, ros_type=Detection2DArray)
    buffer = TransformBuffer(net.b, "rostest/cam", ros=True)
    try:
        time.sleep(0.5)
        image = (np.arange(24, dtype=np.uint8).reshape(4, 6))
        frame = None
        for _ in range(40):
            stamp = time.time_ns()
            camera.publish(image, stamp)
            boxes.publish(Detection2DArray(Header(Time.from_ns(stamp), "rostest_cam")))
            moving.publish([TransformStamped(Header(Time.from_ns(stamp), "rostest_cam_body"), "rostest_cam_moving",
                                             Transform(Vector3(0, 0, 1)))])
            frame = frame or images.latest(timeout=0.05)
            if frame is not None and got_boxes:
                break
        assert frame is not None and (frame.image == image).all() and frame.seq is None
        assert images.camera_info.width == 6
        assert got_boxes and got_boxes[0].header.frame_id == "rostest_cam"
        world_from_moving = buffer.lookup("world", "rostest_cam_moving", stamp, timeout=2.0)
        assert np.allclose(world_from_moving[:3, 3], [1, 2, 4])
        with pytest.raises(ValueError, match="needs a session with --ros"):
            ImageSubscriber(net.observer, "rostest/cam/color", transport="ros")
    finally:
        for p in (images, reader, buffer, camera, static, boxes, moving):
            p.close()
        net.close()


def test_a_tf_frame_with_two_parents_is_one_error_line(caplog):
    net = Network(with_ros=True)
    try:
        a = StaticTransformPublisher(net.a, "rostest/cam1", [TransformStamped(Header(Time(1, 0), "world"), "same")])
        b = StaticTransformPublisher(net.a, "rostest/cam2", [TransformStamped(Header(Time(1, 0), "map"), "same")])
        b2 = StaticTransformPublisher(net.a, "rostest/cam3", [TransformStamped(Header(Time(1, 0), "odom"), "same")])
        errors = [r for r in caplog.records if "frame 'same' has parent" in r.getMessage()]
        assert len(errors) == 1 and "'world'" in errors[0].getMessage() and "'map'" in errors[0].getMessage()
        for p in (a, b, b2):
            p.close()
    finally:
        net.close()
