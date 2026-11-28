"""ROS 2 without a ROS install: the options (ROS on by default, ``--no-ros``, ``XBLICK_ROS``), that nothing of ROS goes
on the wire without it, and that with it the streams, transforms and parameters appear on rmw_zenoh's keys with its
attachment and tokens, read back by our own ROS readers and clients; one node per program, its name, frames and
/clock. tests/test_ros_live.py checks the same against ROS 2's own programs."""

import argparse
import socket
import time

import numpy as np
import pytest
import zenoh

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


def test_the_default_node_name_is_the_parsers_program(monkeypatch):
    """Without prog the default ROS node name is the parser's prog (a node started from python -c or a test)."""
    parser = argparse.ArgumentParser(prog="xblick-my-node")
    session.add_arguments(parser)
    assert session.ros_options_from_args(parser.parse_args(["--ros"])).node == "xblick_my_node"
    assert session.ros_options_from_args(parser.parse_args(["--ros"]), prog="other").node == "other"


def test_ros_options(monkeypatch):
    """ROS is on by default; --ros changes nothing; --no-ros and XBLICK_ROS=0 turn it off, and the ROS options then
    are errors; XBLICK_ROS takes 0 and 1 only."""
    monkeypatch.delenv("ROS_DOMAIN_ID", raising=False)
    monkeypatch.delenv("XBLICK_ROS", raising=False)
    default = session.RosOptions(0, "xblick_cam", follow_parameters=True)
    assert session.ros_options_from_args(parse([]), prog="xblick cam") == default
    assert session.ros_options_from_args(parse(["--ros"]), prog="xblick cam") == default
    assert session.ros_options_from_args(parse(["--no-ros"]), prog="xblick cam") is None
    monkeypatch.setenv("XBLICK_ROS", "1")
    assert session.ros_options_from_args(parse([]), prog="xblick cam") == default
    monkeypatch.setenv("XBLICK_ROS", "0")
    assert session.ros_options_from_args(parse(["--ros"]), prog="xblick cam") is None
    with pytest.raises(ValueError, match="XBLICK_ROS=0"):
        session.ros_options_from_args(parse(["--ros-domain", "3"]), prog="x")
    for value in ("", "yes", "2", " 1"):
        monkeypatch.setenv("XBLICK_ROS", value)
        with pytest.raises(ValueError, match="0 .* or 1"):
            session.ros_options_from_args(parse([]), prog="x")
    monkeypatch.delenv("XBLICK_ROS")
    monkeypatch.setenv("ROS_DOMAIN_ID", "12")
    eye = session.ros_options_from_args(parse(["--ros-node", "eye"]), prog="x")
    assert (eye.domain, eye.namespace, eye.node, eye.follow_parameters) == (12, "/", "eye", False)
    lab = session.ros_options_from_args(parse(["--ros-node", "/lab/eye"]), prog="x")
    assert (lab.namespace, lab.node) == ("/lab", "eye")
    assert session.ros_options_from_args(parse(["--ros-domain", "3"]), prog="x").domain == 3
    assert not session.ros_options_from_args(parse([]), prog="x").params_writable
    assert session.ros_options_from_args(parse(["--ros-params-writable"]), prog="x").params_writable
    assert session.ros_options_from_args(parse(["--ros-clock", "sim/clock"]), prog="x").clock == "sim/clock"
    for argv in (["--no-ros", "--ros-domain", "3"], ["--no-ros", "--ros-node", "eye"],
                 ["--no-ros", "--ros-params-writable"], ["--no-ros", "--ros-clock", "sim/clock"],
                 ["--ros-node", "a-b"], ["--ros-node", "/a-b/c"], ["--ros-domain", "-1"]):
        with pytest.raises(ValueError):
            session.ros_options_from_args(parse(argv), prog="x")


def test_the_default_node_is_in_the_cameras_namespace(monkeypatch):
    """A program with --camera (a node): its node in the camera's namespace; a name ROS does not take is written as ROS
    takes it, so that it does not stop a program that did not ask for ROS."""
    monkeypatch.delenv("XBLICK_ROS", raising=False)
    parser = argparse.ArgumentParser(prog="xblick-yolox")
    session.add_arguments(parser)
    parser.add_argument("--camera")
    options = session.ros_options_from_args(parser.parse_args(["--camera", "test/cam-2"]))
    assert (options.namespace, options.node, options.follow_parameters) == ("/test/cam_2", "xblick_yolox", True)
    assert session.ros_options_from_args(parse([]), prog="x", camera="lab/1st").namespace == "/lab/_1st"


def test_open_session_takes_part_by_default(monkeypatch):
    monkeypatch.delenv("XBLICK_ROS", raising=False)
    with session.open_session(local_router=False, program="xblick probe") as s:
        assert s.ros is not None and s.ros.node.fully_qualified == "/xblick_probe" and s.ros.follow_parameters
    with session.open_session(local_router=False, ros_options=False) as s:
        assert s.ros is None
    monkeypatch.setenv("XBLICK_ROS", "0")
    with session.open_session(local_router=False) as s:
        assert s.ros is None
    with session.open_session(local_router=False, ros_options=True) as s:  # asked for explicitly
        assert s.ros is not None
    monkeypatch.setenv("XBLICK_ROS", "on")
    with pytest.raises(ValueError, match="XBLICK_ROS"):
        session.open_session(local_router=False)


class Network:
    """Two sessions connected directly (one listens on a free port), both in ROS 2 or both not, and an observer
    session (not in ROS 2) that sees every key and every ROS token."""

    def __init__(self, with_ros: bool):
        port = free_port()
        self.endpoint = f"tcp/127.0.0.1:{port}"
        options = session.RosOptions(DOMAIN, "xblick_test") if with_ros else False
        self.a = session.open_session(listen=[self.endpoint], ros_options=options)
        self.b = session.open_session(connect=[self.endpoint], ros_options=options and session.RosOptions(
            DOMAIN, "xblick_reader"))
        self.observer = session.open_session(connect=[self.endpoint], declare=False, ros_options=False)
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
        assert nodes == {"/xblick_test", "/xblick_reader"}  # one node per program, its parameters' services on it
        publishers = {(t.topic, t.qos.key()) for t in tokens if t.kind == ros.PUBLISHER}
        assert ("/rostest/cam/color/image_raw", "::,1:,:,:,,") in publishers
        assert ("/tf_static", ":1:,1:,:,:,,") in publishers
        services = {t.topic for t in tokens if t.kind == ros.SERVICE}
        assert {f"/xblick_test/{s}" for s in ("get_parameters", "set_parameters", "list_parameters",
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
        with pytest.raises(ValueError, match="takes part in ROS 2"):
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


def test_command_keys_never_go_to_ros(caplog):
    """What a robot is told (a part ``command``) is never offered to ROS, whatever its type: a simulated robot's
    command/joints and command/tf, a controller's command; also through xblick play."""
    from types import SimpleNamespace

    from xblick import topics
    from xblick.camera import CountedPublisher, cdr_encoding
    from xblick.msg import JointState, RobotState, TFMessage
    from xblick.recording import Player

    net = Network(with_ros=True)
    keys = [(topics.command("robot", "joints"), JointState), (topics.command("robot", "tf"), TFMessage),
            (topics.controller_command("robot", "arms"), RobotState)]
    try:
        for key, kind in keys:
            assert ros.commanding(key)
            assert ros.mirror(net.a, key, cdr_encoding(kind)) is None
            assert ros.mirror_tf(net.a, key=key) is None
            publisher = CountedPublisher(net.a, key, cdr_encoding(kind), ros_tf=kind is TFMessage)
            assert publisher._ros is None and not publisher.ros_matched
            publisher.close()
            assert Player._ros_publisher(SimpleNamespace(session=net.a), key, str(cdr_encoding(kind))) is None
        assert any("a command key" in r.getMessage() for r in caplog.records)
        assert not ros.commanding("robot/joint_states") and not ros.commanding("rostest/cam/tf")
        assert not [t for t in net.tokens if "command" in t]
    finally:
        net.close()


def test_matched_is_our_key_and_ros_matched_ros():
    """CountedPublisher.matched means a subscriber of our key, as before; a ROS subscriber shows in ros_matched."""
    from xblick.camera import CountedPublisher, cdr_encoding
    from xblick.msg import JointState

    endpoint = f"tcp/127.0.0.1:{free_port()}"
    a = session.open_session(listen=[endpoint], ros_options=session.RosOptions(DOMAIN, "xblick_a"))
    b = session.open_session(connect=[endpoint], ros_options=session.RosOptions(DOMAIN, "xblick_b"))
    publisher = CountedPublisher(a, "rostest/robot/joint_states", cdr_encoding(JointState))
    try:
        ros_reader = b.ros.node.subscriber("/rostest/robot/joint_states", JointState.TYPE, lambda *_: None)
        wait_until(lambda: publisher.ros_matched)
        time.sleep(0.3)
        assert not publisher.matched
        ours = b.declare_subscriber("rostest/robot/joint_states", lambda s: None)
        wait_until(lambda: publisher.matched)
        ours.undeclare()
        ros_reader.close()
    finally:
        publisher.close()
        b.close()
        a.close()


def test_a_part_that_cannot_be_closed_keeps_no_other_open():
    """session._close_parts closes the declaration when the part in ROS raises; Ros.close closes every part when one
    raises (the services without parameters stop their thread, the node its token) and raises the first error."""

    class Part:
        def __init__(self, fails):
            self.fails, self.closed = fails, False

        def close(self):
            self.closed = True
            if self.fails:
                raise zenoh.ZError("closed")

    taking_part, declared = Part(True), Part(False)
    session._close_parts(declared, taking_part)
    assert taking_part.closed and declared.closed

    a = session.open_session(local_router=False, ros_options=session.RosOptions(DOMAIN, "xblick_parts"))
    try:
        graph, none = a.ros.graph, a.ros._no_parameters
        closes = graph.close

        def failing():
            closes()
            raise zenoh.ZError("the graph's close failed")

        graph.close = failing
        server, node = none._server, a.ros.node
        with pytest.raises(zenoh.ZError, match="the graph's close failed"):
            a.ros.close()
        server.join(timeout=2.0)
        assert not server.is_alive() and a.ros._duplicates is None and node._token is None and not a.ros._nodes
    finally:
        a.close()


def test_a_rename_that_races_a_close_leaves_no_token():
    """RosNode.rename against an entity's close in another thread: the close waits for the rename's new token and
    undeclares it, and no token of the publisher remains in ROS's graph."""
    import threading

    a = session.open_session(local_router=False, ros_options=session.RosOptions(DOMAIN, "xblick_race"))
    try:
        publisher = a.ros.node.publisher("/rostest/race/image_raw", "sensor_msgs/msg/Image")
        live = publisher._live
        closer = threading.Thread(target=publisher.close)

        class Racing:  # the old token: its undeclare lets the close start in the middle of the rename
            def undeclare(self):
                live.undeclare()
                closer.start()
                closer.join(0.3)  # with the entity's lock the close waits for the rename

        publisher._live = Racing()
        a.ros.node.rename("xblick_raced")
        closer.join(5.0)
        assert not closer.is_alive() and publisher._live is None
        tokens = [ros.parse_token(str(r.ok.key_expr))
                  for r in session.replies(a, "@ros2_lv/**", 1.0, liveliness=True) if r.ok is not None]
        assert not [t for t in tokens if t.topic == "/rostest/race/image_raw"]
        publisher._rename()  # closed: nothing declared again
        assert publisher._live is None
    finally:
        a.close()


def test_closed_parameters_of_their_own_leave_the_sessions_nodes():
    a = session.open_session(local_router=False, ros_options=session.RosOptions(DOMAIN, "xblick_nodes"))
    try:
        first = Parameters(a, "first", "rostest/nodes")
        count = len(a.ros._nodes)
        for _ in range(3):
            more = Parameters(a, "more", "rostest/nodes")
            assert more._ros_own_node and len(a.ros._nodes) == count + 1
            more.close()
            assert len(a.ros._nodes) == count
        first.close()
    finally:
        a.close()


def test_one_node_per_program_named_after_its_parameters(capsys):
    """A default node name gives way to the name of the program's first parameters: one node, which carries the
    publishers made before (their tokens declared anew), the six services and the publishers made after. Further
    parameters are a node of their own."""
    endpoint = f"tcp/127.0.0.1:{free_port()}"
    a = session.open_session(listen=[endpoint], ros_options=session.RosOptions(DOMAIN, "xblick_x",
                                                                              follow_parameters=True))
    observer = session.open_session(connect=[endpoint], declare=False, ros_options=False)
    tokens: set[str] = set()
    live = observer.liveliness().declare_subscriber(
        "@ros2_lv/**", lambda s: tokens.add(str(s.key_expr)) if s.kind == zenoh.SampleKind.PUT
        else tokens.discard(str(s.key_expr)), history=True)
    a.ros.prog = "xblick-x"
    camera = CameraPublisher(a, "rostest/one/color", encoding="mono8", frame_id="rostest_one")
    params = Parameters(a, "boxer", "rostest/one")
    boxes = DetectionPublisher(a, "rostest/one/boxer/boxes", Detection2DArray)
    more = Parameters(a, "other", "rostest/one")
    try:
        assert a.ros.node.fully_qualified == "/rostest/one/boxer"
        assert "in ROS 2 as node /rostest/one/boxer (the name of its parameters)" in capsys.readouterr().err
        time.sleep(0.5)
        parsed = [ros.parse_token(t) for t in set(tokens)]
        assert {t.fully_qualified_node for t in parsed if t.kind == ros.NODE} == {"/rostest/one/boxer",
                                                                                   "/rostest/one/other"}
        by_node = {(t.topic, t.fully_qualified_node) for t in parsed if t.kind != ros.NODE}
        for topic in ("/rostest/one/color/image_raw", "/rostest/one/boxer/boxes", "/rostest/one/boxer/get_parameters"):
            assert (topic, "/rostest/one/boxer") in by_node, topic
        assert ("/rostest/one/other/get_parameters", "/rostest/one/other") in by_node
        assert not [t for t in parsed if t.node == "xblick_x"]  # the old name's tokens are gone
        assert camera._ros_raw.gid == ros.gid(camera._ros_raw.token.key())
    finally:
        for p in (more, boxes, params, camera):
            p.close()
        live.undeclare()
        observer.close()
        a.close()


def test_a_second_node_of_one_name_is_one_warning_line(caplog):
    endpoint = f"tcp/127.0.0.1:{free_port()}"
    a = session.open_session(listen=[endpoint], ros_options=session.RosOptions(DOMAIN, "xblick_same"))
    b = session.open_session(connect=[endpoint], ros_options=session.RosOptions(DOMAIN, "xblick_same"))
    try:
        wait_until(lambda: len([r for r in caplog.records if "ROS node /xblick_same is in ROS's graph already"
                                in r.getMessage()]) == 2)
        time.sleep(0.3)
        assert len([r for r in caplog.records if "/xblick_same is in ROS's graph" in r.getMessage()]) == 2
    finally:
        b.close()
        a.close()


def test_keys_ros_does_not_take_are_left_out_with_a_warning(caplog):
    """Our names are ROS names already (topics.check_name); a key that is not (a recording's, played) is not offered
    to ROS: one warning line, no error, so that it does not stop a program that did not ask for ROS."""
    from xblick.camera import cdr_encoding
    from xblick.msg import JointState

    with session.open_session(local_router=False, ros_options=session.RosOptions(DOMAIN, "xblick_names")) as a:
        assert ros.mirror(a, "rostest/robot-1/joint_states", cdr_encoding(JointState)) is None
        assert any("rostest/robot-1/joint_states: not offered to ROS" in r.getMessage() for r in caplog.records)


def test_a_static_frame_another_program_sends_otherwise_is_one_error_line(caplog):
    """Two programs (sessions) send one child frame on /tf_static with different parents: each says it once. The same
    transform from both is no clash."""
    net = Network(with_ros=True)
    same = TransformStamped(Header(Time(1, 0), "world"), "rostest_shared", Transform(Vector3(1, 2, 3)))
    other = TransformStamped(Header(Time(1, 0), "map"), "rostest_clash", Transform(Vector3(1, 2, 3)))
    mine = TransformStamped(Header(Time(1, 0), "world"), "rostest_clash", Transform(Vector3(1, 2, 3)))
    a = StaticTransformPublisher(net.a, "rostest/cam1", [same, mine])
    time.sleep(0.5)
    b = StaticTransformPublisher(net.b, "rostest/cam2", [same, other])
    try:
        def said(frame):
            return [r for r in caplog.records if f"frame {frame!r} is sent by another program too" in r.getMessage()]
        wait_until(lambda: len(said("rostest_clash")) == 2)
        time.sleep(0.5)
        assert len(said("rostest_clash")) == 2 and not said("rostest_shared")
        assert "parent 'map' there, 'world' here" in said("rostest_clash")[0].getMessage() + \
            said("rostest_clash")[1].getMessage()
    finally:
        b.close()
        a.close()
        net.close()


def test_the_clock_key_goes_to_ros_clock():
    from xblick.camera import CountedPublisher, cdr_encoding
    from xblick.msg import Clock, JointState

    options = session.RosOptions(DOMAIN, "xblick_sim", clock="rostest/world/clock")
    with session.open_session(local_router=False, ros_options=options) as a:
        clock = CountedPublisher(a, "rostest/world/clock", cdr_encoding(Clock))
        other = CountedPublisher(a, "rostest/other/clock", cdr_encoding(Clock))
        try:
            assert clock._ros.topic == "/clock" and other._ros.topic == "/rostest/other/clock"
            with pytest.raises(ros.RosError, match="not rosgraph_msgs/msg/Clock"):
                CountedPublisher(a, "rostest/world/clock", cdr_encoding(JointState))
        finally:
            clock.close()
            other.close()


def test_a_program_without_parameters_answers_ros2_param_list():
    """A program node without parameters still serves the six services (none, every set refused), so that
    ``ros2 param list`` on it answers instead of waiting."""
    from xblick.parameters import ParameterClient

    net = Network(with_ros=True)
    client = ParameterClient(net.b, "xblick_test", timeout=3.0, ros=True)
    try:
        assert client.list() == []
        (refused,) = client.set({"x": 1})
        assert not refused.successful
    finally:
        client.close()
        net.close()


def test_after_its_parameters_close_the_node_answers_ros2_param_list_again():
    """The program's node, renamed after its parameters, serves the six services without parameters again once they
    are closed (and the program's next Parameters take them over)."""
    from xblick.parameters import ParameterClient

    net = Network(with_ros=True)
    a = session.open_session(connect=[net.endpoint], ros_options=session.RosOptions(
        DOMAIN, "xblick_probe", follow_parameters=True))
    params = Parameters(a, "probe", "rostest/cam")
    params.declare("threshold", 0.5)
    client = ParameterClient(net.b, "rostest/cam/probe", timeout=3.0, ros=True)
    try:
        assert client.list() == ["threshold"]
        params.close()
        end = time.monotonic() + 5.0
        while time.monotonic() < end and not (a.ros._no_parameters is not None and client.list() == []):
            time.sleep(0.1)
        assert client.list() == []
        again = Parameters(a, "probe", "rostest/cam")
        again.declare("threshold", 0.7)
        assert a.ros._no_parameters is None
        wait_until(lambda: client.list() == ["threshold"])
        again.close()
    finally:
        client.close()
        a.close()
        net.close()


def test_a_clock_key_the_program_never_publishes_is_one_warning(monkeypatch, caplog):
    monkeypatch.setattr(ros, "CLOCK_WAIT_S", 0.2)
    from xblick.camera import CountedPublisher, cdr_encoding
    from xblick.msg import Clock

    options = session.RosOptions(DOMAIN, "xblick_sim", clock="rostest/world/clock")
    with session.open_session(local_router=False, ros_options=options):
        time.sleep(0.4)
    assert [r for r in caplog.records if "--ros-clock rostest/world/clock: this program has published no" in
            r.getMessage()]
    caplog.clear()
    with session.open_session(local_router=False, ros_options=options) as a:
        clock = CountedPublisher(a, "rostest/world/clock", cdr_encoding(Clock))
        time.sleep(0.4)
        clock.close()
    assert not [r for r in caplog.records if "--ros-clock" in r.getMessage()]


@pytest.mark.parametrize("with_ros", [False, True])
def test_a_session_dropped_without_close_is_released(with_ros):
    """A Session a program drops without close() is released as without ROS: its tokens (declaration, ROS node) go,
    and its threads (declaration, the ROS node's empty parameter services, duplicate watch, graph) end."""
    import gc
    import threading

    endpoint = f"tcp/127.0.0.1:{free_port()}"
    observer = session.open_session(listen=[endpoint], declare=False, ros_options=False)
    before = set(threading.enumerate())
    options = session.RosOptions(DOMAIN, "xblick_dropped") if with_ros else False
    dropped = session.open_session(connect=[endpoint], ros_options=options)
    zid = str(dropped.info.zid())
    if with_ros:
        ImageSubscriber(dropped, "/dropped/image_raw", transport="ros").close()  # makes the graph too

    def tokens():  # its declaration's and its ROS node's (** matches no @ chunk: asked apart)
        return [str(r.ok.key_expr) for selector in ("**", "@ros2_lv/**")
                for r in session.replies(observer, selector, 1.0, liveliness=True)
                if r.ok is not None and zid in str(r.ok.key_expr)]

    try:
        wait_until(lambda: len(tokens()) >= (2 if with_ros else 1))
        del dropped
        gc.collect()
        wait_until(lambda: not tokens(), timeout=8.0)
        wait_until(lambda: not [t for t in threading.enumerate() if t not in before and t.is_alive()], timeout=8.0)
    finally:
        observer.close()
