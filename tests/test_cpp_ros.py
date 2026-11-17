"""C++ and Python with ``--ros`` against each other: the keys, names, type hashes, attachments, QoS strings and tokens
of cpp/include/xblick/ros.hpp byte for byte against xblick.ros (xblick-cpp-probe --ros-wire), the C++ programs'
option family, nothing of ROS on the wire without ``--ros``, and the C++ programs' ROS side seen by our Python ROS
readers. Live against ROS 2 Jazzy's own programs as tests/test_ros_live.py does (its router, rosprobe built with g++),
skipped without /opt/ros/jazzy.

Needs the C++ build: XBLICK_CPP_BUILD (as tests/test_cpp_interop.py; skipped without). The point cloud cases need
xblick-points in it (a build with XBLICK_POINTS) and a GPU."""

import json
import os
import re
import subprocess
import time

import numpy as np
import pytest

from xblick import _core, ros
from xblick.camera import (CameraPublisher, ImageSubscriber, StaticTransformSubscriber, TransformSubscriber)
from xblick.encodings import numpy_to_image
from xblick.msg import CameraInfo, Header, Parameter, ParameterValue, SetParametersRequest, Time
from xblick.parameters import READ_ONLY_ACCESS, ROS_SETS_OFF, ParameterClient
from xblick.points import PointCloudSubscriber
from xblick.session import RosOptions, open_session, replies

from test_cpp_interop import BUILD, H, W, checksum, cloud_pattern, first_frame, free_port, pattern, tool
from test_ros_parameters import ros_set
import test_ros_live as live

pytestmark = pytest.mark.skipif(not BUILD, reason="XBLICK_CPP_BUILD (the C++ build directory) is not set")

# rmw_zenoh_cpp 0.2.13's own bytes (tests/test_ros.py): the talker's /chatter publisher and one of its samples
TALKER_CHATTER = ("@ros2_lv/87/78cf7f7f723020cd89f943acf3eb2710/0/11/MP/%/%/talker/%chatter/std_msgs::msg::dds_::"
                  "String_/RIHS01_df668c740482bbd48fb39d76a70dfd4bd59db1288021743503259e948f6b1a18/::,7:,:,:,,")
CHATTER_ATTACHMENT = "0300000000000000dca5f02d39ace8181031b1bb10e2fda68e3b7018e216540748"
ZID = "78cf7f7f723020cd89f943acf3eb2710"

# ------------------------------------------------------------------------------------- byte for byte, no session


def cpp_wire(cases: list[list[str]], env: dict | None = None) -> list[str]:
    """What the C++ side makes of each case (xblick-cpp-probe --ros-wire), one line each."""
    text = "".join("\t".join(map(str, case)) + "\n" for case in cases)
    out = subprocess.run([tool("xblick-cpp-probe"), "--ros-wire"], input=text, capture_output=True, text=True,
                         timeout=60, check=True, env=env).stdout
    lines = out.split("\n")
    assert lines[-1] == "" and len(lines) == len(cases) + 1, out
    return lines[:-1]


def python_wire(case: list[str]) -> str:
    """What Python's xblick.ros makes of the same case."""
    op, *f = [str(x) for x in case]
    try:
        if op == "topic_of":
            return ros.topic_of(f[0])
        if op == "node_name_of":
            return ros.node_name_of(f[0])
        if op == "check_node_name":
            return ros.check_node_name(f[0])
        if op == "split_node":
            return "\t".join(ros.split_node(f[0]))
        if op == "dds_type":
            return ros.dds_type(f[0])
        if op == "data_key":
            return ros.data_key(int(f[0]), f[1], f[2], ros.hash_of(f[2]))
        if op == "hashes":
            return ";".join(f"{t}={h}" for t, h in sorted(ros.TYPE_HASHES.items()))
        if op == "attachment":
            return ros.encode_attachment(int(f[0]), int(f[1]), bytes.fromhex(f[2])).hex()
        if op == "decode_attachment":
            seq, stamp, gid = ros.decode_attachment(bytes.fromhex(f[0]))
            return f"{seq}\t{stamp}\t{gid.hex()}"
        if op == "token":
            return ros.parse_token(f[0]).key()
        if op == "node_token":
            return ros.Token(int(f[0]), f[1], f[2], f[2], ros.NODE, "/", f[3], ros.check_node_name(f[4])).key()
        if op == "entity":
            domain, zid, node_id, entity_id, kind, ns, node, name, type_name, r, d, h, depth = f
            qos = ros.Qos(reliability=int(r), durability=int(d), history=int(h), depth=int(depth))
            token = ros.Token(int(domain), zid, node_id, entity_id, kind, "/", ns, node, ros.check_ros_name(name),
                              type_name, ros.hash_of(type_name), qos)
            if qos.reliability == ros.BEST_EFFORT:
                return "ERROR"  # the C++ API offers no best-effort entity (zenoh-c sets reliability unstably only)
            return f"{token.key()}\t{ros.data_key(int(domain), name, type_name, token.type_hash)}"
        if op == "qos":
            return ros.Qos.parse(f[0]).key()
        if op == "commanding":
            return "1" if ros.commanding(f[0]) else "0"
        if op == "domain":
            return str(ros.domain_from(int(f[0]) if f[0] else None))
    except ros.RosError:
        return "ERROR"
    raise AssertionError(f"no case {op}")


TYPES = sorted(ros.TYPE_HASHES)
CASES = (
    [["topic_of", k] for k in ("test/camera/color/image_raw", "/a/b/", "rc-visard/color", "test/3d/points", "a//b",
                               "x", "_x/y_1/z")]
    + [["node_name_of", p] for p in ("xblick test-camera", "3d", "xblick-cpp-camera", "", "ünï", "a.b c")]
    + [["check_node_name", n] for n in ("ok_name", "1x", "a-b", "", "_x")]
    + [["split_node", n] for n in ("/test/camera/boxer", "/talker", "talker", "/", "/a//b")]
    + [["dds_type", t] for t in TYPES + ["sensor_msgs/Image", "pkg/act/X", "a/msg/b/c"]]
    + [["data_key", 87, name, t] for name in ("/chatter", "/a/b/c", "chatter", "/a-b") for t in
       ("std_msgs/msg/String", "rcl_interfaces/srv/GetParameters", "xblick_msgs/msg/ShmImage")]
    + [["hashes"]]
    + [["attachment", seq, ns, gid] for seq, ns, gid in ((1, 0, "00" * 16), (3, 1_763_000_000_123_456_789, "ab" * 16),
                                                          (-1, -5, "0f" * 16), (2**63 - 1, 2**62, "11" * 16))]
    + [["decode_attachment", a] for a in (CHATTER_ATTACHMENT, CHATTER_ATTACHMENT[:-2], "",
                                          CHATTER_ATTACHMENT[:32] + "0f" + CHATTER_ATTACHMENT[34:])]
    + [["token", t] for t in (TALKER_CHATTER, TALKER_CHATTER.replace("/MP/", "/XX/"), TALKER_CHATTER + "/x",
                              f"@ros2_lv/87/{ZID}/0/0/NN/%/%/talker", f"@ros2_lv/x/{ZID}/0/0/NN/%/%/talker")]
    + [["node_token", 87, ZID, 0, ns, n] for ns, n in (("/", "xblick_cpp_camera"), ("/test/camera", "boxer"),
                                                       ("/", "bad-name"))]
    + [["entity", 93, ZID, 0, e, kind, ns, node, name, t, r, d, h, depth]
       for e, kind, ns, node, name, t, r, d, h, depth in (
           (1, "MP", "/", "xblick_cpp_camera", "/t/cpp/color/image_raw", "sensor_msgs/msg/Image", 1, 2, 1, 1),
           (2, "MP", "/", "xblick_cpp_camera", "/tf_static", "tf2_msgs/msg/TFMessage", 1, 1, 1, 1),
           (3, "MP", "/", "xblick_cpp_camera", "/tf", "tf2_msgs/msg/TFMessage", 1, 2, 1, 100),
           (4, "SS", "/t/cpp", "cpp_camera", "/t/cpp/cpp_camera/get_parameters", "rcl_interfaces/srv/GetParameters",
            1, 2, 1, 1000),
           (5, "MS", "/", "probe", "/roscam/image_raw", "sensor_msgs/msg/Image", 1, 2, 1, 1),
           (6, "MP", "/", "probe", "/x", "sensor_msgs/msg/Image", 2, 2, 1, 5),
           (7, "MP", "/", "probe", "/x", "xblick_msgs/msg/ShmImage", 1, 2, 1, 1),
           (8, "MP", "/", "probe", "x-y", "sensor_msgs/msg/Image", 1, 2, 1, 1))]
    + [["qos", q] for q in ("::,7:,:,:,,", ":1:,1000:,:10,0:,,", ":1:,1:,:,:,,", "::,:,:,:,,", ":7:,:,:,,", "x::,1:,:,:,,",
                            "::,1:,:,:,,,", "")]
    + [["domain", d] for d in ("", "0", "7", "-1")]
)


def test_cpp_ros_wire_is_pythons():
    """Every case gives the same bytes in C++ and Python (a refusal: ERROR on both sides)."""
    env = {k: v for k, v in os.environ.items() if k != "ROS_DOMAIN_ID"}
    got = cpp_wire(CASES, env)
    mismatches = [(case, mine, python_wire(case)) for case, mine in zip(CASES, got) if mine != python_wire(case)]
    assert not mismatches, mismatches


@pytest.mark.parametrize("value", ["42", " 5 ", "abc", "-3", ""])
def test_cpp_domain_from_the_environment_is_pythons(value, monkeypatch):
    monkeypatch.setenv("ROS_DOMAIN_ID", value)
    (got,) = cpp_wire([["domain", ""]], dict(os.environ))
    assert got == python_wire(["domain", ""])


def test_cpp_gid_is_rmw_zenohs():
    """The C++ gid (XXH3-128 of the token, ros_gid.hpp) is the one rmw_zenoh put into the talker's samples, and
    _core.ros_gid and Python's ros.gid are the same function."""
    gid = bytes.fromhex(CHATTER_ATTACHMENT)[17:].hex()
    (got,) = cpp_wire([["gid", TALKER_CHATTER]])
    assert got == gid == _core.ros_gid(TALKER_CHATTER).hex() == ros.gid(TALKER_CHATTER).hex()


def test_cpp_commanding_is_pythons():
    if not hasattr(ros, "commanding"):
        pytest.skip("xblick.ros has no commanding yet")
    cases = [["commanding", k] for k in ("robot/controller/arm/command", "robot/command/joints", "/robot/command/",
                                         "robot/commands/x", "test/camera/color/image_raw", "command")]
    assert cpp_wire(cases) == [python_wire(c) for c in cases]


# ------------------------------------------------------------------------------- C++ programs with and without --ros

DOMAIN = 92
BASE = "rc/cpp/color"


class CppCamera:
    """xblick-cpp-camera listening on a free port (camera tool arguments ``extra``), and an observer session that sees
    every key it sends (its subscriber on ``**`` also makes every publisher, ROS's too, send) and every ROS token."""

    def __init__(self, tmp_path, *extra):
        self.port = free_port()
        self.log = tmp_path / f"camera-{self.port}.log"
        with open(self.log, "w") as log:
            self.proc = subprocess.Popen(
                [tool("xblick-cpp-camera"), "--listen", f"127.0.0.1:{self.port}", "--base", BASE, "--encoding", "rgb8",
                 "--size", str(W), str(H), "--fps", "20", "--seconds", "60", *map(str, extra)],
                stdout=subprocess.PIPE, stderr=log, text=True)
        assert self.proc.stdout.readline().strip() == "ready", self.log.read_text()
        self.observer = open_session(connect=[f"tcp/127.0.0.1:{self.port}"], declare=False)
        self.keys: dict[str, bytes | None] = {}
        self.tokens: set[str] = set()
        self._sub = self.observer.declare_subscriber(
            "**", lambda s: self.keys.setdefault(str(s.key_expr), s.attachment.to_bytes() if s.attachment else None))
        self._live = self.observer.liveliness().declare_subscriber(
            "@ros2_lv/**", lambda s: self.tokens.add(str(s.key_expr)), history=True)

    def ros_session(self, node="py_reader"):
        return open_session(connect=[f"tcp/127.0.0.1:{self.port}"], ros_options=RosOptions(DOMAIN, node))

    def close(self):
        self._sub.undeclare()
        self._live.undeclare()
        self.observer.close()
        self.proc.terminate()
        self.proc.wait(10)


def wait_until(condition, timeout=10.0):
    end = time.monotonic() + timeout
    while not condition():
        if time.monotonic() > end:
            raise TimeoutError
        time.sleep(0.05)


def ours(keys) -> set[str]:
    return {k for k in keys if "::dds_::" not in k and not k[:1].isdigit()}


def test_cpp_without_ros_nothing_of_ros_goes_on_the_wire(tmp_path):
    """Without --ros a C++ program sends what it sent before: our keys only, no rmw_zenoh key, no ROS token; with
    --ros its own keys are the same."""
    seen = {}
    for flag in ([], ["--ros", "--ros-domain", DOMAIN]):
        camera = CppCamera(tmp_path, "--tf", "--diagnostics", "--params", *flag)
        try:
            wait_until(lambda: f"{BASE}/image_raw" in camera.keys and "rc/cpp/tf" in camera.keys)
            time.sleep(1.5)
            seen[bool(flag)] = (dict(camera.keys), set(camera.tokens))
        finally:
            camera.close()
    keys, tokens = seen[False]
    assert ours(keys) == set(keys) and tokens == set()
    assert ours(seen[True][0]) == set(keys)  # our keys unchanged by --ros
    assert set(seen[True][0]) - set(keys) and seen[True][1]  # and ROS's beside them


def test_cpp_with_ros_rmw_zenoh_keys_tokens_and_attachments(tmp_path):
    camera = CppCamera(tmp_path, "--tf", "--diagnostics", "--ros", "--ros-domain", DOMAIN)
    try:
        topics = {"/rc/cpp/color/image_raw": "sensor_msgs/msg/Image",
                  "/rc/cpp/color/camera_info": "sensor_msgs/msg/CameraInfo",
                  "/rc/cpp/color/image_raw/compressed": "sensor_msgs/msg/CompressedImage",
                  "/tf": "tf2_msgs/msg/TFMessage", "/rc/cpp/diagnostics": "diagnostic_msgs/msg/DiagnosticArray"}
        data = {ros.data_key(DOMAIN, t, typ, ros.hash_of(typ)): t for t, typ in topics.items()}
        wait_until(lambda: set(data) <= set(camera.keys))
        parsed = [ros.parse_token(t) for t in camera.tokens]
        (node,) = [t for t in parsed if t.kind == ros.NODE]
        assert (node.domain, node.namespace, node.node, node.node_id) == (DOMAIN, "/", "xblick_cpp_camera", "0")
        publishers = {t.topic: t for t in parsed if t.kind == ros.PUBLISHER}
        assert set(topics) | {"/tf_static"} <= set(publishers)
        assert publishers["/rc/cpp/color/image_raw"].qos == ros.STREAM_QOS
        assert publishers["/tf"].qos == ros.TF_QOS and publishers["/tf_static"].qos == ros.TF_STATIC_QOS
        for key, topic in data.items():
            seq, stamp, gid = ros.decode_attachment(camera.keys[key])
            assert seq >= 1 and abs(stamp - time.time_ns()) < 60e9
            assert gid == _core.ros_gid(publishers[topic].key())  # rmw_zenoh's gid of the publisher's token
        assert all(t.zid == node.zid and t.node_id == node.node_id for t in parsed)
        assert "xblick-cpp-camera: in ROS 2 as node /xblick_cpp_camera on domain 92" in camera.log.read_text()
    finally:
        camera.close()


def test_cpp_camera_read_through_ros_by_python(tmp_path):
    """Our Python ROS readers read the C++ camera's ROS topics: image_raw with camera_info (the image's header),
    image_raw/compressed is there for ROS, /tf, and /tf_static published before the reader started."""
    camera = CppCamera(tmp_path, "--tf", "--ros", "--ros-domain", DOMAIN)
    reader = camera.ros_session()
    try:
        time.sleep(1.5)  # /tf_static was put at the start: a late reader gets it from the publisher's cache
        fixed, moving = [], []
        static = StaticTransformSubscriber(reader, "unused", fixed.append, ros=True)
        tf = TransformSubscriber(reader, "unused", moving.append, ros=True)
        images = ImageSubscriber(reader, BASE, transport="ros", camera_info=True)
        frame = first_frame(images)
        expected = pattern("rgb8", int(frame.image[0, 0, 0]))  # the pattern's seq mod 256 is its first byte
        np.testing.assert_array_equal(frame.image, expected)
        assert frame.seq is None and frame.header.frame_id == "TestCamera"
        wait_until(lambda: images.camera_info is not None and fixed and moving)
        assert images.camera_info.width == W and images.camera_info.header.frame_id == "TestCamera"
        poses = [t for m in fixed for t in m.transforms]  # once or twice (ros.hpp: the cache's stand-in)
        pose = poses[0]
        assert 1 <= len(poses) <= 2 and all(p == pose for p in poses)
        assert (pose.header.frame_id, pose.child_frame_id, pose.transform.translation.z) == ("world", "TestCamera", 3.0)
        assert moving[-1].transforms[0].child_frame_id == "TestCamera_moving"
        for s in (images, static, tf):
            s.close()
    finally:
        reader.close()
        camera.close()


def test_cpp_probe_reads_a_ros_topic(tmp_path):
    """xblick-cpp-probe --transport ros (ImageSubscriber, Transport::Ros) reads a Python publisher's ROS topic and
    its camera_info."""
    port = free_port()
    with open_session(listen=[f"tcp/127.0.0.1:{port}"], ros_options=RosOptions(DOMAIN, "py_camera")) as session:
        camera = CameraPublisher(session, "rc/py/color", encoding="rgb8", frame_id="py_cam",
                                 camera_info=CameraInfo(height=H, width=W, k=[400.0, 0, 0, 0, 400.0, 0, 0, 0, 1]))
        proc = subprocess.Popen([tool("xblick-cpp-probe"), "--connect", f"127.0.0.1:{port}", "--base", "rc/py/color",
                                 "--transport", "ros", "--camera-info", "--frames", "3", "--timeout", "20", "--ros",
                                 "--ros-domain", str(DOMAIN)], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        seq = 0
        while proc.poll() is None:
            seq += 1
            camera.publish(pattern("rgb8", seq))
            time.sleep(0.05)
        out, err = proc.communicate()
        camera.close()
    lines = [json.loads(line) for line in out.splitlines()]
    assert proc.returncode == 0 and len(lines) == 3, err
    for line in lines:
        assert line["seq"] == -1 and line["transport"] == "ros" and line["encoding"] == "rgb8"
        assert 1 <= _seq_of(line) <= seq  # the pixels of one of the frames published
    assert any(line["camera_info"] == {"width": W, "fx": 400.0} for line in lines)


def _seq_of(line) -> int:
    """The frame of a probe line: the published patterns' checksums are distinct for seq 1..255."""
    for seq in range(1, 256):
        if checksum(pattern("rgb8", seq)) == line["checksum"]:
            return seq
    raise AssertionError(f"no pattern has checksum {line['checksum']}")


def test_cpp_points_through_ros(tmp_path):
    """A C++ PointCloudPublisher with --ros offers the plain cloud on /<base>/cloud; our reader with transport ros
    reads it."""
    port = free_port()
    proc = subprocess.Popen([tool("xblick-cpp-camera"), "--listen", f"127.0.0.1:{port}", "--points", "--base",
                             "rc/cpp/points", "--size", str(W), str(H), "--fps", "20", "--seconds", "60", "--ros",
                             "--ros-domain", str(DOMAIN)], stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
    try:
        assert proc.stdout.readline().strip() == "ready"
        with open_session(connect=[f"tcp/127.0.0.1:{port}"], ros_options=RosOptions(DOMAIN, "py_points")) as s:
            clouds = PointCloudSubscriber(s, "rc/cpp/points", transport="ros")
            end = time.monotonic() + 15
            got = None
            while got is None and time.monotonic() < end:
                got = clouds.latest(timeout=0.1)
            clouds.close()
        assert got is not None and got.seq is None
        data = got.points.tobytes()
        assert data == cloud_pattern(data[0]).tobytes()
    finally:
        proc.terminate()
        proc.wait(10)


def test_cpp_ros_options_refused():
    for argv in (["--ros-domain", "3"], ["--ros-node", "x"], ["--ros-params-writable"], ["--ros", "--ros-node", "a-b"],
                 ["--ros", "--ros-domain", "-1"]):
        done = subprocess.run([tool("xblick-cpp-camera"), "--listen", f"127.0.0.1:{free_port()}", "--seconds", "1",
                               *argv], capture_output=True, text=True, timeout=30)
        assert done.returncode == 1 and "xblick-cpp-camera:" in done.stderr, (argv, done.stderr)


# ------------------------------------------------------------------- C++ Parameters as a ROS node's (command path)

NODE = "rc/cpp/cpp_camera"  # xblick-cpp-camera --params: fps, pattern.shift (refuses 13), pattern.auto_shift, codec


def test_cpp_parameters_are_a_ros_node(tmp_path):
    """The C++ node's parameters: the ROS node /rc/cpp/cpp_camera with the six services (rcl_interfaces, QoS of
    ROS's parameters), read through ROS's keys by our ROS client; sets refused by default with Python's reason."""
    camera = CppCamera(tmp_path, "--params", "--ros", "--ros-domain", DOMAIN)
    reader = camera.ros_session()
    client = ParameterClient(reader, NODE, timeout=5.0, ros=True)
    try:
        wait_until(lambda: sum(ros.parse_token(t).kind == ros.SERVICE for t in camera.tokens) == 6)
        parsed = [ros.parse_token(t) for t in camera.tokens]
        services = {t.topic: t for t in parsed if t.kind == ros.SERVICE}
        assert set(services) == {f"/{NODE}/{s}" for s in ("list_parameters", "describe_parameters", "get_parameters",
                                                          "get_parameter_types", "set_parameters",
                                                          "set_parameters_atomically")}
        assert all(t.qos == ros.PARAMETERS_QOS and t.type_name.startswith("rcl_interfaces/srv/")
                   for t in services.values())
        assert any(t.kind == ros.NODE and t.fully_qualified_node == f"/{NODE}" for t in parsed)
        assert sorted(client.list()) == ["codec", "fps", "pattern.auto_shift", "pattern.shift"]
        assert [v.value() for v in client.get(["fps", "codec"])] == [20.0, "jpeg"]
        assert client.describe(["codec"])[0].read_only
        (result,) = client.set({"fps": 10.0})
        assert not result.successful and result.reason == ROS_SETS_OFF
        assert not client.set_atomically({"fps": 10.0}).successful
        assert ParameterClient(camera.observer, NODE).get(["fps"])[0].value() == 20.0  # nothing changed
    finally:
        client.close()
        reader.close()
        camera.close()


def test_cpp_parameters_writable_through_ros(tmp_path):
    """With --ros-params-writable a ROS client's sets are executed (unchecked: no node, no deadline), each
    parameter's checks and its writer apply, and the change is an event on ROS's /parameter_events."""
    camera = CppCamera(tmp_path, "--params", "--ros", "--ros-domain", DOMAIN, "--ros-params-writable")
    reader = camera.ros_session()
    client = ParameterClient(reader, NODE, timeout=5.0, ros=True)
    events = []
    client.on_event(events.append)
    ours = ParameterClient(camera.observer, NODE)
    try:
        (result,) = client.set({"fps": 10.0})
        assert result.successful and not client.last_checked
        assert ours.get(["fps"])[0].value() == 10.0
        (refused,) = client.set({"pattern.shift": 13})
        assert not refused.successful and refused.reason == "the camera refused 13"
        (refused,) = client.set({"codec": "png"})
        assert not refused.successful and "read-only" in refused.reason
        assert not client.set_atomically({"fps": 200.0}).successful and ours.get(["fps"])[0].value() == 10.0
        wait_until(lambda: any(p.name == "fps" for e in events for p in e.changed_parameters))
    finally:
        client.close()
        reader.close()
        camera.close()


def test_cpp_read_only_access_holds_on_the_ros_path(tmp_path):
    camera = CppCamera(tmp_path, "--params", "--params-read-only", "--ros", "--ros-domain", DOMAIN,
                       "--ros-params-writable")
    reader = camera.ros_session()
    try:
        time.sleep(1.0)
        (answer,) = ros_set(reader, NODE, "fps", 10.0, timeout=5.0)
        assert not answer.results[0].successful and answer.results[0].reason == READ_ONLY_ACCESS
        assert ParameterClient(camera.observer, NODE).get(["fps"])[0].value() == 20.0
    finally:
        reader.close()
        camera.close()


def test_cpp_malformed_ros_requests_are_refused_and_it_goes_on(tmp_path):
    """A ROS call without ROS's attachment, with CDR that does not decode, or with an empty payload: one error reply
    each, nothing changed; the node answers afterwards on both paths."""
    camera = CppCamera(tmp_path, "--params", "--ros", "--ros-domain", DOMAIN, "--ros-params-writable")
    reader = camera.ros_session()
    client = reader.ros.node.client(f"/{NODE}/set_parameters", "rcl_interfaces/srv/SetParameters")
    try:
        wait_until(lambda: client.matched)
        good = client.request_options(SetParametersRequest([Parameter("fps", ParameterValue.of(10.0))]).serialize())
        no_attachment = {k: v for k, v in good.items() if k != "attachment"}
        bad_cdr = dict(client.request_options(b"\x00\x01\x00\x00\xff"))
        for options in (no_attachment, bad_cdr, {**client.request_options(b""), "payload": b""}):
            answers = replies(reader, client.key, 2.0, **options)
            assert [r.err is not None for r in answers] == [True], answers
        assert ParameterClient(camera.observer, NODE).get(["fps"])[0].value() == 20.0
        assert ParameterClient(reader, NODE, timeout=5.0, ros=True).get(["fps"])[0].value() == 20.0
        assert camera.proc.poll() is None
    finally:
        client.close()
        reader.close()
        camera.close()


# ---------------------------------------------------------------- live: against ROS 2 Jazzy's programs (rmw_zenoh)

JAZZY, LIVE_DOMAIN, run = live.JAZZY, live.DOMAIN, live.run
ros_env, rosprobe = live.ros_env, live.rosprobe  # its fixtures: ROS's router on a free port, rosprobe built with g++

needs_jazzy = pytest.mark.skipif(not (JAZZY / "lib/rmw_zenoh_cpp/rmw_zenohd").exists(),
                                 reason="needs ROS 2 Jazzy with rmw_zenoh_cpp in /opt/ros/jazzy")


class CppOnRouter:
    """A C++ program with --ros on ROS's router (rmw_zenohd of ros_env), ended by SIGTERM."""

    def __init__(self, port: int, program: str, *args, ready: bool = True):
        self.proc = subprocess.Popen([tool(program), "--connect", f"127.0.0.1:{port}", "--ros", "--ros-domain",
                                      str(LIVE_DOMAIN), *map(str, args)], stdout=subprocess.PIPE,
                                     stderr=subprocess.DEVNULL, text=True)
        if ready:
            assert self.proc.stdout.readline().strip() == "ready"

    def close(self):
        if self.proc.poll() is None:
            self.proc.terminate()
        self.proc.wait(10)


def camera_args(*extra):
    return ["--base", "live/cpp/color", "--encoding", "rgb8", "--size", W, H, "--fps", "20", "--seconds", "90", *extra]


@needs_jazzy
def test_rclcpp_reads_the_cpp_camera(ros_env, rosprobe):
    env, port = ros_env
    camera = CppOnRouter(port, "xblick-cpp-camera", *camera_args())
    try:
        # camera_info goes out with the frames, which the camera renders while somebody takes images (subscribed)
        reading = subprocess.Popen([str(rosprobe), "sub", "/live/cpp/color/image_raw", "sensor_msgs/msg/Image", "15"],
                                   env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        info = run(env, [str(rosprobe), "sub", "/live/cpp/color/camera_info", "sensor_msgs/msg/CameraInfo", "2"], 30)
        jpeg = run(env, [str(rosprobe), "sub", "/live/cpp/color/image_raw/compressed",
                         "sensor_msgs/msg/CompressedImage", "2"], 30)
        image, _ = reading.communicate(timeout=60)
    finally:
        if reading.poll() is None:
            reading.terminate()
            reading.wait(10)
        camera.close()
    totals = [int(re.search(r"TOTAL (\d+)", out)[1]) for out in (image, info, jpeg)]
    assert min(totals) > 3, (totals, image[-1000:])
    assert b"TestCamera".hex() in image  # the frame id in the bytes rclcpp got


@needs_jazzy
def test_cpp_tf_static_to_a_late_ros_listener(ros_env):
    """The C++ camera's /tf_static, put once at its start, reaches tf2_echo started later (the cache's stand-in)."""
    env, port = ros_env
    camera = CppOnRouter(port, "xblick-cpp-camera", *camera_args("--tf"))
    try:
        time.sleep(1.0)
        out = run(env, [str(JAZZY / "lib/tf2_ros/tf2_echo"), "world", "TestCamera"], 6)
    finally:
        camera.close()
    assert "Translation: [1.000, 2.000, 3.000]" in out, out[-2000:]


@needs_jazzy
@pytest.mark.parametrize("writable", [False, True])
def test_ros_parameter_client_on_the_cpp_node(ros_env, rosprobe, writable):
    env, port = ros_env
    camera = CppOnRouter(port, "xblick-cpp-camera",
                         *camera_args("--params", *(["--ros-params-writable"] if writable else [])))
    try:
        listed = run(env, [str(rosprobe), "params", "/live/cpp/cpp_camera", "list"], 30)
        got = run(env, [str(rosprobe), "params", "/live/cpp/cpp_camera", "get", "pattern.shift"], 30)
        set_shift = run(env, [str(rosprobe), "params", "/live/cpp/cpp_camera", "set", "pattern.shift", "42"], 30)
        set_13 = run(env, [str(rosprobe), "params", "/live/cpp/cpp_camera", "set", "pattern.shift", "13"], 30)
    finally:
        camera.close()
    assert "NAME fps" in listed and "NAME pattern.shift" in listed, listed[-2000:]
    assert "VALUE pattern.shift 0" in got, got[-2000:]
    if writable:
        assert "SET 1" in set_shift and "SET 0 the camera refused 13" in set_13, set_shift + set_13
    else:
        assert f"SET 0 {ROS_SETS_OFF}" in set_shift and "SET 0" in set_13, set_shift + set_13


@needs_jazzy
def test_cpp_probe_reads_a_ros_image(ros_env, rosprobe):
    env, port = ros_env
    picture = np.arange(4 * 6 * 3, dtype=np.uint8).reshape(4, 6, 3)
    payload = bytes(numpy_to_image(picture, "rgb8", Header(Time(5, 0), "ros_cam")).serialize())
    publisher = subprocess.Popen([str(rosprobe), "pub", "/live/roscam/image_raw", "sensor_msgs/msg/Image",
                                  payload.hex(), "60"], env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    probe = CppOnRouter(port, "xblick-cpp-probe", "--base", "live/roscam", "--transport", "ros", "--frames", "2",
                        "--timeout", "20", ready=False)
    try:
        out, _ = probe.proc.communicate(timeout=60)
    finally:
        probe.close()
        publisher.terminate()
        publisher.wait(10)
    lines = [json.loads(line) for line in out.splitlines()]
    assert probe.proc.returncode == 0 and len(lines) == 2, out
    assert all(line["checksum"] == checksum(picture) and line["width"] == 6 and line["seq"] == -1 for line in lines)
