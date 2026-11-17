"""C++ and Python with ``--ros`` against each other: the keys, names, type hashes, attachments, QoS strings and tokens
of cpp/include/xblick/ros.hpp byte for byte against xblick.ros (xblick-cpp-probe --ros-wire), the C++ programs'
option family, nothing of ROS on the wire without ``--ros``, and the C++ programs' ROS side seen by our Python ROS
readers. Live against ROS 2 Jazzy's own programs as tests/test_ros_live.py does (its router, rosprobe built with g++),
skipped without /opt/ros/jazzy.

Needs the C++ build: XBLICK_CPP_BUILD (as tests/test_cpp_interop.py; skipped without). The point cloud cases need
xblick-points in it (a build with XBLICK_POINTS) and a GPU."""

import os
import subprocess

import pytest

from xblick import _core, ros

from test_cpp_interop import BUILD, tool

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
    _core.ros_gid is the same function. Python's ros.gid is a SHA-256 prefix until it uses _core.ros_gid."""
    gid = bytes.fromhex(CHATTER_ATTACHMENT)[17:].hex()
    (got,) = cpp_wire([["gid", TALKER_CHATTER]])
    assert got == gid == _core.ros_gid(TALKER_CHATTER).hex()


def test_cpp_commanding_is_pythons():
    if not hasattr(ros, "commanding"):
        pytest.skip("xblick.ros has no commanding yet")
    cases = [["commanding", k] for k in ("robot/controller/arm/command", "robot/command/joints", "/robot/command/",
                                         "robot/commands/x", "test/camera/color/image_raw", "command")]
    assert cpp_wire(cases) == [python_wire(c) for c in cases]
