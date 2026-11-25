"""xblick.ros: rmw_zenoh_cpp 0.2.13's keys, attachment, QoS strings and tokens against bytes captured from ROS 2
Jazzy's demo nodes (talker, listener, tf2_ros static_transform_publisher), the tokens of rmw_zenoh_cpp 0.10.6 (ROS 2
Lyrical) with their backends part, and the RIHS01 hash against ROS's own hashes: always against the table and
msgdefs.py, and, where ROS 2 Jazzy is installed in /opt/ros/jazzy, against every message and service type there."""

import json
import re
from pathlib import Path

import pytest

from xblick import ros
from xblick.msgdefs import MSGDEFS

JAZZY = Path("/opt/ros/jazzy/share")
needs_ros = pytest.mark.skipif(not JAZZY.is_dir(), reason="needs ROS 2 Jazzy in /opt/ros/jazzy")

# captured on domain 87 from rmw_zenoh_cpp 0.2.13
TALKER_CHATTER = ("@ros2_lv/87/78cf7f7f723020cd89f943acf3eb2710/0/11/MP/%/%/talker/%chatter/std_msgs::msg::dds_::"
                  "String_/RIHS01_df668c740482bbd48fb39d76a70dfd4bd59db1288021743503259e948f6b1a18/::,7:,:,:,,")
LISTENER_CHATTER = ("@ros2_lv/87/f54a4cdf922715d7d8be8d57bc314b9b/0/11/MS/%/%/listener/%chatter/std_msgs::msg::dds_::"
                    "String_/RIHS01_df668c740482bbd48fb39d76a70dfd4bd59db1288021743503259e948f6b1a18/::,10:,:,:,,")
TALKER_NODE = "@ros2_lv/87/78cf7f7f723020cd89f943acf3eb2710/0/0/NN/%/%/talker"
TALKER_ROSOUT = ("@ros2_lv/87/78cf7f7f723020cd89f943acf3eb2710/0/1/MP/%/%/talker/%rosout/rcl_interfaces::msg::dds_::"
                 "Log_/RIHS01_e28ce254ca8abc06abf92773b74602cdbf116ed34fbaf294fb9f81da9f318eac/:1:,1000:,:10,0:,,")
STATIC_TF = ("@ros2_lv/87/35014fbe89d585c32f08f47d73e62323/0/11/MP/%/%/static_transform_publisher_QjRTOJWJVsBx8onc/"
             "%tf_static/tf2_msgs::msg::dds_::TFMessage_/RIHS01_e369d0f05a23ae52508854b66f6aa0437f3449d652e8cbf22d5abe8"
             "5d020f087/:1:,1:,:,:,,")
TALKER_GET = ("@ros2_lv/87/78cf7f7f723020cd89f943acf3eb2710/0/2/SS/%/%/talker/%talker%get_parameters/rcl_interfaces::"
              "srv::dds_::GetParameters_/RIHS01_bf9803d5c74cf989a5de3e0c2e99444599a627c7ff75f97b8c05b01003675cbc/::,"
              "1000:,:,:,,")
# captured on domain 77 from rmw_zenoh_cpp 0.10.6 (ROS 2 Lyrical): image_tools' cam2image publisher and showimage
# subscription of an Image topic, and the gid in the attachment of a cam2image sample (XXH3-128 of the whole token)
LYRICAL_CAM2IMAGE = ("@ros2_lv/77/1b98790c445491809533e78cc3304fd5/0/10/MP/%/%/cam2image/%lyrg%image_raw/sensor_msgs::"
                     "msg::dds_::Image_/RIHS01_d31d41a9a4c4bc8eae9be757b0beed306564f7526c88ea6a4588fb9582527d47/"
                     "::2,10:,:,:,,/backends:cpu:")
LYRICAL_CAM2IMAGE_GID = "3d71c655915d7801d6885bcb18590281"
LYRICAL_SHOWIMAGE = ("@ros2_lv/77/16a25e588aeb51c11d8c2340714f13f/0/10/MS/%/%/showimage/%lyrg%image_raw/sensor_msgs::"
                     "msg::dds_::Image_/RIHS01_d31d41a9a4c4bc8eae9be757b0beed306564f7526c88ea6a4588fb9582527d47/"
                     "::2,10:,:,:,,/backends:cpu:")
CHATTER_KEY = ("87/chatter/std_msgs::msg::dds_::String_/"
               "RIHS01_df668c740482bbd48fb39d76a70dfd4bd59db1288021743503259e948f6b1a18")
CHATTER_ATTACHMENT = bytes.fromhex("0300000000000000dca5f02d39ace8181031b1bb10e2fda68e3b7018e216540748")
CHATTER_PAYLOAD = bytes.fromhex("000100000f00000048656c6c6f20576f726c643a203300")  # "Hello World: 3"


def test_data_key_of_a_topic_and_a_service():
    assert ros.data_key(87, "/chatter", "std_msgs/msg/String", ros.hash_of("std_msgs/msg/String")) == CHATTER_KEY
    assert ros.data_key(87, "/talker/get_parameters", "rcl_interfaces/srv/GetParameters",
                        ros.hash_of("rcl_interfaces/srv/GetParameters")) == (
        "87/talker/get_parameters/rcl_interfaces::srv::dds_::GetParameters_/" + ros.hash_of(
            "rcl_interfaces/srv/GetParameters"))


def test_type_names():
    assert ros.dds_type("sensor_msgs/msg/Image") == "sensor_msgs::msg::dds_::Image_"
    assert ros.dds_type("rcl_interfaces/srv/SetParameters") == "rcl_interfaces::srv::dds_::SetParameters_"
    assert ros.type_of_dds("tf2_msgs::msg::dds_::TFMessage_") == "tf2_msgs/msg/TFMessage"
    with pytest.raises(ros.RosError):
        ros.dds_type("sensor_msgs/Image")


def test_topic_of_a_key_and_names_ros_refuses():
    assert ros.topic_of("test/camera/color/image_raw") == "/test/camera/color/image_raw"
    for bad in ("rc-visard/color/image_raw", "test/3d/points", "a//b"):
        with pytest.raises(ros.RosError):
            ros.topic_of(bad)
    assert ros.node_name_of("xblick test-camera") == "xblick_test_camera"
    assert ros.node_name_of("3d") == "_3d"
    assert ros.split_node("/test/camera/boxer") == ("/test/camera", "boxer")
    assert ros.split_node("/talker") == ("/", "talker")


def test_attachment_of_a_talker_sample():
    seq, stamp, gid = ros.decode_attachment(CHATTER_ATTACHMENT)
    assert seq == 3 and stamp > 1_700_000_000 * 10**9 and len(gid) == 16
    assert ros.encode_attachment(seq, stamp, gid) == CHATTER_ATTACHMENT
    assert ros.gid(TALKER_CHATTER) == gid  # rmw_zenoh's gid of the publisher's token: XXH3-128
    for bad in (b"", CHATTER_ATTACHMENT[:-1], CHATTER_ATTACHMENT[:16] + b"\x0f" + CHATTER_ATTACHMENT[17:],
                b'{"v":1,"seq":1}'):
        with pytest.raises(ros.RosError):
            ros.decode_attachment(bad)


@pytest.mark.parametrize("key", [TALKER_CHATTER, LISTENER_CHATTER, TALKER_NODE, TALKER_ROSOUT, STATIC_TF, TALKER_GET,
                                 LYRICAL_CAM2IMAGE, LYRICAL_SHOWIMAGE])
def test_tokens_of_ros_nodes_read_and_written_back(key):
    token = ros.parse_token(key)
    assert token.key() == key


def test_token_parts():
    t = ros.parse_token(STATIC_TF)
    assert (t.domain, t.kind, t.namespace, t.topic, t.type_name) == (87, "MP", "/", "/tf_static",
                                                                      "tf2_msgs/msg/TFMessage")
    assert t.qos == ros.TF_STATIC_QOS
    assert ros.parse_token(TALKER_GET).topic == "/talker/get_parameters"
    assert ros.parse_token(TALKER_ROSOUT).qos.lifespan == ("10", "0")
    assert ros.parse_token(TALKER_NODE).fully_qualified_node == "/talker"
    for bad in ("@ros2_lv/87/x", TALKER_CHATTER.replace("/MP/", "/XX/"), TALKER_CHATTER + "/extra",
                TALKER_CHATTER.replace(":,7:", ":7:")):
        with pytest.raises(ros.RosError):
            ros.parse_token(bad)


def test_lyrical_tokens_with_backends():
    """rmw_zenoh 0.10.x's optional 14th part: read and kept (the gid hashes the whole token); any other 14th part, and
    a 15th, refused."""
    t = ros.parse_token(LYRICAL_CAM2IMAGE)
    assert (t.kind, t.type_name, t.backends) == ("MP", "sensor_msgs/msg/Image", "backends:cpu:")
    assert ros.parse_token(LYRICAL_SHOWIMAGE).backends == ros.CPU_BACKENDS
    assert ros.parse_token(TALKER_CHATTER).backends is None
    assert ros.gid(LYRICAL_CAM2IMAGE).hex() == LYRICAL_CAM2IMAGE_GID
    for bad in (LYRICAL_CAM2IMAGE.replace("/backends:", "/b:"), LYRICAL_CAM2IMAGE + "/backends:cpu:",
                TALKER_NODE + "/backends:cpu:"):
        with pytest.raises(ros.RosError):
            ros.parse_token(bad)


def test_buffer_types_say_cpu_backends():
    """A volatile subscription of a buffer type says backends:cpu: and reads <key>/_buf_cpu as well; transient local,
    publishers, services and other types say nothing; a message type of unknown buffer status is refused."""
    assert ros.backends_of(ros.SUBSCRIPTION, "sensor_msgs/msg/Image", ros.STREAM_QOS) == "backends:cpu:"
    assert ros.backends_of(ros.SUBSCRIPTION, "sensor_msgs/msg/PointCloud2", ros.Qos(depth=10)) == "backends:cpu:"
    assert ros.backends_of(ros.SUBSCRIPTION, "sensor_msgs/msg/CompressedImage", ros.STREAM_QOS) == "backends:cpu:"
    assert ros.backends_of(ros.SUBSCRIPTION, "sensor_msgs/msg/Image",
                           ros.Qos(durability=ros.TRANSIENT_LOCAL, depth=1)) is None
    assert ros.backends_of(ros.PUBLISHER, "sensor_msgs/msg/Image", ros.STREAM_QOS) is None
    assert ros.backends_of(ros.SERVICE, "rcl_interfaces/srv/GetParameters", ros.PARAMETERS_QOS) is None
    for plain in ("sensor_msgs/msg/CameraInfo", "tf2_msgs/msg/TFMessage", "rcl_interfaces/msg/ParameterEvent"):
        assert ros.backends_of(ros.SUBSCRIPTION, plain, ros.STREAM_QOS) is None
    with pytest.raises(ros.RosError, match="BUFFER_FIELDS"):
        ros.backends_of(ros.SUBSCRIPTION, "sensor_msgs/msg/Range", ros.STREAM_QOS)
    key = ros.data_key(77, "/lyrg/image_raw", "sensor_msgs/msg/Image", ros.hash_of("sensor_msgs/msg/Image"))
    assert ros.cpu_group_key(key) == key + "/_buf_cpu"


def _has_uint8_sequence(type_name: str, lookup) -> bool:
    """Lyrical's rule (rosidl_typesupport_fastrtps_c's has_buffer_fields): an unbounded uint8[] (also char[]) field of
    the type or of a type it uses."""
    full = ros._full(ros._individual(type_name, lookup(type_name)), lookup)
    described = [full["type_description"], *full["referenced_type_descriptions"]]
    return any(f["type"]["type_id"] == 3 + 144 for d in described for f in d["fields"])


def test_buffer_fields_holds_every_message_type():
    """Every message type offered to ROS has its buffer status; for those in msgdefs.py it follows Lyrical's rule."""
    assert set(ros.BUFFER_FIELDS) == {t for t in ros.TYPE_HASHES if "/msg/" in t}
    both = sorted(set(MSGDEFS) & set(ros.BUFFER_FIELDS))
    assert len(both) >= 14
    for t in both:
        assert _has_uint8_sequence(t, ros.msgdefs_lookup(t, MSGDEFS[t])) == ros.BUFFER_FIELDS[t], t


def test_qos_strings():
    assert ros.Qos(depth=7).key() == "::,7:,:,:,,"
    assert ros.TF_STATIC_QOS.key() == ":1:,1:,:,:,,"
    assert ros.PARAMETERS_QOS.key() == "::,1000:,:,:,,"
    assert ros.Qos().key() == "::,:,:,:,,"
    assert ros.Qos(reliability=ros.BEST_EFFORT, depth=5).key() == "2::,5:,:,:,,"
    assert ros.Qos.parse(":1:,1000:,:10,0:,,").key() == ":1:,1000:,:10,0:,,"


def test_domain_id(monkeypatch):
    monkeypatch.delenv("ROS_DOMAIN_ID", raising=False)
    assert ros.domain_from(None) == 0
    monkeypatch.setenv("ROS_DOMAIN_ID", "42")
    assert ros.domain_from(None) == 42
    assert ros.domain_from("7") == 7
    for bad in ("-1", "x"):
        with pytest.raises(ros.RosError):
            ros.domain_from(bad)
    monkeypatch.setenv("ROS_DOMAIN_ID", "abc")
    with pytest.raises(ros.RosError, match="ROS_DOMAIN_ID"):
        ros.domain_from(None)


def test_hash_of_msgdefs_types_is_the_table():
    """The hash implementation over msgdefs.py gives the table's hash of every type both have (ROS's own hashes for
    the types of a Jazzy install, the computed ones for vision_msgs)."""
    both = sorted(set(MSGDEFS) & set(ros.TYPE_HASHES))
    assert len(both) >= 14
    for t in both:
        assert ros.message_hash(t, ros.msgdefs_lookup(t, MSGDEFS[t])) == ros.TYPE_HASHES[t], t


def test_unknown_type_is_not_offered():
    with pytest.raises(ros.RosError, match="not offered to ROS"):
        ros.hash_of("xblick_msgs/msg/ShmImage")


def _jazzy_lookup(name: str) -> str:
    pkg, kind, typ = name.split("/")
    return (JAZZY / pkg / kind / f"{typ}.msg").read_text()


def _ros_hash(path: Path) -> str:
    name = "/".join(path.with_suffix("").parts[-3:])
    data = json.loads(path.read_text())
    return next(h["hash_string"] for h in data["type_hashes"] if h["type_name"] == name)


@needs_ros
def test_table_holds_ros_own_hashes():
    for t, h in ros.TYPE_HASHES.items():
        path = JAZZY / f"{t}.json"
        if not t.startswith("vision_msgs/"):
            assert _ros_hash(path) == h, t


@needs_ros
def test_buffer_fields_by_the_definitions_of_ros():
    for t, buffer in ros.BUFFER_FIELDS.items():
        if not t.startswith("vision_msgs/"):
            assert _has_uint8_sequence(t, _jazzy_lookup) == buffer, t


@needs_ros
def test_hash_of_every_installed_message_and_service():
    """rosidl's algorithm, reimplemented, against every .msg and .srv of a ROS 2 Jazzy install with its .json."""
    checked = 0
    for path in sorted(JAZZY.glob("*/msg/*.msg")) + sorted(JAZZY.glob("*/srv/*.srv")):
        if not path.with_suffix(".json").exists():
            continue
        name = "/".join(path.with_suffix("").parts[-3:])
        if path.suffix == ".msg":
            got = ros.message_hash(name, _jazzy_lookup)
        else:
            request, response = re.split(r"^---[ \t]*$", path.read_text(), maxsplit=1, flags=re.M)
            got = ros.service_hash(name, request, response, _jazzy_lookup)
        assert got == _ros_hash(path.with_suffix(".json")), name
        checked += 1
    assert checked > 200  # 252 types in a ROS 2 Jazzy install with demo_nodes_cpp and tf2_ros (2026-11-17)
