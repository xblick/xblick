"""ROS 2 on Zenoh: the keys, type names, type hashes, attachments, QoS strings and liveliness tokens of rmw_zenoh_cpp
0.2.13 (ROS 2 Jazzy) and 0.10.6 or newer (ROS 2 Lyrical), so that every program takes part in ROS 2 as well (by
default; ``--no-ros`` or ``XBLICK_ROS=0`` turn it off; docs/integrations/ros2.md). Durchblick's payloads are the
CDR bytes ROS 2 writes already (protocol.md); ROS needs them on its own keys, with its own attachment, and announces
every entity by a liveliness token.

* data key: ``<domain>/<topic without the leading />/<pkg>::msg::dds_::<Name>_/RIHS01_<type hash>``;
  a service's key the same with the service's name and ``<pkg>::srv::dds_::<Name>_`` and the service type's hash;
* attachment of every sample, service request and reply: int64 sequence number, int64 source timestamp (ns since
  the epoch), the gid (one length byte 16, 16 bytes), zenoh-ext's serialization, 33 bytes; a ROS subscriber drops a
  sample without it;
* liveliness token of a node: ``@ros2_lv/<domain>/<zid>/<node id>/<node id>/NN/<enclave>/<namespace>/<node>``; of a
  publisher (MP), subscription (MS), service (SS), client (SC): ``@ros2_lv/<domain>/<zid>/<node id>/<entity
  id>/<kind>/<enclave>/<namespace>/<node>/<topic>/<type>/RIHS01_<hash>/<qos>``, every ``/`` inside a name written
  as ``%`` (``/`` alone: ``%``); Lyrical adds an optional 14th part ``backends:<name>:<meta>;...`` (below);
* buffer types (Lyrical): a message type with a ``uint8[]`` field, its own or a nested type's (``BUFFER_FIELDS``),
  goes from a Lyrical publisher to a volatile subscription only on ``<key>/_buf_cpu`` (a subscription whose token
  says ``backends:cpu:``) and ``<key>/_buf/<subscription gid>`` (any other), never on the key itself while it knows
  every subscription. Our volatile subscriptions of such a type say ``backends:cpu:`` and take ``<key>/_buf_cpu``
  as well as the key; the bytes there are the same CDR with the same attachment. A publisher sends each message on
  the key or on these keys, never on both. Jazzy reads the 14th part and ignores it; a Lyrical subscription reads the
  key whatever the publisher's token says, so our publishers stay as they are;
* a transient-local publisher (``/tf_static``) is a zenoh-ext AdvancedPublisher with a cache of ``depth`` samples
  and publisher detection (and sample-miss detection with a sporadic heartbeat of 0.5 s when reliable); a
  transient-local subscription an AdvancedSubscriber that asks late-detected publishers for their history;
* a service is a queryable declared ``complete`` (a ROS client queries with target ALL_COMPLETE and reaches no other
  queryable); its reply carries the request's sequence number and client gid with the reply's own timestamp.

The gid: rmw_zenoh derives an entity's gid from its token key with XXH3-128 (low 64 bits, then high 64 bits, little
endian). ``gid`` does the same through ``_core.ros_gid`` (the C++ core's, cpp/include/xblick/ros_gid.hpp), so a
sample's gid is the one of its entity in ROS's graph.

Type hashes (RIHS01, REP 2011): ``TYPE_HASHES`` for every type Durchblick carries to ROS. All are ROS's own
(``share/<pkg>/msg/<Name>.json`` of ROS 2 Jazzy, vision_msgs those of ros-jazzy-vision-msgs 4.1.1). ``message_hash`` and
``service_hash`` are rosidl's algorithm, checked against every type of a ROS 2 Jazzy install by tests/test_ros.py.
"""

from __future__ import annotations

import dataclasses
import hashlib
import itertools
import json
import logging
import os
import re
import struct
import sys
import threading
import time
from collections.abc import Callable

import zenoh

log = logging.getLogger("xblick")

RMW_ZENOH = "0.2.13"           # the rmw_zenoh_cpp version this module follows (ROS 2 Jazzy)
ADMIN_SPACE = "@ros2_lv"       # the liveliness tokens' prefix
DOMAIN_ENV = "ROS_DOMAIN_ID"   # ROS's variable for the domain id
DEFAULT_DOMAIN = 0             # ROS's default domain id
GID_SIZE = 16
_ATTACHMENT = struct.Struct(f"<qqB{GID_SIZE}s")  # seq, source timestamp, length byte, gid
ATTACHMENT_SIZE = _ATTACHMENT.size              # 33
HEARTBEAT_S = 0.5  # sample-miss detection of a reliable transient-local publisher (rmw_zenoh: 500 ms, sporadic)

# rmw's QoS enums (rmw/types.h)
SYSTEM_DEFAULT, RELIABLE, BEST_EFFORT = 0, 1, 2
TRANSIENT_LOCAL, VOLATILE = 1, 2
KEEP_LAST, KEEP_ALL = 1, 2

CPU_BACKENDS = "backends:cpu:"  # the 14th token part of a subscription that takes buffer types as plain CDR
CPU_GROUP = "_buf_cpu"          # below a buffer type's key: what Lyrical's publishers send to such subscriptions

NODE, PUBLISHER, SUBSCRIPTION, SERVICE, CLIENT = "NN", "MP", "MS", "SS", "SC"
KINDS = (NODE, PUBLISHER, SUBSCRIPTION, SERVICE, CLIENT)

_TOKEN_NAME = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*$")
_TYPE_NAME = re.compile(r"^([A-Za-z_][A-Za-z0-9_]*)/(msg|srv)/([A-Za-z_][A-Za-z0-9_]*)$")


class RosError(ValueError):
    """A name, a type or a value that ROS 2 does not take."""


# --------------------------------------------------------------------------------------------------- names and keys


def check_ros_name(name: str, what: str = "topic") -> str:
    """``name`` as a fully qualified ROS name (``/a/b``): a leading ``/``, tokens of letters, digits and ``_`` that
    do not start with a digit, no empty token. RosError otherwise, saying why."""
    if not name.startswith("/") or name == "/":
        raise RosError(f"{what} {name!r} is no fully qualified ROS name (/a/b)")
    for token in name[1:].split("/"):
        if not _TOKEN_NAME.match(token):
            raise RosError(f"{what} {name!r} is no ROS name: {token!r} is not letters, digits and _ (not starting "
                           "with a digit), as ROS requires of each part")
    return name


def topic_of(key: str) -> str:
    """The ROS topic of a Durchblick key: ``test/camera/color/image_raw`` -> ``/test/camera/color/image_raw``
    (one rule, no namespace of its own; RosError for a key that is no ROS name, e.g. one with ``-``)."""
    return check_ros_name("/" + key.strip("/"), "the ROS topic of key")


def check_node_name(name: str) -> str:
    if not _TOKEN_NAME.match(name):
        raise RosError(f"ROS node name {name!r}: letters, digits and _, not starting with a digit")
    return name


def node_name_of(program: str) -> str:
    """The ROS node name of a program by default: its name with every character other than a letter, digit or ``_``
    written as ``_`` (``xblick cam`` -> ``xblick_cam``, ``xblick-yolox`` -> ``xblick_yolox``)."""
    name = re.sub(r"[^A-Za-z0-9_]", "_", program) or "xblick"
    return name if not name[0].isdigit() else "_" + name


def split_node(fully_qualified: str) -> tuple[str, str]:
    """``/a/b/c`` -> (``/a/b``, ``c``); ``/c`` -> (``/``, ``c``)."""
    check_ros_name(fully_qualified, "node")
    namespace, _, name = fully_qualified.rpartition("/")
    return namespace or "/", name


def namespace_of(name: str) -> str:
    """A Durchblick name (a camera, ``test/cam-1``) as a ROS namespace: each part as ``node_name_of`` writes it
    (``/test/cam_1``); ``/`` for an empty name."""
    parts = [node_name_of(p) for p in str(name).strip("/").split("/") if p]
    return "/" + "/".join(parts) if parts else "/"


def node_of(name: str) -> tuple[str, str]:
    """(namespace, node) of a Durchblick name as a ROS node, each part as ``node_name_of`` writes it:
    ``test/cam-1/probe`` -> (``/test/cam_1``, ``probe``). RosError for an empty name."""
    namespace, _, last = str(name).strip("/").rpartition("/")
    if not last:
        raise RosError(f"{name!r}: no node name")
    return namespace_of(namespace), node_name_of(last)


ENABLE_ENV = "XBLICK_ROS"  # 0: no part in ROS 2; 1 or unset: ROS 2 beside our own keys (the default)


def enabled_by_environment() -> bool:
    """False when ``XBLICK_ROS`` is 0, True when it is 1 or unset; RosError for any other value."""
    value = os.environ.get(ENABLE_ENV)
    if value is None or value == "1":
        return True
    if value == "0":
        return False
    raise RosError(f"{ENABLE_ENV}={value!r}: 0 (no part in ROS 2) or 1 (ROS 2 beside our own keys, the default)")


# what a reader of ROS's topics needs, said when it was opened without
NEEDS_ROS = ("a session that takes part in ROS 2 (the default of open_session and every program; not one opened with "
             "--no-ros, XBLICK_ROS=0 or ros_options=False, nor a plain zenoh.open)")


def offered_topic(key: str) -> str | None:
    """The ROS topic of one of our keys (``topic_of``), or None with one warning line for a key that is no ROS name
    (``-`` in it): such a key stays ours only, the program goes on."""
    try:
        return topic_of(key)
    except RosError as error:
        log.warning(f"{key}: not offered to ROS ({error})")
        return None


def dds_type(type_name: str) -> str:
    """``sensor_msgs/msg/Image`` -> ``sensor_msgs::msg::dds_::Image_`` (a service: ``pkg/srv/Name`` ->
    ``pkg::srv::dds_::Name_``), as rmw_zenoh writes types into keys and tokens."""
    m = _TYPE_NAME.match(type_name)
    if not m:
        raise RosError(f"type {type_name!r}: <package>/msg/<Name> or <package>/srv/<Name> expected")
    return f"{m[1]}::{m[2]}::dds_::{m[3]}_"


def type_of_dds(dds: str) -> str:
    """``sensor_msgs::msg::dds_::Image_`` -> ``sensor_msgs/msg/Image``."""
    m = re.match(r"^([A-Za-z0-9_]+)::(msg|srv)::dds_::([A-Za-z0-9_]+)_$", dds)
    if not m:
        raise RosError(f"{dds!r} is no rmw_zenoh type name")
    return f"{m[1]}/{m[2]}/{m[3]}"


def data_key(domain: int, name: str, type_name: str, type_hash: str) -> str:
    """The Zenoh key of a topic or service (``name`` fully qualified)."""
    check_ros_name(name)
    return f"{domain}/{name.strip('/')}/{dds_type(type_name)}/{type_hash}"


def mangle(name: str) -> str:
    return name.replace("/", "%")


def demangle(name: str) -> str:
    return name.replace("%", "/")


def domain_from(value) -> int:
    """A domain id given as an option, else ``ROS_DOMAIN_ID``, else 0. RosError for one that is no integer >= 0."""
    if value is None:
        value = os.environ.get(DOMAIN_ENV, "").strip() or DEFAULT_DOMAIN
        where = f"{DOMAIN_ENV}={value!r}"
    else:
        where = f"--ros-domain {value!r}"
    try:
        domain = int(value)
    except (TypeError, ValueError):
        raise RosError(f"{where}: a ROS domain id is an integer from 0") from None
    if domain < 0:
        raise RosError(f"{where}: a ROS domain id is an integer from 0")
    return domain


# ------------------------------------------------------------------------------------------------------ attachment


def encode_attachment(seq: int, source_ns: int, gid: bytes) -> bytes:
    """rmw_zenoh's attachment: zenoh-ext serializes int64, int64 and std::array<uint8_t, 16> (a length byte 16,
    then the bytes)."""
    if len(gid) != GID_SIZE:
        raise RosError(f"a gid has {GID_SIZE} bytes, not {len(gid)}")
    return _ATTACHMENT.pack(seq, source_ns, GID_SIZE, bytes(gid))


def decode_attachment(data) -> tuple[int, int, bytes]:
    """(sequence number, source timestamp ns, gid) of an rmw_zenoh attachment; RosError for anything else."""
    data = bytes(data) if data is not None else b""
    if len(data) != ATTACHMENT_SIZE or data[16] != GID_SIZE:
        raise RosError(f"no rmw_zenoh attachment ({len(data)} bytes; {ATTACHMENT_SIZE} expected, length byte 16)")
    seq, stamp, _, gid = _ATTACHMENT.unpack(data)
    return seq, stamp, gid


def gid(token_key: str) -> bytes:
    """The gid of an entity, as rmw_zenoh makes it: XXH3-128 of its token key (module docstring)."""
    from . import _core
    return _core.ros_gid(token_key)


# ------------------------------------------------------------------------------------------------------------- QoS

_INFINITE = ("9223372036", "854775807")  # RMW_DURATION_INFINITE (sec, nsec): rmw_zenoh's default deadline and others


@dataclasses.dataclass(frozen=True)
class Qos:
    """The QoS of a ROS entity as rmw_zenoh writes it into the token (qos_to_keyexpr): each field empty when it is
    rmw_zenoh's default (reliable, volatile, keep last, depth 42, deadline, lifespan and lease infinite, liveliness
    automatic). Durations as (sec, nsec) text, empty for the default."""

    reliability: int = RELIABLE
    durability: int = VOLATILE
    history: int = KEEP_LAST
    depth: int = 42
    deadline: tuple = ("", "")
    lifespan: tuple = ("", "")
    liveliness: str = ""
    lease: tuple = ("", "")

    def key(self) -> str:
        d = Qos()
        r = "" if self.reliability == d.reliability else str(self.reliability)
        u = "" if self.durability == d.durability else str(self.durability)
        h = "" if self.history == d.history else str(self.history)
        n = "" if self.depth == d.depth else str(self.depth)
        return (f"{r}:{u}:{h},{n}:{self.deadline[0]},{self.deadline[1]}:{self.lifespan[0]},{self.lifespan[1]}:"
                f"{self.liveliness},{self.lease[0]},{self.lease[1]}")

    @classmethod
    def parse(cls, text: str) -> Qos:
        """The QoS of a token's last part; RosError for one rmw_zenoh would not read."""
        parts = text.split(":")
        try:
            history, depth = parts[2].split(",")
            deadline = tuple(parts[3].split(","))
            lifespan = tuple(parts[4].split(","))
            liveliness, *lease = parts[5].split(",")
            if len(parts) != 6 or len(deadline) != 2 or len(lifespan) != 2 or len(lease) != 2:
                raise ValueError
            d = cls()
            return cls(int(parts[0]) if parts[0] else d.reliability, int(parts[1]) if parts[1] else d.durability,
                       int(history) if history else d.history, int(depth) if depth else d.depth, deadline, lifespan,
                       liveliness, tuple(lease))
        except (ValueError, IndexError):
            raise RosError(f"{text!r} is no rmw_zenoh QoS") from None


STREAM_QOS = Qos(depth=1)                                    # our streams: reliable, volatile, the latest only
TF_QOS = Qos(depth=100)                                      # /tf, as tf2's broadcaster and listener
TF_STATIC_QOS = Qos(durability=TRANSIENT_LOCAL, depth=1)     # /tf_static, as tf2's static broadcaster
TF_STATIC_READ_QOS = Qos(durability=TRANSIENT_LOCAL, depth=100)  # tf2's static listener
PARAMETERS_QOS = Qos(depth=1000)                             # rmw_qos_profile_parameters: services, events


# ---------------------------------------------------------------------------------------------------------- tokens


@dataclasses.dataclass(frozen=True)
class Token:
    """A liveliness token of rmw_zenoh, its parts (names demangled)."""

    domain: int
    zid: str
    node_id: str
    entity_id: str
    kind: str
    enclave: str
    namespace: str
    node: str
    topic: str | None = None
    type_name: str | None = None  # pkg/msg/Name
    type_hash: str | None = None
    qos: Qos | None = None
    backends: str | None = None   # the optional 14th part (rmw_zenoh 0.10.5 and later), ``backends:...``

    def key(self) -> str:
        parts = [ADMIN_SPACE, str(self.domain), self.zid, self.node_id, self.entity_id, self.kind, mangle(self.enclave),
                 mangle(self.namespace), mangle(self.node)]
        if self.kind != NODE:
            parts += [mangle(self.topic), dds_type(self.type_name), self.type_hash, self.qos.key()]
            if self.backends is not None:
                parts.append(self.backends)
        return "/".join(parts)

    @property
    def fully_qualified_node(self) -> str:
        return self.namespace.rstrip("/") + "/" + self.node


def parse_token(key: str) -> Token:
    """The parts of a token key; RosError for one rmw_zenoh would refuse."""
    parts = str(key).split("/")
    if len(parts) < 9 or parts[0] != ADMIN_SPACE or any(not p for p in parts) or parts[5] not in KINDS:
        raise RosError(f"{key!r} is no rmw_zenoh liveliness token")
    try:
        domain = int(parts[1])
    except ValueError:
        raise RosError(f"{key!r}: domain {parts[1]!r}") from None
    base = (domain, parts[2], parts[3], parts[4], parts[5], demangle(parts[6]), demangle(parts[7]), demangle(parts[8]))
    if parts[5] == NODE:
        if len(parts) != 9:
            raise RosError(f"{key!r}: a node's token has 9 parts")
        return Token(*base)
    if len(parts) not in (13, 14):
        raise RosError(f"{key!r}: an entity's token has 13 parts, or 14 with backends")
    if len(parts) == 14 and not parts[13].startswith("backends:"):
        raise RosError(f"{key!r}: an entity's 14th token part starts with 'backends:'")
    return Token(*base, demangle(parts[9]), type_of_dds(parts[10]), parts[11], Qos.parse(parts[12]),
                 parts[13] if len(parts) == 14 else None)


# ------------------------------------------------------------------------------------------- type hashes (RIHS01)

# from ROS 2 Jazzy's share/<package>/<msg|srv>/<Name>.json, vision_msgs from ros-jazzy-vision-msgs 4.1.1 (not part of
# a base install); tests/test_ros.py checks them against an install and against message_hash over msgdefs.py
TYPE_HASHES = {
    "builtin_interfaces/msg/Time": "RIHS01_b106235e25a4c5ed35098aa0a61a3ee9c9b18d197f398b0e4206cea9acf9c197",
    "diagnostic_msgs/msg/DiagnosticArray": "RIHS01_5a8a36efb05fb25070fa0fb3810290c0e6cd4862b54a8fb975a1ee8dc55a333e",
    "geometry_msgs/msg/WrenchStamped": "RIHS01_8dc3deaf06b2ab281f9f9a742a8961c328ca7cec16e3fd6586d3a5c83fa78f77",
    "nav_msgs/msg/Odometry": "RIHS01_3cc97dc7fb7502f8714462c526d369e35b603cfc34d946e3f2eda2766dfec6e0",
    "rcl_interfaces/msg/ParameterEvent": "RIHS01_043e627780fcad87a22d225bc2a037361dba713fca6a6b9f4b869a5aa0393204",
    "rcl_interfaces/srv/DescribeParameters": "RIHS01_845b484d71eb0673dae682f2e3ba3c4851a65a3dcfb97bddd82c5b57e91e4cff",
    "rcl_interfaces/srv/GetParameterTypes": "RIHS01_da199c878688b3e530bdfe3ca8f74cb9fa0c303101e980a9e8f260e25e1c80ca",
    "rcl_interfaces/srv/GetParameters": "RIHS01_bf9803d5c74cf989a5de3e0c2e99444599a627c7ff75f97b8c05b01003675cbc",
    "rcl_interfaces/srv/ListParameters": "RIHS01_3e6062bfbb27bfb8730d4cef2558221f51a11646d78e7bb30a1e83afac3aad9d",
    "rcl_interfaces/srv/SetParameters": "RIHS01_56eed9a67e169f9cb6c1f987bc88f868c14a8fc9f743a263bc734c154015d7e0",
    "rcl_interfaces/srv/SetParametersAtomically":
        "RIHS01_0e192ef259c07fc3c07a13191d27002222e65e00ccec653ca05e856f79285fcd",
    "rosgraph_msgs/msg/Clock": "RIHS01_692f7a66e93a3c83e71765d033b60349ba68023a8c689a79e48078bcb5c58564",
    "sensor_msgs/msg/CameraInfo": "RIHS01_b3dfd68ff46c9d56c80fd3bd4ed22c7a4ddce8c8348f2f59c299e73118e7e275",
    "sensor_msgs/msg/CompressedImage": "RIHS01_15640771531571185e2efc8a100baf923961a4d15d5569652e6cb6691e8e371a",
    "sensor_msgs/msg/Image": "RIHS01_d31d41a9a4c4bc8eae9be757b0beed306564f7526c88ea6a4588fb9582527d47",
    "sensor_msgs/msg/Imu": "RIHS01_7d9a00ff131080897a5ec7e26e315954b8eae3353c3f995c55faf71574000b5b",
    "sensor_msgs/msg/JointState": "RIHS01_a13ee3a330e346c9d87b5aa18d24e11690752bd33a0350f11c5882bc9179260e",
    "sensor_msgs/msg/PointCloud2": "RIHS01_9198cabf7da3796ae6fe19c4cb3bdd3525492988c70522628af5daa124bae2b5",
    "std_msgs/msg/Float64MultiArray": "RIHS01_1025ddc6b9552d191f89ef1a8d2f60f3d373e28b283d8891ddcc974e8c55397f",
    "std_msgs/msg/String": "RIHS01_df668c740482bbd48fb39d76a70dfd4bd59db1288021743503259e948f6b1a18",
    "tf2_msgs/msg/TFMessage": "RIHS01_e369d0f05a23ae52508854b66f6aa0437f3449d652e8cbf22d5abe85d020f087",
    "vision_msgs/msg/Classification": "RIHS01_f74db28d0eaef3afb78f48bf2161cd709a1383a0309d3454383b25be11345143",
    "vision_msgs/msg/Detection2DArray": "RIHS01_214274120007f4a683866481336b70310268cfa107aebe9d594a6acf75bda968",
    "vision_msgs/msg/Detection3DArray": "RIHS01_793f5eab9595ac1c0da07873cc859db42eae51f0ee76056db8562515d830e018",
}


# Every message type of TYPE_HASHES: True when ROS 2 Lyrical's typesupport says it has Buffer fields (rosidl
# has_buffer_fields_<pkg>__msg__<Name>(): an unbounded uint8[] field of its own or of a type it uses), asked of a
# Lyrical install (rmw_zenoh_cpp 0.10.6, vision_msgs 4.2.0); tests/test_ros.py checks that every message type is here
BUFFER_FIELDS = {
    "builtin_interfaces/msg/Time": False,
    "diagnostic_msgs/msg/DiagnosticArray": False,
    "geometry_msgs/msg/WrenchStamped": False,
    "nav_msgs/msg/Odometry": False,
    "rcl_interfaces/msg/ParameterEvent": False,
    "rosgraph_msgs/msg/Clock": False,
    "sensor_msgs/msg/CameraInfo": False,
    "sensor_msgs/msg/CompressedImage": True,
    "sensor_msgs/msg/Image": True,
    "sensor_msgs/msg/Imu": False,
    "sensor_msgs/msg/JointState": False,
    "sensor_msgs/msg/PointCloud2": True,
    "std_msgs/msg/Float64MultiArray": False,
    "std_msgs/msg/String": False,
    "tf2_msgs/msg/TFMessage": False,
    "vision_msgs/msg/Classification": False,
    "vision_msgs/msg/Detection2DArray": False,
    "vision_msgs/msg/Detection3DArray": False,
}


def backends_of(kind: str, type_name: str, qos: Qos) -> str | None:
    """The 14th token part of an entity of ours: ``CPU_BACKENDS`` for a subscription of a buffer type that is not
    transient local (rmw_zenoh uses no buffer keys for transient local), else None. RosError for a subscription's
    message type of unknown buffer status (not in ``BUFFER_FIELDS``)."""
    if kind != SUBSCRIPTION:
        return None
    if type_name not in BUFFER_FIELDS:
        raise RosError(f"{type_name}: not known whether ROS 2 Lyrical sends it on buffer keys (ros.BUFFER_FIELDS)")
    return CPU_BACKENDS if BUFFER_FIELDS[type_name] and qos.durability != TRANSIENT_LOCAL else None


def cpu_group_key(key: str) -> str:
    """The key on which Lyrical's publishers send a buffer type to ``backends:cpu:`` subscriptions."""
    return f"{key}/{CPU_GROUP}"


def hash_of(type_name: str) -> str:
    """The RIHS01 hash of a type Durchblick carries to ROS (``TYPE_HASHES``); RosError for another."""
    try:
        return TYPE_HASHES[type_name]
    except KeyError:
        raise RosError(f"{type_name}: no ROS 2 type hash known for it, so it is not offered to ROS") from None


# rosidl's type ids (type_description_interfaces/msg/FieldType.msg); a .msg's char is uint8, byte is octet
_PRIMITIVE_IDS = {"int8": 2, "uint8": 3, "char": 3, "int16": 4, "uint16": 5, "int32": 6, "uint32": 7, "int64": 8,
                  "uint64": 9, "float32": 10, "float64": 11, "bool": 15, "byte": 16, "string": 17, "wstring": 18}
_NESTED, _BOUNDED_STRING, _BOUNDED_WSTRING = 1, 21, 22
_ARRAY, _BOUNDED_SEQUENCE, _UNBOUNDED_SEQUENCE = 48, 96, 144  # added to the element's id
_FIELD_TYPE = re.compile(r"^(?P<base>[A-Za-z_][A-Za-z0-9_/]*)(<=(?P<string_bound>\d+))?"
                         r"(\[(?P<bounded><=)?(?P<size>\d*)\])?$")


def _field_type(spec: str, package: str) -> dict:
    m = _FIELD_TYPE.match(spec)
    if not m:
        raise RosError(f"field type {spec!r}")
    base, string_bound = m["base"], m["string_bound"]
    nested = ""
    if base in _PRIMITIVE_IDS:
        type_id = _PRIMITIVE_IDS[base]
        if string_bound is not None:
            if base not in ("string", "wstring"):
                raise RosError(f"field type {spec!r}: a bound on a non-string")
            type_id = _BOUNDED_STRING if base == "string" else _BOUNDED_WSTRING
    else:
        type_id = _NESTED
        if "/" in base:
            pkg, _, name = base.partition("/")
            nested = f"{pkg}/msg/{name.split('/')[-1]}"
        elif base == "Header":
            nested = "std_msgs/msg/Header"
        else:
            nested = f"{package}/msg/{base}"
    capacity = 0
    if m.group(5) is not None or "[" in spec:
        if m["size"] == "":
            type_id += _UNBOUNDED_SEQUENCE
        elif m["bounded"]:
            type_id, capacity = type_id + _BOUNDED_SEQUENCE, int(m["size"])
        else:
            type_id, capacity = type_id + _ARRAY, int(m["size"])
    return {"type_id": type_id, "capacity": capacity, "string_capacity": int(string_bound or 0),
            "nested_type_name": nested}


def parse_fields(text: str, package: str) -> list[dict]:
    """The fields of a .msg text (constants and comments left out), as rosidl's type description has them (without
    default values, which the hash leaves out). A message without fields gets rosidl's one uint8 field."""
    fields = []
    for line in text.splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        tokens = line.split()
        if len(tokens) < 2:
            raise RosError(f"definition line {line!r}")
        if "=" in tokens[1] or (len(tokens) > 2 and tokens[2].startswith("=")):
            continue  # a constant
        fields.append({"name": tokens[1].split("#")[0], "type": _field_type(tokens[0], package)})
    if not fields:
        fields.append({"name": "structure_needs_at_least_one_member", "type": _field_type("uint8", package)})
    return fields


def _individual(type_name: str, text: str) -> dict:
    return {"type_name": type_name, "fields": parse_fields(text, type_name.split("/")[0])}


def _full(top: dict, lookup: Callable[[str], str], extra: dict | None = None) -> dict:
    """``top`` with every type it refers to (sorted by name), read by ``lookup(type_name) -> .msg text`` unless in
    ``extra`` (descriptions at hand)."""
    known = dict(extra or {})
    referenced, queue = {}, [f["type"]["nested_type_name"] for f in top["fields"] if f["type"]["nested_type_name"]]
    while queue:
        name = queue.pop()
        if name in referenced:
            continue
        referenced[name] = known.get(name) or _individual(name, lookup(name))
        queue.extend(f["type"]["nested_type_name"] for f in referenced[name]["fields"]
                     if f["type"]["nested_type_name"])
    return {"type_description": top, "referenced_type_descriptions": [referenced[n] for n in sorted(referenced)]}


def rihs01(full_description: dict) -> str:
    """The hash of a full type description (rosidl_generator_type_description.calculate_type_hash)."""
    text = json.dumps(full_description, ensure_ascii=True, separators=(", ", ": "), sort_keys=False)
    return "RIHS01_" + hashlib.sha256(text.encode("utf-8")).hexdigest()


def message_hash(type_name: str, lookup: Callable[[str], str]) -> str:
    """The RIHS01 hash of a message type (``pkg/msg/Name``) from its definition and those it uses
    (``lookup(type_name) -> .msg text``)."""
    return rihs01(_full(_individual(type_name, lookup(type_name)), lookup))


def service_hash(type_name: str, request: str, response: str, lookup: Callable[[str], str]) -> str:
    """The RIHS01 hash of a service type (``pkg/srv/Name``) from the request's and the response's definitions (the
    two halves of its .srv) and ``lookup`` for the types they use (service_msgs/msg/ServiceEventInfo among them)."""
    parts = {f"{type_name}_Request": _individual(f"{type_name}_Request", request),
             f"{type_name}_Response": _individual(f"{type_name}_Response", response)}
    pkg = type_name.split("/")[0]
    event = {"type_name": f"{type_name}_Event", "fields": [
        {"name": "info", "type": _field_type("service_msgs/ServiceEventInfo", pkg)},
        {"name": "request", "type": {**_field_type("uint8[<=1]", pkg), "type_id": _NESTED + _BOUNDED_SEQUENCE,
                                     "nested_type_name": f"{type_name}_Request"}},
        {"name": "response", "type": {**_field_type("uint8[<=1]", pkg), "type_id": _NESTED + _BOUNDED_SEQUENCE,
                                      "nested_type_name": f"{type_name}_Response"}}]}
    parts[f"{type_name}_Event"] = event
    top = {"type_name": type_name, "fields": [
        {"name": f"{role}_message", "type": {"type_id": _NESTED, "capacity": 0, "string_capacity": 0,
                                             "nested_type_name": f"{type_name}_{suffix}"}}
        for role, suffix in (("request", "Request"), ("response", "Response"), ("event", "Event"))]}
    return rihs01(_full(top, lookup, parts))


def msgdefs_lookup(type_name: str, text: str) -> Callable[[str], str]:
    """``lookup`` for a ros2msg definition as rosbags and MCAP write it (msgdefs.py): the type's own text, then each
    used type after a line of ``=`` and ``MSG: pkg/Name``."""
    texts = {}
    blocks = re.split(r"^=+\s*$", text, flags=re.M)
    texts[type_name] = blocks[0]
    for block in blocks[1:]:
        head, _, body = block.strip("\n").partition("\n")
        m = re.match(r"^MSG: ([A-Za-z0-9_]+)/(?:msg/)?([A-Za-z0-9_]+)$", head.strip())
        if not m:
            raise RosError(f"{type_name}: definition block {head!r}")
        texts[f"{m[1]}/msg/{m[2]}"] = body

    def lookup(name: str) -> str:
        try:
            return texts[name]
        except KeyError:
            raise RosError(f"{type_name}: its definition lacks {name}") from None
    return lookup


# ---------------------------------------------------------------------------------------- taking part on the wire


def _raw(session):
    """The zenoh.Session inside session.Session (zenoh-ext's functions take only Zenoh's own)."""
    return getattr(session, "_session", session)


POLL_PERIOD_S = 0.05  # how often a _Polled subscriber looks for samples


class _Polled:
    """A declared subscriber's channel, read by a daemon thread of its own (``try_recv`` every POLL_PERIOD_S), for a
    subscription the program did not ask for: zenoh-python's callback thread is no daemon and would keep a program
    alive that ends without closing its session, and a channel cannot be undeclared during a blocking ``recv``.
    ``undeclare`` ends the thread first."""

    def __init__(self, declared, take: Callable, name: str):
        self.declared = declared
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, args=(take,), name=name, daemon=True)
        self._thread.start()

    def _run(self, take: Callable):
        while not self._stop.is_set():
            try:
                while (sample := self.declared.try_recv()) is not None:
                    try:
                        take(sample)
                    except Exception:  # noqa: BLE001 - a bad sample must not end the subscription
                        log.exception("a ROS sample its reader could not take")
            except zenoh.ZError:  # closed with its session
                return
            self._stop.wait(POLL_PERIOD_S)

    def undeclare(self):
        self._stop.set()
        if self._thread is not threading.current_thread():
            self._thread.join(timeout=2.0)
        self.declared.undeclare()


# ------------------------------------------------------------------------------------ ROS cameras as ROS names them

COMPRESSED = "compressed"  # image_transport's compressed transport: the topic <image topic>/compressed
CAMERA_INFO = "camera_info"  # image_transport's camera_info: the topic camera_info beside the image topic
IMAGE_TYPE, COMPRESSED_TYPE = "sensor_msgs/msg/Image", "sensor_msgs/msg/CompressedImage"
CLOUD_TYPE, CAMERA_INFO_TYPE = "sensor_msgs/msg/PointCloud2", "sensor_msgs/msg/CameraInfo"
CAMERA_TYPES = (IMAGE_TYPE, COMPRESSED_TYPE, CLOUD_TYPE, CAMERA_INFO_TYPE)  # what xblick list shows of ROS's topics


def is_topic(name: str) -> bool:
    """True for a ROS topic as ROS's tools write it (``/image_raw``, ``/resize/image``): a name with a leading ``/``.
    A name without one is one of our stream bases (``test/camera/color``)."""
    return str(name).startswith("/")


def compressed_topic(image_topic: str) -> str:
    """image_transport's compressed topic of an image topic: ``/cam/image_raw`` -> ``/cam/image_raw/compressed``."""
    return f"{image_topic}/{COMPRESSED}"


def camera_info_topic(image_topic: str) -> str:
    """image_transport's camera_info topic of an image topic: ``camera_info`` in the image topic's namespace
    (``/cam/image_raw`` -> ``/cam/camera_info``, ``/image_raw`` -> ``/camera_info``, ``/resize/image`` ->
    ``/resize/camera_info``)."""
    return f"{image_topic.rpartition('/')[0]}/{CAMERA_INFO}"


def publisher_pattern(domain: int, topic: str | None = None) -> str:
    """The liveliness key expression of the publishers' tokens on ``domain``, of one ``topic`` or of all."""
    return "/".join([ADMIN_SPACE, str(domain), "*", "*", "*", PUBLISHER, "*", "*", "*"]
                    + ([mangle(topic), "**"] if topic is not None else ["**"]))


def publishers(session, timeout: float = 1.0, topic: str | None = None) -> list[Token]:
    """The publishers in ROS's graph (their tokens) on the domain of ``session`` (one in ROS 2), of ``topic`` or of
    every topic, asked once (a session just opened knows none yet: ``ask``'s loop is the caller's)."""
    from .session import replies
    found = []
    for reply in replies(session, publisher_pattern(session.ros.domain, topic), timeout, liveliness=True):
        if reply.ok is not None:
            try:
                found.append(parse_token(str(reply.ok.key_expr)))
            except RosError:
                continue
    return found


class Graph:
    """The publishers in ROS's graph on a session's domain, as a liveliness subscriber with history keeps their tokens
    (made at its first use, ``Ros.graph``, closed with the session's part in ROS). ``listen(callback)``: called in its
    thread when a publisher came or went. ``by_xblick(zid)``: whether a publisher's session is a program of ours (its
    declaration, declaration.py), asked once per session: the ROS topics such a program publishes are the copies of
    its own keys."""

    def __init__(self, ros_: Ros):
        self._ros = ros_
        self._lock = threading.Lock()
        self._tokens: dict[str, Token] = {}
        self._listeners: list[Callable] = []
        self._xblick: dict[str, bool] = {}
        self._subscriber = _Polled(_raw(ros_.session).liveliness().declare_subscriber(
            publisher_pattern(ros_.domain), history=True), self._seen, f"ros graph {ros_.domain}")

    def _seen(self, sample):
        key = str(sample.key_expr)
        with self._lock:
            if sample.kind == zenoh.SampleKind.PUT:
                try:
                    self._tokens[key] = parse_token(key)
                except RosError:
                    return
            elif self._tokens.pop(key, None) is None:
                return
            listeners = list(self._listeners)
        for callback in listeners:
            callback()

    def listen(self, callback: Callable):
        with self._lock:
            self._listeners.append(callback)

    def unlisten(self, callback: Callable):
        with self._lock:
            if callback in self._listeners:
                self._listeners.remove(callback)

    def publishers(self, topic: str | None = None) -> list[Token]:
        """The publishers known now, of ``topic`` or of all, this session's own left out."""
        with self._lock:
            return [t for t in self._tokens.values() if (topic is None or t.topic == topic) and t.zid != self._ros.zid]

    def by_xblick(self, zid: str) -> bool:
        """True when the session ``zid`` declares a program of ours (its declaration's token; a liveliness query, kept
        per session). A program declares itself before it takes part in ROS (session.open_session), so its declaration
        is known wherever its ROS tokens are."""
        with self._lock:
            known = self._xblick.get(zid)
        if known is not None:
            return known
        from . import declaration
        from .session import replies
        found = any(r.ok is not None for r in replies(_raw(self._ros.session), declaration.key(zid), 1.0,
                                                       liveliness=True))
        with self._lock:
            self._xblick[zid] = found
        return found

    def close(self):
        self._subscriber.undeclare()
        with self._lock:
            self._listeners = []


CLOCK_TOPIC = "/clock"  # ROS's global clock, which a node with use_sim_time reads
TF_STATIC_TOPIC = "/tf_static"  # ROS's global static transforms (tf2's static broadcaster and listener)
TF_TOLERANCE = 1e-6     # m and quaternion units: two static transforms of one frame that differ less are the same


def _static_of(t) -> tuple:
    """(parent, translation, rotation) of a TransformStamped, for comparing static transforms."""
    tr, q = t.transform.translation, t.transform.rotation
    return t.header.frame_id, (tr.x, tr.y, tr.z), (q.x, q.y, q.z, q.w)


def _static_differs(a: tuple, b: tuple) -> str:
    """Why two static transforms of one frame (``_static_of``) clash, or empty when they are the same (q and -q are
    the same rotation)."""
    if a[0] != b[0]:
        return f"parent {b[0]!r} there, {a[0]!r} here"
    same_t = all(abs(x - y) <= TF_TOLERANCE for x, y in zip(a[1], b[1]))
    same_q = (all(abs(x - y) <= TF_TOLERANCE for x, y in zip(a[2], b[2]))
              or all(abs(x + y) <= TF_TOLERANCE for x, y in zip(a[2], b[2])))
    return "" if same_t and same_q else f"parent {a[0]!r} in both, another transform"


class Ros:
    """A session's part in ROS 2 (every program's by default): ``domain``, and the nodes it shows ROS: its own
    ``node``, made at the start, which carries its publishers, subscribers and its first parameters' services
    (``parameters_node``; until then six services without parameters, Parameters.none_on), and those ``add_node``
    adds (further parameters of one program). Entity ids count up per
    session, as rmw_zenoh's per context. ``params_writable`` (``--ros-params-writable``): ROS clients may set the
    parameters of this session's nodes; by default the ROS parameter services answer reads only and refuse sets.
    ``follow_parameters``: the node's name is a default, and the program's first parameters give it theirs.
    ``clock`` (``--ros-clock``): the key of ours that goes to ROS's ``/clock`` (``mirror``).

    Another node of the node's fully qualified name in ROS's graph (at the start or later) is said in one warning
    line, as ROS's tools only warn. A static transform of ours whose frame another program sends on ``/tf_static``
    with another parent or transform is said in one error line (``check_tf``)."""

    def __init__(self, session, domain: int, node: str, namespace: str = "/", params_writable: bool = False,
                 follow_parameters: bool = False, clock: str | None = None):
        self.session = session
        self.domain = domain
        self.params_writable = params_writable
        self.follow_parameters = follow_parameters
        self.clock = clock.strip("/") if clock else None
        self.prog: str | None = None  # set by session.open_from_args: a rename of the node is said as "<prog>: ..."
        self.zid = str(_raw(session).info.zid())
        self._ids = itertools.count()
        self._lock = threading.Lock()
        self._nodes: list[RosNode] = []
        self._tf_parents: dict[str, str] = {}  # child frame -> parent, of what this program sent to /tf, /tf_static
        self._tf_clashes: set[str] = set()     # frames said already
        self._parameters_taken = False         # the node serves a Parameters' services
        self._own_gids: set[bytes] = set()     # every gid an entity of this session had: its own samples
        self._static_mine: dict[str, tuple] = {}    # child -> _static_of, sent by this program on /tf_static
        self._static_others: dict[str, tuple] = {}  # child -> _static_of, sent by other programs
        self._static_said: set[str] = set()
        self._static_reader: RosSubscriber | None = None
        self._duplicates = None
        self._duplicates_said: set[str] = set()
        self._no_parameters = None
        self._graph: Graph | None = None
        self.node = self.add_node(node, namespace)
        self._watch_duplicates()
        from .parameters import Parameters
        self._no_parameters = Parameters.none_on(self.node)  # until the program's first Parameters

    def next_id(self) -> str:
        with self._lock:
            return str(next(self._ids))

    def add_node(self, name: str, namespace: str = "/") -> RosNode:
        node = RosNode(self, check_node_name(name), namespace)
        with self._lock:
            self._nodes.append(node)
        return node

    def _watch_duplicates(self):
        """A liveliness subscriber (with the tokens there now) on node tokens of the node's fully qualified name: one
        warning line for the first one that is not ours. Declared anew when the node is renamed."""
        if self._duplicates is not None:
            self._duplicates.undeclare()
        own, name = self.node.token.key(), self.node.fully_qualified
        pattern = "/".join([ADMIN_SPACE, str(self.domain), "*", "*", "*", NODE, "*", mangle(self.node.namespace),
                            mangle(self.node.name)])

        def seen(sample):
            if sample.kind != zenoh.SampleKind.PUT or str(sample.key_expr) == own:
                return
            with self._lock:
                if name in self._duplicates_said:
                    return
                self._duplicates_said.add(name)
            log.warning(f"ROS node {name} is in ROS's graph already (another program, or another session of this "
                        f"one, on domain {self.domain}): ROS's tools then show two nodes of one name and reach one of "
                        "them; give each program a node name of its own (--ros-node)")

        self._duplicates = _Polled(_raw(self.session).liveliness().declare_subscriber(pattern, history=True), seen,
                                   f"ros node {name}")

    def parameters_node(self, name: str) -> tuple[RosNode, bool]:
        """The ROS node that serves the parameters ``name`` (their Durchblick name, ``test/camera/probe``), and True
        when it is a node of their own. The program's first parameters: its node, which takes their name
        (``node_of``) when its own is a default (``follow_parameters``); its publishers and subscribers made before
        are declared anew under it. Further parameters of the program: a node of their own of their name."""
        namespace, node = node_of(name)
        with self._lock:
            first, self._parameters_taken = not self._parameters_taken, True
            no_parameters, self._no_parameters = self._no_parameters, None
        if not first:
            return self.add_node(node, namespace), True
        if no_parameters is not None:
            no_parameters.close()
        if self.follow_parameters and (namespace, node) != (self.node.namespace, self.node.name):
            self.node.rename(node, namespace)
            self._watch_duplicates()
            if self.prog is not None:
                print(f"{self.prog}: in ROS 2 as node {self.node.fully_qualified} (the name of its parameters)",
                      file=sys.stderr, flush=True)
        return self.node, False

    def release_parameters(self):
        """The node's parameters closed: the next Parameters serve on it again."""
        with self._lock:
            self._parameters_taken = False

    def own(self, gid_: bytes):
        with self._lock:
            self._own_gids.add(gid_)

    @property
    def graph(self) -> Graph:
        """ROS's publishers on the domain (Graph), made at the first use: a program that reads no ROS topic keeps
        none."""
        with self._lock:
            if self._graph is None:
                self._graph = Graph(self)
            return self._graph

    def close(self):
        with self._lock:
            graph, self._graph = self._graph, None
        if graph is not None:
            graph.close()
        if self._no_parameters is not None:
            self._no_parameters.close()
            self._no_parameters = None
        if self._duplicates is not None:
            self._duplicates.undeclare()
            self._duplicates = None
        with self._lock:
            nodes, self._nodes = self._nodes, []
        for node in nodes:
            node.close()

    def _read_static(self):
        """The subscription to ROS's /tf_static that compares other programs' static transforms with this program's
        (``check_tf``), made with this program's first static transform: a program that sends none reads none."""
        with self._lock:
            if self._static_reader is not None:
                return
            self._static_reader = True  # being made
        from .msg import TFMessage
        self._static_reader = self.node.subscriber("/tf_static", TFMessage.TYPE, self._on_static, TF_STATIC_READ_QOS,
                                                   daemon=True)

    def _on_static(self, payload: bytes, seq, stamp, publisher: bytes):
        from .msg import TFMessage
        with self._lock:
            if publisher in self._own_gids:
                return
        for t in TFMessage.deserialize(payload).transforms:
            theirs = _static_of(t)
            with self._lock:
                self._static_others[t.child_frame_id] = theirs
                mine = self._static_mine.get(t.child_frame_id)
            if mine is not None:
                self._say_static(t.child_frame_id, mine, theirs)

    def _say_static(self, child: str, mine: tuple, theirs: tuple):
        why = _static_differs(mine, theirs)
        with self._lock:
            if not why or child in self._static_said:
                return
            self._static_said.add(child)
        log.error(f"ROS /tf_static: frame {child!r} is sent by another program too, {why}; tf2 keeps one of them: "
                  "give each camera and robot frame names of its own (--frame-id)")

    def check_tf(self, payload, static: bool):
        """One error line per frame that this program gives two parents on ROS's /tf and /tf_static (tf2 takes one
        parent per frame: frame names must be unique across cameras and robots). Static ones are also compared with
        what other programs send on /tf_static (``_read_static``): one error line per frame another program sends
        with another parent or transform. The transforms still go out: a consumer sees the clash too."""
        from .msg import TFMessage
        try:
            transforms = TFMessage.deserialize(bytes(payload)).transforms
        except Exception as error:  # noqa: BLE001 - a payload of ours that does not decode: said, not raised
            log.error(f"/tf{'_static' if static else ''}: a message that is no TFMessage ({error})")
            return
        compare = []
        with self._lock:
            for t in transforms:
                child, parent = t.child_frame_id, t.header.frame_id
                known = self._tf_parents.setdefault(child, parent)
                if known != parent and child not in self._tf_clashes:
                    self._tf_clashes.add(child)
                    log.error(f"ROS tf: frame {child!r} has parent {known!r} and parent {parent!r} in this program; "
                              "tf2 takes one parent per frame: give each camera and robot frame names of its own "
                              "(--frame-id)")
                if static:
                    self._static_mine[child] = _static_of(t)
                    if child in self._static_others:
                        compare.append((child, self._static_mine[child], self._static_others[child]))
        for clash in compare:
            self._say_static(*clash)
        if static:
            self._read_static()


def commanding(key: str) -> bool:
    """True for a key that carries what a robot is told: a part ``command`` (a controller's
    ``<robot>/controller/<instance>/command``, a simulated robot's ``<robot>/command/joints`` and ``command/tf``).
    Such a key is never offered to ROS, whatever its type: a ROS driver could take its targets."""
    from .topics import COMMAND
    return COMMAND in str(key).strip("/").split("/")


def mirror(session, key: str, encoding, topic: str | None = None, qos: Qos = STREAM_QOS) -> RosPublisher | None:
    """The ROS publisher that mirrors one of our keys when ``session`` takes part in ROS (the default), else None.
    ``encoding``: the key's Zenoh encoding (``application/cdr;<ROS type>``); ``topic``: ROS's topic, by default the
    key's (``topic_of``), ``/clock`` for the session's clock key (``--ros-clock``; RosError when its type is not
    rosgraph_msgs/msg/Clock). A command key (``commanding``), a key whose type ROS has no hash for (Durchblick's own
    types, JSON, moveit_msgs) and a key that is no ROS name (``-`` in it) are not offered: one warning line, None."""
    taking_part = getattr(session, "ros", None)
    if taking_part is None:
        return None
    if commanding(key):
        log.warning(f"{key}: not offered to ROS (a command key: what a robot is told never goes to ROS)")
        return None
    schema = str(encoding).partition(";")
    type_name = schema[2] if schema[0] == "application/cdr" else ""
    if taking_part.clock is not None and str(key).strip("/") == taking_part.clock:
        if type_name != "rosgraph_msgs/msg/Clock":
            raise RosError(f"--ros-clock {key}: its type is {type_name or str(encoding)}, not rosgraph_msgs/msg/Clock")
        topic = CLOCK_TOPIC
    if not type_name or "@" in type_name or type_name not in TYPE_HASHES:
        log.warning(f"{key}: not offered to ROS ({type_name or str(encoding)} has no ROS type hash here)")
        return None
    topic = topic or offered_topic(key)
    if topic is None:
        return None
    return taking_part.node.publisher(topic, type_name, qos)


def mirror_tf(session, static: bool = False, key: str = "") -> RosPublisher | None:
    """The publisher on ROS's global ``/tf`` (``/tf_static`` with ``static``, transient local) when ``session`` takes
    part in ROS, else None; None too for a command key (``commanding``, one warning line). Its ``put`` checks the
    frames for a second parent and, static, for other programs' transforms of the same frames (``Ros.check_tf``)."""
    taking_part = getattr(session, "ros", None)
    if taking_part is None:
        return None
    if commanding(key):
        log.warning(f"{key}: not offered to ROS (a command key: what a robot is told never goes to ROS)")
        return None
    from .msg import TFMessage
    publisher = taking_part.node.publisher("/tf_static" if static else "/tf", TFMessage.TYPE,
                                           TF_STATIC_QOS if static else TF_QOS)
    publisher.check = lambda payload: taking_part.check_tf(payload, static)
    return publisher


class RosNode:
    """A ROS node of a session: its token, and the entities below it."""

    def __init__(self, ros: Ros, name: str, namespace: str = "/"):
        if namespace != "/":
            check_ros_name(namespace, "namespace")
        self.ros, self.name, self.namespace = ros, name, namespace
        self.id = ros.next_id()
        self.token = Token(ros.domain, ros.zid, self.id, self.id, NODE, "/", namespace, name)
        self._token = _raw(ros.session).liveliness().declare_token(self.token.key())
        self._entities: list = []
        self._lock = threading.Lock()

    @property
    def fully_qualified(self) -> str:
        return self.token.fully_qualified_node

    def rename(self, name: str, namespace: str = "/"):
        """The node under another name (its id kept): its token and those of its entities declared anew (their gids
        follow the token). Their keys stay as they are."""
        check_node_name(name)
        if namespace != "/":
            check_ros_name(namespace, "namespace")
        with self._lock:
            self.name, self.namespace = name, namespace
            self.token = Token(self.ros.domain, self.ros.zid, self.id, self.id, NODE, "/", namespace, name)
            if self._token is not None:
                self._token.undeclare()
                self._token = _raw(self.ros.session).liveliness().declare_token(self.token.key())
            entities = list(self._entities)
        for entity in entities:
            entity._rename()

    def entity_token(self, kind: str, name: str, type_name: str, qos: Qos) -> Token:
        return Token(self.ros.domain, self.ros.zid, self.id, self.ros.next_id(), kind, "/", self.namespace, self.name,
                     check_ros_name(name), type_name, hash_of(type_name), qos, backends_of(kind, type_name, qos))

    def _add(self, entity):
        with self._lock:
            self._entities.append(entity)
            stale = (entity.token.namespace, entity.token.node) != (self.namespace, self.name)
        if stale:  # made while the node was renamed
            entity._rename()
        return entity

    def _forget(self, entity):
        with self._lock:
            if entity in self._entities:
                self._entities.remove(entity)

    def publisher(self, topic: str, type_name: str, qos: Qos = STREAM_QOS) -> RosPublisher:
        return self._add(RosPublisher(self, topic, type_name, qos))

    def subscriber(self, topic: str, type_name: str, callback: Callable, qos: Qos = STREAM_QOS,
                   daemon: bool = False) -> RosSubscriber:
        return self._add(RosSubscriber(self, topic, type_name, callback, qos, daemon))

    def service(self, name: str, type_name: str, qos: Qos = PARAMETERS_QOS) -> RosService:
        return self._add(RosService(self, name, type_name, qos))

    def client(self, name: str, type_name: str, qos: Qos = PARAMETERS_QOS) -> RosClient:
        return self._add(RosClient(self, name, type_name, qos))

    def close(self):
        with self._lock:
            entities, self._entities = self._entities, []
        for entity in entities:
            entity.close()
        if self._token is not None:
            self._token.undeclare()
            self._token = None


def _reliability(qos: Qos) -> zenoh.Reliability:
    return zenoh.Reliability.BEST_EFFORT if qos.reliability == BEST_EFFORT else zenoh.Reliability.RELIABLE


def _congestion(qos: Qos) -> zenoh.CongestionControl:
    # rmw_zenoh: blocking only for reliable keep-all
    blocking = qos.reliability != BEST_EFFORT and qos.history == KEEP_ALL
    return zenoh.CongestionControl.BLOCK if blocking else zenoh.CongestionControl.DROP


class _Entity:
    def __init__(self, node: RosNode, kind: str, name: str, type_name: str, qos: Qos):
        self.node = node
        self.topic = name
        self.type_name = type_name
        self.qos = qos
        self.token = node.entity_token(kind, name, type_name, qos)
        self.key = data_key(node.ros.domain, name, type_name, self.token.type_hash)
        if not re.match(r"\d+/", self.key):  # a ROS key starts with the domain: no key of ours does (topics.check_name)
            raise RosError(f"{self.key!r} is no rmw_zenoh key")
        self.gid = gid(self.token.key())
        node.ros.own(self.gid)
        self._raw = _raw(node.ros.session)
        self._declared = None
        self._live = None

    def _announce(self):
        self._live = self._raw.liveliness().declare_token(self.token.key())

    def _rename(self):
        """Its node renamed (RosNode.rename): its token and gid of the node's new name."""
        node = self.node
        token = dataclasses.replace(self.token, namespace=node.namespace, node=node.name)
        self.token, self.gid = token, gid(token.key())
        node.ros.own(self.gid)
        if self._live is not None:
            self._live.undeclare()
            self._announce()

    def close(self):
        """Undeclares it and its token (again: nothing)."""
        self.node._forget(self)
        if self._live is not None:
            self._live.undeclare()
            self._live = None
        if self._declared is not None:
            self._declared.undeclare()
            self._declared = None


class RosPublisher(_Entity):
    """A ROS publisher: ``put(payload)`` sends the CDR bytes on ROS's key with ROS's attachment. Transient local
    (``/tf_static``): zenoh-ext's AdvancedPublisher with a cache, as rmw_zenoh, so that a late subscriber gets the
    last ``depth`` samples."""

    def __init__(self, node: RosNode, topic: str, type_name: str, qos: Qos = STREAM_QOS):
        super().__init__(node, PUBLISHER, topic, type_name, qos)
        self.check = None  # called with each payload before it goes out (mirror_tf: the frames' parents)
        self._seq = itertools.count(1)
        self._seq_lock = threading.Lock()
        if qos.durability == TRANSIENT_LOCAL:
            import zenoh.ext as ext
            miss = ext.MissDetectionConfig(heartbeat=None, sporadic_heartbeat=HEARTBEAT_S) \
                if qos.reliability != BEST_EFFORT else None
            self._declared = ext.declare_advanced_publisher(
                self._raw, self.key, cache=ext.CacheConfig(qos.depth), publisher_detection=True,
                sample_miss_detection=miss, reliability=_reliability(qos), congestion_control=_congestion(qos))
        else:
            self._declared = self._raw.declare_publisher(self.key, reliability=_reliability(qos),
                                                         congestion_control=_congestion(qos))
        self._announce()

    @property
    def matched(self) -> bool:
        """A ROS subscriber (or any other) of the key is known. Always True for a transient-local publisher: zenoh-ext's
        AdvancedPublisher has no matching status, and its samples go into its cache for later subscribers anyway."""
        if self.qos.durability == TRANSIENT_LOCAL:
            return True
        return self._declared.matching_status.matching

    def put(self, payload):
        if self.check is not None:
            self.check(payload)
        with self._seq_lock:
            seq = next(self._seq)
        self._declared.put(bytes(payload), attachment=encode_attachment(seq, time.time_ns(), self.gid))


class RosSubscriber(_Entity):
    """A ROS subscription: ``callback(payload: bytes, seq, source_ns, gid)`` for each sample (in Zenoh's thread). A
    sample without ROS's attachment is dropped with a warning, as ROS drops it. Transient local: zenoh-ext's
    AdvancedSubscriber, which asks publishers it detects for their cache. A buffer type (``backends_of``): also the
    key's ``_buf_cpu``, where Lyrical's publishers send it. ``daemon``: the samples in a daemon thread of its own
    instead (a subscription the program did not ask for, such as ``Ros``'s of ``/tf_static``: zenoh-python's callback
    thread is no daemon and would keep a program alive that ends without closing its session)."""

    def __init__(self, node: RosNode, topic: str, type_name: str, callback: Callable, qos: Qos = STREAM_QOS,
                 daemon: bool = False):
        super().__init__(node, SUBSCRIPTION, topic, type_name, qos)
        self._callback = callback
        self._warned = False
        handler = None if daemon else self._on_sample  # None: zenoh's channel, read by _Polled
        if qos.durability == TRANSIENT_LOCAL:
            import zenoh.ext as ext
            recovery = ext.RecoveryConfig(periodic_queries=None, heartbeat=True) \
                if qos.reliability != BEST_EFFORT else None
            self._declared = ext.declare_advanced_subscriber(
                self._raw, self.key, handler,
                history=ext.HistoryConfig(detect_late_publishers=True, max_samples=qos.depth), recovery=recovery,
                subscriber_detection=True)
        else:
            self._declared = self._raw.declare_subscriber(self.key, handler)
        self._cpu_group = self._raw.declare_subscriber(cpu_group_key(self.key), handler) \
            if self.token.backends == CPU_BACKENDS else None
        if daemon:
            self._declared = _Polled(self._declared, self._on_sample, f"ros {topic}")
            if self._cpu_group is not None:
                self._cpu_group = _Polled(self._cpu_group, self._on_sample, f"ros {topic} cpu")
        self._announce()

    def close(self):
        super().close()
        if self._cpu_group is not None:
            self._cpu_group.undeclare()
            self._cpu_group = None

    def _on_sample(self, sample: zenoh.Sample):
        try:
            seq, stamp, publisher = decode_attachment(sample.attachment.to_bytes() if sample.attachment else None)
        except RosError as error:
            if not self._warned:
                self._warned = True
                log.warning(f"{self.topic}: a sample dropped, as ROS drops it: {error}")
            return
        try:
            self._callback(sample.payload.to_bytes(), seq, stamp, publisher)
        except Exception:  # noqa: BLE001 - a bad sample must not end the subscription
            log.exception(f"{self.topic}: a sample its reader could not take")


class RosService(_Entity):
    """A ROS service: a queryable on ROS's key, declared complete (ROS clients ask with target ALL_COMPLETE), its
    queries in a channel (``try_recv``, as Parameters reads its own). ``reply`` answers a query with ROS's attachment
    (the request's sequence number and client gid, the reply's time)."""

    def __init__(self, node: RosNode, name: str, type_name: str, qos: Qos = PARAMETERS_QOS):
        super().__init__(node, SERVICE, name, type_name, qos)
        self._declared = self._raw.declare_queryable(self.key, complete=True)
        self._announce()

    @property
    def closed(self) -> bool:
        """Closed (``close``): its queries end; a server stops reading it."""
        return self._declared is None

    def try_recv(self):
        return self._declared.try_recv()

    @staticmethod
    def request_of(query: zenoh.Query) -> tuple[int, int, bytes]:
        """(sequence number, source ns, client gid) of a ROS request; RosError when it has no ROS attachment."""
        return decode_attachment(query.attachment.to_bytes() if query.attachment is not None else None)

    def reply(self, query: zenoh.Query, payload, request: tuple[int, int, bytes]):
        seq, _, client = request
        query.reply(self.key, bytes(payload), attachment=encode_attachment(seq, time.time_ns(), client))


class RosClient(_Entity):
    """A ROS service client, as rmw_zenoh's: its token, and ``request_options(payload)``, the options of one request's
    query (target ALL_COMPLETE, which reaches ROS's complete service queryables and no other, and ROS's attachment
    with a sequence number of its own), for session.replies or session.command. ``matched``: a service of the key is
    known."""

    def __init__(self, node: RosNode, name: str, type_name: str, qos: Qos = PARAMETERS_QOS):
        super().__init__(node, CLIENT, name, type_name, qos)
        self._seq = itertools.count(1)
        self._seq_lock = threading.Lock()
        self._declared = self._raw.declare_querier(self.key, target=zenoh.QueryTarget.ALL_COMPLETE)
        self._announce()

    @property
    def matched(self) -> bool:
        return self._declared.matching_status.matching

    def request_options(self, payload) -> dict:
        """A new request's options: each request its own sequence number (a ROS service refuses a second pending
        request of one client with the same number)."""
        with self._seq_lock:
            seq = next(self._seq)
        return {"payload": bytes(payload), "target": zenoh.QueryTarget.ALL_COMPLETE,
                "attachment": encode_attachment(seq, time.time_ns(), self.gid)}
