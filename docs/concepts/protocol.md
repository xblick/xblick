# Protocol

The protocol is the agreement between publishers and subscribers about the bytes on a topic. This page explains
which messages Durchblick sends, how it adds its own fields, and how versions protect you from misread data.

## ROS 2 messages without ROS

Every payload is a ROS 2 message, for example `sensor_msgs/msg/Image` or `vision_msgs/msg/Detection3DArray`.
Durchblick serialises it as CDR, the binary format of ROS 2, in little-endian byte order with the 4-byte
encapsulation header. The bytes are identical to the bytes that ROS 2 writes.

We chose ROS 2 messages because engineers know them and tools read them. Durchblick does not need a ROS
installation. It has its own implementation of the messages in Python and in C++.

Two tests keep this promise. `tests/test_cdr.py` compares the bytes of every message type with
[rosbags](https://gitlab.com/ternaris/rosbags), which uses the definitions of ROS 2 Lyrical. A second test compares
the C++ messages with the Python bytes.

## The attachment

A ROS 2 header has a stamp and a frame id. Durchblick needs three more fields per sample: a sequence number, the publish
time and the source. These fields travel beside the payload in Zenoh's
[*attachment*](https://docs.rs/zenoh/1.10.1/zenoh/sample/struct.Sample.html), as a small JSON object:

```json
{"v": 1, "seq": 4711, "publish_ns": 1794300000000000000, "source": "robot", "publish_called_ns": 1794299999990000000}
```

A reader takes the fields that it knows and ignores the others. [Timestamps](timestamps.md) explains each field.

## One definition, with versions

`cpp/include/xblick/protocol.hpp` defines the whole protocol: the topic names, the message types and their
versions. The Python package reads these values from the C++ core, so both languages always agree.

Every sample names its schema in its Zenoh [encoding](https://docs.rs/zenoh/1.10.1/zenoh/bytes/struct.Encoding.html):

```
application/cdr;sensor_msgs/msg/Image                 a ROS 2 type
application/cdr;xblick_msgs/msg/ShmImage@1            a Durchblick type, version 1
```

Durchblick's own types are the descriptors for the rings. Their version changes whenever their layout changes. A
subscriber that receives another version raises `ProtocolError` and names both versions. It does not try to read
the sample. The rings and their handshake carry versions in the same way.

For you this means: all publishers and subscribers of one setup speak the same protocol. The
[changelog](../../CHANGELOG.md) states which versions do.

## Every program declares itself

A Zenoh router knows which keys a session declares, and it does not know which program the session is. So every
program on Durchblick's library declares itself when it opens its session. Commands, nodes and the C++ tools do this
in `open_session` and `open_from_args`, and a node needs no code of its own for it.

| Key | Kind | Content |
| --- | --- | --- |
| `xblick/sessions/<zid>/program` | liveliness token | alive while the session is open |
| `xblick/sessions/<zid>/program` | queryable | the program record, JSON, answered at once |

`<zid>` is the Zenoh id of the session. The token appears and disappears with the session. A process that is killed
loses its token when its links close, which the kernel does at once for a local process. The record looks like this:

```json
{"v": 1, "kind": "program", "zid": "5a1c...", "program": "xblick-yolox", "version": "0.3.0",
 "xblick": "0.5.9", "language": "python", "names": ["test/camera/yolox"], "host": "robot-pc",
 "command": ["/usr/bin/python3", "/opt/venv/bin/xblick-yolox", "--camera", "test/camera"],
 "pid": 4711, "started_ns": 1794300000000000000}
```

| Field | Meaning |
| --- | --- |
| `program` | the name the program was started by, or the command, such as `xblick view` |
| `version` | the program's version: the package's for `xblick` and its commands, the version of the distribution that installs a node's console script, `null` when nothing names it |
| `xblick` | the version of Durchblick's core in that program |
| `names` | the camera or node names the program publishes for: a camera publisher adds its camera, `node.Reporter` adds `<camera>/<node>`, a recorder its name |
| `command` | the whole command line, the interpreter included |
| `started_ns` | the start of the process, in ns since the epoch |

The command line is visible to every program that reaches the network. It contains the paths and the arguments of
the program. It never contains the environment. Durchblick's programs take secrets from the environment, so do not
pass a secret as an argument.

`xblick router` writes the same record into its Zenoh metadata. The admin space of the router shows it under
`metadata`. A program of Durchblick 0.5.8 or older and any other Zenoh program declare nothing. They keep working,
and the terminal interface shows `–` as their program.

## A command's node and deadline

A command is a query that makes a node act: a parameter set, a controller manager's command, a torque step or a
command to the record and play node. Before a command, the caller reads the same node, and the command goes to the
node that answered. Three attachments carry what the node and the caller need for that. Each JSON object has the
field `xblick_command`, which holds the version 1.

A receiver puts this attachment on its replies, to the read and to the command:

```json
{"xblick_command": 1, "zid": "5a1c...", "clock_ns": 86400000000000, "checked": ["node", "deadline"]}
```

The caller then puts this attachment on the command's query:

```json
{"xblick_command": 1, "node": "5a1c...", "deadline_ns": 86402000000000}
```

| Field | Meaning |
| --- | --- |
| `zid` | the Zenoh id of the receiver's session |
| `clock_ns` | the receiver's `CLOCK_BOOTTIME` when it replied, in ns |
| `checked` | what the receiver checked before it executed the command. The list is empty for a read. |
| `node` | the Zenoh id of the session that the command is meant for |
| `deadline_ns` | the moment at which the caller stops waiting for the answer, on the receiver's `CLOCK_BOOTTIME` |

A receiver that takes part refuses a command with an error reply. Its encoding names the case:
`text/plain;xblick-not-target` for a command meant for another node, and `text/plain;xblick-expired` for a command
whose deadline has passed. It checks the node when the command arrives and the deadline just before the effect.

These rules decide how a receiver reads an attachment, the same in Python and C++:

- An attachment whose bytes do not contain `"xblick_command"` belongs to another program. The receiver ignores it.
- Otherwise the attachment must be a JSON object in UTF-8 of at most 64 KiB, nested at most 16 deep, with
  `xblick_command` at its top level and nothing after it. A receiver refuses anything else with
  `text/plain;xblick-invalid-command`.
- The known fields have fixed types: integers within int64 for `xblick_command`, `deadline_ns` and `clock_ns`,
  strings for `node` and `zid`, and a list of strings for `checked`. `true` is no integer, and `1.0` is none either.
- A receiver ignores unknown fields of any type. Version 1 grows by new fields. A receiver keeps answering version
  1, because a caller sends nothing to a read answer of another version.
- A deadline that is negative, or more than 24 h ahead of the receiver's clock, is refused as invalid.

A read answer without the reply attachment means only that its receiver does not take part, whatever its version.
The identity is the session's Zenoh id, which is random unless a Zenoh configuration sets `id`. Two programs given
the same id count as one node. A program older than 0.5.13 sends none of these attachments and reads none. The
[Python reference](../reference/python-api.md#node-and-deadline) explains the clocks and what the caller may claim.

## ROS 2 on rmw_zenoh

A program with `--ros` also sends on the keys of `rmw_zenoh_cpp`: 0.2.13 is the RMW of ROS 2 Jazzy, and 0.10.6 or
newer are those of ROS 2 Lyrical. The payload stays
the same CDR bytes. Only the key, the attachment and the announcements differ. `src/xblick/ros.py` implements them,
and [Durchblick and ROS 2](../integrations/ros2.md) explains what takes part.

| Part | Format |
| --- | --- |
| data key of a topic | `<domain>/<topic without the leading />/<pkg>::msg::dds_::<Name>_/RIHS01_<type hash>` |
| data key of a service | the same with the service's name, `<pkg>::srv::dds_::<Name>_` and the service type's hash |
| attachment | 33 bytes: the sequence number (int64), the source time in ns (int64), the length byte 16, the gid (16 bytes) |
| node token | `@ros2_lv/<domain>/<zid>/<node id>/<node id>/NN/<enclave>/<namespace>/<node>` |
| entity token | `@ros2_lv/<domain>/<zid>/<node id>/<entity id>/<MP\|MS\|SS\|SC>/<enclave>/<namespace>/<node>/<topic>/<type>/RIHS01_<hash>/<qos>`, in Lyrical optionally followed by `/backends:<name>:<meta>;...` |
| QoS in a token | `<reliability>:<durability>:<history>,<depth>:<deadline>:<lifespan>:<liveliness>`, each part empty where it is `rmw_zenoh`'s default |

Inside a token every `/` of a name is written as `%`. A ROS subscriber drops a sample without the attachment. A
service reply carries the request's sequence number and client gid with the reply's own time.

Four rules of `rmw_zenoh` decide how a program takes part:

- A transient-local publisher, such as one on `/tf_static`, is zenoh-ext's advanced publisher with a cache of `depth`
  samples. A subscriber that starts later asks it for them.
- A ROS service is a queryable declared complete. A ROS client queries with the target `ALL_COMPLETE`, which reaches
  only complete queryables. Our own queryables are never complete, and our keys never match ROS's keys.
- A type hash (RIHS01) is the SHA-256 of the type's description in a fixed JSON form. Durchblick computes it as
  ROS does. `tests/test_ros.py` compares the result with every type of a ROS 2 Jazzy installation.
- In Lyrical, a message type with a `uint8[]` field, its own or one of a type it uses, is a buffer type. Image,
  CompressedImage and PointCloud2 are the buffer types among ours. A Lyrical publisher of a buffer type sends to a
  volatile subscription on `<key>/_buf_cpu` when its token ends in `backends:cpu:`. It sends to any other
  subscription on `<key>/_buf/<the subscription's gid>`. It uses the key itself only while it has not yet seen
  every subscription, and it sends each message on one of these ways only. Our volatile subscriptions of buffer
  types therefore end their token in `backends:cpu:` and also read `<key>/_buf_cpu`. The bytes and the attachment
  there are the same as on the key. Jazzy reads such a token and ignores the last part.

## Related pages

- [Durchblick and ROS 2](../integrations/ros2.md): what is compatible with ROS 2 and what `--ros` adds.
- [Topic reference](../reference/topics.md): which type travels on which topic.
- [Development](../project/development.md): what you must do when you change a message.
