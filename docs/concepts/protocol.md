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

## Related pages

- [Durchblick and ROS 2](../integrations/ros2.md): what is compatible with ROS 2 and what is not.
- [Topic reference](../reference/topics.md): which type travels on which topic.
- [Development](../project/development.md): what you must do when you change a message.
