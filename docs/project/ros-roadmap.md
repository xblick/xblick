# ROS 2 by default: facts for a decision

Today a program takes part in ROS 2 only with `--ros` ([Durchblick and ROS 2](../integrations/ros2.md)). This page
collects what we know about a later step, in which Durchblick speaks ROS 2 without the option. It takes no decision.
Each area lists the facts with their source and then the options with their consequences.

The facts refer to ROS 2 Jazzy with `rmw_zenoh_cpp` 0.2.13. "rmw_zenoh" below means that version. Its source is the
tag [0.2.13](https://github.com/ros2/rmw_zenoh/tree/0.2.13/rmw_zenoh_cpp/src/detail) on GitHub.

## What "by default" would mean

A program with `--ros` sends on our keys and also on rmw_zenoh's keys. It announces itself twice, once by our
declaration and once by ROS's liveliness tokens. "ROS by default" can mean one of three things:

| Option | Our keys | ROS keys | Consequence |
| --- | --- | --- | --- |
| A. `--ros` on by default | stay | added | The double publication of today on every PC. Older programs keep working. |
| B. ROS keys for every ROS type, our keys for the rest | only for types without a ROS type | the main keys | A program of 0.5.x no longer finds the streams. This changes the wire. |
| C. ROS keys only | gone | everything | Rings, RVL and JSON need ROS types of their own. This changes the wire. |

Options B and C change the protocol, so they need a new minor version (0.6). Option A changes a default and the
traffic, but no byte of our own keys.

## Keys

| Part | Our key | rmw_zenoh's key |
| --- | --- | --- |
| form | `test/camera/color/image_raw` | `<domain>/test/camera/color/image_raw/sensor_msgs::msg::dds_::Image_/RIHS01_<hash>` |
| message type | in the Zenoh encoding, `application/cdr;sensor_msgs/msg/Image` | in the key, with its type hash |
| encoding field | our type string | `zenoh/bytes` |
| domain | none | the ROS domain id as the first part |

Sources: `src/xblick/ros.py` (`data_key`), [protocol](../concepts/protocol.md#ros-2-on-rmw_zenoh), and keys
captured from ROS's `talker` in `tests/test_ros.py`.

What follows from the key format:

- A ROS key holds the type and its hash. A reader with another hash of the same type never matches the key, and it
  receives nothing.
- A ROS key starts with the domain. Every program would need a domain, and `ROS_DOMAIN_ID` (default 0) would decide
  who sees whom. Our keys have no domain today.
- ROS names allow letters, digits and `_`, and no part may start with a digit. A key that breaks this rule stops a
  program with `--ros` at start. All default keys of the ten Python nodes and of the Azure Kinect and rc_visard
  programs pass this rule (checked with `ros.topic_of` on their branches `ros2`).
- A recording keeps each channel's Zenoh key, encoding and info record ([recording](../guides/recording.md)).
  Recordings made before a change of keys hold our keys. `xblick play --ros` already maps them to ROS keys, so
  playback of old recordings stays possible if `play` keeps that mapping.

## The per-sample attachment

rmw_zenoh puts 33 bytes on every sample, request and reply. Our keys carry a JSON object.

| Field | rmw_zenoh (33 bytes) | Ours (JSON, [timestamps](../concepts/timestamps.md)) |
| --- | --- | --- |
| sequence number | int64, per publisher, from 1, one step per sample | `seq`, the count of the publisher's frames, the same on all topics of one frame |
| time | int64 source timestamp: system time in `rmw_publish`, right before the put | `publish_ns`: right before the put of this sample |
| publisher | gid, 16 bytes (XXH3-128 of the entity's token) | `source`, a name, and in some samples `zid` |
| call time | none | `publish_called_ns`: when the code called `publish()` |
| version | none | `v` |

Sources: [rmw_publisher_data.cpp](https://github.com/ros2/rmw_zenoh/blob/0.2.13/rmw_zenoh_cpp/src/detail/rmw_publisher_data.cpp#L300-L315)
and [attachment_helpers.cpp](https://github.com/ros2/rmw_zenoh/blob/0.2.13/rmw_zenoh_cpp/src/detail/attachment_helpers.cpp#L67-L82).

ROS's time field is close to our `publish_ns`, since both are taken right before the put. ROS's sequence number
counts samples per publisher. Ours counts frames and is shared by all topics of a frame. rmw_zenoh reports lost
messages whenever the number on one publisher jumps by more than one
([rmw_subscription_data.cpp](https://github.com/ros2/rmw_zenoh/blob/0.2.13/rmw_zenoh_cpp/src/detail/rmw_subscription_data.cpp#L528-L545)).
A frame count in ROS's field would therefore raise lost-message events whenever a publisher skips frames on that key.
`source`, `publish_called_ns` and `v` have no place in ROS's 33 bytes.

### A longer attachment

rmw_zenoh reads its three fields with zenoh-ext's `Deserializer` and never checks that the bytes end there
(`AttachmentData(const zenoh::Bytes &)`, lines 76 to 82 of `attachment_helpers.cpp`). A probe published on ROS's
`/chatter` key with modified attachments, and ROS's `listener` received or rejected the samples as follows
(2026-11-17, ten samples per case):

| Attachment | ROS's `listener` |
| --- | --- |
| ROS's 33 bytes | heard 10, no log line |
| 33 bytes and 1 byte more | heard 10, no log line |
| 33 bytes and our JSON object (108 bytes) | heard 10, no log line |
| 33 bytes and 4096 bytes more | heard 10, no log line |
| none | heard 0, one error line per sample ("Unable to obtain attachment"), the process goes on |
| 32 bytes | heard 0, the process aborts (SIGABRT after "Failed to deserialize the payload") |
| our JSON object alone | heard 0, the process aborts in the same way |

The abort means that every program that writes on a ROS key must write at least ROS's 33 well-formed bytes. A short
or foreign attachment on a ROS key ends every rmw_zenoh 0.2.13 subscriber of that key.

Our own readers today, from the same probe and the code:

- `ros.decode_attachment` (Python) and `xblick::ros::decode_attachment` (C++) take exactly 33 bytes. Our ROS
  subscriber dropped a sample with 33 bytes and our JSON appended, with one warning line. That line says "as ROS drops
  it", which holds for shorter attachments and is wrong for longer ones.
- `camera.parse_attachment` (Python) raises `UnicodeDecodeError` on ROS's 33 bytes, with or without our JSON
  appended. The C++ reader parses the attachment as JSON too. Our readers never see ROS's bytes today, because the
  keys differ.

To carry our fields inside ROS's attachment, these parts would change:

1. The ROS publishers append our fields after the 33 bytes, as JSON or in a fixed binary form.
2. Both `decode_attachment` functions accept 33 bytes or more, check byte 16, and hand on the rest.
3. Our readers of a ROS key parse the rest as our fields, and they treat a sample without it as a ROS sample.
4. If our own keys used the same layout, `parse_attachment` in both languages would need to skip the 33 bytes first.

That rmw_zenoh ignores the extra bytes is a property of 0.2.13 that it does not document. Other versions are
untested.

## Discovery

| | Ours | rmw_zenoh |
| --- | --- | --- |
| program | token and queryable `xblick/sessions/<zid>/program`, a JSON record ([protocol](../concepts/protocol.md#every-program-declares-itself)) | node token `@ros2_lv/<domain>/<zid>/<node id>/<node id>/NN/<enclave>/<namespace>/<node>` |
| stream | the queryable `<key>/info`, an info record with the kind and the type | one token per publisher, subscriber, service and client, with type, hash and QoS |
| used by | `xblick list`, `xblick rerun`, the TUI, our readers | `ros2 node list`, `ros2 topic list`, zenoh-ext's detection of late publishers |

With `--ros` a program declares both today. The options are both kinds (as today), or ROS's tokens only. ROS's
tokens carry no program record and no info record. Our tools would then need another source for the program, its
version and a stream's kind.

## Type hashes

`ros.TYPE_HASHES` and the C++ table hold the RIHS01 hash of every type that Durchblick sends to ROS. All of them are
ROS's own: those of a Jazzy install and those of `ros-jazzy-vision-msgs` 4.1.1. Our implementation of the hash gives
ROS's hash for every message and service type of a Jazzy install. `tests/test_ros.py` checks this when ROS is
installed. A typed `rclcpp` subscription deserialises our `Detection2DArray` and `Detection3DArray`
(`tests/test_ros_live.py`).

A hash follows from the type's definition. A ROS distribution that changes a definition changes the hash, and with
it the key. A table per distribution would then be needed. Only Jazzy is tested.

## Forms without a ROS type

These forms have no ROS type: ring descriptors in host and GPU memory, RVL depth, clouds on a ray table, JSON keys and
the `xblick_msgs` types. With `--ros` they stay on our keys only, with one warning line each.

| Option | Consequence |
| --- | --- |
| They stay on our keys, beside ROS keys for the rest | Two key schemes in one system. Our readers of rings find them as today. |
| A ROS package `xblick_msgs` with these types | ROS nodes could read them after installing it. Their hashes join the table. A ring descriptor is still useless on another PC. |

## Transforms

Ours: each camera, robot and world has its own `<camera>/tf` and `<camera>/tf_static`. With `--ros` all of them also
go to ROS's global `/tf` and `/tf_static`. tf2 keeps one parent per frame. A program that gives one frame two parents
writes one error line. A clash between two programs shows only in tf2
([guide](../integrations/ros2.md#several-cameras-and-tf)).

By default, every program would share one tf tree. Frame names would then have to be unique across all programs
of a domain. Our per-camera keys keep two cameras apart today, even when both use the same frame names.

## Parameters and services

Our set is a command with the node and the deadline in its attachment. Our callers count the nodes of the name first
([protocol](../concepts/protocol.md#a-commands-node-and-deadline)). A ROS client does neither, and it reaches every
complete queryable of the key. These points came from the reviews of the branch, and each has a test that must stay
true:

| Rule | Test |
| --- | --- |
| ROS sets are refused unless `--ros-params-writable` | `test_ros_sets_are_refused_by_default`, live in `test_ros_parameter_client_on_our_node` |
| two nodes of one name both execute a ROS set, and our client refuses to send it | `test_two_nodes_of_one_name_and_a_ros_set` |
| a key with a part `command` never goes to ROS | `test_command_keys_never_go_to_ros` |
| `matched` counts our key only, `ros_matched` the ROS key | `test_matched_is_our_key_and_ros_matched_ros` |
| only ROS keys have complete queryables | the source pin in `tests/test_queries.py` |
| at most four ROS calls per service and round (Python) | `ROS_CALLS_PER_ROUND` in `parameters.py` |
| read-only access holds on the ROS path | `test_read_only_access_holds_on_the_ros_path` |

With ROS by default, every node would answer ROS clients on its domain. The read-only default keeps a ROS tool from
changing a node unless its start allows it.

## QoS and transient local

Our ROS publishers use reliable, volatile, keep-last 1 for streams, and transient local with depth 1 for
`/tf_static` (`STREAM_QOS`, `TF_STATIC_QOS` in `ros.py`). rmw_zenoh keeps transient-local samples with zenoh-ext's
advanced publisher. Python uses the same API. The standalone zenoh-c that our C++ links lacks it. The C++ publisher
answers late subscribers from its own cache, so a late ROS subscriber may get a message twice.

## Zenoh version

rmw_zenoh 0.2.13 on Jazzy links zenoh-c 1.10.1 (`zenoh_cpp_vendor`). We depend on `eclipse-zenoh>=1.10,<2`. With
ROS by default, our Zenoh version would follow what each ROS distribution's rmw_zenoh brings. A Zenoh update on our
side would then wait for ROS, or break the link to ROS.

## Routers and ports

`xblick router` and ROS's `rmw_zenohd` both listen on 7447. Either one serves both sides, so one router per PC is
enough. Our tests ran our sessions against `rmw_zenohd`, and ROS's `talker` and `listener` through `xblick router`,
both on other ports. Two PCs are untested.

## The cost of the double publication

A mirror sends only while a ROS subscriber is there, except `/tf_static`. While one is there, every sample is put
twice, on our key and on ROS's key. Every ROS entity also holds a liveliness token, with or without a subscriber. We
have not measured the cost in time, bandwidth or memory.

## A possible path in steps

Each step is an option, with what it changes.

1. Our ROS decoders accept attachments of 33 bytes or more. This changes no byte on the wire.
2. `--ros` becomes the default, with `--no-ros` to switch it off. The traffic doubles while ROS subscribers are
   there. Older programs keep working.
3. ROS publishers append our fields after ROS's 33 bytes, and our ROS readers read them. This is a wire change for
   ROS keys only.
4. Our readers read ROS keys for every form with a ROS type. Our own keys remain for the other forms. This is a wire
   change: 0.5.x programs no longer find those streams.
5. Our keys disappear for forms with a ROS type, or a ROS package carries the other forms too.

## Open questions and risks

- A sample on a ROS key with a short or foreign attachment aborts rmw_zenoh 0.2.13 subscribers. Every writer on ROS
  keys must be correct, including `xblick play` and tools made by hand.
- Other ROS distributions, other rmw_zenoh versions and other RMWs are untested.
- `ros2` command line tools, `rviz2` and two PCs are untested.
- The default domain, and how a lab keeps several setups apart, is open.
- A node with parameters shows as two ROS nodes, the program's and its parameters'.
- `/clock` is `/<world>/clock`, and diagnostics go to `/<camera>/diagnostics`. ROS tools expect `/clock` and
  `/diagnostics`.
- A key with `-` or a part that starts with a digit cannot become a ROS name. A default would have to reject or rename
  such keys.
