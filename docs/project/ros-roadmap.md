# ROS 2 by default: facts for a decision

Every program now takes part in ROS 2 by default, beside our own keys, and `--no-ros` turns it off
([Durchblick and ROS 2](../integrations/ros2.md)). This is option A below. This page collects the facts behind that
step and behind the options B and C, which would change our own keys. Where a passage says "with `--ros`", it means
a program in ROS 2. Each area lists the facts with their source and then the options with their consequences.

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
- ROS names allow letters, digits and `_`, and no part may start with a digit. Most of our publishers check the same
  rule for their keys, with or without `--ros` ([Topic names](#topic-names)). A key that breaks it and reaches a ROS
  mirror stops a program with `--ros` at start. All default keys of the ten Python nodes and of the Azure Kinect
  and rc_visard programs pass this rule: `ros.topic_of` takes each of them on their branches `ros2`.
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

That rmw_zenoh ignores the extra bytes is a property of its code that it does not document. The same decoding code
is in the releases for Humble, Kilted and Rolling (next section); for them this follows from the code alone.

### Which versions use these 33 bytes

ROS 2 requires metadata of each message, not a byte layout. An RMW fills `rmw_message_info_t` with the publisher's
gid, the sequence numbers and the source and reception times. How these travel is the RMW's own choice. A DDS RMW
has no Zenoh attachment at all. The 33 bytes are therefore rmw_zenoh's format, and they are not a ROS 2 format.

The layout in rmw_zenoh's tags (`rmw_zenoh_cpp/src/detail/attachment_helpers.cpp`, compared by its source):

| rmw_zenoh | ROS 2 | Attachment |
| --- | --- | --- |
| 0.1.0 | Humble | a name before each field: `"sequence_number"`, `"source_timestamp"`, `"source_gid"` |
| 0.1.12 (branch `humble`) | Humble | the 33 bytes |
| 0.2.0 to 0.2.3 | Jazzy | a name before each field, as 0.1.0 |
| 0.2.4 to 0.2.13 (branch `jazzy`) | Jazzy | the 33 bytes |
| 0.6.10 (branch `kilted`) | Kilted | the 33 bytes, the same code as 0.2.13 |
| 0.14.1 (branch `rolling`) | Rolling | the 33 bytes, the same reading and writing code |

The layout changed once within Jazzy, between 0.2.3 and 0.2.4. A subscriber of one layout that gets the other cannot
deserialise it, and our probe showed that such a failure aborts the subscriber process. Humble's rmw_zenoh writes
`TypeHashNotSupported` where Jazzy's key holds the type hash (`rmw_publisher_data.cpp` on branch `humble`). Our keys
therefore never match a Humble node.

What follows:

- Our compatibility is a stated requirement, "`rmw_zenoh_cpp` 0.2.13 on Jazzy", not "ROS 2".
- A program could check the version of the ROS side before it publishes on ROS keys. rmw_zenoh's tokens carry no
  version, so such a check needs another source, such as an option or a test of the attachment of a received sample.
- Only a test matrix tells which versions work: Jazzy, Kilted and Rolling, in container images or RoboStack
  environments. None of these exists yet.

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
it the key. A table per distribution would then be needed.

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
enough. It may listen on another port when both sides name that endpoint. A setup across two PCs is an open
question (below).

## The cost of the double publication

A mirror sends only while Zenoh reports a subscriber of its ROS key, except `/tf_static`. The code checks this before
each put (`ros_matched` in `camera.py`, `matched` in `detections.py`, `diagnostics.py`, `points.py` and
`parameters.py`). Any Zenoh subscriber of the key counts, so a storage or recorder on `**` also starts the mirrors.
While a subscriber is there, every sample is put twice, on our key and on ROS's key. Every ROS entity also holds a
liveliness token, with or without a subscriber. We have not measured the cost in time, bandwidth or memory.

## ROS and our programs on the same topics

Our key and ROS's key for one topic can never be equal, because ROS's key also holds the domain, the type and the
hash. "The same topic" therefore means two keys that a program connects. Three variants exist.

| Variant | What it does | Wire | Version |
| --- | --- | --- | --- |
| 1. Readers take both | A reader subscribes to our key and to the ROS topic of the same name. It takes ours when both arrive, and ROS's when only a ROS publisher exists, such as a ROS camera driver. | unchanged | fits 0.5 |
| 2. Publishers serve both by default | `--ros` is on by default. The ROS copy goes out only while a subscriber of its key is there. | our keys unchanged, ROS keys added | a changed default |
| 3. One key for both | Programs publish only on ROS's key, with ROS's 33 bytes and our fields after them. | changed | 0.6 |

Variant 1 in detail:

- Our form carries more: rings, RVL, the ray-table clouds and our attachment fields. A reader therefore prefers it.
- Two copies of one frame arrive when a program publishes both. The reader has to drop one, for example by the
  header's stamp and frame id. No such code exists today.
- A frame read from ROS has no `seq`, `source` or publish time.

Variant 2 in detail:

- The cost is the tokens of every program, plus the copies while a subscriber is there.
- The domain is 0 unless `ROS_DOMAIN_ID` says otherwise. Our programs would then appear in every ROS 2 system on the
  same network and domain, for example in `ros2 node list` and in rviz.

Variant 3 depends on the undocumented tolerance of the 33-byte reader for a longer attachment. It needs the version
matrix of the previous section first.

Rules for every variant:

- A key with a part `command` never goes to ROS (`ros.commanding`, used by `ros.mirror`, `ros.mirror_tf` and
  `xblick play`). This rule covers the sending side only. A reader that takes ROS topics would need the same rule, so
  that no robot command is taken from a ROS topic.
- Forms without a ROS type stay on our keys only.

## Topic names

A ROS topic is another name for one of our keys: the key `cam/x/color` is the topic `/cam/x/color` (`ros.topic_of`).

### Characters

Our rule (`topics.check_name`): each part of a key consists of letters, digits and `_` and does not start with a
digit (`[A-Za-z_][A-Za-z0-9_]*`). Leading and trailing `/` are removed. The publishers of streams, clouds,
detections, poses, parameters and the controller manager check their keys with it, with or without `--ros`. A key
`cam-1/color` or `0cam/color` is therefore refused today. Internal keys such as `xblick/sessions/<zid>/program` do
not pass this check, since a Zenoh id may start with a digit.

ROS's rule for a full topic name (`rmw_validate_full_topic_name` in `rmw`, Jazzy):

- It starts with `/` and does not end with `/`.
- It has no `//`.
- It uses letters, digits, `_` and `/` only, and no part starts with a digit.
- It has at most 247 characters (`RMW_TOPIC_MAX_NAME_LENGTH`).

Before a name gets there, `rcl` also expands `~` (the node's name) and `{}` substitutions. A part that starts with `_`
makes the name hidden: `ros2 topic list` does not show it without `--include-hidden-topics` (rclpy,
`topic_or_service_is_hidden`). The design article
[Topic and service names](https://design.ros2.org/articles/topic_and_service_names.html) also asks for no repeated
`_`, but `rcl` and `rmw` do not check it.

Both rules allow the same characters per part. The differences are these:

| Name | Ours | ROS |
| --- | --- | --- |
| a part that starts with `_` | allowed | allowed, the topic is hidden |
| `__` in a part | allowed | allowed by the code, against the design article |
| more than 247 characters | allowed | refused |

All default keys of the ten Python nodes and of the Azure Kinect and rc_visard programs pass `ros.topic_of`. No
default name would change if ROS's rule applied to all names. Such a rule would change what we accept, and no byte
on the wire.

### Layout

A camera stream is one key base for us, with its forms below it. ROS splits it into sibling topics by the
convention of `image_transport`: `image_raw`, `image_raw/compressed` and `camera_info`. Our mapping to ROS follows
that convention ([guide](../integrations/ros2.md#what-ros-2-receives)). The reverse direction reads a foreign ROS
camera by the same convention, which `ImageSubscriber(..., transport="ros")` does. For other ROS types there is no
general rule that groups topics into one stream.

Other differences of layout:

- Transforms: ours are per camera, robot and world. ROS has one `/tf` and one `/tf_static`.
- Clouds: ROS gets the plain `PointCloud2` on `/<stream>/cloud`. Our compressed and ray-table forms have no ROS topic.

### What a ROS name carries beyond the name

- The type: one name with two types is two ROS keys, and a subscriber sees only the one of its type.
- The domain: the same name in two domains is two topics.
- Names with a role in ROS: `/rosout`, `/parameter_events`, `/tf`, `/tf_static`, `/clock` and `/diagnostics`, and
  the parameter services below each node's name. A key of ours with such a name would mean something to ROS tools.

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
- Whether ROS distributions other than Jazzy and Lyrical, other rmw_zenoh versions and other RMWs work is open.
- Whether the `ros2` command line tools, `rviz2` and a setup across two PCs work is open.
- The default domain, and how a lab keeps several setups apart, is open.
- A node with parameters shows as two ROS nodes, the program's and its parameters'.
- `/clock` is `/<world>/clock`, and diagnostics go to `/<camera>/diagnostics`. ROS tools expect `/clock` and
  `/diagnostics`.
- A key that does not pass `topics.check_name` may hold `-` or a part that starts with a digit. It cannot become a ROS
  name. A default would have to reject or rename such keys.
