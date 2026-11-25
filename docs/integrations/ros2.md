# Durchblick and ROS 2

Durchblick uses the message types and conventions of ROS 2, and it needs no ROS installation. This page states
exactly what is compatible with ROS 2 and what is not.

## What is identical

| Aspect | State |
| --- | --- |
| Message definitions and bytes on the wire | identical. Tests compare every message type with an independent CDR implementation ([protocol](../concepts/protocol.md)). |
| Topic layout of a camera stream | identical to `image_transport`: `image_raw`, `image_raw/compressed`, `camera_info` |
| Transforms, diagnostics, clock | the types and roles of `/tf`, `/tf_static`, `/diagnostics` and `/clock` |
| Parameters | the messages and services of `rcl_interfaces`, with the same bytes. Only the keys follow Durchblick's naming ([parameters](../reference/parameters.md)). |
| Calibration files | the YAML format of `camera_calibration`. `CameraInfo.read_file` reads it and `write_file` writes it. |
| Recordings | MCAP files with ROS 2 schemas, which ROS 2 tools open ([recording](../guides/recording.md)) |

## What is not compatible

A ROS 2 node on `rmw_zenoh` does not see Durchblick's streams without a bridge. `rmw_zenoh` puts the domain id, the
type name and a type hash into the Zenoh key, and it announces every entity with a liveliness token. Durchblick's
keys are the plain topic names.

Durchblick's own topics for the rings (`image_raw/shm/<host>`, `image_raw/cuda/<host>/<gpu>`) have no counterpart in ROS 2.

## ROS 2 on DDS through a bridge

ROS 2 nodes on a DDS middleware (`rmw_cyclonedds_cpp`, `rmw_fastrtps_cpp`) read Durchblick's topics through
`zenoh-bridge-ros2dds` of Eclipse Zenoh. The bridge runs next to the ROS nodes. It connects to a Durchblick router as
a Zenoh client and writes each Zenoh sample unchanged to DDS. Durchblick's payloads are already ROS 2 messages in CDR,
so the ROS nodes read them as they are. Durchblick's programs need no option for this.

You need `zenoh-bridge-ros2dds` of the Zenoh version that Durchblick uses (1.10). The bridge needs no ROS
installation.

What works through the bridge:

- Topics with a ROS 2 message reach ROS nodes on Cyclone DDS and on Fast DDS. Examples are `image_raw`,
  `image_raw/compressed`, `camera_info`, `cloud`, `<camera>/tf` and `<camera>/diagnostics`.
- A ROS subscriber with the durability "transient local" gets `<camera>/tf_static` also when it starts after the
  camera. Durchblick answers the bridge's request for the latched message.
- `ros2 service call` reaches the parameter services of a Durchblick node, when the ROS side uses Cyclone DDS. The
  bridge offers services to Cyclone DDS only.

### Run the bridge

The steps show one PC on which Durchblick and the ROS nodes run, with ROS discovery limited to that PC.

1. Start the router and a camera, here the test camera with a fixed pose:

   ```bash
   xblick router
   xblick test-camera --pose 0 0 1.5 0 10 0
   ```

2. Start the bridge in client mode. Give it the ROS side's domain id and distribution:

   ```bash
   CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface address="127.0.0.1"/></Interfaces></General></Domain></CycloneDDS>' \
   ROS_DISTRO=jazzy zenoh-bridge-ros2dds client -e tcp/127.0.0.1:7447 --no-multicast-scouting \
     -d 0 --ros-automatic-discovery-range LOCALHOST
   ```

   `CYCLONEDDS_URI` binds the bridge's DDS to the loopback interface. With `LOCALHOST`, the ROS nodes talk over
   loopback. The bridge's DDS chooses a network interface of its own, and without this setting the two sides do not
   find each other. Leave the setting out when the ROS nodes run on other PCs.

3. Read a topic on the ROS side. Name the message type:

   ```bash
   export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
   ros2 topic echo --once /test/camera/color/camera_info sensor_msgs/msg/CameraInfo
   ros2 topic echo --once --qos-durability transient_local /test/camera/tf_static tf2_msgs/msg/TFMessage
   ```

   `ros2 topic list` shows a Durchblick topic only while a ROS subscriber of it exists. The bridge creates the DDS
   writer of a topic when a ROS node subscribes, with the type that the subscriber names. For the same reason,
   `ros2 topic echo` and `ros2 topic hz` need the type on the command line.

4. Call a parameter service of a node, for example of `cam/azurekinect`:

   ```bash
   ros2 service call /cam/azurekinect/list_parameters rcl_interfaces/srv/ListParameters "{}"
   ```

## What could be added

- The key format of `rmw_zenoh` would make the streams visible to ROS 2 nodes on `rmw_zenoh`. Nobody has needed it
  so far.
- ROS 2 Lyrical has buffer backends (`rosidl::Buffer`), and NVIDIA provides one for CUDA. A ROS mode of Durchblick
  could offer its CUDA ring as such a backend. ROS 2 nodes, including Isaac ROS, could then read Durchblick's frames
  in place. [Alternatives](../project/alternatives.md) describes these projects.

To look at streams you need no ROS. Use [`xblick rerun`](../guides/viewer.md).

## Status and limits

These limits hold for ROS 2 nodes behind `zenoh-bridge-ros2dds`:

- A camera's transforms arrive on `/<camera>/tf` and `/<camera>/tf_static`. ROS's tf2 reads `/tf` and `/tf_static`.
  The bridge's option `-n /<camera>` maps the camera's keys to the ROS root, which serves one camera per bridge.
- `ros2 param` does not find Durchblick's nodes, because the ROS graph has no node of their names. Use
  `ros2 service call` for their parameter services.
- The bridge carries data from Durchblick to ROS only. Durchblick's subscribers do not read what ROS nodes publish
  through the bridge. Such samples carry no Durchblick encoding, and no publisher answers their stream's info query.
- ROS subscribers receive every frame as a message over Zenoh and DDS. Shared memory and the CUDA ring serve
  Durchblick's own subscribers only.
