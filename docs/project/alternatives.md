# Alternatives

Other projects that pass camera frames or tensors from process to process without a copy, in particular in GPU
memory. We surveyed them in November 2026 from documentation, source code, announcements and pull requests. We did
not run them. Statements marked "unverified" could not be confirmed in the sources.

## Summary

ROS 2 Lyrical with the CUDA buffer backend is the closest alternative. Like the CUDA ring of Durchblick, it shares
GPU memory between processes on one host. Isaac ROS builds on it. [ROS 2 Lyrical](#ros-2-lyrical) compares the two.

We found no project that combines the following properties of Durchblick in one package.

- One publisher serves host rings, CUDA rings and compressed or raw network streams at the same time, and each
  subscriber chooses.
- A reader on another GPU of the same host receives the frames by a copy from GPU to GPU.
- A producer never waits for a reader, and a stream holds a fixed number of frames.
- The messages are ROS 2 messages, and no ROS is needed.
- Python and C++ have the same stream API.
- Camera nodes, a recorder and a viewer belong to it.

## ROS 2 Lyrical

ROS 2 Lyrical was released in May 2026. It can pass GPU memory between processes. In C++, every `uint8[]` field of
a message is a [`rosidl::Buffer`](https://docs.ros.org/en/lyrical/ROS-Framework/interfaces/Working-with-interfaces/Buffer-Backends/About-Buffer-Backends.html),
for example the pixels of `sensor_msgs/msg/Image`. A buffer backend decides where the memory of such a field lives.
The CUDA backend [`cuda_buffer_backend`](https://index.ros.org/p/cuda_buffer_backend/) is part of
[rosidl_buffer_backends](https://github.com/ros2/rosidl_buffer_backends), whose packages name NVIDIA staff as
maintainers. The ROS 2 documentation calls buffer backends experimental.

The publisher takes the memory of a frame from a pool on its GPU. A subscriber that accepts the CUDA backend receives
a descriptor of at most 4 KiB and maps that memory into its own process. Every other subscriber receives the message
serialised on the CPU.

The GPU path has these conditions in Lyrical:

- The middleware is `rmw_fastrtps_cpp`, or `rmw_zenoh_cpp` 0.10.5 or newer. `rmw_cyclonedds_cpp` and
  `rmw_connextdds` do not implement buffer backends.
- Publisher and subscriber run on the same host, on the same GPU and as the same Linux user. Otherwise the
  subscriber receives a copy through the CPU.
- The message travels on a topic with volatile durability. Services and actions always serialise on the CPU.
- The ROS 2 documentation names C++ as the language of the backend API. The repository also has Python packages,
  which give PyTorch tensors. These packages are newer than its release 0.1.2.

Isaac ROS nodes use `rosidl::Buffer` and the CUDA backend since Isaac ROS 5.0
([NVIDIA blog](https://developer.nvidia.com/blog/?p=122571)). Isaac ROS 5.1 marks NITROS as deprecated.

### Where they differ

| Need | ROS 2 Lyrical | Durchblick |
| --- | --- | --- |
| GPU frames between processes on one host | The CUDA backend shares them for the same GPU and user. | A [CUDA ring](../concepts/gpu-frames.md) shares them for the same user, in Python and C++. |
| A reader on another GPU of the host | The subscriber receives a copy through the CPU. | The reader copies each frame from GPU to GPU. The copy goes through host memory only where the GPUs cannot reach each other. |
| Subscribers with and without the GPU path on one topic | On `rmw_fastrtps_cpp`, one matched subscriber without buffer support turns the buffer path off for every subscriber of the topic. | Each subscriber chooses its [transport](../concepts/transports.md). The choice of one subscriber changes nothing for the others. |
| Memory of a stream | The backend takes blocks from a pool, which grows while no free block is ready. The publisher reuses a block at the earliest 100 ms after its last reader released it. | A [ring](../concepts/rings.md) holds 32 images or 4 point clouds. The writer overwrites the oldest slot and never waits. A reader detects a frame that was overwritten during its read. |
| Frames on the CPU of one host | Fast DDS uses shared memory between processes on one host by default. `rmw_zenoh_cpp` can put the serialised message into shared memory, which is off by default. | A host ring holds each frame once for all readers on the host. |
| Pairing and rate | `message_filters` pairs streams by stamp, in C++ and Python. `topic_tools throttle` limits the rate of one topic. | In Python, `RGBDInputs` limits the rate of the primary stream and gives the newest stamp for which every stream has a frame. It reads a partner frame only when it pairs it ([pairing](../guides/pairing-streams.md)). |
| Depth and point clouds over the network | `compressed_depth_image_transport` writes PNG or RVL. `point_cloud_transport` offers Draco, which loses precision, and the lossless zlib and zstd. | Depth travels as RVL or PNG. A camera point cloud travels as the ranges on its rays plus colours. A subscriber with `auto` on another host takes the compressed form. |
| Video codecs | `ffmpeg_image_transport` encodes H.264 and HEVC, also on the GPU with NVENC. | Durchblick has no video codec for streams. |
| Age of a frame | `ros2 topic delay` shows the age at receipt from the header stamp. `rclcpp` in Lyrical has topic statistics. | Every image and point cloud sample carries its publish time. A subscriber gets the age at send and at receipt of every frame ([timestamps](../concepts/timestamps.md)). |
| Simulated time | Every node can follow `/clock` with `use_sim_time`. | A simulation publishes a clock, and `xblick play` can follow it. Nodes work from the stamps of their inputs and have no simulated clock. |
| Tools | launch, lifecycle nodes, composition, the `ros2` command, rqt and rosbag2 | the `xblick` command with a terminal interface, recording and playback. A node is a plain program without a base class or an executor. |
| Several Linux users on one host | The CUDA backend sends a copy through the CPU to a subscriber of another user. | Rings serve only the user who runs the publisher. A subscriber of another user must ask for `raw` or `compressed`. |
| Installation | a ROS 2 installation. The ROS 2 documentation suggests one virtual environment per workspace and warns that conda is likely incompatible with the ROS 2 binaries. | a pip package without ROS. Each node can run in a Python environment of its own. |
| Ecosystem | many packages, among them Isaac ROS. NVIDIA maintains the GPU path. | the camera and model nodes of Durchblick |

### Which one to choose

Choose ROS 2 when your system already runs on ROS 2, or when it needs the packages and tools of ROS 2, simulated
time or video encoding on the GPU. Choose it also when the GPU path should have the support of NVIDIA.

Choose Durchblick for perception nodes in Python or C++ that share cameras between programs, GPUs and hosts without
a ROS installation. It fits when each node needs its own Python environment, when subscribers mix GPU, host and
network transports, or when the memory of a stream must stay fixed.

Durchblick uses the message types of ROS 2. Every program also takes part in ROS 2 on
`rmw_zenoh_cpp`, and ROS 2 nodes on DDS read its topics through a bridge. ROS 2 nodes receive serialised messages
on both ways, not the rings. [Durchblick and ROS 2](../integrations/ros2.md) explains both.

## Other projects for ROS 2

| Project | What it does | Limits |
| --- | --- | --- |
| [Isaac ROS NITROS](https://nvidia-isaac-ros.github.io/concepts/nitros/index.html) (before Isaac ROS 5.0) | type adaptation and negotiation (REP 2007/2009), GPU memory between nodes | Zero-copy only within one process. The ROS 1 bridge uses CUDA IPC. Isaac ROS 5.1 marks it as deprecated. |
| [ros2_cuda_ipc](https://discourse.openrobotics.org/t/ros2-cuda-ipc-zero-copy-gpu-data-sharing-between-ros-2-processes/57157) (2026-08) | CUDA VMM file descriptors and IPC events between ROS 2 processes, lifetime handling for several consumers, DLPack | experimental, one author |
| [Autoware cuda_blackboard](https://awesome.ecosyste.ms/projects/github.com%2Fautowarefoundation%2Fcuda_blackboard) | GPU data between ROS 2 nodes | Each further subscriber costs a copy from device to device. |
| [Agnocast](https://discourse.openrobotics.org/t/agnocast-callback-isolated-executor-true-zero-copy-ipc-and-middleware-transparent-scheduling-for-ros-2/52678) (Autoware Foundation) | zero-copy between ROS 2 processes | CPU memory only |

## Zenoh and Rust

| Project | What it does | Limits |
| --- | --- | --- |
| [Zenoh shared memory](https://zenoh-cpp.readthedocs.io/en/1.10.1/shared_memory.html) | zero-copy payloads on one host | host memory only |
| [hiroz](https://discourse.openrobotics.org/t/introducing-hiroz-by-zettascale/55081) (ZettaScale, formerly ROS-Z) | ROS 2 in Rust on Zenoh. An open pull request adds a CUDA IPC tensor transport (`zenoh-cuda`). | Not merged. It depends on an open Zenoh pull request (eclipse-zenoh/zenoh#2463). |
| [dora-rs](https://github.com/dora-rs/dora) | a dataflow framework. Zenoh shared memory for host data, CUDA IPC handles for GPU data ([issue #1872](https://github.com/dora-rs/dora/issues/1872)). | The GPU path is an example. Frames that are made on the CPU and used on the GPU are a known gap. |

## NVIDIA pipeline frameworks and GStreamer

| Project | What it does | Limits |
| --- | --- | --- |
| [Holoscan SDK](https://docs.nvidia.com/holoscan/sdk-user-guide/introduction/relevant-technologies) | UCX between fragments (CUDA IPC, GPUDirect RDMA). HoloMQ is a pool of exportable CUDA slots: it sends a handle and tensor metadata, and signals readiness on a CUDA stream ([RidgeRun](https://www.ridgerun.com/post/optimizing-interprocess-gpu-frame-transport-in-holoscan-sdk)). | Documented with one sender and one receiver. A held slot is not reused, so a slow reader can hold the producer up. |
| [DeepStream Gst-nvunixfd](https://docs.nvidia.com/metropolis/deepstream/9.0/text/DS_plugin_gst-nvunixfd.html) | NvBufSurface handles over a Unix socket, from one sink to several sources | GStreamer pipelines, one device |
| [GStreamer cudaipcsink/cudaipcsrc](https://gstreamer.freedesktop.org/documentation/nvcodec/cudaipcsink.html) (since 1.24) | CUDA memory between GStreamer processes, with legacy CUDA IPC or VMM | GStreamer pipelines |
| [NvSciStream](https://developer.nvidia.com/docs/drive/drive-os/7.0.3/public/drive-os-linux-sdk/core-concepts/NvStreams/NvSciStreamSampleApplication1.html) (DRIVE OS) | a camera producer multicast to up to 4 consumers in other processes | NVIDIA's automotive platforms |

## Smaller projects

Three projects solve the same problem inside one application each:
[IsaacTeleop](https://github.com/NVIDIA/IsaacTeleop/pull/1078) sends camera frames as CUDA VMM handles over a Unix
socket, [Isaac Lab](https://github.com/isaac-sim/IsaacLab/pull/8410) sends images to Cosmos by CUDA IPC, and
[cuda-link](https://github.com/forkni/cuda-link) is a CUDA IPC ring between TouchDesigner and Python.

## What this means for Durchblick

We follow two developments: the buffer backends of ROS 2 and the CUDA transport that is proposed for Zenoh.

- A ROS mode of Durchblick could offer its CUDA ring as a `rosidl::Buffer` backend. ROS 2 nodes with that backend
  installed, Isaac ROS among them, could then read the frames in place. See
  [Durchblick and ROS 2](../integrations/ros2.md).
- If Zenoh gains GPU payloads, the descriptors of the ring could use them.
