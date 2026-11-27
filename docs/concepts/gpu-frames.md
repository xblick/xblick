# Frames on the GPU

A CUDA ring is a [ring](rings.md) in GPU memory. A producer on the GPU writes a frame into it, and a consumer on the
same GPU reads the frame there. The frame never passes through CPU memory. This page explains how processes share
GPU memory, why a GPU array can be unfinished when your code already holds it, and what you must do when you publish
one.

## Why the copy to the CPU matters

A renderer, a JPEG decoder on the GPU and a neural network all hold their data in GPU memory. If a middleware
accepts only CPU arrays, the producer downloads each frame to the CPU and the consumer uploads it again. Both copies
cross the bus between CPU and GPU, and both processes wait for them. With a CUDA ring, the only copy is from one
place in GPU memory to another. For a 720p `rgb8` frame this copy takes 0.02 ms.

## How two processes share GPU memory

GPU memory normally belongs to one process. CUDA can export an allocation as a file descriptor, in the same way as
Linux exports shared memory. Durchblick uses this as follows.

1. The publisher allocates the ring with CUDA's virtual memory management (`cuMemCreate`) and receives a file
   descriptor for it.
2. A reader connects to the publisher over a Unix socket and receives the file descriptor. Only processes of the
   same user can connect.
3. The reader maps the allocation into its own CUDA context. Publisher and reader now see the same GPU memory.

Durchblick compares GPUs by their UUID. The numbering in `CUDA_VISIBLE_DEVICES` therefore does not matter: a node
on `cuda:0` with `CUDA_VISIBLE_DEVICES=1` and a publisher on GPU 1 use the same CUDA ring.

## A reader on another GPU

A PC can have several GPUs, and a node can work on another GPU than the publisher. The reader then copies each
frame from the publisher's ring straight into memory on its own GPU. The frame does not pass through CPU memory,
and the publisher does no extra work.

| 1080p `rgb8` to a reader on another GPU | Time per frame |
| --- | --- |
| Copy from GPU to GPU | 0.16 ms |
| Host ring and upload | 0.53 ms |
| For comparison: reader on the publisher's GPU | 0.014 ms |

Measured on 2026-11-11 between two RTX PRO 6000.

Three conditions apply.

- The reader's process must see the publisher's GPU. If `CUDA_VISIBLE_DEVICES` hides it, the reader takes the host
  ring and uploads. Select a node's GPU with its `--device` option instead.
- The reader's process holds a CUDA context on the publisher's GPU. In the measurement this took 552 MiB of that
  GPU's memory per reader process.
- The copy goes over PCIe only where the mainboard lets the two GPUs reach each other. Otherwise the driver copies
  through host memory. The result is the same. `CudaRingReader.direct` tells which case applies.

A reader that asks for views (`copy=False`) gets copies from a ring on another GPU, because a view would be memory
on the publisher's GPU.

A reader gets a *device array*, which is an array in GPU memory. The array has the `__cuda_array_interface__`
attribute, so PyTorch, CuPy and Warp accept it without a copy:

```python
gpu = db.ImageSubscriber(session, "demo/camera/color", transport="cuda")
tensor = torch.as_tensor(gpu.latest(timeout=1.0).image, device="cuda")
```

## Why a GPU array can be unfinished

A GPU executes work asynchronously. When `y = model(x)` returns, PyTorch has only placed the operations in a queue
on the GPU. CUDA calls this queue a *CUDA stream*. The values of `y` exist a few milliseconds later. Code in the
same process does not notice the delay, because every later operation on `y` waits in the same CUDA stream.

A reader in another process does not wait in that CUDA stream. If Durchblick copied `y` into the ring immediately,
the reader could receive a frame that the GPU has not finished. A second failure is possible: PyTorch gives the
memory of `y` to the next tensor as soon as your code drops `y`, so a late copy could read the next frame.

`test_device_frames_published_complete_and_before_reuse` in `tests/test_rings.py` covers both failures. When the
test names a wrong CUDA stream on purpose, all 30 frames arrive with wrong content.

## Publishing a device array

`publish()` waits until the GPU has completed the array, copies the array into the ring, and returns when the copy
is done. You may then free or overwrite the array.

Durchblick must know which CUDA stream produced the array. Some libraries record the CUDA stream in the array.
PyTorch does not, so you name it:

```python
camera.publish(tensor, stamp_ns=capture_time_ns, stream=torch.cuda.current_stream())
```

| You pass | Durchblick waits for | Time per publish |
| --- | --- | --- |
| `stream=...` | the work on this CUDA stream | 0.1 ms |
| no stream, and the array names none | all work of the process on this GPU | 7 ms |

Measured on 2026-10-30 on an RTX PRO 6000 with a 720p `rgb8` frame, while 10 ms of other work waited on another
CUDA stream.

Without a name the result is still correct. The publisher only waits longer than necessary. In one measurement,
Boxer ran on one GPU with two other nodes at 60 frames per second. Its latency was 27 ms when it named its CUDA
stream and 32 ms when it did not. We measured each case once, and the latency varied by more than this difference
between identical setups. At 30 frames per second we measured no difference.

The reader has the same problem in the other direction, because the copy into the slot is also asynchronous GPU
work. Each slot therefore has an event that both processes can see. The reader's GPU work waits for this event, so
it starts after the copy into the slot has finished. You do not need to do anything for this.

## ROS 2 readers

A ROS 2 Lyrical node can accept image data as a CUDA buffer through ROS 2's CUDA buffer backend
(`cuda_buffer_backend`, experimental upstream). Isaac ROS nodes do this. Such a reader gets our GPU frames on the GPU
as well. [GPU frames for ROS readers](../integrations/ros2.md#gpu-frames-for-ros-readers) lists what the reader
needs. In short, it runs on `rmw_zenoh` with the backend's plugin, as the same Linux user on the same PC and GPU, and
it shares `/tmp`, the process ids and the network with the publisher. The publisher reaches its `/dev/shm`: the
same one, or the reader's own through `XBLICK_ROS_CUDA_SHM`.

The backend's reader maps one whole CUDA allocation from its start. A slot of our CUDA ring is a part of one
allocation, so the reader cannot map it. The publisher therefore keeps a small pool of blocks, each one allocation of
exactly the frame's size in whole steps of 2 MiB.

1. A ROS reader that accepts the backend announces it in its liveliness token.
2. The publisher finds the reader in the backend's registry in `/dev/shm`. The registry tells whether the reader
   runs on this PC, as this user and on this GPU.
3. For each frame, the publisher copies the slot into a free block on the GPU and waits for the copy. It then sends
   a descriptor of the block to the reader in place of the pixels.
4. The reader receives the block's file descriptor over a Unix socket once, maps the block, and keeps the mapping.
   The socket accepts only processes of the publisher's user.
5. The reader counts its hold in a small record in `/dev/shm` while it uses the message. The publisher rewrites a
   block only when the count is zero.

The count rises only when the reader takes a message from its queue. A message that waits in the queue holds
nothing. The publisher therefore also waits until as many frames as the reader's queue holds have gone out since
the block's last frame. A reader that takes a message whose block was rewritten drops it. An Isaac ROS node then
aborts, so this rule matters.

The copy into the block costs the publisher little compared with the bytes that it sends otherwise:

| 1080p `rgb8` to one ROS reader that accepts CUDA buffers | Publisher's time per frame, median | 95th percentile |
| --- | --- | --- |
| CUDA buffer: copy into a block, descriptor | 0.5 ms | 0.7 ms |
| Bytes: download, serialisation, put | 13.4 ms | 15.2 ms |

Measured on 2026-11-26 on an RTX PRO 6000 with a stock Lyrical reader in a container, whose queue holds 10 messages,
twice 300 frames at 30 Hz per row. Both rows include the copy into the CUDA ring. Other jobs used the PC's CPUs at the
time. The pool had 11 blocks, which take 66 MiB of GPU memory.

## ROS 2 publishers

A ROS 2 Lyrical publisher with the CUDA buffer backend sends the frames that it has as CUDA buffers as such to our
subscribers that work on a GPU. [GPU frames from ROS publishers](../integrations/ros2.md#gpu-frames-from-ros-publishers)
lists what the publisher needs. In short, it runs as the same Linux user on the same PC and GPU, and it shares our
program's `/dev/shm`, process ids and network.

1. The subscriber announces the backend in its liveliness token and enters itself in the backend's registry before
   that. The publisher decides from that entry, once, whether it sends CUDA buffers or the bytes.
2. When the subscriber takes a frame, it adds one to the count in the block's record and compares the block's uid
   with the descriptor's. Another uid means that the publisher has rewritten the block, and the frame is stale.
3. The subscriber fetches the block's file descriptor from the publisher once and maps the block on its GPU.
4. Its copy waits for the publisher's CUDA event, which marks the end of the publisher's write into the block.
5. After the copy, the subscriber compares the uid again and takes its count back. The publisher may then reuse the
   block.

The subscriber hands out its own copy, as from a CUDA ring, so the publisher's pool stays small.

## What follows for your node

- Name your CUDA stream when you publish a device array.
- Ask for `transport="auto"` and pass your `device`. The subscriber then delivers every frame of a CUDA ring on
  your GPU, and takes the host ring where it cannot reach the ring. [Transports](transports.md) has the rules.
- PyTorch needs the reader's copy of a frame, which is the default. The read-only views of `copy=False` do not work
  with PyTorch.
- Durchblick opens the CUDA driver (`libcuda.so.1`) at run time. The package therefore loads on a host without a
  GPU, and only the CUDA ring needs one.

## Status and limits

- A process cannot open a CUDA ring that it created itself. Tests read a CUDA ring from a child process.
- A reader sets up CUDA when its first frame arrives. When 16 readers opened one ring at the same time, each reader
  needed about 3 s for this.
- We measured the copy between GPUs on one machine with two GPUs of the same model.
- A ROS reader gets CUDA buffers only from a `CameraPublisher` with `cuda_device`, only on the publisher's GPU,
  and only through a backend whose wire format matches ours. The backend is experimental and has no version on the wire.
- A ROS reader on another GPU gets the bytes. The backend's reader cannot copy between GPUs.
- Our subscriber takes CUDA buffers only from a ROS publisher on its GPU, and gets the bytes from one on another GPU.
  A ROS publisher that starts after our subscriber decides by its CUDA ordinal alone. On another GPU with the same
  ordinal, its frames do not reach our subscriber.

## Related pages

- [Rings](rings.md): slots, counters and descriptors.
- [Making a node fast](../guides/making-a-node-fast.md): what else we learned about GPU work in nodes.
- [Alternatives](../project/alternatives.md): other projects that share GPU memory between processes.
- `cpp/include/xblick/cuda_ring.hpp`: the handshake and the slot events.
- `cpp/include/xblick/ros_cuda.hpp`: the CUDA buffer backend's protocol and the pool of blocks.
