"""The GPU link to ROS 2 readers (docs/concepts/gpu-frames.md, docs/integrations/ros2.md): a ROS subscription of an
image topic that accepts ROS 2's CUDA buffer backend (``cuda_buffer_backend``, experimental upstream; its token says
``backends:...cuda:``) gets the frames the publisher has on the GPU as CUDA buffers instead of their bytes, over
rmw_zenoh's conventions (ros.py). The CUDA ring stays as it is: each frame is copied on the GPU into a block of a small
export pool, and the block's descriptor goes on ``<key>/_buf/<the reader's gid>``. Nothing is allocated before the
first such reader.

The readers' decisions, the plan of each message, the pool and the protocol are the C++ core's
(``_core.RosCudaLink``, cpp/include/xblick/ros_cuda.hpp), which the C++ CameraPublisher uses as well (ros.hpp
GpuLink); this module is the Zenoh part: the readers' tokens, the buffer keys, the log lines.

A reader gets the descriptor only when the backend could import it, else the CPU form as before:

* it is in the backend's host registry (``/dev/shm/ros2_hem_d0_h<hash>``, written by every ROS process that loads the
  backend; the publisher only reads it) under its gid, as a subscription of this Linux user that can import (the
  backend's own rule); so on this host, with this ``/dev/shm``;
* its process is one this program can see (``/proc/<pid>`` maps that registry: the same pid namespace);
* it uses this GPU: with one GPU visible to it, that GPU (by the device files it has open; a process without one open
  yet is looked up again); with several, the same ``CUDA_VISIBLE_DEVICES`` and ``CUDA_DEVICE_ORDER`` as this program
  and the same ordinal (the backend compares ordinals only);
* it keeps the last so many messages (its QoS depth), not all, and no more than the pool holds: a block is rewritten
  only after as many frames as the deepest reader queue holds (a message that a reader takes from a rewritten block is
  dropped by the backend, and Isaac ROS 5.1's ResizeNode then aborts), so the pool has up to depth + 1 +
  SPARE_BLOCKS blocks, at most MAX_BLOCKS (a depth of MAX_BLOCKS - 1 - SPARE_BLOCKS at most).

A registry that cannot be read (another layout) gives every reader the CPU form, said once.

While at least one reader gets descriptors, a message goes on the buffer keys only, as a Lyrical publisher sends it (a
Lyrical reader reads the topic's key as well and would get it twice): ``<key>/_buf_cpu`` for ``backends:cpu:``
readers (ours among them), ``<key>/_buf/<gid>`` in CPU form for the other buffer-aware ones. A reader without a
backends part (ROS 2 Jazzy, older xblick) reads the key only: while one is there, every reader gets the CPU form on
the key. A program that reads the key without a token (no ROS entity) gets nothing while the link is on.

The other direction (``GpuReceiver``, the core's ``_core.RosCudaReceiver``): a subscription of ours that reads a ROS
image topic and works on a GPU (``ros.RosSubscriber`` with ``cuda_device``; an ImageSubscriber of a ROS form with a
``device``) takes the frames a ROS publisher has as CUDA buffers on that GPU without a copy through the CPU, as a
device buffer of its own (the CUDA ring reader's copy). Its token says ``backends:cuda:``. A publisher decides once,
when it discovers the subscription, by the backend's rule (the subscription's registry entry has its Linux user and
CUDA ordinal and says "can import"), so the subscription enters the registry before its token, from the publishers
known then. Without a registry yet (no ROS process with the backend has run on this host), it first makes the
registry and its semaphore as the backend makes them, so that a publisher that starts later attaches to it and finds
the entry. The entry says "can import" with a publisher's ordinal when that publisher uses this
GPU, as this Linux user, in a process seen from here, with this program's ``/dev/shm`` (CUDA's events of the buffers
are there); "cannot import" when publishers are known and none does; with none known as a stock subscription enters
itself (this program's ordinal). Every other publisher sends it the CPU form (one line each says why). A descriptor is
read under the backend's hold (the count in the block's record, the uid compared before and after the copy): a frame
whose block was rewritten before it was taken is stale.

``XBLICK_ROS_CUDA=0`` turns the link off (both directions); ``XBLICK_ROS_CUDA_SOCKETS`` is where the sockets go
(default ``/tmp``, the backend's; a reader in a container needs it at the same path, release 0.1.2 reads only the
file form), ``XBLICK_ROS_CUDA_SHM`` the ROS nodes' ``/dev/shm`` as this program sees it (default ``/dev/shm``)."""

from __future__ import annotations

import logging
import threading

from . import _core, ros
from .session import close_at_exit, forget_at_exit

log = logging.getLogger("xblick")

ENABLE_ENV = _core.ROS_CUDA_ENABLE_ENV     # 0: no GPU link; 1 or unset: the link (the default)
SOCKETS_ENV = _core.ROS_CUDA_SOCKETS_ENV   # where the fd sockets go
SHM_ENV = _core.ROS_CUDA_SHM_ENV           # the readers' /dev/shm
HOLD_LIMIT_S = _core.ROS_CUDA_HOLD_LIMIT_S      # a block held longer than this since its publish counts as free
MIN_REUSE_S = _core.ROS_CUDA_MIN_REUSE_S        # a block is not rewritten sooner after its publish
SPARE_BLOCKS = _core.ROS_CUDA_SPARE_BLOCKS      # blocks beyond the readers' queue depth, for messages they hold
MAX_BLOCKS = _core.ROS_CUDA_MAX_BLOCKS          # blocks of one frame size at most
RECHECK_S = _core.ROS_CUDA_RECHECK_S   # a reader not (yet) in the registry is looked up again after this long
UNKNOWN_S = _core.ROS_CUDA_UNKNOWN_S   # ... and said once after this long
LINK_TYPES = ("sensor_msgs/msg/Image",)  # the message types the link carries


def enabled_by_environment() -> bool:
    try:
        return _core.ros_cuda_enabled_by_environment()
    except ValueError as error:
        raise ros.RosError(str(error)) from None


def backends_of(token: ros.Token) -> dict[str, str] | None:
    """The backends of a token's 14th part (``backends:cpu:;cuda:`` -> {"cpu": "", "cuda": ""}); None without it."""
    return None if token.backends is None else _core.ros_cuda_backends_of(token.backends)


def gpu_minors(pid: int | str) -> set[int]:
    """The GPUs (minor numbers of /dev/nvidia<N>) a process has open: those CUDA sees in it."""
    return _core.ros_cuda_gpu_minors(str(pid))


class GpuLink:
    """The GPU link of one ROS publisher of images (``ros.RosPublisher`` of sensor_msgs/msg/Image): the subscription
    tokens of its topic into the core's link (decisions, plan, pool), and the buffer keys. ``plan()`` before each frame
    (None: everything as before, on the key); ``send_gpu`` and ``send_cpu`` with that plan. Not thread-safe apart
    from the tokens (one publisher)."""

    def __init__(self, publisher: ros.RosPublisher, device: int, socket_dir: str | None = None,
                 shm_dir: str | None = None, max_blocks: int = MAX_BLOCKS, hold_limit_s: float = HOLD_LIMIT_S,
                 min_reuse_s: float = MIN_REUSE_S):
        if publisher.type_name not in LINK_TYPES:
            raise ros.RosError(f"{publisher.type_name}: the GPU link carries {', '.join(LINK_TYPES)}")
        self.publisher = publisher
        self.device = device
        try:
            self._link = _core.RosCudaLink(publisher.topic, device,
                                           socket_dir or _core.ros_cuda_socket_dir_from_environment(),
                                           shm_dir or _core.ros_cuda_shm_dir_from_environment(),
                                           max_blocks, hold_limit_s, min_reuse_s)
        except ValueError as error:
            raise ros.RosError(str(error)) from None
        self.socket_dir, self.shm_dir = self._link.socket_dir, self._link.shm_dir
        self.hold_limit_s = hold_limit_s
        self._lock = threading.Lock()
        self._outs: dict[str, object] = {}  # buffer key -> its zenoh publisher
        token = publisher.token
        pattern = "/".join([ros.ADMIN_SPACE, str(token.domain), "*", "*", "*", ros.SUBSCRIPTION, "*", "*", "*",
                            ros.mangle(publisher.topic), ros.dds_type(publisher.type_name), token.type_hash, "**"])
        self._tokens = ros._Polled(publisher._raw.liveliness().declare_subscriber(pattern, history=True),
                                   self._on_token, f"ros gpu {publisher.topic}")
        # its thread calls Zenoh and the core, and the pool serves its readers on a socket: closed at the end, before
        # the session, also when the publisher is left open (its pool's files then go too)
        close_at_exit(self, f"the GPU link of {publisher.topic}")

    # Each call reads ``_link`` once: close() may set it to None meanwhile (the tokens' thread, a camera's publish
    # thread at exit); a closed link then does nothing (plan None: the key as before).

    def _on_token(self, sample):
        link = self._link
        if link is None:
            return
        key = str(sample.key_expr)
        if sample.kind != ros.zenoh.SampleKind.PUT:
            gid = link.remove(key)
            with self._lock:
                out = self._outs.pop(self._buf_key(gid), None) if gid is not None else None
            if out is not None:
                out.undeclare()
            return
        try:
            token = ros.parse_token(key)
        except ros.RosError:
            return
        qos = token.qos or ros.Qos()
        link.add(key, ros.gid(key), token.backends, qos.history == ros.KEEP_ALL, qos.depth,
                 token.fully_qualified_node)

    def _buf_key(self, gid: bytes) -> str:
        return f"{self.publisher.key}/_buf/{gid.hex()}"

    def _say(self, link):
        for warning, text in link.notes():
            (log.warning if warning else log.info)(text)

    def _importable(self, entry: dict) -> tuple[bool, str]:
        """Whether a reader of this registry entry can import our blocks, and why not (the core's rule)."""
        return self._link.importable(entry)

    def plan(self):
        """Where the next message goes (``_core.RosCudaPlan``: ``gpu`` [(gid, device_id)], ``cpu_group``, ``cpu``
        [gid], ``depth``, ``needs_cpu``), or None: on the topic's key as before (no reader gets descriptors, or one
        reads the key only, or the link is closed)."""
        link = self._link
        if link is None:
            return None
        plan = link.plan()
        self._say(link)
        return plan

    def _out(self, key: str):
        with self._lock:
            out = self._outs.get(key)
        if out is None:
            qos = self.publisher.qos
            out = self.publisher._raw.declare_publisher(key, reliability=ros._reliability(qos),
                                                        congestion_control=ros._congestion(qos))
            with self._lock:
                self._outs[key] = out
        return out

    def send_cpu(self, plan, payload, attachment: bytes) -> list[str]:
        """The CPU form (the message's CDR) to the readers of ``plan`` that take it."""
        sent = []
        keys = ([ros.cpu_group_key(self.publisher.key)] if plan.cpu_group else []) + \
            [self._buf_key(gid) for gid in plan.cpu]
        for key in keys:
            self._out(key).put(bytes(payload), attachment=attachment)
            sent.append(key)
        return sent

    def send_gpu(self, plan, ring, slot: int, nbytes: int, header, height: int, width: int, encoding: str,
                 step: int, attachment: bytes) -> list[str]:
        """The frame in slot ``slot`` of the CUDA ring ``ring`` (``_core.CudaRingWriter``, this link's device) into
        a free block, its descriptor to the GPU readers of ``plan``. Nothing when every block is held (the pool
        counts it) or the link is closed: the writer never waits."""
        link = self._link
        if link is None:
            return []
        messages = link.gpu_messages(plan, ring, slot, nbytes, header.stamp.sec, header.stamp.nanosec,
                                     header.frame_id, height, width, encoding, step)
        self._say(link)
        sent = []
        for gid, payload in messages:
            key = self._buf_key(gid)
            self._out(key).put(payload, attachment=attachment)
            sent.append(key)
        return sent

    def stats(self) -> dict | None:
        """sent, dropped, expired, blocks, accepts, refused; None once closed."""
        link = self._link
        return link.stats() if link is not None else None

    def close(self):
        """Ends the tokens' thread, the buffer keys and the pool (again: nothing)."""
        forget_at_exit(self)
        tokens, self._tokens = self._tokens, None
        if tokens is None:
            return
        tokens.undeclare()
        with self._lock:
            outs, self._outs = list(self._outs.values()), {}
        for out in outs:
            out.undeclare()
        self._link = None  # removes its pool's sockets, records and blocks


class Received(bytes):
    """The payload of a sample in the buffer form (a sensor_msgs/Image whose data is a CUDA buffer descriptor), from a
    subscription with a GpuReceiver: ``parsed`` (``_core.ros_cuda_parse_image``: the header's fields and
    ``descriptor``) and ``receiver``, which reads it (``GpuReceiver.take``) when the frame is taken."""

    parsed: dict
    receiver: GpuReceiver


class GpuReceiver:
    """The link from ROS publishers of one subscription (``ros.RosSubscriber`` with ``cuda_device``): the publisher
    tokens of its topic into the core's receiver (decisions, registry entry, blocks), the samples in the buffer form
    marked (``received``), and their frames copied onto the GPU (``take``)."""

    def __init__(self, subscriber: ros.RosSubscriber, device: int, shm_dir: str | None = None):
        if subscriber.type_name not in LINK_TYPES:
            raise ros.RosError(f"{subscriber.type_name}: CUDA buffers come in {', '.join(LINK_TYPES)}")
        self.subscriber = subscriber
        self.device = device
        try:
            self._receiver = _core.RosCudaReceiver(subscriber.topic, device,
                                                   shm_dir or _core.ros_cuda_shm_dir_from_environment(),
                                                   subscriber.gid)
        except ValueError as error:
            raise ros.RosError(str(error)) from None
        self.shm_dir = self._receiver.shm_dir
        # entered in the registry for the publishers known now (ROS's graph), before the subscription's token: a
        # publisher decides once, when it discovers the subscription
        for known in subscriber.node.ros.graph.publishers(subscriber.topic):
            if known.type_name == subscriber.type_name:
                self._receiver.add_publisher(known.key(), ros.gid(known.key()), known.fully_qualified_node)
        self._receiver.enter()
        self._say(self._receiver)
        token = subscriber.token
        pattern = "/".join([ros.ADMIN_SPACE, str(token.domain), "*", "*", "*", ros.PUBLISHER, "*", "*", "*",
                            ros.mangle(subscriber.topic), ros.dds_type(subscriber.type_name), token.type_hash, "**"])
        self._tokens = ros._Polled(subscriber._raw.liveliness().declare_subscriber(pattern, history=True),
                                   self._on_token, f"ros gpu from {subscriber.topic}")

    # Each call reads ``_receiver`` once: close() may set it to None meanwhile (the tokens' thread, Zenoh's sample
    # threads); a closed receiver then does nothing.

    def _on_token(self, sample):
        receiver = self._receiver
        if receiver is None:
            return
        key = str(sample.key_expr)
        if sample.kind != ros.zenoh.SampleKind.PUT:
            receiver.remove_publisher(key)
            return
        try:
            token = ros.parse_token(key)
        except ros.RosError:
            return
        receiver.add_publisher(key, ros.gid(key), token.fully_qualified_node)
        self._refresh(receiver)

    def _refresh(self, receiver):
        receiver.refresh()
        self._say(receiver)

    def _say(self, receiver):
        for warning, text in receiver.notes():
            (log.warning if warning else log.info)(text)

    def set_gid(self, gid: bytes):
        """The subscription's token changes (its node renamed): entered anew with the new gid, before the new token."""
        receiver = self._receiver
        if receiver is None:
            return
        receiver.set_gid(gid)
        self._say(receiver)

    def received(self, payload: bytes) -> bytes:
        """A sample's payload: as it is in the CPU form, a Received in the buffer form. The registry looked at again
        (cheap once every publisher is decided). RingError for a buffer of another backend."""
        receiver = self._receiver
        if receiver is None:  # closed: a sample that Zenoh handed over meanwhile
            return payload
        self._refresh(receiver)
        parsed = _core.ros_cuda_parse_image(payload)
        if parsed is None:
            return payload
        received = Received(payload)
        received.parsed, received.receiver = parsed, self
        return received

    def take(self, received: Received):
        """The frame of a Received, copied onto this GPU (``_core.DeviceBuffer``, complete), or None when it is stale
        (its block rewritten or gone before it was taken, or this receiver closed) or not read (said once)."""
        receiver = self._receiver
        if receiver is None:
            return None
        buffer = receiver.take(received.parsed["descriptor"])
        self._say(receiver)
        return buffer

    def stats(self) -> dict | None:
        """copied, stale, refused (descriptors), blocks (mapped now), registered (the entry is in the registry); None
        once closed."""
        receiver = self._receiver
        return receiver.stats() if receiver is not None else None

    def close(self):
        """Ends the tokens' thread, makes the registry entry inactive and unmaps the blocks (again: nothing)."""
        tokens, self._tokens = self._tokens, None
        receiver, self._receiver = self._receiver, None
        if tokens is not None:
            tokens.undeclare()
        if receiver is not None:
            receiver.close()  # without the GIL: it may wait for a take() or the registry's semaphore
