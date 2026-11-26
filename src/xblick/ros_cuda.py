"""The GPU link to ROS 2 readers (docs/concepts/gpu-frames.md, docs/integrations/ros2.md): a ROS subscription of an
image topic that accepts ROS 2's CUDA buffer backend (``cuda_buffer_backend``, experimental upstream; its token says
``backends:...cuda:``) gets the frames the publisher has on the GPU as CUDA buffers instead of their bytes, over
rmw_zenoh's conventions (ros.py). The CUDA ring stays as it is: each frame is copied on the GPU into a block of a small
export pool (``_core.RosCudaPool``, cpp/include/xblick/ros_cuda.hpp, which also has the protocol), and the block's
descriptor goes on ``<key>/_buf/<the reader's gid>``. Nothing is allocated before the first such reader.

A reader gets the descriptor only when the backend could import it, else the CPU form as before:

* it is in the backend's host registry (``/dev/shm/ros2_hem_d0_h<hash>``, written by every ROS process that loads the
  backend; read here, never written) under its gid, as a subscription of this Linux user that can import (the
  backend's own rule); so on this host, with this ``/dev/shm``;
* its process is one this program can see (``/proc/<pid>`` maps that registry: the same pid namespace);
* it uses this GPU: with one GPU visible to it, that GPU (by the device files it has open); with several, the same
  ``CUDA_VISIBLE_DEVICES`` and ``CUDA_DEVICE_ORDER`` as this program and the same ordinal (the backend compares
  ordinals only);
* it keeps the last so many messages (its QoS depth), not all: a block is rewritten only after as many frames as the
  deepest reader queue holds (a message that a reader takes from a rewritten block is dropped by the backend, and
  Isaac ROS 5.1's ResizeNode then aborts), so the pool has up to depth + 1 + SPARE_BLOCKS blocks.

While at least one reader gets descriptors, a message goes on the buffer keys only, as a Lyrical publisher sends it (a
Lyrical reader reads the topic's key as well and would get it twice): ``<key>/_buf_cpu`` for ``backends:cpu:``
readers (ours among them), ``<key>/_buf/<gid>`` in CPU form for the other buffer-aware ones. A reader without a
backends part (ROS 2 Jazzy, older xblick) reads the key only: while one is there, every reader gets the CPU form on
the key. A program that reads the key without a token (no ROS entity) gets nothing while the link is on.

``XBLICK_ROS_CUDA=0`` turns the link off; ``XBLICK_ROS_CUDA_SOCKETS`` is where the sockets go (default ``/tmp``, the
backend's; a reader in a container needs it at the same path, release 0.1.2 reads only the file form),
``XBLICK_ROS_CUDA_SHM`` the reader's ``/dev/shm`` as this program sees it (default ``/dev/shm``)."""

from __future__ import annotations

import dataclasses
import logging
import os
import stat
import threading
import time

from . import _core, ros

log = logging.getLogger("xblick")

ENABLE_ENV = "XBLICK_ROS_CUDA"            # 0: no GPU link; 1 or unset: the link (the default)
SOCKETS_ENV = "XBLICK_ROS_CUDA_SOCKETS"   # where the fd sockets go
SHM_ENV = "XBLICK_ROS_CUDA_SHM"           # the readers' /dev/shm
HOLD_LIMIT_S = _core.ROS_CUDA_HOLD_LIMIT_S      # a block held longer than this since its publish counts as free
MIN_REUSE_S = _core.ROS_CUDA_MIN_REUSE_S        # a block is not rewritten sooner after its publish
SPARE_BLOCKS = _core.ROS_CUDA_SPARE_BLOCKS      # blocks beyond the readers' queue depth, for messages they hold
MAX_BLOCKS = _core.ROS_CUDA_MAX_BLOCKS          # blocks of one frame size at most
RECHECK_S = 0.5     # a reader not (yet) in the registry is looked up again after this long
UNKNOWN_S = 3.0     # ... and said once after this long
CUDA_BACKEND = "cuda"
NVIDIA_MAJOR = 195  # the character devices /dev/nvidia<minor>; 254 and 255 are nvidia-modeset and nvidiactl
GPU_MINORS = range(254)
LINK_TYPES = ("sensor_msgs/msg/Image",)  # the message types the link carries


def enabled_by_environment() -> bool:
    value = os.environ.get(ENABLE_ENV)
    if value is None or value == "1":
        return True
    if value == "0":
        return False
    raise ros.RosError(f"{ENABLE_ENV}={value!r}: 0 (no GPU link to ROS readers) or 1 (the link, the default)")


def backends_of(token: ros.Token) -> dict[str, str] | None:
    """The backends of a token's 14th part (``backends:cpu:;cuda:`` -> {"cpu": "", "cuda": ""}); None without it."""
    if token.backends is None:
        return None
    out = {}
    for part in token.backends[len("backends:"):].split(";"):
        if part:
            name, _, meta = part.partition(":")
            out[name] = meta
    return out


def gpu_minors(pid: int | str) -> set[int]:
    """The GPUs (minor numbers of /dev/nvidia<N>) a process has open: those CUDA sees in it. OSError when its file
    descriptors cannot be read (another user, gone)."""
    minors = set()
    base = f"/proc/{pid}/fd"
    for fd in os.listdir(base):
        try:
            st = os.stat(f"{base}/{fd}")
        except OSError:
            continue  # closed meanwhile
        if stat.S_ISCHR(st.st_mode) and os.major(st.st_rdev) == NVIDIA_MAJOR and os.minor(st.st_rdev) in GPU_MINORS:
            minors.add(os.minor(st.st_rdev))
    return minors


def minor_of(uuid: str) -> int | None:
    """The minor number of the GPU of a UUID (32 hex digits) from /proc/driver/nvidia/gpus; None if none says so."""
    want = uuid.replace("-", "").lower()
    root = "/proc/driver/nvidia/gpus"
    try:
        buses = os.listdir(root)
    except OSError:
        return None
    for bus in buses:
        try:
            with open(f"{root}/{bus}/information") as f:
                fields = dict(line.split(":", 1) for line in f if ":" in line)
        except OSError:
            continue
        their = fields.get("GPU UUID", "").strip().removeprefix("GPU-").replace("-", "").lower()
        if their == want:
            return int(fields["Device Minor"].strip())
    return None


def _environment(pid: int) -> dict[str, str]:
    with open(f"/proc/{pid}/environ", "rb") as f:
        pairs = [item.partition(b"=") for item in f.read().split(b"\0") if item]
    return {k.decode(errors="replace"): v.decode(errors="replace") for k, _, v in pairs}


GPU_ENV = ("CUDA_VISIBLE_DEVICES", "CUDA_DEVICE_ORDER")


@dataclasses.dataclass
class _Reader:
    key: str
    token: ros.Token
    gid: bytes
    backends: dict[str, str] | None
    importable: bool | None = None  # None: not decided yet (not in the registry)
    device_id: int = 0              # its CUDA ordinal (the descriptor's device_id)
    seen: float = dataclasses.field(default_factory=time.monotonic)
    checked: float = 0.0
    said: bool = False

    @property
    def wants_cuda(self) -> bool:
        return self.backends is not None and CUDA_BACKEND in self.backends

    @property
    def cpu_only(self) -> bool:
        return self.token.backends == ros.CPU_BACKENDS


@dataclasses.dataclass(frozen=True)
class Plan:
    """Where one message goes while the link is on: descriptors to ``gpu`` (readers), the CPU form on ``_buf_cpu``
    (``cpu_group``) and on ``_buf/<gid>`` of ``cpu``; ``depth``: the deepest queue of the ``gpu`` readers."""

    gpu: tuple
    cpu_group: bool
    cpu: tuple
    depth: int

    @property
    def needs_cpu(self) -> bool:
        return self.cpu_group or bool(self.cpu)


class GpuLink:
    """The GPU link of one ROS publisher of images (``ros.RosPublisher`` of sensor_msgs/msg/Image): the subscription
    tokens of its topic, a decision per reader, and the export pool once a reader gets descriptors. ``plan()`` before
    each frame (None: everything as before, on the key); ``send_gpu`` and ``send_cpu`` with that plan. Not
    thread-safe apart from the tokens (one publisher)."""

    def __init__(self, publisher: ros.RosPublisher, device: int, socket_dir: str | None = None,
                 shm_dir: str | None = None, max_blocks: int = MAX_BLOCKS, hold_limit_s: float = HOLD_LIMIT_S,
                 min_reuse_s: float = MIN_REUSE_S):
        if publisher.type_name not in LINK_TYPES:
            raise ros.RosError(f"{publisher.type_name}: the GPU link carries {', '.join(LINK_TYPES)}")
        self.publisher = publisher
        self.device = device
        self.socket_dir = socket_dir or os.environ.get(SOCKETS_ENV) or _core.ROS_CUDA_SOCKET_DIR
        self.shm_dir = shm_dir or os.environ.get(SHM_ENV) or _core.ROS_CUDA_SHM_DIR
        for what, path in (("sockets", self.socket_dir), ("shm", self.shm_dir)):
            if not os.path.isdir(path):
                raise ros.RosError(f"GPU link to ROS readers: {path} ({what}) is no directory "
                                   f"({SOCKETS_ENV if what == 'sockets' else SHM_ENV})")
        self.max_blocks = max_blocks
        self.hold_limit_s = hold_limit_s
        self.min_reuse_s = min_reuse_s
        self.pool = None  # _core.RosCudaPool, made for the first reader that gets descriptors
        self.sent = 0     # descriptors sent (one per reader and frame)
        self._said_drop = False
        self._lock = threading.Lock()
        self._readers: dict[str, _Reader] = {}
        self._outs: dict[str, object] = {}  # buffer key -> its zenoh publisher
        self._own_minor: int | None = None
        token = publisher.token
        pattern = "/".join([ros.ADMIN_SPACE, str(token.domain), "*", "*", "*", ros.SUBSCRIPTION, "*", "*", "*",
                            ros.mangle(publisher.topic), ros.dds_type(publisher.type_name), token.type_hash, "**"])
        self._tokens = ros._Polled(publisher._raw.liveliness().declare_subscriber(pattern, history=True),
                                   self._on_token, f"ros gpu {publisher.topic}")

    # ------------------------------------------------------------------------------------------------- readers

    def _on_token(self, sample):
        key = str(sample.key_expr)
        if sample.kind != ros.zenoh.SampleKind.PUT:
            with self._lock:
                gone = self._readers.pop(key, None)
                out = self._outs.pop(self._buf_key(gone.gid), None) if gone is not None else None
            if out is not None:
                out.undeclare()
            return
        try:
            token = ros.parse_token(key)
        except ros.RosError:
            return
        with self._lock:
            self._readers[key] = _Reader(key, token, ros.gid(key), backends_of(token))

    def _buf_key(self, gid: bytes) -> str:
        return f"{self.publisher.key}/_buf/{gid.hex()}"

    def _say(self, reader: _Reader, text: str):
        if not reader.said:
            reader.said = True
            node = reader.token.fully_qualified_node
            log.warning(f"{self.publisher.topic}: ROS reader {node} accepts CUDA buffers, gets the CPU form: {text}")

    def _decide(self, reader: _Reader, now: float):
        """Sets ``importable`` once the reader is in the registry (or says why it is not, once)."""
        if reader.importable is not None or now - reader.checked < RECHECK_S:
            return
        reader.checked = now
        entry = _core.ros_cuda_registry_lookup(self.shm_dir, reader.gid)
        if entry is None:
            if now - reader.seen > UNKNOWN_S:
                self._say(reader, f"it is not in the CUDA buffer backend's registry in {self.shm_dir} (another host, "
                                  "another /dev/shm, or no backend plugin)")
            return
        if reader.token.qos.history == ros.KEEP_ALL:
            reader.importable, why = False, "it keeps all messages (KEEP_ALL), more than any pool of blocks holds"
        else:
            reader.importable, why = self._importable(entry)
        if reader.importable:
            reader.device_id = entry["device_id"]
            log.info(f"{self.publisher.topic}: ROS reader {reader.token.fully_qualified_node} gets the frames as CUDA "
                     "buffers")
        else:
            self._say(reader, why)

    def _importable(self, entry: dict) -> tuple[bool, str]:
        if entry["entity_type"] != _core.ROS_CUDA_SUBSCRIPTION:
            return False, "its registry entry is no subscription"
        if entry["uid"] != os.getuid():
            return False, f"it runs as Linux user {entry['uid']}, this program as {os.getuid()}"
        if not entry["ipc_capable"]:
            return False, "its backend cannot import CUDA buffers (no IPC-capable memory pool)"
        pid = entry["pid"]
        registry = _core.ros_cuda_registry_name()
        try:
            with open(f"/proc/{pid}/maps") as f:
                if not any(line.rstrip().endswith("/" + registry) for line in f):
                    raise OSError("the process there does not map the registry")
            theirs = gpu_minors(pid)
            env = _environment(pid)
        except OSError as error:
            return False, (f"its process {pid} cannot be seen from here ({error}): it needs this program's pid "
                           "namespace")
        if self._own_minor is None:
            self._own_minor = minor_of(_core.cuda_device_uuid(self.device))
            if self._own_minor is None:
                return False, "this program cannot tell its own GPU's device file (/proc/driver/nvidia/gpus)"
        if len(theirs) == 1:
            if theirs != {self._own_minor}:
                return False, f"it uses GPU /dev/nvidia{min(theirs)}, this program /dev/nvidia{self._own_minor}"
            return True, ""
        mine = {k: os.environ.get(k) for k in GPU_ENV}
        if any(env.get(k) != mine[k] for k in GPU_ENV) or theirs != gpu_minors("self"):
            return False, ("it sees several GPUs, numbered otherwise than here (its CUDA_VISIBLE_DEVICES and "
                           "CUDA_DEVICE_ORDER, its GPUs): the backend compares CUDA ordinals only")
        if entry["device_id"] != self.device:
            return False, f"it uses CUDA device {entry['device_id']}, this program {self.device}"
        return True, ""

    def plan(self) -> Plan | None:
        """Where the next message goes, or None: on the topic's key as before (no reader gets descriptors, or one
        reads the key only)."""
        now = time.monotonic()
        with self._lock:
            readers = list(self._readers.values())
        if not any(r.wants_cuda for r in readers):
            return None
        if any(r.backends is None for r in readers):
            return None  # Jazzy or an older xblick: the key, for everyone
        for r in readers:
            if r.wants_cuda:
                self._decide(r, now)
        gpu = tuple(r for r in readers if r.wants_cuda and r.importable)
        if not gpu:
            return None
        cpu_group = any(r.cpu_only for r in readers)
        cpu = tuple(r for r in readers if not r.cpu_only and not (r.wants_cuda and r.importable))
        return Plan(gpu, cpu_group, cpu, max(r.token.qos.depth for r in gpu))

    # ---------------------------------------------------------------------------------------------- sending

    def _out(self, key: str):
        out = self._outs.get(key)
        if out is None:
            qos = self.publisher.qos
            out = self.publisher._raw.declare_publisher(key, reliability=ros._reliability(qos),
                                                        congestion_control=ros._congestion(qos))
            with self._lock:
                self._outs[key] = out
        return out

    def send_cpu(self, plan: Plan, payload, attachment: bytes) -> list[str]:
        """The CPU form (the message's CDR) to the readers of ``plan`` that take it."""
        sent = []
        keys = ([ros.cpu_group_key(self.publisher.key)] if plan.cpu_group else []) + \
            [self._buf_key(r.gid) for r in plan.cpu]
        for key in keys:
            self._out(key).put(bytes(payload), attachment=attachment)
            sent.append(key)
        return sent

    def send_gpu(self, plan: Plan, ring, slot: int, nbytes: int, header, height: int, width: int, encoding: str,
                 step: int, attachment: bytes) -> list[str]:
        """The frame in slot ``slot`` of the CUDA ring ``ring`` (``_core.CudaRingWriter``, this link's device) into
        a free block, its descriptor to the GPU readers of ``plan``. Nothing when every block is held (the pool
        counts it): the writer never waits."""
        if self.pool is None:
            self.pool = _core.RosCudaPool(self.device, self.socket_dir, self.shm_dir, self.max_blocks,
                                          self.hold_limit_s, self.min_reuse_s)
        index = self.pool.acquire(nbytes, plan.depth)
        if index < 0:
            if not self._said_drop:
                self._said_drop = True
                log.warning(f"{self.publisher.topic}: every block of the GPU link is held or queued by ROS readers; "
                            f"frames for them are dropped until one is free (hold limit {self.hold_limit_s} s)")
            return []
        ring.copy_slot(slot, self.pool.block_ptr(index), nbytes)
        descriptor = self.pool.publish(index, nbytes)
        sent = []
        payloads = {}
        for reader in plan.gpu:
            payload = payloads.get(reader.device_id)
            if payload is None:
                payload = payloads[reader.device_id] = _core.ros_cuda_image_payload(
                    header.stamp.sec, header.stamp.nanosec, header.frame_id, height, width, encoding, 0, step,
                    {**descriptor, "device_id": reader.device_id})
            key = self._buf_key(reader.gid)
            self._out(key).put(payload, attachment=attachment)
            sent.append(key)
        self.sent += len(sent)
        return sent

    def stats(self) -> dict:
        pool = self.pool
        return {"sent": self.sent, "dropped": pool.dropped if pool else 0, "expired": pool.expired if pool else 0,
                "blocks": pool.blocks if pool else 0, "accepts": pool.accepts if pool else 0,
                "refused": pool.refused if pool else 0}

    def close(self):
        self._tokens.undeclare()
        with self._lock:
            outs, self._outs = list(self._outs.values()), {}
        for out in outs:
            out.undeclare()
        self.pool = None  # removes its sockets, records and blocks
