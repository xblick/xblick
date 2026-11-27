"""Publishing and receiving one camera stream (topics.py: image_raw, image_raw/compressed, camera_info).

The publisher produces a representation only while somebody subscribes to it (Zenoh's matching status): no JPEG is
encoded without a subscriber to ``image_raw/compressed``, no raw image is sent without one to ``image_raw``. A
producer can ask ``subscribed`` before rendering a frame at all.

Payloads are the CDR bytes of the ROS 2 message (msg.py); the Zenoh encoding is ``application/cdr`` with the ROS
type as schema (``application/cdr;sensor_msgs/msg/Image``). What ROS's header has no field for travels in the
Zenoh attachment, a small JSON object (ATTACHMENT_VERSION): the sequence number, the publish time and the source
(e.g. ``sim`` or ``robot``).

Streams are latest-value data: publishers drop under congestion instead of queueing, subscribers keep the newest
sample and decode it only when it is read.

On one host, frames can stay in shared memory instead (``image_raw/shm``: a host ring; ``image_raw/cuda``: a ring in
device memory, CUDA virtual memory management). The publisher then writes each frame once, however many readers
there are, and sends only a descriptor (msg.ShmImage, msg.CudaImage); the rings are the C++ core (xblick._core,
cpp/include/xblick/ring.hpp). A frame overwritten before a reader got to it is dropped (``stale``), never shown
torn.

Every sample's Zenoh encoding names its type and, for Durchblick's own types, the protocol version
(``application/cdr;xblick_msgs/msg/ShmImage@1``): a subscriber refuses samples of another version
(``ProtocolError``) instead of misreading them. Each publisher answers ``<base>/info`` with its host, version and
transports; a subscriber with ``transport="auto"`` uses it to take the host ring on the publisher's host and the
network elsewhere.
"""

from __future__ import annotations

import collections
import json
import logging
import struct
import threading
import time
from dataclasses import dataclass, field, replace

import numpy as np
import zenoh

from . import _core, declaration, ros, ros_cuda, topics
from .codecs import JPEG, PNG, check_codec, compress, decompress
from .encodings import check_array, image_to_numpy, info, numpy_to_image
from .gpu import (CudaArray, CudaImage, check_cuda_array, cuda_array, device_ordinal, device_uuid, is_cuda_array,
                  write_ring)
from .msg import CameraInfo, CompressedImage, Header, Image, Message, ShmImage, TFMessage, Time, TransformStamped
from .msg import CudaImage as CudaImageMsg
from .session import ASK_PERIOD, ask, close_at_exit, forget_at_exit, replies

log = logging.getLogger("xblick")

FIND_PERIOD = ASK_PERIOD  # find_streams asks again this often: publishers become known to a session over time
DEFAULT_SLOTS = 32  # a reader has DEFAULT_SLOTS - 1 frame periods to read a frame (about a second at 30 Hz)

ATTACHMENT_VERSION = _core.protocol.ATTACHMENT_VERSION
INFO_VERSION = _core.protocol.INFO_VERSION


def ring_gpu(base: str, cuda_device: int | None) -> str | None:
    """The UUID of the GPU of a publisher's CUDA ring (in its key and its info record); None without ``cuda_device``."""
    if cuda_device is None:
        return None
    uuid = device_uuid(cuda_device)
    if not uuid:
        raise RuntimeError(f"{base}: a publisher with cuda_device {cuda_device} needs the CUDA driver (libcuda.so.1)")
    return uuid


class ProtocolError(RuntimeError):
    """A sample of another protocol version or type than the subscriber reads."""


def cdr_encoding(message_type: type[Message]) -> zenoh.Encoding:
    return zenoh.Encoding(message_type.ENCODING)


def _attachment(seq: int, source: str, zid: str | None = None, called_ns: int | None = None) -> bytes:
    """The attachment (protocol.hpp), its ``publish_ns`` now: make it right before the put. ``zid``: the publishing
    session's, for samples whose key several sessions publish on (diagnostics); ``called_ns``: when the publisher's
    publish was called (``publish_called_ns``; the difference to publish_ns is what came between: encoding, a ring
    write). Readers take the fields they know."""
    meta = {"v": ATTACHMENT_VERSION, "seq": seq, "publish_ns": time.time_ns(), "source": source}
    if called_ns is not None:
        meta["publish_called_ns"] = called_ns
    if zid is not None:
        meta["zid"] = zid
    return json.dumps(meta, separators=(",", ":")).encode()


class FrameAttachment:
    """The attachment of one frame's samples: ``now()`` at each put, so that each transport's ``publish_ns`` is when
    that sample went out (after its encoding, its copy into a ring), and ``publish_called_ns`` when publish was
    called."""

    __slots__ = ("seq", "source", "called_ns")

    def __init__(self, seq: int, source: str):
        self.seq, self.source, self.called_ns = seq, source, time.time_ns()

    def now(self) -> bytes:
        return _attachment(self.seq, self.source, called_ns=self.called_ns)


def parse_attachment(raw: bytes | None) -> dict:
    if not raw:
        return {}
    meta = json.loads(raw)
    if not isinstance(meta, dict) or meta.get("v") != ATTACHMENT_VERSION:
        raise ValueError(f"attachment {raw[:80]!r} is not a Durchblick attachment of version {ATTACHMENT_VERSION}")
    return meta


SENT_VERSION = _core.protocol.SENT_VERSION


def congestion(blocking: bool) -> zenoh.CongestionControl:
    """BLOCK for a blocking publisher (a slow subscriber holds it up instead of losing samples), else DROP."""
    return zenoh.CongestionControl.BLOCK if blocking else zenoh.CongestionControl.DROP


class SentCount:
    """A blocking publisher's count of one of its keys, answered on ``<key>/sent`` (protocol.hpp): the samples put on
    the key and the attachment seq of the last one. A recording is checked against it (recording.py)."""

    def __init__(self, session: zenoh.Session, publisher: zenoh.Publisher):
        self.key = str(publisher.key_expr)
        self._publisher = publisher
        self._lock = threading.Lock()  # sent and seq change together
        self.sent = 0
        self.seq = 0
        self._queryable = session.declare_queryable(topics.sent(self.key), self._answer)

    def count(self, seq: int):
        """One more sample put on the key, with attachment seq ``seq``."""
        with self._lock:
            self.sent += 1
            self.seq = seq

    def record(self) -> dict:
        with self._lock:
            return {"v": SENT_VERSION, "key": self.key, "sent": self.sent, "seq": self.seq,
                    "matched": self._publisher.matching_status.matching}

    def _answer(self, query: zenoh.Query):
        query.reply(topics.sent(self.key), json.dumps(self.record(), separators=(",", ":")),
                    encoding=zenoh.Encoding.APPLICATION_JSON)

    def close(self):
        self._queryable.undeclare()


class CountedPublisher:
    """The samples of one key, each with the attachment and a seq that grows by one. ``blocking``: blocking congestion
    control and the count on ``<key>/sent`` (SentCount), for recordings that must have every sample (the default
    drops: nothing is held up unless its publisher asks for it). Not thread-safe (call put from one thread).

    A session in ROS 2 also offers the samples to ROS 2 (ros.mirror), on the key's topic (``/<key>``), or on
    ROS's global ``/tf`` with ``ros_tf`` (a camera's, a robot's, a world's transforms); while a ROS subscriber is
    there."""

    def __init__(self, session: zenoh.Session, key: str, encoding: zenoh.Encoding, blocking: bool = False,
                 source: str = "", ros_tf: bool = False):
        self.key = topics.check_name(key)
        self.blocking = blocking
        self.source = source
        self.seq = 0
        self._publisher = session.declare_publisher(self.key, encoding=encoding,
                                                    congestion_control=congestion(blocking))
        self._sent = SentCount(session, self._publisher) if blocking else None
        self._ros = ros.mirror_tf(session, key=self.key) if ros_tf else ros.mirror(session, self.key, encoding)

    @property
    def matched(self) -> bool:
        """True if a subscriber of our key is known (Zenoh's matching status); ROS's subscribers: ``ros_matched``."""
        return self._publisher.matching_status.matching

    @property
    def ros_matched(self) -> bool:
        """True if a subscriber of the ROS topic this key is mirrored to is known (session.ros); False without one."""
        return self._ros is not None and self._ros.matched

    def put(self, payload: bytes):
        self.seq += 1
        self._publisher.put(payload, attachment=_attachment(self.seq, self.source))
        if self._sent is not None:
            self._sent.count(self.seq)
        if self.ros_matched:
            self._ros.put(payload)

    def close(self):
        if self._sent is not None:
            self._sent.close()
        self._publisher.undeclare()
        if self._ros is not None:
            self._ros.close()


class CameraPublisher:
    """One camera stream. ``publish()`` is not thread-safe (call it from one thread).

    ``image_raw/shm`` is always offered. ``image_raw/cuda`` only with ``cuda_device`` (the CUDA ordinal the ring is
    allocated on, normally the one the frames are rendered on). The rings are created at the first frame a reader
    asks for, sized for that frame, and made anew for a larger one (a camera back with another size): readers follow
    the new ring by its name. ``camera_info`` and ``frame_id`` may be set at any time (they go with the next frame). ``codec`` compresses ``image_raw/compressed``
    (``jpeg``; for 16-bit grey and depth ``rvl``, lossless and fast, or ``png``, lossless and smaller for label
    images); ``None``: no compressed transport, for encodings none of them
    takes (float disparity, feature maps). ``labels``: the pixels are ids (0 none, e.g. a segmenter's instances); the
    info record says so, and ``xblick rerun`` draws them over the camera's images. ``blocking``: ``image_raw``,
    ``image_raw/compressed`` and ``camera_info`` with blocking congestion control (a slow subscriber, a recorder, holds
    ``publish`` up instead of losing frames), and their counts on ``<key>/sent`` (SentCount); the rings drop as
    always. The default drops.

    A session in ROS 2 (``session.ros``, the default) also offers the stream to ROS 2 under ``/<base>/image_raw`` and
    ``/<base>/camera_info`` (each frame's camera_info with the image's header, as image_transport has it), and under
    ``/<base>/image_raw/compressed`` with codec jpeg or png (image_transport's compressed format); RVL has no ROS
    topic. Each only while a ROS subscriber is there; the rings and our keys stay as they are. With ``cuda_device``, a
    ROS reader that accepts CUDA buffers gets a frame that is in the CUDA ring as one (ros_cuda.py)."""

    def __init__(self, session: zenoh.Session, base: str, *, encoding: str, frame_id: str,
                 camera_info: CameraInfo | None = None, source: str = "", codec: str | None = JPEG,
                 jpeg_quality: int = 85, slots: int = DEFAULT_SLOTS, cuda_device: int | None = None,
                 labels: bool = False, blocking: bool = False):
        self.base = topics.check_name(base)
        declaration.add_name(session, self.base.rpartition("/")[0])  # the camera (a node's: the node) it publishes
        self.encoding = encoding
        if labels and encoding not in ("mono8", "mono16"):
            raise ValueError(f"{base}: a label image is mono8 or mono16, not {encoding}")
        self.labels = labels
        if codec is not None:
            check_codec(encoding, codec)
        self.frame_id = frame_id
        self.source = source
        self.codec = codec
        self.jpeg_quality = jpeg_quality
        self.camera_info = camera_info
        self.slots = slots
        self.cuda_device = cuda_device
        self.blocking = blocking
        self.seq = 0
        self._host = _core.host_id()
        self._gpu = ring_gpu(self.base, cuda_device)
        self._info_queryable = session.declare_queryable(topics.info(self.base), self._answer_info)
        # tells the subscribers of the stream that a publisher came or went: they choose their transport again
        self._alive = session.liveliness().declare_token(
            topics.alive(self.base, self._host, topics.new_publisher_id()))
        self._host_ring: _core.HostRingWriter | None = None
        self._cuda_ring: _core.CudaRingWriter | None = None
        drop, net = zenoh.CongestionControl.DROP, congestion(blocking)
        self._raw = session.declare_publisher(topics.image_raw(self.base), encoding=cdr_encoding(Image),
                                              congestion_control=net)
        self._compressed = (session.declare_publisher(topics.compressed(self.base),
                                                      encoding=cdr_encoding(CompressedImage), congestion_control=net)
                            if codec is not None else None)
        self._shm = session.declare_publisher(topics.shm(self.base, self._host), encoding=cdr_encoding(ShmImage),
                                              congestion_control=drop)
        self._cuda = (session.declare_publisher(topics.cuda(self.base, self._host, self._gpu),
                                                encoding=cdr_encoding(CudaImageMsg), congestion_control=drop)
                      if cuda_device is not None else None)
        self._info = session.declare_publisher(topics.camera_info(self.base), encoding=cdr_encoding(CameraInfo),
                                               congestion_control=net)
        # key -> its count (blocking only)
        self._sent = ({str(p.key_expr): SentCount(session, p) for p in (self._raw, self._compressed, self._info)
                       if p is not None} if blocking else {})
        self._ros_raw = self._ros_compressed = self._ros_info = None
        self._ros_gpu: ros_cuda.GpuLink | None = None
        taking_part = getattr(session, "ros", None)
        # a base that is no ROS name (a "-" in it): not offered to ROS, one warning line (ros.offered_topic)
        if taking_part is not None and ros.offered_topic(topics.image_raw(self.base)) is not None:
            node = taking_part.node
            self._ros_raw = node.publisher(ros.topic_of(topics.image_raw(self.base)), Image.TYPE)
            if cuda_device is not None and ros_cuda.enabled_by_environment():
                # frames on the GPU go to ROS readers that accept CUDA buffers as such (ros_cuda.py)
                self._ros_gpu = ros_cuda.GpuLink(self._ros_raw, cuda_device)
            if codec in (JPEG, PNG):  # image_transport's compressed plugin; RVL is no ROS format there
                self._ros_compressed = node.publisher(ros.topic_of(topics.compressed(self.base)), CompressedImage.TYPE)
            self._ros_info = node.publisher(ros.topic_of(topics.camera_info(self.base)), CameraInfo.TYPE)

    def _ring_uuid(self) -> str | None:
        """The CUDA ring's GPU, also before a reader made the ring exist (local auto compares it with its own)."""
        return self._gpu

    def info_record(self) -> dict:
        """What ``<base>/info`` answers (protocol.hpp describes the fields)."""
        transports = (["raw"] + (["compressed"] if self._compressed is not None else []) + ["shm"]
                      + (["cuda"] if self._cuda is not None else []))
        return {"v": INFO_VERSION, "kind": "image", "xblick": _core.__version__, "host": self._host,
                "transports": transports, "encoding": self.encoding, "frame_id": self.frame_id, "source": self.source,
                "cuda_device_uuid": self._ring_uuid(),
                **({"labels": True} if self.labels else {}), **({"blocking": True} if self.blocking else {})}

    def _answer_info(self, query: zenoh.Query):
        query.reply(topics.info(self.base), json.dumps(self.info_record(), separators=(",", ":")))

    def _image_publishers(self):
        return [p for p in (self._raw, self._compressed, self._shm, self._cuda) if p is not None]

    def _ros_wants(self) -> tuple[bool, bool]:
        """(raw, compressed): a ROS subscriber takes the stream's image_raw, its image_raw/compressed."""
        return (self._ros_raw is not None and self._ros_raw.matched,
                self._ros_compressed is not None and self._ros_compressed.matched)

    @property
    def subscribed(self) -> bool:
        """True if anybody takes images of this stream (any representation, ROS's too)."""
        return any(p.matching_status.matching for p in self._image_publishers()) or any(self._ros_wants())

    def publish(self, image, stamp_ns: int | None = None, stream=None) -> list[str]:
        """Publish one frame captured at ``stamp_ns`` (nanoseconds since the epoch; default: now). ``image`` is a NumPy
        array or a device array (``__cuda_array_interface__``, e.g. a torch CUDA tensor; needs ``cuda_device``).
        A device array is copied after the work that completes it and before this returns (it may be freed or
        reused then): ``stream`` is the stream that work is on (a CUstream handle or a stream object such as
        ``torch.cuda.current_stream()``); without it, the stream the array's interface names, else (PyTorch's names
        none) all of the process's work on the device is waited for. Returns the topics it went to (empty without
        subscribers)."""
        gpu = is_cuda_array(image)
        if gpu:
            if self.cuda_device is None:
                raise ValueError("a device array needs a publisher with cuda_device")
            array = cuda_array(image, stream)
            check_cuda_array(array, self.encoding)
        else:
            check_array(image, self.encoding)
        header = Header(stamp=Time.from_ns(time.time_ns() if stamp_ns is None else stamp_ns), frame_id=self.frame_id)
        self.seq += 1
        attachment = FrameAttachment(self.seq, self.source)
        want = self._wants()
        sent = []
        self._send(array if gpu else image, gpu, header, attachment, want, sent, encode=True)
        self._send_info(header, attachment, sent)
        return sent

    def publish_compressed(self, message: CompressedImage, image: np.ndarray | None = None) -> list[str]:
        """Publish one frame that is compressed already (a recording's ``image_raw/compressed``, a camera's JPEG),
        with its header (stamp and frame): compressed subscribers get it as it is (not encoded a second time), raw,
        shm and cuda ones the image decoded once (only if there are any). ``image``: the frame decoded already (a
        host array of the stream's encoding), taken for them instead. Returns the topics it went to."""
        if self._compressed is None:
            raise ValueError(f"{self.base}: a publisher without codec takes no compressed frames")
        if image is not None:
            check_array(image, self.encoding)
        self.seq += 1
        attachment = FrameAttachment(self.seq, self.source)
        want = self._wants()
        sent = []
        if want[self._compressed] or want["ros_compressed"]:
            payload = message.serialize()
            if want[self._compressed]:
                self._put(self._compressed, payload, attachment, sent)
            if want["ros_compressed"]:
                self._put_ros(self._ros_compressed, payload, sent)
        if (want[self._raw] or want["ros_raw"] or want[self._shm]
                or (self._cuda is not None and want[self._cuda])):
            if image is None:
                image, encoding = decompress(message)
                if encoding != self.encoding:
                    raise ProtocolError(f"{self.base}: a compressed frame of {encoding}, the stream is "
                                        f"{self.encoding}")
            self._send(image, False, message.header, attachment, want, sent, encode=False)
        self._send_info(message.header, attachment, sent)
        return sent

    def _wants(self) -> dict:
        """Which of the image publishers have subscribers now: each publisher, and ``ros_raw``,
        ``ros_compressed``."""
        want = {p: p.matching_status.matching for p in self._image_publishers()}
        want["ros_raw"], want["ros_compressed"] = self._ros_wants()
        return want

    def _send(self, image, gpu: bool, header: Header, attachment: FrameAttachment, want: dict, sent: list,
              encode: bool):
        """The frame (host array, or the device array if ``gpu``) to the image publishers that have subscribers
        (``want``, ``_wants``); ``encode``: compressed ones too. The raw and compressed bytes are made once for ours
        and ROS's."""
        cuda_slot = None
        if gpu or (self._cuda is not None and want[self._cuda]):
            cuda_slot = self._write_cuda(image)
            if self._cuda is not None and want[self._cuda]:
                self._cuda.put(self._descriptor(CudaImageMsg, header, cuda_slot[2], cuda_slot[:2]).serialize(),
                               attachment=attachment.now())
                sent.append(str(self._cuda.key_expr))
        compressed = encode and self._compressed is not None and want[self._compressed]
        ros_compressed = encode and want["ros_compressed"]
        # the GPU link (ros_cuda.py): a frame in the CUDA ring goes to ROS readers that accept CUDA buffers as one,
        # the message then on the buffer keys only; the CPU form only if another ROS reader needs it
        plan = (self._ros_gpu.plan() if self._ros_gpu is not None and want["ros_raw"] and cuda_slot is not None
                else None)
        ros_raw = want["ros_raw"] and (plan is None or plan.needs_cpu)
        ros_attachment = self._ros_raw.attachment() if plan is not None else None
        if plan is not None:
            channels = 1 if len(image.shape) == 2 else image.shape[2]
            sent += self._ros_gpu.send_gpu(plan, self._cuda_ring, cuda_slot[0], image.nbytes, header, image.shape[0],
                                           image.shape[1], self.encoding,
                                           image.shape[1] * channels * image.dtype.itemsize, ros_attachment)
        raw = want[self._raw] or ros_raw
        if raw or compressed or ros_compressed or want[self._shm]:
            host = image
            if gpu:  # one download for all CPU representations
                host = np.empty(image.shape, image.dtype)
                self._cuda_ring.read_host(cuda_slot[0], host)
            if want[self._shm]:
                written = self._write_host(host)
                self._shm.put(self._descriptor(ShmImage, header, host, written).serialize(),
                              attachment=attachment.now())
                sent.append(str(self._shm.key_expr))
            if raw:
                payload = numpy_to_image(host, self.encoding, header).serialize()
                if want[self._raw]:
                    self._put(self._raw, payload, attachment, sent)
                if ros_raw and plan is not None:
                    sent += self._ros_gpu.send_cpu(plan, payload, ros_attachment)
                elif ros_raw:
                    self._put_ros(self._ros_raw, payload, sent)
            if compressed or ros_compressed:
                payload = compress(host, self.encoding, self.codec, self.jpeg_quality, header).serialize()
                if compressed:
                    self._put(self._compressed, payload, attachment, sent)
                if ros_compressed:
                    self._put_ros(self._ros_compressed, payload, sent)

    def _send_info(self, header: Header, attachment: FrameAttachment, sent: list):
        if self.camera_info is None:
            return
        ours, theirs = self._info.matching_status.matching, self._ros_info is not None and self._ros_info.matched
        if ours or theirs:
            payload = replace(self.camera_info, header=header).serialize()
            if ours:
                self._put(self._info, payload, attachment, sent)
            if theirs:
                self._put_ros(self._ros_info, payload, sent)

    @staticmethod
    def _put_ros(publisher: ros.RosPublisher, payload, sent: list):
        publisher.put(payload)
        sent.append(publisher.key)

    def _put(self, publisher: zenoh.Publisher, payload, attachment: FrameAttachment, sent: list):
        """A sample of the current frame (seq) on one of the network keys, its payload made (encoded) already: its
        publish_ns now. Counted if blocking."""
        publisher.put(payload, attachment=attachment.now())
        key = str(publisher.key_expr)
        sent.append(key)
        if key in self._sent:
            self._sent[key].count(self.seq)

    def _descriptor(self, kind, header: Header, image, written) -> Message:
        shape = image.shape
        itemsize = image.dtype.itemsize
        channels = 1 if len(shape) == 2 else shape[2]
        fields = dict(header=header, height=shape[0], width=shape[1], encoding=self.encoding, is_bigendian=0,
                      step=shape[1] * channels * itemsize, host=self._host, slot=written[0], seq=written[1])
        if kind is ShmImage:
            return ShmImage(**fields, segment=self._host_ring.name)
        return CudaImageMsg(**fields, endpoint=self._cuda_ring.endpoint, device_uuid=self._cuda_ring.device_uuid)

    def _write_host(self, image: np.ndarray) -> tuple[int, int]:
        image = np.ascontiguousarray(image)
        if self._host_ring is None or image.nbytes > self._host_ring.slot_size:  # first, or a larger frame
            self._host_ring = _core.HostRingWriter(self.slots, image.nbytes)
        return self._host_ring.write(image)

    def _write_cuda(self, image) -> tuple[int, int, object]:
        """(slot, seq, image) of the frame in the CUDA ring; a host image is uploaded."""
        nbytes = image.nbytes
        if self._cuda_ring is None or nbytes > self._cuda_ring.slot_size:  # first, or a larger frame
            self._cuda_ring = _core.CudaRingWriter(self.cuda_device, self.slots, nbytes)
        if isinstance(image, CudaArray):
            slot, seq = write_ring(self._cuda_ring, image)
        else:
            slot, seq = self._cuda_ring.write_host(np.ascontiguousarray(image))
        return slot, seq, image

    def close(self):
        for count in self._sent.values():
            count.close()
        for publisher in self._image_publishers() + [self._info]:
            publisher.undeclare()
        if self._ros_gpu is not None:
            self._ros_gpu.close()
        for publisher in (self._ros_raw, self._ros_compressed, self._ros_info):
            if publisher is not None:
                publisher.close()
        self._alive.undeclare()
        self._info_queryable.undeclare()
        self._host_ring = None  # unlinks the shared memory; readers keep their mapping until they let go
        self._cuda_ring = None


@dataclass
class Frame:
    image: object  # np.ndarray, or gpu.CudaImage for the cuda transport
    encoding: str
    header: Header
    key: str
    receive_ns: int
    seq: int | None = None
    source: str = ""
    publish_ns: int | None = None
    meta: dict = field(default_factory=dict)
    valid: object = None  # zero-copy frames: callable telling whether the ring still holds this frame

    @property
    def stamp_ns(self) -> int:
        return self.header.stamp.to_ns()

    @property
    def age_s(self) -> float:
        """Seconds from capture to reception (meaningful only with synchronised clocks)."""
        return (self.receive_ns - self.stamp_ns) / 1e9

    @property
    def age_at_send_ns(self) -> int | None:
        """Nanoseconds from capture to the put of this frame's sample, on the publisher's clock alone: with its
        encoding (compressed) or its copy into the ring (shm, cuda) (stamps.py); None without publish_ns."""
        if not self.stamp_ns or self.publish_ns is None:
            return None
        return self.publish_ns - self.stamp_ns

    def is_valid(self) -> bool:
        """False once the publisher has begun overwriting a zero-copy frame: whatever was computed from it since may
        be wrong. Always True for copied frames."""
        return True if self.valid is None else bool(self.valid())


class _Latest:
    """The newest sample of a stream, kept as received with the transport it came by; ``take()`` hands it out once.
    ``keep``: also the last so many samples, not decoded (``recent``), for a partner stream read at a stamp
    (Subscription.keep_recent); ``on_put``: called after each sample (a partner's arrival wakes the node).

    The samples come by routes (a subscription's subscribers, numbered); those of ``route`` are kept. When a
    subscription changes its transport (``move``), it reads the old and the new route side by side until the new one
    delivers its first sample (or ``leave_old``). From then on it reads only the new one: a sample of the old route is
    left out, also one that Zenoh hands over while the old subscriber is undeclared. While both routes are read, and
    after that for HANDOVER_S or until the new route passes the newest frame kept (a relay's ring lags the network by
    a frame or more), a sample whose header stamp is not newer than the newest one kept is the same frame by the other
    route and is left out (``duplicates``). Only on the new route, a stamp more than HANDOVER_NS behind the newest one
    is a restart of the stream (a camera or a simulation): it is kept and ends the window. The old route's frames of a
    restart come again by the new route. So the stamps handed out during a move only grow, except after a restart of
    the stream. What remains:

    * copies on the new route more than HANDOVER_NS behind (a relay that queued frames while it started) are taken for
      a restart and handed out again: stamps alone do not tell the two apart;
    * a new route that stops for longer than HANDOVER_S after a first sample behind the newest one hands out older
      frames again;
    * frames that carry one stamp on purpose, or a restart less than HANDOVER_NS back, are not handed out during the
      window;
    * frames are lost when the new route is ahead of the old one (those the old one had still to bring)."""

    def __init__(self, keep: int = 0, on_put=None):
        self._cond = threading.Condition()
        self._sample = None
        self._new = False
        self.received = 0
        # samples replaced before anybody read them; with keep, none: a replaced sample stays in recent for at()
        self.overwritten = 0
        self.recent = collections.deque(maxlen=keep) if keep else None
        self._on_put = on_put
        self.duplicates = 0
        self._newest_stamp: int | None = None
        self._route = 0  # the route whose samples are kept
        self._old: int | None = None  # the route moved from, read until the new one delivers
        self._compare_until: float | None = None  # after the move: the new route's samples are compared until then

    def keep(self, n: int, on_put):
        with self._cond:
            self.recent = collections.deque(self.recent or (), maxlen=n) if n else None
            self._on_put = on_put

    def move(self, to: int):
        """The samples of route ``to`` are kept from now on; with a route before it, that one's too until ``to``
        delivers its first sample. Before its subscriber is declared."""
        with self._cond:
            if self._route:
                self._old = self._route
            self._route = to

    def wait_moved(self, timeout: float) -> bool:
        """True once the new route delivered its first sample (waits at most ``timeout`` seconds)."""
        with self._cond:
            return self._cond.wait_for(lambda: self._old is None, timeout=timeout)

    def leave_old(self):
        """Leaves the old route out from now on, also before the new one delivered: before its subscriber is
        undeclared."""
        with self._cond:
            self._leave_old_locked()

    def _leave_old_locked(self):
        if self._old is not None:
            self._old = None
            self._compare_until = time.monotonic() + HANDOVER_S
            self._cond.notify_all()

    def put(self, sample: zenoh.Sample, transport: str | None = None, route: int = 0):
        payload = sample.payload.to_bytes()
        self.offer((payload, sample.attachment.to_bytes() if sample.attachment is not None else None,
                    str(sample.key_expr), time.time_ns(), str(sample.encoding), transport), route)

    def offer(self, item: tuple, route: int = 0):
        """Keeps ``item`` (payload, attachment, key, receive_ns, encoding, transport), a sample of route ``route``,
        unless it is left out."""
        payload = item[0]
        stamp = _stamp_of(payload) if len(payload) >= 12 else None  # a sample without a header: refused when read
        with self._cond:
            if route != self._route and route != self._old:
                return  # of a route left already: its frames come by the new route
            if route == self._route:
                self._leave_old_locked()
            compare = self._old is not None or (self._compare_until is not None
                                                and time.monotonic() < self._compare_until)
            newest = self._newest_stamp
            # a restart (far back) only by the route that is kept: the old one's frames come again by the new one
            if (compare and stamp is not None and newest is not None and stamp <= newest
                    and (route != self._route or newest - HANDOVER_NS <= stamp)):
                self.duplicates += 1
                return
            if route == self._route:
                # newer than the frames of the old route, or a restart: none of its later samples is one of them
                self._compare_until = None
            self._newest_stamp = stamp
            if self._new and self.recent is None:
                self.overwritten += 1
            self._sample, self._new = item, True
            self.received += 1
            if self.recent is not None:
                self.recent.append(item)
            self._cond.notify_all()
        if self._on_put is not None:
            self._on_put()

    def take(self, consume: bool, timeout: float | None):
        with self._cond:
            if timeout is not None and not self._new:
                self._cond.wait_for(lambda: self._new, timeout=timeout)
            if self._sample is None or (consume and not self._new):
                return None
            if consume:
                self._new = False
            return self._sample


ROS = "ros"  # a transport: a ROS 2 topic (a ROS name, or ours: /<base>/image_raw, /<base>/cloud), for a session in ROS 2
# the transport of image_transport's compressed topic (<image topic>/compressed) of a ROS name, or the one auto takes
# when a ROS publisher offers only that
ROS_COMPRESSED = "ros_compressed"
# the transport of image_transport's compressedDepth topic (<image topic>/compressedDepth, ROS's compressedDepth plugin:
# 16-bit or 32FC1 depth, codecs.compressed_depth), the one auto takes when a ROS publisher offers only that
ROS_COMPRESSED_DEPTH = "ros_compressed_depth"
ROS_FORMS = (ROS, ROS_COMPRESSED, ROS_COMPRESSED_DEPTH)
TRANSPORTS = {"compressed": topics.compressed, "raw": topics.image_raw, "shm": topics.shm, "cuda": topics.cuda,
              ROS: topics.image_raw}
_TYPES = {"compressed": CompressedImage, "raw": Image, "shm": ShmImage, "cuda": CudaImageMsg, ROS: Image,
          ROS_COMPRESSED: CompressedImage, ROS_COMPRESSED_DEPTH: CompressedImage}
AUTO = "auto"
# how long a subscription that changes its transport keeps the old one for the new one's first frame; after that, as
# long again the new route's samples are compared with the newest frame kept (_Latest), and a stamp at most as far
# behind it is the same frame
HANDOVER_S = 2.0
HANDOVER_NS = int(HANDOVER_S * 1_000_000_000)  # the window in stamp time, an integer as C++'s
WAIT_HINT_S = 5.0  # a subscription that has no publisher this long says once what may be wrong
ROS_RECHECK_S = 1.0  # auto on a ROS name looks at ROS's graph this often besides when its listener tells it
SETTLE_ROUNDS = 3  # info queries after a publisher came or went (its queryable can be known later than its token)
# what a subscriber reads, for its start line: where the frames are (host or GPU memory) or how they come
READS = {"shm": "the host ring", "cuda": "the CUDA ring", "compressed": "compressed frames over Zenoh",
         "raw": "raw frames over Zenoh", ROS: "the ROS 2 topic", ROS_COMPRESSED: "the ROS 2 compressed topic",
         ROS_COMPRESSED_DEPTH: "the ROS 2 compressedDepth topic"}
LOCAL = ("auto", "shm", "cuda")
REMOTE = ("compressed", "raw")


def _check_encoding(key: str, got: str, message_type: type[Message]):
    if got != message_type.ENCODING:
        raise ProtocolError(f"{key}: the publisher sends {got!r}, this subscriber reads {message_type.ENCODING!r} "
                            f"(Durchblick {_core.__version__}); run the same Durchblick version on both sides")


def query_info(session: zenoh.Session, base: str, timeout: float = 1.0) -> list[dict]:
    """The info records of the publishers of a stream (usually one), asked once (a subscriber's chooser asks in its
    own loop; a session just opened finds nothing yet: ``wait_for_info``)."""
    return _info_records(base, replies(session, topics.info(base), timeout))


def wait_for_info(session: zenoh.Session, base: str, timeout: float = 1.0) -> list[dict]:
    """As ``query_info``, asked again until a publisher answers or ``timeout`` passed (session.ask): for a session
    just opened, as a command line tool's."""
    return _info_records(base, ask(session, topics.info(base), timeout))


def _info_records(base: str, answers: list) -> list[dict]:
    records = []
    for reply in answers:
        if reply.ok is None:
            continue
        record = json.loads(reply.ok.payload.to_string())
        if not isinstance(record, dict) or record.get("v") != INFO_VERSION:
            raise ProtocolError(f"{base}: info record {str(record)[:120]} is not of version {INFO_VERSION} "
                                f"(Durchblick {_core.__version__}); run the same Durchblick version on both sides")
        records.append(record)
    return records


_not_info: set[str] = set()  # keys whose answer to <key>/info was no info record (logged once)


class StreamGivenAsCamera(SystemExit):
    """A stream given where its camera belongs, such as a node's ``--camera cam/video2/color``: nothing answers the
    stream the program reads (``cam/video2/color/color``), and the name before its last part answers as a stream
    itself. A SystemExit with one line (exit code 1): the program ends, it does not read the camera in its place."""


def stream_given_as_camera(base: str) -> str:
    """The line of StreamGivenAsCamera for a stream ``base`` whose parent is a stream."""
    stream = base.rpartition("/")[0]
    camera = stream.rpartition("/")[0]
    return (f"{base}: no publisher answers it, and {stream} is a stream: the camera is {camera}. A node takes the "
            f"camera: --camera {camera}" if camera else
            f"{base}: no publisher answers it, and {stream} is a stream (a node takes the camera, the name before a "
            "stream's last part)")


def find_streams(session: zenoh.Session, prefix: str = "", timeout: float = 2.0) -> list[tuple[str, dict]]:
    """(stream base, info record) of every stream below ``prefix`` that answers ``<base>/info`` within ``timeout``,
    sorted by base; with relays (xblick-relay) the source's record, the relays' hosts in its "relayed_by" (a
    stream only relays answer for is left out: no source). Costs the publishers nothing (subscribing to everything would make them produce every
    representation). A query reaches only the publishers the session knows of when it is sent: right after the
    session opened none (through a router their declarations arrive some 0.1-0.3 s later), so it asks again every
    FIND_PERIOD seconds until ``timeout``. A reply that is no info record (not JSON; seen once from a publisher
    being stopped) is left out and logged once per key: discovery goes on."""
    selector = (prefix.strip("/") + "/**/" if prefix else "**/") + topics.INFO
    found = {}
    relays = {}
    end = time.monotonic() + timeout
    while (remaining := end - time.monotonic()) > 0:
        for reply in replies(session, selector, remaining):
            if reply.ok is not None:
                key = str(reply.ok.key_expr)
                try:
                    record = json.loads(reply.ok.payload.to_bytes())
                    if not isinstance(record, dict):
                        raise ValueError(f"{type(record).__name__}, not an object")
                except ValueError as error:
                    if key not in _not_info:
                        _not_info.add(key)
                        log.warning(f"{key}: an answer that is no info record, left out ({error})")
                    continue
                base = key[: -len(topics.INFO) - 1]
                if record.get("relayed_from"):  # a relay's (xblick-relay): noted in the source's record
                    relays.setdefault(base, set()).add(str(record.get("host", "")))
                else:
                    found[base] = record
        time.sleep(max(0.0, min(FIND_PERIOD, end - time.monotonic())))
    # one record per stream, the source's (a relay offers only its own host's rings); its relays' hosts in "relayed_by"
    return sorted((base, dict(record, relayed_by=sorted(relays.get(base, ())))) for base, record in found.items())


def _declare_advanced_history(session: zenoh.Session, key: str, answer) -> zenoh.Queryable:
    """The queryable ``topics.advanced_history`` of the once-published ``key``: ``answer(query)`` replies on ``key``
    itself, as zenoh-ext's AdvancedPublisher answers from its cache. zenoh-bridge-ros2dds asks it for a transient-local
    ROS subscriber (``/tf_static``) that starts after the put."""
    def history(query: zenoh.Query):
        # a reply on the key itself lies outside the query's key: only a query with _anyke accepts it, as
        # zenoh-ext's AdvancedSubscriber sends; another query gets no reply
        if query.accepts_replies() == zenoh.ReplyKeyExpr.ANY:
            answer(query)
    return session.declare_queryable(topics.advanced_history(key, str(session.info.zid())), history)


class StaticTransformPublisher:
    """A camera's fixed transforms between its streams' frames on ``<camera>/tf_static``: published once and answered
    on request until :meth:`close`, as ROS's latched ``/tf_static``. Also answered to zenoh-ext's AdvancedSubscriber
    (``_declare_advanced_history``), so that a late transient-local ROS subscriber behind zenoh-bridge-ros2dds gets
    them. A session in ROS 2 also publishes them once on ROS 2's global ``/tf_static``, transient local with a
    cache of one message (as tf2's static broadcaster), so that a ROS subscriber that starts later gets them until
    :meth:`close`."""

    def __init__(self, session: zenoh.Session, camera: str, transforms: list[TransformStamped]):
        self.key = topics.tf_static(camera)
        self._payload = bytes(TFMessage(transforms=list(transforms)).serialize())
        # the queryables first: a subscriber that starts after the put asks for it
        self._queryable = session.declare_queryable(self.key, self._answer)
        self._history = _declare_advanced_history(session, self.key, self._answer)
        session.put(self.key, self._payload, encoding=cdr_encoding(TFMessage))
        self._ros = ros.mirror_tf(session, static=True)
        if self._ros is not None:
            self._ros.put(self._payload)  # kept in its cache for later ROS subscribers

    def _answer(self, query: zenoh.Query):
        query.reply(self.key, self._payload, encoding=cdr_encoding(TFMessage))

    def close(self):
        self._history.undeclare()
        self._queryable.undeclare()
        if self._ros is not None:
            self._ros.close()


class StaticTransformSubscriber:
    """Calls ``callback(TFMessage)`` with a camera's fixed transforms: those its publisher answers when asked (at the
    start, in a thread of its own, asked again until an answer comes or ``timeout`` ends: right after a session
    connects, a query can end at once without answers) and every later publication. Errors (another protocol version)
    are kept and raised by :meth:`check`.

    ``ros`` (a session in ROS 2): ROS 2's global ``/tf_static`` instead, every publisher's (transient local:
    the messages of publishers that were there before come from their caches), ``camera`` unused."""

    def __init__(self, session: zenoh.Session, camera: str, callback, timeout: float = 10.0, ros: bool = False):
        self.key = "/tf_static" if ros else topics.tf_static(camera)
        self._callback = callback
        self._error: BaseException | None = None
        self._closed = False
        self._query = None
        if ros:
            self._subscriber = _ros_tf_subscriber(session, True, self._deliver_ros)
            return
        self._subscriber = session.declare_subscriber(self.key, lambda s: self._deliver(s.key_expr, s.encoding,
                                                                                        s.payload))
        self._query = threading.Thread(target=self._ask, args=(session, timeout), daemon=True)
        self._query.start()
        close_at_exit(self, f"the subscriber of {self.key}")  # its thread asks over Zenoh: stopped at the end

    def _deliver_ros(self, payload: bytes, *_):
        if self._error is not None:
            return
        try:
            self._callback(TFMessage.deserialize(payload))
        except BaseException as error:  # noqa: BLE001 - raised in the caller's thread by check()
            self._error = error

    def _ask(self, session: zenoh.Session, timeout: float):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and not self._closed:
            answered = False
            for reply in replies(session, self.key, 0.5):
                if reply.ok is not None:
                    answered = True
                    self._deliver(reply.ok.key_expr, reply.ok.encoding, reply.ok.payload)
            if answered:
                return
            time.sleep(0.1)

    def _deliver(self, key, encoding, payload):
        if self._error is not None:
            return
        try:
            _check_encoding(str(key), str(encoding), TFMessage)
            self._callback(TFMessage.deserialize(payload.to_bytes()))
        except BaseException as error:  # noqa: BLE001 - raised in the caller's thread by check()
            self._error = error

    def check(self):
        if self._error is not None:
            raise self._error

    def close(self):
        forget_at_exit(self)
        self._closed = True
        _undeclare(self._subscriber)
        if self._query is not None:
            self._query.join()


class TransformPublisher:
    """A camera's moving transforms on ``<camera>/tf``, as ROS's ``/tf``: its pose in the world (parent frame
    ``topics.WORLD``), one message per image, stamped as that image, so that a reader finds the pose an image was taken
    from by its stamp. ``blocking`` as CountedPublisher's."""

    def __init__(self, session: zenoh.Session, camera: str, blocking: bool = False):
        self.key = topics.tf(camera)
        self._publisher = CountedPublisher(session, self.key, cdr_encoding(TFMessage), blocking=blocking, ros_tf=True)

    def publish(self, transforms: list[TransformStamped]):
        self._publisher.put(bytes(TFMessage(transforms=list(transforms)).serialize()))

    def close(self):
        self._publisher.close()


def _ros_tf_subscriber(session, static: bool, callback):
    """A subscription to ROS 2's global ``/tf`` or ``/tf_static`` (as tf2's listener: depth 100, the static one
    transient local) for a session in ROS 2."""
    if getattr(session, "ros", None) is None:
        raise ValueError(f"reading ROS 2's /tf{'_static' if static else ''} needs {ros.NEEDS_ROS}")
    return session.ros.node.subscriber("/tf_static" if static else "/tf", TFMessage.TYPE, callback,
                                       ros.TF_STATIC_READ_QOS if static else ros.TF_QOS)


class TransformSubscriber:
    """Calls ``callback(TFMessage)`` with every message on a camera's ``<camera>/tf``, in Zenoh's thread. Errors
    (another protocol version) stop the deliveries; :meth:`check` raises them. ``ros`` (a session in ROS 2):
    ROS 2's global ``/tf`` instead, ``camera`` unused."""

    def __init__(self, session: zenoh.Session, camera: str, callback, ros: bool = False):
        self.key = "/tf" if ros else topics.tf(camera)
        self._callback = callback
        self._error: BaseException | None = None
        if ros:
            self._subscriber = _ros_tf_subscriber(session, False, lambda payload, *_: self._take(payload))
        else:
            self._subscriber = session.declare_subscriber(self.key, self._on_sample)

    def _take(self, payload: bytes):
        if self._error is not None:
            return
        try:
            self._callback(TFMessage.deserialize(payload))
        except BaseException as error:  # noqa: BLE001 - raised in the caller's thread by check()
            self._error = error

    def _on_sample(self, sample: zenoh.Sample):
        if self._error is not None:
            return
        try:
            _check_encoding(self.key, str(sample.encoding), TFMessage)
            self._callback(TFMessage.deserialize(sample.payload.to_bytes()))
        except BaseException as error:  # noqa: BLE001 - raised in the caller's thread by check()
            self._error = error

    def check(self):
        if self._error is not None:
            raise self._error

    def close(self):
        _undeclare(self._subscriber)


class MaxRate:
    """At most ``hz`` frames a second, decided by their stamps before they are decoded: a frame is taken when its stamp
    lies in a later period of 1/hz s than the last frame taken (periods counted from the epoch, so that the streams of
    one camera, which share their stamps, take the same frames and still pair). The frame taken is the newest there is
    then; the later ones of its period are skipped. A stamp that went back (a camera's restart) is taken."""

    def __init__(self, hz: float):
        if not hz > 0:
            raise ValueError(f"max-hz {hz}: a rate above 0 (none: unlimited)")
        self.hz = hz
        self.period_ns = round(1e9 / hz)
        self._last: int | None = None

    def admit(self, stamp_ns: int) -> bool:
        period = stamp_ns // self.period_ns
        if period == self._last:
            return False
        self._last = period
        return True


def _stamp_of(payload: bytes) -> int:
    """The header stamp of a stream's sample (every representation starts with a std_msgs/Header), not decoded."""
    if len(payload) < 12:
        raise ProtocolError(f"a sample of {len(payload)} bytes has no header")
    sec, nanosec = struct.unpack_from("<iI", payload, 4)
    return sec * 1_000_000_000 + nanosec


def _undeclare(subscriber):
    """A Zenoh subscriber undeclared, a ROS one (ros.RosSubscriber) closed."""
    if isinstance(subscriber, ros.RosSubscriber):
        subscriber.close()
    else:
        subscriber.undeclare()


def check_local(subscriber, local: str):
    """Raises when ``local="cuda"`` was asked for and the publisher, on this host, offers no CUDA ring: ``auto`` reads
    its shared memory instead, unseen (a node told to keep frames on the GPU calls this once a frame came)."""
    if local == "cuda" and subscriber.transport == "shm":
        raise RuntimeError(f"{subscriber.base}: its publisher on this host offers no CUDA ring; start it with a CUDA "
                           "device, or read with --local shm")


class Subscription:
    """The subscription of one stream (image or point cloud): a Zenoh subscriber on the topic of the transport in use,
    keeping the newest sample. With ``transport="auto"`` the transport is chosen in a thread of its own from the
    publishers' info records (whose ``kind`` must be ``KIND``): ``local`` on a publisher's host (``"shm"``, or
    ``"cuda"``: the CUDA ring where the publisher offers one, else shm), ``remote`` elsewhere. The thread chooses again
    whenever a publisher of the stream comes or goes (their liveliness tokens), so that the order in which programs
    start does not matter: a subscription that reads a remote stream over the network moves to the ring of a relay
    that starts later on its host, and back when the relay stops. While it moves it reads both routes, and the stamps
    it hands out only grow, except after a restart of the stream: no frame twice, none out of order (``_Latest`` names
    the other exceptions, and where frames are lost). Subclasses say what their transports are and decode the
    samples. ``max_hz``: at most so many frames a second (MaxRate), the others skipped before they are decoded or read
    from their ring and counted in ``skipped_max_hz``; None: every frame.

    In a session in ROS 2, ``auto`` reads a stream of ours that no publisher of ours answers from its ROS form
    (``ros_forms``: ``/<base>/image_raw``, else ``/<base>/image_raw/compressed``, else
    ``/<base>/image_raw/compressedDepth``; a cloud's ``/<base>/cloud``) when a
    ROS publisher of another program offers it. The ROS copies that our programs publish are never read in place of a
    stream's own forms, and once a publisher of ours answers, the subscription moves to it.

    ``base`` with a leading ``/`` is a ROS 2 topic as ROS's tools write it (``/image_raw``, ``/resize/image``, a
    cloud's ``/points``), read from ROS publishers (``ros_topic``; a session in ROS 2): ``auto`` takes the form its
    publishers offer (``ros_forms``: the topic itself, else image_transport's compressed topic of an image topic),
    ``ros`` and ``raw`` the topic itself, ``compressed`` image_transport's compressed topic. After WAIT_HINT_S
    without a sample of a ROS form, ROS's graph is looked at: a topic of another type, or a name that is no topic but
    has image topics below it, is an error (a ValueError naming them), raised by ``latest``; nothing is chosen in
    its place."""

    KIND = ""
    TRANSPORTS: dict = {}  # transport -> topic function
    TYPES: dict = {}  # transport -> message type
    REMOTE: tuple = ()

    def __init__(self, session: zenoh.Session, base: str, transport: str, local: str, remote: str, copy: bool,
                 max_hz: float | None = None, device=None):
        if transport not in self.TRANSPORTS and transport != AUTO:
            raise ValueError(f"transport {transport!r}: auto or one of {', '.join(self.TRANSPORTS)}")
        self.ros_topic = ros.is_topic(base)
        if self.ros_topic:
            if getattr(session, "ros", None) is None:
                raise ValueError(f"{base}: a ROS 2 topic (a leading /); reading it needs {ros.NEEDS_ROS}")
            transport = self._ros_transport(ros.check_ros_name(base), transport)
        elif transport == ROS and getattr(session, "ros", None) is None:
            raise ValueError(f"transport ros reads the stream's ROS 2 topic: it needs {ros.NEEDS_ROS}")
        if local not in LOCAL:
            raise ValueError(f"local {local!r}: one of {', '.join(LOCAL)}")
        if remote not in self.REMOTE:
            raise ValueError(f"remote {remote!r}: one of {', '.join(self.REMOTE)}")
        if not copy and (transport in self.REMOTE or transport in ROS_FORMS):
            raise ValueError("copy=False is for the shm and cuda transports (and auto)")
        self.base = base if self.ros_topic else topics.check_name(base)
        self._ros_forms_error: Exception | None = None
        try:
            self.ros_forms = self._forms() if getattr(session, "ros", None) is not None else {}
        except ros.RosError as error:  # a base that is no ROS name ("-" in it): no ROS form
            self.ros_forms, self._ros_forms_error = {}, error
        if transport in ROS_FORMS and transport not in self.ros_forms:
            raise self._ros_forms_error or ValueError(f"{self.base}: no ROS form {transport}")
        self.requested = transport
        self.local = local
        # the GPU it works on (local auto: the CUDA ring if it is on it); None or "cpu": the CPU, so auto takes shm
        self._on_cpu = device is None or str(device) == "cpu"
        self.device = None if self._on_cpu else device_ordinal(device)
        self.transport_reason = ""  # why it took the transport it took (auto)
        self.remote = remote
        self.copy = copy
        self.stale = 0  # descriptors whose data was overwritten before it could be read
        self.max_rate = MaxRate(max_hz) if max_hz is not None else None
        self.skipped_max_hz = 0  # frames left out for max_hz, not decoded
        self.decoded = 0  # frames decoded (or read from their ring) and handed out
        self._keep, self._on_put = 0, None  # keep_recent
        self._session = session
        self._host = _core.host_id()
        self._lock = threading.Lock()
        self._chosen = threading.Event()
        self._closed = threading.Event()
        self._error: BaseException | None = None
        self.transport: str | None = None
        self._subscriber = None
        self._rings = {}  # transport -> (name, reader)
        self._gone: set[str] = set()  # rings of descriptors that could not be opened (their publisher stopped)
        self._latest = _Latest()
        self._using: tuple[str, str] | None = None  # (transport, gpu of its CUDA ring) of the subscriber in use
        self._route = 0  # the subscribers declared so far (numbers the routes of _Latest)
        self._recheck = threading.Event()  # a publisher came or went: choose again
        self._alive = None
        self._chooser = None
        self._want = None  # the liveliness token that asks this host's relay for the stream (_tell_relay)
        self._started = time.monotonic()
        self._ros_checked = False  # the graph looked at, once, for a ROS form that brought nothing (_check_ros)
        self._graph = None  # ROS's graph, listened to by auto for a ROS name
        if self.ros_forms and (self.ros_topic or transport in ROS_FORMS):
            self._graph = session.ros.graph  # made now, so that it knows the publishers when it is asked
        if transport == AUTO:
            if self.ros_topic:
                self._graph.listen(self._recheck.set)
            else:
                self._alive = session.liveliness().declare_subscriber(topics.alive_all(self.base),
                                                                      lambda sample: self._recheck.set(), history=True)
            self._chooser = threading.Thread(target=self._choose, name=f"xblick-auto {self.base}", daemon=True)
            self._chooser.start()
            # the chooser keeps the subscription alive: left open, its ring readers would never be freed (nanobind
            # reports them as leaked when the program ends)
            close_at_exit(self, f"the subscription of {self.base}")
        else:
            self._use(transport)

    def _ros_transport(self, topic: str, transport: str) -> str:
        """The transport of a ROS name (``ros_topic``): auto, ros for ``ros`` and ``raw`` (the topic itself),
        ROS_COMPRESSED for ``compressed`` (image_transport's compressed topic, if the kind has one)."""
        if transport in (ROS, "raw"):
            return ROS
        if transport == AUTO:
            return AUTO
        if transport == "compressed" and ROS_COMPRESSED in self.TYPES:
            return ROS_COMPRESSED
        kinds = "auto, ros, raw" + (" or compressed" if ROS_COMPRESSED in self.TYPES else "")
        raise ValueError(f"{topic} is a ROS 2 topic: transport {kinds}, not {transport}")

    def _forms(self) -> dict:
        """ROS form -> (topic, message type), in the order auto takes them (``ros_forms``). Subclasses."""
        return {}

    def keep_recent(self, n: int, on_put=None):
        """Keep the last ``n`` samples as they came (not decoded) for ``at``, a partner stream's frames taken at the
        stamps of another's (history.PartnerHistory); ``on_put`` is called after each sample. Before any frame."""
        with self._lock:
            self._keep, self._on_put = n, on_put
            self._latest.keep(n, on_put)

    def recent_stamps(self) -> list[int]:
        """The stamps of the samples kept (keep_recent), oldest first, none of them decoded."""
        latest = self._latest
        with latest._cond:
            items = list(latest.recent or ())
        return [_stamp_of(item[0]) for item in items]

    def _recent_at(self, stamp_ns: int, max_offset_ns: int):
        """(payload, meta, key, receive_ns, transport) of the kept sample nearest to ``stamp_ns``, at most
        ``max_offset_ns`` away; None if there is none."""
        if self._error is not None:
            raise self._error
        latest = self._latest
        if self.transport is None:
            return None
        with latest._cond:
            items = list(latest.recent or ())
        best = min(items, key=lambda item: abs(_stamp_of(item[0]) - stamp_ns), default=None)
        if best is None or abs(_stamp_of(best[0]) - stamp_ns) > max_offset_ns:
            return None
        payload, attachment, key, receive_ns, encoding_of_sample, transport = best
        _check_encoding(key, encoding_of_sample, self.TYPES[transport])
        return payload, parse_attachment(attachment), key, receive_ns, transport

    # ------------------------------------------------------------------------------------- choosing (auto)

    def _key(self, transport: str, gpu: str = topics.ANY_GPU) -> str:
        """The key of the stream's ``transport`` for this subscriber: the rings' keys name this host, a CUDA ring's
        the GPU too (``gpu``: its UUID; the default: the rings on every GPU of this host)."""
        topic = self.TRANSPORTS[transport]
        if transport == "shm":
            return topic(self.base, self._host)
        if transport == "cuda":
            return topic(self.base, self._host, gpu)
        return topic(self.base)

    def _use(self, transport: str, gpu: str = topics.ANY_GPU):
        """Reads the stream by ``transport`` from now on. With a transport in use already, both are read until the
        new one delivered its first frame (at most HANDOVER_S: a stream that sends nothing loses nothing), each frame
        handed out once (_Latest)."""
        latest = self._latest
        with self._lock:
            self._route += 1
            route = self._route
            old = self._subscriber
            latest.move(route)
            if transport in ROS_FORMS:
                self._subscriber = self._ros_subscriber(transport, latest, route)
            else:
                self._subscriber = self._session.declare_subscriber(
                    self._key(transport, gpu), lambda sample: latest.put(sample, transport, route))
            self.transport, self._using = transport, (transport, gpu)
            self._chosen.set()
        self._used(transport)
        if old is None:
            return
        end = time.monotonic() + HANDOVER_S
        while not self._closed.is_set() and time.monotonic() < end and not latest.wait_moved(0.05):
            pass
        # before the undeclare: Zenoh can still hand over a sample of the old subscriber while it is undeclared
        latest.leave_old()
        _undeclare(old)

    def _used(self, transport: str):
        """Called after the subscription took ``transport`` (ImageSubscriber: its camera_info follows)."""

    def _ros_subscriber(self, transport: str, latest: _Latest, route: int):
        """The ROS 2 subscription of a ROS form (``ros_forms``): its samples as ours of TYPES[transport], without our
        attachment (no seq, source or publish time)."""
        topic, message_type = self.ros_forms[transport]
        return self._session.ros.node.subscriber(
            topic, message_type.TYPE,
            lambda payload, seq, stamp, publisher: latest.offer((payload, None, topic, time.time_ns(),
                                                                 message_type.ENCODING, transport), route),
            cuda_device=self._ros_cuda_device(message_type))

    def _ros_cuda_device(self, message_type) -> int | None:
        """The GPU on which a ROS form's frames are taken as CUDA buffers from ROS publishers that have them there
        (ros_cuda.GpuReceiver): this subscriber's ``device`` for an image topic, unless XBLICK_ROS_CUDA is 0; None for
        a subscriber on the CPU (it gets the bytes)."""
        if self._on_cpu or message_type.TYPE not in ros_cuda.LINK_TYPES or not ros_cuda.enabled_by_environment():
            return None
        return self.device

    def ros_gpu_stats(self) -> dict | None:
        """The counters of the CUDA buffers taken from ROS publishers (ros_cuda.GpuReceiver.stats) while a ROS form
        is read with a device; None otherwise."""
        with self._lock:
            subscriber = self._subscriber
        gpu = getattr(subscriber, "gpu", None)
        return gpu.stats() if gpu is not None else None

    def _ros_choice(self, foreign_only: bool = False) -> str | None:
        """The first ROS form (``ros_forms``) that a ROS publisher offers now, of the form's type; None without one.
        ``foreign_only``: publishers of other programs than ours (Graph.by_xblick): the ROS copies of a stream of
        ours are never read in place of its own forms."""
        for form, (topic, message_type) in self.ros_forms.items():
            for token in self._graph.publishers(topic):
                if token.type_name == message_type.TYPE and not (foreign_only and self._graph.by_xblick(token.zid)):
                    return form
        return None

    def _check_ros(self):
        """Once, WAIT_HINT_S after the start without a sample, for a subscription that reads or looks for a ROS form:
        a ROS publisher of its topic with another type is a ProtocolError, a ROS name that is no topic but has topics
        of the kind below it a ValueError naming them (both kept and raised by ``latest``); else one line that it
        waits."""
        if (self._ros_checked or self._latest.received or not self.ros_forms
                or time.monotonic() - self._started < WAIT_HINT_S):
            return
        if not (self.ros_topic or self.transport in ROS_FORMS):
            return
        self._ros_checked = True
        forms = ({self.transport: self.ros_forms[self.transport]} if self.transport in ROS_FORMS
                 else self.ros_forms)
        kinds = {m.TYPE for _, m in self.ros_forms.values()}
        for topic, message_type in forms.values():
            other = sorted({t.type_name for t in self._graph.publishers(topic)} - {message_type.TYPE})
            if other:
                hint = ""
                if other == [ros.COMPRESSED_TYPE] and topic.endswith("/" + ros.COMPRESSED):
                    hint = (f": image_transport's compressed topic of {topic.rpartition('/')[0]}, which transport "
                            "compressed reads")
                elif other == [ros.COMPRESSED_TYPE] and topic.endswith("/" + ros.COMPRESSED_DEPTH):
                    hint = (f": image_transport's compressedDepth topic of {topic.rpartition('/')[0]}, which auto "
                            "reads where it is the only form")
                self._error = ProtocolError(f"{topic}: its ROS publishers send {', '.join(other)}, this subscriber "
                                            f"reads {message_type.TYPE}{hint}")
                raise self._error
        if self.ros_topic and not any(self._graph.publishers(t) for t, _ in self.ros_forms.values()):
            below = sorted({t.topic for t in self._graph.publishers() if t.type_name in kinds
                            and t.topic.startswith(self.base + "/")})
            if below:
                self._error = ValueError(f"{self.base}: no ROS topic of this name, and these are below it: "
                                         f"{', '.join(below)}; name one of them")
                raise self._error
        log.info(f"{self.base}: no ROS publisher of {' or '.join(t for t, _ in forms.values())} within "
                 f"{WAIT_HINT_S:g} s, waiting for one")

    def _choice(self, records: list[dict]) -> tuple[str, str, str, bool]:
        """(transport, gpu, why, local) for the publishers that answered ``<base>/info``."""
        for r in records:
            if r.get("kind") != self.KIND:
                raise ProtocolError(f"{self.base} is a stream of kind {r.get('kind')!r}, this subscriber "
                                    f"reads {self.KIND!r}")
        local = [r for r in records if r.get("host") == self._host and "shm" in r.get("transports", ())]
        if not local:
            # a relay (xblick-relay) offers only its own host's rings: the network forms are the source's
            sources = [r for r in records if not r.get("relayed_from")] or records
            if any(self.remote not in r.get("transports", (self.remote,)) for r in sources):
                raise ProtocolError(f"{self.base}: its publisher on another host offers no {self.remote} "
                                    "transport (a stream without a compressed form: subscribe with raw)")
            return self.remote, topics.ANY_GPU, "publisher on another host", False
        chosen, why = self._local_choice(local)
        gpus = {str(r.get("cuda_device_uuid")) for r in local}
        # the key of the one ring the records name; rings on several GPUs (two publishers): all of this host
        gpu = gpus.pop() if chosen == "cuda" and len(gpus) == 1 else topics.ANY_GPU
        return chosen, gpu, why, True

    def _tell_relay(self, records: list[dict]):
        """Holds the token ``<base>/want/<host>/<id>`` while the stream has no source with a ring on this host: the
        relay of this host (xblick-relay, where one runs) then brings the stream into rings here, and this
        subscription moves to them. Without a relay nothing follows from it."""
        here = any(r.get("host") == self._host and "shm" in r.get("transports", ()) and not r.get("relayed_from")
                   for r in records)
        if not here and self._want is None:
            self._want = self._session.liveliness().declare_token(
                topics.want(self.base, self._host, topics.new_publisher_id()))
        elif here and self._want is not None:
            self._want.undeclare()
            self._want = None

    def _choose(self):
        """The chooser's thread: chooses when a publisher is found and again whenever one comes or goes (the
        liveliness subscriber sets ``_recheck``). Without a publisher it asks every 0.5 s (a publisher without a
        liveliness token, such as a camera program on an embedded board, is found this way)."""
        try:
            settle = 0
            waiting_since = time.monotonic()
            hinted = False
            while True:
                self._recheck.clear()
                if self._closed.is_set():  # after the clear: close() sets closed, then recheck
                    break
                if self.ros_topic:  # a ROS name: ROS's publishers only, told by the graph's listener
                    self._choose_ros()
                    self._recheck.wait(ROS_RECHECK_S)
                    continue
                records = query_info(self._session, self.base)
                if not records and not self._closed.is_set():
                    self._check_not_below_a_stream()
                    if self._choose_ros_only():
                        waiting_since, hinted = time.monotonic(), False
                    elif not hinted and time.monotonic() - waiting_since >= WAIT_HINT_S:
                        hinted = True
                        log.info(f"{self.base}: no publisher answers within {WAIT_HINT_S:g} s, waiting for it: a "
                                 "camera that has not started yet, or a wrong name (a node's --camera is the camera, "
                                 "such as cam/video0; its streams are below it, such as cam/video0/color)")
                elif records:
                    waiting_since, hinted = time.monotonic(), False
                if records and not self._closed.is_set():
                    self._tell_relay(records)
                    chosen, gpu, why, local = self._choice(records)
                    if (chosen, gpu) != self._using:
                        before = self.transport_reason
                        self.transport_reason = f"{chosen} ({why})"
                        where = "this host" if local else "host " + ", ".join(str(r.get("host"))[:8] for r in records)
                        sources = ", ".join(sorted({str(r.get("source")) for r in records if r.get("source")}))
                        log.info(f"{self.base}: publisher found on {where}" + (f" ({sources})" if sources else "")
                                 + f", reading {READS.get(chosen, chosen)}: {self.transport_reason}"
                                 + (f" (was {before})" if before else ""))
                        self._use(chosen, gpu)
                if self._recheck.wait(0.5 if not records or settle else None):
                    settle = SETTLE_ROUNDS
                elif settle:
                    settle -= 1
        except BaseException as error:  # noqa: BLE001 - raised in the caller's thread by latest()
            self._error = error
            self._chosen.set()

    def _choose_ros(self, foreign_only: bool = False, why: str = "") -> bool:
        """Takes the ROS form a ROS publisher offers (``_ros_choice``) when it is not the one in use. True when one is
        offered."""
        chosen = self._ros_choice(foreign_only)
        if chosen is None:
            return False
        if (chosen, topics.ANY_GPU) != self._using:
            before = self.transport_reason
            self.transport_reason = f"{chosen} ({why or 'a ROS publisher'})"
            log.info(f"{self.base}: ROS publisher found, reading {READS[chosen]} {self.ros_forms[chosen][0]}"
                     + (f" (was {before})" if before else ""))
            self._use(chosen)
        return True

    def _choose_ros_only(self) -> bool:
        """auto on a stream of ours that no publisher of ours answers: its ROS form (``ros_forms``) when a ROS
        publisher of another program than ours offers it (a ROS camera, ``ros2 bag play``), never our programs' ROS
        copies. Our forms keep priority: the next round with an answer takes them. True when a ROS form is offered."""
        if not self.ros_forms:
            return False
        if self._graph is None:  # made at the first need: a stream of ours that answers never needs it
            self._graph = self._session.ros.graph
        return self._choose_ros(foreign_only=True, why="only a ROS publisher offers it")

    def _check_not_below_a_stream(self):
        """While no publisher answers the stream: StreamGivenAsCamera when the name before its last part answers as a
        stream (a node given a stream as its camera reads ``<stream>/color``, which nobody publishes). Else the
        subscription waits: the camera may not have started yet."""
        stream = self.base.rpartition("/")[0]
        if stream and query_info(self._session, stream):
            raise StreamGivenAsCamera(stream_given_as_camera(self.base))

    def _own_uuid(self) -> str:
        """The UUID of the GPU this subscriber works on (empty without one)."""
        return device_uuid(self.device)

    def _local_choice(self, records: list[dict]) -> tuple[str, str]:
        """(transport, why) for a publisher on this host: ``local`` shm; cuda (its CUDA ring where it offers one,
        else shm); auto (its CUDA ring if that is on this subscriber's GPU, ``device``, by the GPU's UUID, which
        CUDA_VISIBLE_DEVICES does not renumber, or on another GPU that this process sees: the frames are then copied
        from GPU to GPU onto ``device``; else shm; without a device, shm)."""
        cuda = all("cuda" in r.get("transports", ()) for r in records)
        if self.local == "shm":
            return "shm", "asked for"
        if self.local == "cuda":
            return ("cuda", "asked for") if cuda else ("shm", "publisher offers no CUDA ring")
        if self._on_cpu:  # no device given: frames for the CPU, whatever the publisher offers
            return "shm", "this subscriber takes frames as arrays in host memory"
        if not cuda:
            return "shm", "publisher offers no CUDA ring"
        mine = self._own_uuid()
        if not mine:
            return "shm", "no CUDA device here"
        rings = {r.get("cuda_device_uuid") for r in records}
        if rings == {mine}:
            return "cuda", "on this subscriber's GPU"
        where = ", ".join(str(u)[:8] for u in sorted(rings, key=str))
        # a ring on another GPU of this host: its frames are copied from GPU to GPU, if this process sees that GPU
        if all(u and _core.cuda_device_ordinal(u) >= 0 for u in rings):
            return "cuda", f"on GPU {where}, copied to this subscriber's GPU"
        return "shm", f"the CUDA ring is on GPU {where}, which this process does not see"

    # -------------------------------------------------------------------------------------------- reading

    @property
    def received(self) -> int:
        return self._latest.received

    def describe(self) -> str:
        """The subscription's state in a few words, for a node's log: what it waits for, or how it reads."""
        if self._error is not None:
            return f"failed: {self._error}"
        if self.transport is None:
            if self.ros_topic:
                return f"no ROS publisher of {self.base} yet"
            return f"no publisher answers {topics.info(self.base)} yet"
        n = self.received
        limit = f", at most {self.max_rate.hz:g} Hz ({self.skipped_max_hz} skipped)" if self.max_rate else ""
        how = self.transport_reason or self.transport
        return (f"{how}, {n} frames since it was chosen" if n else f"{how}, no frame yet") + limit

    @property
    def overwritten(self) -> int:
        """Frames replaced by a newer one before ``latest`` took them; none with ``keep_recent`` (a partner's
        frames stay available to ``at`` until they leave the kept window)."""
        return self._latest.overwritten

    def _take(self, consume: bool, timeout: float | None):
        """(payload, meta, key, receive_ns, transport) of the newest sample, its type checked; None if there is
        none."""
        start = time.monotonic()
        if self._error is not None:
            raise self._error
        self._check_ros()
        if not self._chosen.wait(timeout if timeout is not None else 0.0):
            return None
        if self._error is not None:
            raise self._error
        while True:
            remaining = None if timeout is None else max(0.0, timeout - (time.monotonic() - start))
            item = self._latest.take(consume, remaining)
            if item is None:
                return None
            payload, attachment, key, receive_ns, encoding_of_sample, transport = item
            _check_encoding(key, encoding_of_sample, self.TYPES[transport])
            if consume and self.max_rate is not None and not self.max_rate.admit(_stamp_of(payload)):
                self.skipped_max_hz += 1  # not decoded, its ring not read: wait for the next within the timeout
                continue
            return payload, parse_attachment(attachment), key, receive_ns, transport

    def _moved(self, message, transport: str) -> bool:
        """True if the ring descriptor ``message`` is of another host: with auto, choose again; else raise."""
        if message.host == self._host:
            return False
        if self.requested == AUTO:  # cannot be by the rings' keys (they name the host): choose again
            log.info(f"{self.base}: a ring descriptor of host {str(message.host)[:8]}: choosing the transport again")
            self._recheck.set()
            return True
        raise RuntimeError(f"{self.base}: the data is in a ring on host {message.host}, this is {self._host}; the "
                           f"{transport} transport works on one host only (use auto or {' or '.join(self.REMOTE)})")

    def _open(self, name: str, transport: str):
        """The ring a descriptor names, opened once; None if it cannot be opened: a descriptor that came before its
        publisher stopped and is read after it removed its ring (a camera's restart), its frame counted as stale and
        the ring logged once."""
        opened = self._rings.get(transport)
        if opened is None or opened[0] != name:  # first time, or the publisher was restarted with a new ring
            try:
                # a CUDA ring's copies arrive on this subscriber's GPU, from another GPU of the host too
                ring = (_core.HostRingReader(name) if transport == "shm"
                        else _core.CudaRingReader(name, -1 if self.device is None else self.device))
            except _core.RingError as error:
                if name not in self._gone:
                    self._gone.add(name)
                    log.warning(f"{self.base}: a frame in a ring that is gone (its publisher stopped?), skipped: "
                                f"{error}")
                return None
            opened = self._rings[transport] = (name, ring)
        return opened[1]

    def _read_ring(self, message, transport: str, shape: tuple, dtype: np.dtype, nbytes: int):
        """(array, valid) of the data a ring descriptor names: a copy, or with ``copy=False`` a view and the callable
        telling whether it still holds; None if it was overwritten before it could be read."""
        if transport == "shm":
            ring = self._open(message.segment, transport)
            if ring is None:
                return None
            if self.copy:
                out = np.empty(shape, dtype)
                return (out, None) if ring.read(message.slot, message.seq, out) else None
            if not ring.acquire(message.slot, message.seq):
                return None
            view = ring.view(message.slot, nbytes).view(dtype).reshape(shape)
            return view, (lambda r=ring, s=message.slot, q=message.seq: r.validate(s, q))
        ring = self._open(message.endpoint, transport)
        if ring is None:
            return None
        if self.copy or ring.device != ring.ring_device:  # a view of a ring on another GPU would be on that GPU
            buffer = ring.copy(message.slot, message.seq)
            if buffer is None:
                return None
            # the reader's own copy: writable (PyTorch refuses read-only device arrays, so it takes copies only)
            return CudaImage(buffer.ptr, shape, dtype, ring.device, owner=buffer, readonly=False), None
        if not ring.acquire(message.slot, message.seq):
            return None
        ring.wait(message.slot, 0)  # the legacy default stream waits for the slot's copy; blocking streams follow it
        array = CudaImage(ring.slot_ptr(message.slot), shape, dtype, ring.ring_device, owner=ring)
        return array, (lambda r=ring, s=message.slot, q=message.seq: r.validate(s, q))

    def close(self):
        forget_at_exit(self)
        self._closed.set()
        self._recheck.set()
        if self._chooser is not None:
            self._chooser.join()  # it lets go of a subscriber it was handing over from
        if self._want is not None:
            self._want.undeclare()
            self._want = None
        if self._alive is not None:
            self._alive.undeclare()
        if self._graph is not None:
            self._graph.unlisten(self._recheck.set)
        with self._lock:
            if self._subscriber is not None:
                _undeclare(self._subscriber)
                self._subscriber = None
        self._rings = {}


class ImageSubscriber(Subscription):
    """The newest frame of one camera stream. ``transport``:

    * ``"auto"``: ``local`` when the publisher is on this host (``"auto"``, the default: its CUDA ring if this process sees
      the ring's GPU, the frames on ``device``, the GPU this subscriber works on, an ordinal or "cuda:N", copied from
      GPU to GPU if the ring is on another one; None, the default, or "cpu": the CPU, so its
      host ring; else its host ring; ``"shm"``;
      ``"cuda"``: its CUDA ring where it offers one, the frames on ``device`` if one is given), else ``remote``
      (``"compressed"``, the default, or ``"raw"``). Chosen from the publisher's info record when it is first found
      (``transport`` says which, None until then) and again when the publisher shows up on another host.
    * ``"compressed"`` (JPEG, PNG or RVL), ``"raw"`` (sensor_msgs/Image), ``"shm"`` (host ring, same host only) or ``"cuda"``
      (CUDA ring, same host only; frames are gpu.CudaImage) to choose it yourself.
    * ``"ros"``: the ROS 2 topic ``/<base>/image_raw`` (sensor_msgs/Image) of a ROS publisher, for a session in ROS 2;
      with ``camera_info`` its ``/<base>/camera_info``. Its frames carry no seq, source or publish time.

    A ``base`` with a leading ``/`` is a ROS 2 image topic of any name, as ROS publishes it (``/image_raw``,
    ``/resize/image``; Subscription): ``auto`` reads the topic, else image_transport's compressed topic
    ``<topic>/compressed``, else its compressedDepth topic ``<topic>/compressedDepth`` (16-bit or 32FC1 depth,
    codecs.compressed_depth), the first a publisher offers; ``ros`` or ``raw`` the topic, ``compressed`` the
    compressed one. Its camera_info is ROS's convention, ``camera_info`` beside the image topic (``/resize/image`` ->
    ``/resize/camera_info``), or the topic ``camera_info`` names. With a ``device``, the frames that a ROS publisher has
    as CUDA buffers on that GPU arrive there as gpu.CudaImage (ros_cuda.GpuReceiver;
    ``ros_gpu_stats``), the others as arrays in host memory.

    With ``copy=False`` the shm and cuda transports hand out views into the ring instead of copies: check
    ``Frame.is_valid()`` after using one. With ``camera_info=True`` the stream's CameraInfo is received as well
    (``camera_info``: the newest one); a ROS topic (``/cam/info``) as ``camera_info`` reads that one while a ROS form
    is read. ``max_hz``: at most so many frames a second, the newest (Subscription)."""

    KIND = "image"
    TRANSPORTS = TRANSPORTS
    TYPES = _TYPES
    REMOTE = REMOTE

    def __init__(self, session: zenoh.Session, base: str, transport: str = "compressed",
                 camera_info: bool | str = False, copy: bool = True, remote: str = "compressed", local: str = "auto",
                 max_hz: float | None = None, device=None):
        if isinstance(camera_info, str):
            ros.check_ros_name(camera_info, "camera_info topic")
            if not (ros.is_topic(base) or transport in (ROS, AUTO)):
                raise ValueError(f"camera_info {camera_info}: a ROS topic, for a ROS form (a base with a leading /, "
                                 f"transport ros or auto), not transport {transport}")
        self._info_latest = _Latest() if camera_info else None
        self._info_named = camera_info if isinstance(camera_info, str) else None
        self._info_lock = threading.Lock()
        self._info_subscriber = None  # our key's
        self._ros_info = None  # a ROS form's, while one is read (_used)
        self._info_hinted = False
        super().__init__(session, base, transport, local, remote, copy, max_hz, device)
        if camera_info and not self.ros_topic and transport != ROS:
            self._info_subscriber = session.declare_subscriber(topics.camera_info(self.base), self._info_latest.put)

    def _forms(self) -> dict:
        image = self.base if self.ros_topic else ros.topic_of(topics.image_raw(self.base))
        return {ROS: (image, Image), ROS_COMPRESSED: (ros.compressed_topic(image), CompressedImage),
                ROS_COMPRESSED_DEPTH: (ros.compressed_depth_topic(image), CompressedImage)}

    def info_topic(self) -> str | None:
        """The ROS topic of the camera_info read with a ROS form: the one named, else image_transport's beside the
        image topic; None without camera_info or a ROS form."""
        if self._info_latest is None or not self.ros_forms:
            return None
        return self._info_named or ros.camera_info_topic(self.ros_forms[ROS][0])

    def _used(self, transport: str):
        """A ROS form taken: its camera_info read from ROS too (``info_topic``); our form taken: no longer."""
        if self._info_latest is None:
            return
        with self._info_lock:
            if transport in ROS_FORMS and self._ros_info is None:
                topic = self.info_topic()
                self._ros_info = self._session.ros.node.subscriber(
                    topic, CameraInfo.TYPE, lambda payload, *_: self._info_latest.offer(
                        (payload, None, topic, time.time_ns(), CameraInfo.ENCODING, ROS)))
            elif transport not in ROS_FORMS and self._ros_info is not None:
                self._ros_info.close()
                self._ros_info = None

    def latest(self, consume: bool = True, timeout: float | None = None) -> Frame | None:
        """The newest frame; with ``consume`` each frame is returned once (None until the next arrives). With a
        ``timeout`` (seconds) wait that long for a new frame. None as well for a frame that was overwritten in its
        ring before it could be read (counted in ``stale``). Raises ProtocolError for a publisher of another
        protocol version."""
        taken = self._take(consume, timeout)
        return None if taken is None else self._frame(*taken)

    def at(self, stamp_ns: int, max_offset_ns: int = 0) -> Frame | None:
        """The kept frame (keep_recent) whose stamp is nearest to ``stamp_ns``, at most ``max_offset_ns`` away,
        decoded now (the others never are); None if there is none (yet), or its ring slot was overwritten."""
        taken = self._recent_at(stamp_ns, max_offset_ns)
        return None if taken is None else self._frame(*taken)

    def _frame(self, payload, meta, key, receive_ns, transport) -> Frame | None:
        valid = None
        if transport in ("compressed", ROS_COMPRESSED, ROS_COMPRESSED_DEPTH):
            message = CompressedImage.deserialize(payload)
            image, encoding = decompress(message)
        elif transport == ROS and getattr(payload, "parsed", None) is not None:  # a CUDA buffer (ros_cuda.Received)
            got = self._from_ros_gpu(payload)
            if got is None:
                self.stale += 1
                return None
            message, image, encoding = got
        elif transport in ("raw", ROS):
            message = Image.deserialize(payload)
            image, encoding = image_to_numpy(message), message.encoding
        else:
            message = self.TYPES[transport].deserialize(payload)
            encoding = message.encoding
            if self._moved(message, transport):
                return None
            got = self._from_ring(message, transport)
            if got is None:
                self.stale += 1
                return None
            image, valid = got
        self.decoded += 1
        return Frame(image=image, encoding=encoding, header=message.header, key=key, receive_ns=receive_ns,
                     seq=meta.get("seq"), source=meta.get("source", ""), publish_ns=meta.get("publish_ns"),
                     meta=meta, valid=valid)

    def _from_ros_gpu(self, received):
        """(message with its header, CudaImage on this subscriber's GPU, encoding) of a ROS publisher's CUDA buffer
        (ros_cuda.Received); None when it is stale."""
        p = received.parsed
        if p["is_bigendian"]:
            raise RuntimeError(f"{self.base}: big-endian images on the GPU are not supported")
        enc = info(p["encoding"])
        shape = (p["height"], p["width"]) if enc.channels == 1 else (p["height"], p["width"], enc.channels)
        row = p["width"] * enc.channels * enc.dtype.itemsize
        if p["step"] != row:
            raise RuntimeError(f"{self.base}: GPU frames of ROS publishers need rows without padding: step "
                               f"{p['step']}, row {row}")
        if p["descriptor"]["size"] != row * p["height"]:
            raise RuntimeError(f"{self.base}: a CUDA buffer of {p['descriptor']['size']} bytes for a "
                               f"{p['width']}x{p['height']} {p['encoding']} image of {row * p['height']} bytes")
        buffer = received.receiver.take(received)
        if buffer is None:
            return None
        header = Header(stamp=Time(sec=p["sec"], nanosec=p["nanosec"]), frame_id=p["frame_id"])
        # the reader's own copy, as a CUDA ring reader's: writable
        image = CudaImage(buffer.ptr, shape, enc.dtype, buffer.device, owner=buffer, readonly=False)
        return Image(header=header), image, p["encoding"]

    def _from_ring(self, m: ShmImage | CudaImageMsg, transport: str):
        if m.is_bigendian:
            raise RuntimeError("big-endian frames in a ring are not supported")
        enc = info(m.encoding)
        shape = (m.height, m.width) if enc.channels == 1 else (m.height, m.width, enc.channels)
        row = m.width * enc.channels * enc.dtype.itemsize
        if m.step != row:
            raise RuntimeError(f"ring frames have rows without padding: step {m.step}, row {row}")
        return self._read_ring(m, transport, shape, enc.dtype, m.step * m.height)

    @property
    def camera_info(self) -> CameraInfo | None:
        if self._info_latest is None:
            raise RuntimeError("this subscriber was created without camera_info=True")
        item = self._info_latest.take(consume=False, timeout=None)
        if item is None:
            self._hint_info()
            return None
        _check_encoding(item[2], item[4], CameraInfo)
        return CameraInfo.deserialize(item[0])

    def _hint_info(self):
        """Once, while a ROS form is read and frames came for WAIT_HINT_S but no camera_info: one line naming the
        CameraInfo topics in ROS's graph (none is taken in its place)."""
        if (self._info_hinted or self._ros_info is None or not self.received
                or time.monotonic() - self._started < WAIT_HINT_S):
            return
        self._info_hinted = True
        topic = self.info_topic()
        others = sorted({t.topic for t in self._graph.publishers() if t.type_name == ros.CAMERA_INFO_TYPE} - {topic})
        log.warning(f"{self.base}: frames, but no CameraInfo on {topic}"
                    + ("" if self._info_named else " (ROS's convention: camera_info beside the image topic)")
                    + (f"; CameraInfo topics in ROS's graph: {', '.join(others)}; name one (camera_info=TOPIC)"
                       if others else "; ROS's graph has no other CameraInfo topic"))

    def close(self):
        super().close()
        if self._info_subscriber is not None:
            self._info_subscriber.undeclare()
            self._info_subscriber = None
        with self._info_lock:
            if self._ros_info is not None:
                self._ros_info.close()
                self._ros_info = None
