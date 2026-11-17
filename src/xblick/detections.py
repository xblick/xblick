"""Results of vision nodes over Durchblick: ROS's vision_msgs/Detection2DArray (objects in an image, boxes in pixels),
Detection3DArray (objects in 3D, box poses in a camera frame) and Classification (classes without a place, e.g. the
actions in a clip), each sample one message, on
``<camera>/<node>/<name>`` (topics.detections, protocol.hpp), stamped as the image it is about. The publisher answers
``<key>/info`` (kind ``detections``, the message type), so that ``xblick list`` and ``xblick rerun`` find it
as they find streams; ``xblick rerun`` draws them.

    boxes = DetectionPublisher(session, "test/camera/boxer/boxes_3d", Detection3DArray)
    boxes.publish(Detection3DArray(header=Header(Time.from_ns(image_stamp_ns), "AzureKinectCamera"), detections=[...]))

    reader = DetectionSubscriber(session, "test/camera/boxer/boxes_3d", callback)  # callback(message)
"""

from __future__ import annotations

import json
from typing import Callable

import zenoh

from . import _core, ros, topics
from .camera import INFO_VERSION, ProtocolError, _attachment, cdr_encoding, parse_attachment
from .msg import Classification, Detection2DArray, Detection3DArray

DETECTION_TYPES = (Detection2DArray, Detection3DArray, Classification)


class DetectionPublisher:
    """Publishes the messages of one type on one key; every message is sent (they are small). Answers ``<key>/info``
    until :meth:`close`. A session with ``--ros`` sends them to the ROS 2 topic ``/<key>`` too, while a ROS
    subscriber is there."""

    def __init__(self, session: zenoh.Session, key: str, message_type: type):
        if message_type not in DETECTION_TYPES:
            raise TypeError(f"{message_type.__name__}: one of " + ", ".join(t.__name__ for t in DETECTION_TYPES))
        self.key = topics.check_name(key)
        self.message_type = message_type
        self.seq = 0
        self._publisher = session.declare_publisher(self.key, encoding=cdr_encoding(message_type))
        self._info = json.dumps({"v": INFO_VERSION, "kind": "detections", "xblick": _core.__version__,
                                 "host": _core.host_id(), "type": message_type.TYPE}, separators=(",", ":"))
        self._info_queryable = session.declare_queryable(topics.info(self.key), self._answer_info)
        self._ros = ros.mirror(session, self.key, cdr_encoding(message_type))

    def _answer_info(self, query: zenoh.Query):
        query.reply(topics.info(self.key), self._info)

    def publish(self, message):
        if not isinstance(message, self.message_type):
            raise TypeError(f"{self.key} carries {self.message_type.__name__}, not {type(message).__name__}")
        self.seq += 1  # the attachment: publish_ns, so that the age at send of a result is known (stamps.py)
        payload = bytes(message.serialize())
        self._publisher.put(payload, attachment=_attachment(self.seq, ""))
        if self._ros is not None and self._ros.matched:
            self._ros.put(payload)

    def close(self):
        self._info_queryable.undeclare()
        self._publisher.undeclare()
        if self._ros is not None:
            self._ros.close()


class DetectionSubscriber:
    """Calls ``callback(message)`` with every message of DETECTION_TYPES on a key (the type from the sample's
    encoding), in Zenoh's thread; ``age_at_send_ns`` is the message's then (stamps.py; None from a publisher before
    0.3.30). A sample of another type or that does not decode stops the deliveries; :meth:`check` raises it.

    ``ros_type`` (one of DETECTION_TYPES, for a session with ``--ros``): reads the ROS 2 topic ``/<key>`` of that type
    instead of our key (ROS's key names the type); its messages carry no seq or publish time."""

    def __init__(self, session: zenoh.Session, key: str, callback: Callable, ros_type: type | None = None):
        self.key = topics.check_name(key)
        self._callback = callback
        self._error: BaseException | None = None
        self.received = 0
        self.age_at_send_ns: int | None = None
        self.seq: int | None = None  # the newest message's attachment seq
        if ros_type is not None:
            if ros_type not in DETECTION_TYPES:
                raise TypeError(f"{ros_type.__name__}: one of " + ", ".join(t.__name__ for t in DETECTION_TYPES))
            if getattr(session, "ros", None) is None:
                raise ValueError(f"{self.key}: reading its ROS 2 topic needs a session with --ros")
            self._subscriber = session.ros.node.subscriber(ros.topic_of(self.key), ros_type.TYPE,
                                                           lambda payload, *_: self._on_ros(ros_type, payload))
        else:
            self._subscriber = session.declare_subscriber(self.key, self._on_sample)

    def _on_ros(self, message_type: type, payload: bytes):
        if self._error is not None:
            return
        try:
            self._callback(message_type.deserialize(payload))
            self.received += 1
        except BaseException as error:  # noqa: BLE001 - raised in the caller's thread by check()
            self._error = error

    def _on_sample(self, sample: zenoh.Sample):
        if self._error is not None:
            return
        try:
            encoding = str(sample.encoding)
            message_type = next((t for t in DETECTION_TYPES if t.ENCODING == encoding), None)
            if message_type is None:
                raise ProtocolError(f"{self.key}: {encoding!r} is none of "
                                    + ", ".join(repr(t.ENCODING) for t in DETECTION_TYPES))
            message = message_type.deserialize(sample.payload.to_bytes())
            meta = parse_attachment(sample.attachment.to_bytes() if sample.attachment is not None else None)
            self.seq = meta.get("seq")
            self.age_at_send_ns = (meta["publish_ns"] - message.header.stamp.to_ns()
                                   if meta.get("publish_ns") is not None and message.header.stamp.to_ns() else None)
            self._callback(message)
            self.received += 1
        except BaseException as error:  # noqa: BLE001 - raised in the caller's thread by check()
            self._error = error

    def describe(self) -> str:
        """Its state in a few words, for a node's log."""
        if self._error is not None:
            return f"failed: {self._error}"
        return f"{self.received} messages" if self.received else "no message yet"

    def check(self):
        if self._error is not None:
            raise self._error

    def close(self):
        if isinstance(self._subscriber, ros.RosSubscriber):
            self._subscriber.close()
        else:
            self._subscriber.undeclare()
