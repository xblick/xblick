"""Reading ROS 2 cameras as ROS publishes them, without a ROS install: an image topic of any name with its camera_info
by ROS's convention or named, image_transport's compressed topic, the errors for a name that is no topic and for a
topic of another type. A stand-in for a ROS node is a session that takes part in ROS and declares no program of ours
(``declare=False``), publishing on ROS's keys only. tests/test_ros_live.py and tests/test_ros_lyrical.py do the same
against ROS 2's own publishers."""

import logging
import socket
import threading
import time

import numpy as np
import pytest

from xblick import ros, session
from xblick.camera import ROS, ROS_COMPRESSED, ROS_COMPRESSED_DEPTH, ImageSubscriber
from xblick.codecs import compress
from xblick.encodings import numpy_to_image
from xblick.msg import (
    CameraInfo,
    CompressedImage,
    Header,
    Image,
    PointCloud2,
    PointField,
    Time,
)
from xblick.points import PointCloudSubscriber

DOMAIN = 93


def free_port():
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def ros_compressed_depth(depth: np.ndarray, header: Header) -> CompressedImage:
    """A 16UC1 image as ROS's compressedDepth plugin writes it with its format png: its 12-byte ConfigHeader (zeros
    for 16-bit), then the PNG."""
    import io

    from PIL import Image as PILImage

    buffer = io.BytesIO()
    PILImage.fromarray(depth).save(buffer, format="PNG")
    return CompressedImage(header=header, format="16UC1; compressedDepth png", data=bytes(12) + buffer.getvalue())


class RosCamera:
    """A stand-in for a ROS camera driver: ROS publishers of the given (topic, type) pairs in a session that declares
    no program of ours, publishing a 4 x 6 mono8 image (or its PNG; on a ``.../compressedDepth`` topic a 16UC1 image
    as ROS's compressedDepth plugin writes it) and a CameraInfo every 50 ms until ``close``. A (topic, type, "16UC1")
    triple publishes the 16UC1 depth image instead: on an Image topic as it is, on a ``.../compressed`` topic as
    image_transport's compressed plugin turns depth into 8-bit JPEG (here an 8-bit PNG of the clamped values)."""

    def __init__(self, endpoint: str, domain: int = DOMAIN, **topics_):
        self.session = session.open_session(connect=[endpoint], declare=False,
                                            ros_options=session.RosOptions(domain, "ros_camera"))
        self.topics = {name: spec[:2] for name, spec in topics_.items()}
        self.depth_topics = {name for name, spec in topics_.items() if spec[2:] == ("16UC1",)}
        self.publishers = {name: self.session.ros.node.publisher(topic, kind)
                           for name, (topic, kind) in self.topics.items()}
        self.image = np.arange(24, dtype=np.uint8).reshape(4, 6)
        self.depth = (np.arange(24, dtype=np.uint16).reshape(4, 6) * 1000 + 7).astype(np.uint16)
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self):
        while not self._stop.wait(0.05):
            header = Header(Time.from_ns(time.time_ns()), "ros_camera_optical")
            for name, publisher in self.publishers.items():
                if publisher.type_name == Image.TYPE and name in self.depth_topics:
                    payload = numpy_to_image(self.depth, "16UC1", header).serialize()
                elif publisher.type_name == Image.TYPE:
                    payload = numpy_to_image(self.image, "mono8", header).serialize()
                elif publisher.type_name == ros.COMPRESSED_TYPE and self.topics[name][0].endswith(
                        "/" + ros.COMPRESSED_DEPTH):
                    payload = ros_compressed_depth(self.depth, header).serialize()
                elif publisher.type_name == ros.COMPRESSED_TYPE and name in self.depth_topics:
                    clamped = compress(np.clip(self.depth, 0, 255).astype(np.uint8), "mono8", "png", header=header)
                    payload = CompressedImage(header, "16UC1; jpeg compressed mono8", clamped.data).serialize()
                elif publisher.type_name == ros.COMPRESSED_TYPE:
                    payload = compress(self.image, "mono8", "png", header=header).serialize()
                elif publisher.type_name == CameraInfo.TYPE:
                    payload = CameraInfo(header, height=4, width=6, distortion_model=name).serialize()
                else:
                    fields = [PointField(n, 4 * i, PointField.FLOAT32) for i, n in enumerate("xyz")]
                    payload = PointCloud2(header, height=1, width=2, fields=fields, point_step=12, row_step=24,
                                          data=np.zeros(6, "<f4").tobytes(), is_dense=True).serialize()
                publisher.put(bytes(payload))

    def close(self):
        self._stop.set()
        self._thread.join()
        self.session.close()


@pytest.fixture
def network():
    """(endpoint, reader): a session in ROS 2 that listens, for RosCamera sessions to connect to."""
    endpoint = f"tcp/127.0.0.1:{free_port()}"
    reader = session.open_session(listen=[endpoint], ros_options=session.RosOptions(DOMAIN, "xblick_reader"))
    yield endpoint, reader
    reader.close()


def first_frame(subscriber, timeout=5.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        frame = subscriber.latest(timeout=0.1)
        if frame is not None:
            return frame
    return None


def test_image_transports_topic_names():
    assert ros.camera_info_topic("/cam/image_raw") == "/cam/camera_info"
    assert ros.camera_info_topic("/image_raw") == "/camera_info"
    assert ros.camera_info_topic("/resize/image") == "/resize/camera_info"
    assert ros.compressed_topic("/cam/image_raw") == "/cam/image_raw/compressed"
    assert ros.is_topic("/image_raw") and not ros.is_topic("test/camera/color")


def test_an_image_topic_of_any_name_with_its_camera_info_beside_it(network):
    """A root /image_raw and an Isaac-style /resize/image, each with camera_info beside it (ROS's convention)."""
    endpoint, reader = network
    camera = RosCamera(endpoint, root=("/image_raw", Image.TYPE), root_info=("/camera_info", CameraInfo.TYPE),
                       resize=("/resize/image", Image.TYPE), resize_info=("/resize/camera_info", CameraInfo.TYPE))
    root = ImageSubscriber(reader, "/image_raw", transport="auto", camera_info=True)
    resize = ImageSubscriber(reader, "/resize/image", transport="ros", camera_info=True)
    try:
        for subscriber, info in ((root, "root_info"), (resize, "resize_info")):
            frame = first_frame(subscriber)
            assert frame is not None and (frame.image == camera.image).all() and frame.seq is None
            assert subscriber.transport == ROS
            end = time.monotonic() + 3.0
            while subscriber.camera_info is None and time.monotonic() < end:
                time.sleep(0.05)
            assert subscriber.camera_info.distortion_model == info
    finally:
        root.close()
        resize.close()
        camera.close()


def test_a_camera_info_topic_named_explicitly(network):
    endpoint, reader = network
    camera = RosCamera(endpoint, image=("/lab/eye/image_rect", Image.TYPE),
                       info=("/lab/eye/calibration", CameraInfo.TYPE))
    subscriber = ImageSubscriber(reader, "/lab/eye/image_rect", transport="raw", camera_info="/lab/eye/calibration")
    try:
        assert first_frame(subscriber) is not None
        end = time.monotonic() + 3.0
        while subscriber.camera_info is None and time.monotonic() < end:
            time.sleep(0.05)
        assert subscriber.camera_info.distortion_model == "info"
        with pytest.raises(ValueError, match="a ROS topic, for a ROS form"):
            ImageSubscriber(reader, "test/camera/color", transport="raw", camera_info="/lab/eye/calibration")
    finally:
        subscriber.close()
        camera.close()


def test_compressed_by_image_transports_convention(network):
    """transport compressed reads <topic>/compressed; auto takes it where a publisher offers only that."""
    endpoint, reader = network
    camera = RosCamera(endpoint, compressed=("/bag/cam/image_raw/compressed", ros.COMPRESSED_TYPE))
    asked = ImageSubscriber(reader, "/bag/cam/image_raw", transport="compressed")
    auto = ImageSubscriber(reader, "/bag/cam/image_raw", transport="auto")
    try:
        for subscriber in (asked, auto):
            frame = first_frame(subscriber)
            assert frame is not None and (frame.image == camera.image).all()
            assert subscriber.transport == ROS_COMPRESSED
    finally:
        asked.close()
        auto.close()
        camera.close()


def test_auto_takes_the_topic_before_its_compressed_one(network):
    endpoint, reader = network
    camera = RosCamera(endpoint, raw=("/both/image_raw", Image.TYPE),
                       compressed=("/both/image_raw/compressed", ros.COMPRESSED_TYPE))
    subscriber = ImageSubscriber(reader, "/both/image_raw", transport="auto")
    try:
        assert first_frame(subscriber) is not None and subscriber.transport == ROS
    finally:
        subscriber.close()
        camera.close()


def test_a_point_cloud_topic_of_any_name(network):
    endpoint, reader = network
    camera = RosCamera(endpoint, cloud=("/velodyne_points", PointCloud2.TYPE))
    subscriber = PointCloudSubscriber(reader, "/velodyne_points")
    try:
        assert first_frame(subscriber) is not None and subscriber.transport == ROS
        with pytest.raises(ValueError, match="transport auto, ros, raw, not compressed"):
            PointCloudSubscriber(reader, "/velodyne_points", transport="compressed")
    finally:
        subscriber.close()
        camera.close()


def test_a_name_that_is_no_topic_lists_the_topics_below_it(network, monkeypatch):
    """No guessing: a namespace with several image topics is an error that names them."""
    monkeypatch.setattr("xblick.camera.WAIT_HINT_S", 1.0)
    endpoint, reader = network
    camera = RosCamera(endpoint, color=("/rs/color/image_raw", Image.TYPE),
                       depth=("/rs/depth/image_rect_raw", Image.TYPE))
    subscriber = ImageSubscriber(reader, "/rs", transport="auto")
    try:
        time.sleep(1.2)
        with pytest.raises(ValueError, match="/rs/color/image_raw, /rs/depth/image_rect_raw; name one of them"):
            subscriber.latest(timeout=0.1)
    finally:
        subscriber.close()
        camera.close()


def test_a_topic_of_another_type_is_an_error(network, monkeypatch):
    """image_transport's compressed topic given as the image topic: the error says which topic and transport fit."""
    monkeypatch.setattr("xblick.camera.WAIT_HINT_S", 1.0)
    endpoint, reader = network
    camera = RosCamera(endpoint, compressed=("/cam/image_raw/compressed", ros.COMPRESSED_TYPE))
    subscriber = ImageSubscriber(reader, "/cam/image_raw/compressed", transport="ros")
    try:
        time.sleep(1.2)
        with pytest.raises(RuntimeError, match="compressed topic of /cam/image_raw, which transport compressed"):
            subscriber.latest(timeout=0.1)
    finally:
        subscriber.close()
        camera.close()


def test_frames_without_camera_info_say_which_camera_info_topics_there_are(network, monkeypatch, caplog):
    monkeypatch.setattr("xblick.camera.WAIT_HINT_S", 0.5)
    endpoint, reader = network
    camera = RosCamera(endpoint, image=("/cam/image_raw", Image.TYPE), info=("/cam/info_rect", CameraInfo.TYPE))
    subscriber = ImageSubscriber(reader, "/cam/image_raw", transport="ros", camera_info=True)
    try:
        assert first_frame(subscriber) is not None
        time.sleep(0.6)
        with caplog.at_level(logging.WARNING, logger="xblick"):
            assert subscriber.camera_info is None
        assert any("no CameraInfo on /cam/camera_info" in r.getMessage() and "/cam/info_rect" in r.getMessage()
                   for r in caplog.records)
    finally:
        subscriber.close()
        camera.close()


def test_a_ros_name_needs_a_session_in_ros():
    with session.open_session(local_router=False, ros_options=False) as s, \
            pytest.raises(ValueError, match="a ROS 2 topic"):
        ImageSubscriber(s, "/image_raw", transport="auto")
    with session.open_session(local_router=False, ros_options=session.RosOptions(DOMAIN, "x")) as s:
        with pytest.raises(ValueError, match="transport auto, ros, raw or compressed, not shm"):
            ImageSubscriber(s, "/image_raw", transport="shm")
        with pytest.raises(ValueError, match="no ROS name"):
            ImageSubscriber(s, "/cam-1/image_raw", transport="auto")


def test_auto_reads_a_stream_only_a_ros_publisher_offers_and_moves_to_ours(network):
    """auto on a stream of ours with only a ROS publisher (a ROS camera on /<base>/image_raw): its ROS form, with its
    camera_info; a publisher of ours that starts later takes over."""
    from xblick.camera import CameraPublisher

    endpoint, reader = network
    camera = RosCamera(endpoint, image=("/roscam/color/image_raw", Image.TYPE),
                       info=("/roscam/color/camera_info", CameraInfo.TYPE))
    subscriber = ImageSubscriber(reader, "roscam/color", transport="auto", camera_info=True)
    ours = None
    try:
        assert first_frame(subscriber) is not None and subscriber.transport == ROS
        assert "only a ROS publisher" in subscriber.transport_reason
        end = time.monotonic() + 3.0
        while subscriber.camera_info is None and time.monotonic() < end:
            time.sleep(0.05)
        assert subscriber.camera_info.distortion_model == "info"
        xblick = session.open_session(connect=[endpoint], ros_options=False)
        ours = (xblick, CameraPublisher(xblick, "roscam/color", encoding="mono8", frame_id="roscam"))
        end = time.monotonic() + 5.0
        while subscriber.transport == ROS and time.monotonic() < end:
            ours[1].publish(np.zeros((4, 6), np.uint8))
            subscriber.latest(timeout=0.05)
        assert subscriber.transport == "shm"
    finally:
        subscriber.close()
        if ours is not None:
            ours[1].close()
            ours[0].close()
        camera.close()


def test_auto_never_reads_the_ros_copy_of_a_stream_of_ours(network, monkeypatch):
    """A program of ours (it declares itself) whose stream is on ROS's keys only: auto does not take that copy. A
    camera of ours is read in its own form, as before."""
    from xblick.camera import CameraPublisher

    endpoint, reader = network
    xblick = session.open_session(connect=[endpoint], ros_options=session.RosOptions(DOMAIN, "xblick_camera"))
    copy = xblick.ros.node.publisher("/copy/color/image_raw", Image.TYPE)
    camera = CameraPublisher(xblick, "mine/color", encoding="mono8", frame_id="mine")
    copied = ImageSubscriber(reader, "copy/color", transport="auto")
    mine = ImageSubscriber(reader, "mine/color", transport="auto")
    try:
        end = time.monotonic() + 2.0
        while time.monotonic() < end:
            payload = numpy_to_image(np.zeros((4, 6), np.uint8), "mono8", Header(Time.from_ns(time.time_ns()))).serialize()
            copy.put(bytes(payload))
            camera.publish(np.zeros((4, 6), np.uint8))
            mine.latest(timeout=0.05)
        assert copied.latest(timeout=0.1) is None and copied.transport is None
        assert mine.transport == "shm"
    finally:
        copied.close()
        mine.close()
        camera.close()
        xblick.close()


def test_list_shows_ros_camera_topics_without_our_copies(network):
    from xblick.camera import CameraPublisher
    from xblick.cli import ros_camera_topics

    endpoint, reader = network
    camera = RosCamera(endpoint, image=("/usb_cam/image_raw", Image.TYPE), info=("/usb_cam/camera_info",
                                                                                 CameraInfo.TYPE))
    xblick = session.open_session(connect=[endpoint], ros_options=session.RosOptions(DOMAIN, "xblick_camera"))
    ours = CameraPublisher(xblick, "mine/color", encoding="mono8", frame_id="mine")
    try:
        end = time.monotonic() + 3.0
        found = []
        while len(found) < 2 and time.monotonic() < end:
            found = ros_camera_topics(reader)
            time.sleep(0.1)
        assert found == [("/usb_cam/camera_info", "CameraInfo", ["/ros_camera"]),
                         ("/usb_cam/image_raw", "Image", ["/ros_camera"])]
        assert ros_camera_topics(reader, "usb_cam/image_raw") == [("/usb_cam/image_raw", "Image", ["/ros_camera"])]
        assert ros_camera_topics(reader, "other") == []
    finally:
        ours.close()
        xblick.close()
        camera.close()


def test_rgbd_inputs_pair_a_ros_only_colour_and_depth_camera(network):
    """RGBDInputs reads with auto: the colour stream of ours that only a ROS node publishes, and a depth named as a ROS
    topic (CameraSpec's depth=/TOPIC), each with its camera_info, paired by stamp."""
    from xblick.rgbd import CameraSpec, RGBDInputs

    endpoint, reader = network
    camera = RosCamera(endpoint, color=("/rgbd/color/image_raw", Image.TYPE),
                       color_info=("/rgbd/color/camera_info", CameraInfo.TYPE),
                       depth=("/rgbd/aligned/image_rect", Image.TYPE),
                       depth_info=("/rgbd/aligned/camera_info", CameraInfo.TYPE))
    spec = CameraSpec.parse("rgbd:depth=/rgbd/aligned/image_rect")
    assert spec.keys == {"color": "rgbd/color", "depth": "/rgbd/aligned/image_rect"}
    with pytest.raises(ValueError, match="without a leading /"):
        CameraSpec.parse("/rgbd")
    inputs = RGBDInputs(reader, spec, 8, "auto", threading.Event())
    try:
        end = time.monotonic() + 8.0
        frames = None
        while frames is None and time.monotonic() < end:
            stamp = inputs.newest_complete()
            frames = inputs.take(stamp) if stamp is not None else None
            time.sleep(0.05)
        assert frames is not None, inputs.waiting()
        assert frames["color"].stamp_ns == frames["depth"].stamp_ns == stamp
        assert {r.subscriber.transport for r in inputs.readers.values()} == {ROS}
        assert inputs.readers["depth"].subscriber.camera_info.distortion_model == "depth_info"
        assert inputs.readers["color"].subscriber.camera_info.distortion_model == "color_info"
    finally:
        inputs.close()
        camera.close()


# ------------------------------------------------------------------------- compressedDepth, the tools (issue #39)


def test_compressed_depth_by_image_transports_convention(network, monkeypatch):
    """auto takes <topic>/compressedDepth where a publisher offers neither the topic nor its compressed one, and
    decodes ROS's header and PNG; transport ros on that topic names the transport that reads it."""
    monkeypatch.setattr("xblick.camera.WAIT_HINT_S", 1.0)
    endpoint, reader = network
    camera = RosCamera(endpoint, depth=("/cam/depth/image_rect_raw/compressedDepth", ros.COMPRESSED_TYPE))
    auto = ImageSubscriber(reader, "/cam/depth/image_rect_raw", transport="auto")
    wrong = ImageSubscriber(reader, "/cam/depth/image_rect_raw/compressedDepth", transport="ros")
    try:
        frame = first_frame(auto)
        assert frame is not None and frame.encoding == "16UC1" and np.array_equal(frame.image, camera.depth)
        assert auto.transport == ROS_COMPRESSED_DEPTH and ros.compressed_depth_topic("/a") == "/a/compressedDepth"
        time.sleep(1.2)
        with pytest.raises(RuntimeError, match="compressedDepth topic of /cam/depth/image_rect_raw, which auto reads"):
            wrong.latest(timeout=0.1)
    finally:
        auto.close()
        wrong.close()
        camera.close()


def test_the_graph_knows_a_program_of_ours_before_its_ros_tokens(network):
    """Graph.by_xblick from the declarations' subscriber: whenever a publisher of a program of ours is in the graph,
    the program is known as ours (no query that a timeout can end); a ROS node's is not."""
    endpoint, reader = network
    graph = reader.ros.graph
    camera = RosCamera(endpoint, image=("/foreign/image_raw", Image.TYPE))
    try:
        for k in range(8):
            ours = session.open_session(connect=[endpoint], ros_options=session.RosOptions(DOMAIN, f"ours_{k}"))
            publisher = ours.ros.node.publisher(f"/ours_{k}/image_raw", Image.TYPE)
            zid = str(ours.info.zid())
            end = time.monotonic() + 5.0
            while not graph.publishers(f"/ours_{k}/image_raw") and time.monotonic() < end:
                time.sleep(0.005)
            assert graph.publishers(f"/ours_{k}/image_raw") and graph.by_xblick(zid)
            publisher.close()
            ours.close()
        foreign = graph.publishers("/foreign/image_raw")
        assert foreign and not graph.by_xblick(foreign[0].zid)
    finally:
        camera.close()


def wait_for(get, expected, timeout=5.0):
    end = time.monotonic() + timeout
    got = get()
    while got != expected and time.monotonic() < end:
        time.sleep(0.05)
        got = get()
    return got


def test_ros_cameras_named_as_their_readers_take_them(network):
    """ros.camera_streams: an Image topic as it is; the image topic of a compressed or compressedDepth topic alone;
    PointCloud2 topics; not a CompressedImage topic of another name, not the ROS copies of our programs."""
    from xblick.camera import CameraPublisher

    endpoint, reader = network
    camera = RosCamera(endpoint, raw=("/a/image_raw", Image.TYPE),
                       compressed=("/b/image_raw/compressed", ros.COMPRESSED_TYPE),
                       depth=("/c/depth/compressedDepth", ros.COMPRESSED_TYPE),
                       cloud=("/points", PointCloud2.TYPE), odd=("/odd", ros.COMPRESSED_TYPE),
                       info=("/a/camera_info", CameraInfo.TYPE))
    xblick = session.open_session(connect=[endpoint], ros_options=session.RosOptions(DOMAIN, "xblick_camera"))
    ours = CameraPublisher(xblick, "mine/color", encoding="mono8", frame_id="mine")
    try:
        expected = {"images": ["/a/image_raw", "/b/image_raw", "/c/depth"], "clouds": ["/points"]}
        assert wait_for(lambda: ros.camera_streams(reader.ros.graph), expected) == expected
        assert ros.camera_streams(reader.ros.graph, "c") == {"images": ["/c/depth"], "clouds": []}
    finally:
        ours.close()
        xblick.close()
        camera.close()


def test_xblick_rerun_bridges_a_ros_camera(network):
    """A ROS camera's image topic in the bridge: decoded frames at the topic's entity, its camera_info as the pinhole,
    no transforms; in discovery it comes with what the info records found, marked ROS in the lines."""
    from test_rerun_bridge import FakeRerun, of

    from xblick.cli import _marked, _with_ros
    from xblick.rerun_bridge import RerunBridge

    endpoint, reader = network
    camera = RosCamera(endpoint, image=("/rgbd/color/image_raw", Image.TYPE),
                       info=("/rgbd/color/camera_info", CameraInfo.TYPE),
                       depth=("/rgbd/depth/image_raw/compressedDepth", ros.COMPRESSED_TYPE))
    empty = {"bases": [], "clouds": [], "detections": [], "segmentations": []}
    found = wait_for(lambda: _with_ros(empty, reader.ros.graph, "")["bases"],
                     ["/rgbd/color/image_raw", "/rgbd/depth/image_raw"])
    assert found == ["/rgbd/color/image_raw", "/rgbd/depth/image_raw"]
    assert _marked(["a/b", "/c/d"]) == "a/b, /c/d (ROS)"
    rr = FakeRerun()
    bridge = RerunBridge(reader, rr, found)
    try:
        end = time.monotonic() + 8.0
        while not (of(rr, "Image") and of(rr, "DepthImage") and of(rr, "Pinhole")) and time.monotonic() < end:
            time.sleep(0.05)
        bridge.check()
        entity, _, fields, _ = of(rr, "Image")[0]
        assert entity == "/rgbd/color/image_raw" and fields["color_model"] == "L"
        assert np.array_equal(fields["image"], camera.image)
        entity, _, fields, _ = of(rr, "DepthImage")[0]
        assert entity == "/rgbd/depth/image_raw" and fields["meter"] == 1000.0
        assert np.array_equal(fields["image"], camera.depth)
        assert of(rr, "Pinhole")[0][0] == "/rgbd/color/image_raw" and bridge.cameras == []
        assert not of(rr, "Transform3D")
    finally:
        bridge.close()
        camera.close()


def test_a_ros_camera_is_recorded_under_its_topics_and_plays_as_a_stream(network, tmp_path, capsys):
    """xblick record in ROS 2: a ROS camera's topics under their ROS names, as ros2 bag record writes them (the form
    that keeps the encoding: compressed for mono8, compressedDepth for 16UC1, never the 8-bit compressed form of depth
    image_transport offers as well; its camera_info), the choice in its line; xblick play makes streams of ours of
    image_transport's names, a compressedDepth camera's frames decoded."""
    from test_recording import mcap_reader

    from xblick import recording

    endpoint, reader = network
    camera = RosCamera(endpoint, image=("/roscam/color/image_raw", Image.TYPE),
                       compressed=("/roscam/color/image_raw/compressed", ros.COMPRESSED_TYPE),
                       info=("/roscam/color/camera_info", CameraInfo.TYPE),
                       depth_raw=("/roscam/depth/image_raw", Image.TYPE, "16UC1"),
                       depth_jpeg=("/roscam/depth/image_raw/compressed", ros.COMPRESSED_TYPE, "16UC1"),
                       depth=("/roscam/depth/image_raw/compressedDepth", ros.COMPRESSED_TYPE),
                       depth_info=("/roscam/depth/camera_info", CameraInfo.TYPE))
    recorder = recording.Recorder(reader, tmp_path, look_every=0.5)
    try:
        recorder.record()
        end = time.monotonic() + 10.0
        while recorder.status()["messages"] < 40 and time.monotonic() < end:
            time.sleep(0.1)
        result = recorder.stop()
    finally:
        camera.close()
    out = capsys.readouterr().out
    assert ("recording the ROS camera /roscam/depth/image_raw as /roscam/depth/image_raw/compressedDepth "
            "(16UC1: its compressedDepth form)") in out, out
    assert "as /roscam/color/image_raw/compressed (mono8: its compressed form)" in out, out
    path = result["recording"]["files"][0]
    with open(path, "rb") as stream:
        channels = {c.topic: c.metadata for c in mcap_reader.make_reader(stream).get_summary().channels.values()}
    assert {"/roscam/color/image_raw/compressed", "/roscam/color/camera_info", "/roscam/depth/image_raw/compressedDepth",
            "/roscam/depth/camera_info"} <= set(channels), channels
    assert not {"/roscam/color/image_raw", "/roscam/depth/image_raw", "/roscam/depth/image_raw/compressed"} & set(
        channels)  # one form each
    assert all("zenoh_key" not in channels[t] for t in channels if t.startswith("/roscam/"))
    played = session.open_session(connect=[endpoint], ros_options=False)
    player = recording.Player(played, [path], loop=True)
    color = ImageSubscriber(reader, "roscam/color", transport="auto", camera_info=True)
    depth = ImageSubscriber(reader, "roscam/depth", transport="auto")
    thread = threading.Thread(target=player.run)
    thread.start()
    try:
        frame = first_frame(color, 8.0)
        assert frame is not None and np.array_equal(frame.image, camera.image) and color.transport == "shm"
        d = first_frame(depth, 8.0)
        assert d is not None and d.encoding == "16UC1" and np.array_equal(d.image, camera.depth)
        assert player.error is None
    finally:
        player.end()
        thread.join()
        for c in (color, depth, player, recorder):
            c.close()
        played.close()
