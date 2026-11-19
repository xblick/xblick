"""Videos played by xblick play (video.py through recording.Recorder.play): two generated videos of different
frame rates (a colour one at 20 frames/s, a 16-bit depth one at 10 frames/s, 3 ms later), played at once: their size
and encoding, stamps from their PTS on one clock (equal PTS: stamps exactly the offset apart), sent at their stamps,
paired by PartnerHistory within a maximum offset, looped with the stamps growing by the longest video's length, the
send skew, the rate changed through the node's parameters, pause and resume; an image sequence; what is refused."""

import time
from pathlib import Path

import numpy as np
import pytest
from test_camera import free_port, sessions, stop_child, wait_until  # noqa: F401 - sessions is a fixture

from xblick import _core, recording
from xblick.camera import ImageSubscriber
from xblick.history import FrameHistory, PartnerHistory
from xblick.parameters import ParameterClient

av = pytest.importorskip("av")  # the extra xblick[video]

W, H = 64, 48
MS = 1_000_000


def write_video(path, fps, n, depth=False):
    """``n`` frames at ``fps``: colour frame i has red i*10, depth frame i the value 1000 + i (mm)."""
    with av.open(str(path), "w") as out:
        s = out.add_stream("ffv1" if depth else "libx264", rate=fps)
        s.width, s.height, s.pix_fmt = W, H, "gray16le" if depth else "yuv420p"
        for i in range(n):
            if depth:
                frame = av.VideoFrame.from_ndarray(np.full((H, W), 1000 + i, np.uint16), format="gray16le")
            else:
                image = np.zeros((H, W, 3), np.uint8)
                image[:, :, 0] = i * 10
                frame = av.VideoFrame.from_ndarray(image, format="rgb24")
            for packet in s.encode(frame):
                out.mux(packet)
        for packet in s.encode():
            out.mux(packet)
    return path


@pytest.fixture
def videos(tmp_path):
    return (write_video(tmp_path / "color.mp4", 20, 20), write_video(tmp_path / "depth.mkv", 10, 10, depth=True))


def test_two_videos_on_one_clock(sessions, videos, tmp_path):  # noqa: F811
    a, b = sessions
    color_mp4, depth_mkv = videos
    recorder = recording.Recorder(a, tmp_path, name="video_test")
    color = FrameHistory(ImageSubscriber(b, "t/cam/color", transport="raw"), 200)
    depth = PartnerHistory(ImageSubscriber(b, "t/cam/depth", transport="raw"), 200, max_offset_ns=5 * MS)
    try:
        player = recorder.play([color_mp4, depth_mkv], streams=["t/cam/color", "t/cam/depth"], offsets_ms=[0.0, 3.0],
                               loop=True)
        assert player.topics == ["t/cam/color", "t/cam/depth"]
        # two passes (1.0 s each: both videos are 1 s long) and a little
        wait_until(lambda: len(color.stamps()) >= 30, timeout=10.0)
        status = recorder.status()
        assert status["state"] == "playing" and status["play"]["kind"] == "video"
        stamps = sorted(color.stamps())
        steps = np.diff(stamps)
        # 50 ms apart, to rounding of the PTS (x264's time base); the pass's start 1.0 s after the first
        assert np.all(np.abs(steps - 50 * MS) < MS), steps
        assert stamps[20] - stamps[0] == pytest.approx(1000 * MS, abs=MS)  # looped: on by the length, growing
        newest = color.at(stamps[-1])
        assert newest.image.shape == (H, W, 3) and newest.encoding == "rgb8"
        assert newest.meta.get("source") == "video" and newest.age_at_send_ns is not None
        assert 0 <= newest.age_at_send_ns < 20 * MS  # sent at its stamp
        # every second colour frame has its depth frame 3 ms later, to the nanosecond of the offset
        wait_until(lambda: depth.partner_of(stamps[2]) is not None, timeout=5.0)
        for i in (0, 2, 4, 20, 22):
            partner = depth.partner_of(stamps[i])
            assert partner is not None and partner - stamps[i] == pytest.approx(3 * MS, abs=MS // 2)
            frame = depth.at(stamps[i])
            assert frame.encoding == "16UC1" and frame.image.dtype == np.uint16
            assert int(frame.image[0, 0]) == 1000 + (i % 20) // 2
        assert depth.partner_of(stamps[1]) is None or abs(depth.partner_of(stamps[1]) - stamps[1]) <= 5 * MS
        assert recorder.status()["play"]["skew_ms"] < 20.0
        assert recorder.status()["play"]["passes"] >= 1

        # the rate through the node's parameters: twice the pace, the stamps 25 ms apart from then on
        client = ParameterClient(b, "xblick/video_test")
        assert client.set({"rate": 2.0})[0].successful
        time.sleep(0.6)
        later = sorted(s for s in color.stamps() if s > stamps[-1])
        quick = np.diff(later[-6:])
        assert np.all(np.abs(quick - 25 * MS) < MS), quick
        # paused: nothing goes out; resumed: on from there, the stamps growing
        recorder.pause()
        time.sleep(0.2)
        held = len(color.stamps())
        time.sleep(0.4)
        assert len(color.stamps()) == held
        last = max(color.stamps())
        recorder.resume()
        wait_until(lambda: max(color.stamps()) > last, timeout=5.0)
        assert min(s for s in color.stamps() if s > last) - last >= 25 * MS
    finally:
        color.close()
        depth.close()
        recorder.close()
    recorder.check()


def test_an_image_sequence_and_what_is_refused(sessions, videos, tmp_path):  # noqa: F811
    a, b = sessions
    color_mp4, _ = videos
    frames = tmp_path / "frames"
    frames.mkdir()
    from PIL import Image

    for i in range(5):
        Image.fromarray(np.full((H, W, 3), i * 40, np.uint8)).save(frames / f"{i:03d}.png")
    recorder = recording.Recorder(a, tmp_path, name="video_test2")
    received = FrameHistory(ImageSubscriber(b, "t/seq/color", transport="raw"), 20)
    try:
        with pytest.raises(ValueError, match="give --fps"):
            recorder.play([frames], streams=["t/seq/color"])
        with pytest.raises(ValueError, match="and 1 --stream .or --camera.: one per video"):
            recorder.play([color_mp4, color_mp4], streams=["t/a/color"])
        with pytest.raises(ValueError, match="2 --offset-ms for 1 video.*once per video"):
            recorder.play([color_mp4], streams=["t/a/color"], offsets_ms=[1.0, 2.0])
        with pytest.raises(ValueError, match="for MCAP recordings, these are videos"):
            recorder.play([color_mp4], streams=["t/a/color"], keys=["x/*"])
        with pytest.raises(ValueError, match="videos need their stream bases"):
            recorder.play([color_mp4])
        fake_mcap = tmp_path / "rec.bin"  # MCAP by its magic bytes, whatever its name
        fake_mcap.write_bytes(b"\x89MCAP0\r\n" + bytes(64))
        with pytest.raises(ValueError, match="MCAP recordings and videos together"):
            recorder.play([fake_mcap, color_mp4], streams=["t/a/color"])
        with pytest.raises(ValueError, match="options for videos, these are MCAP recordings"):
            recorder.play([fake_mcap], streams=["t/a/color"], fps=10.0)
        with pytest.raises(ValueError, match="of 640x480, the stream is 64x48"):
            calibration = tmp_path / "cal.yaml"
            from xblick.msg import CameraInfo

            calibration.write_text(CameraInfo.pinhole(640, 480, 500, 500, 320, 240).to_yaml("cam"))
            recorder.play([color_mp4], streams=["t/a/color"], camera_infos=[calibration])
        player = recorder.play([str(frames)], streams=["t/seq/color"], fps=10.0, size=(32, 24))
        wait_until(lambda: len(received.stamps()) == 5, timeout=5.0)
        stamps = sorted(received.stamps())
        assert np.all(np.abs(np.diff(stamps) - 100 * MS) < MS)
        frame = received.at(stamps[-1])
        assert frame.image.shape == (24, 32, 3) and int(frame.image[0, 0, 0]) == 160
        wait_until(lambda: player.finished, timeout=5.0)
    finally:
        received.close()
        recorder.close()


def test_video_controls_as_parameters(sessions, videos, tmp_path):  # noqa: F811
    """The play node's parameters on a video: duration and file, paused, a seek (to a frame: decoded on from the key
    frame before it; the stamps growing), loop switched off while it plays (it ends then), the end announced."""
    a, b = sessions
    color_mp4, _ = videos
    recorder = recording.Recorder(a, tmp_path, name="video_ctl")
    color = FrameHistory(ImageSubscriber(b, "t/ctl/color", transport="raw"), 400)
    client = ParameterClient(b, "xblick/video_ctl")
    events = []
    client.on_event(lambda e: events.append({p.name: p.value.value() for p in e.changed_parameters}))
    try:
        recorder.play([color_mp4], streams=["t/ctl/color"], loop=True)
        wait_until(lambda: len(color.stamps()) >= 3, timeout=5.0)
        values = dict(zip(client.list(), [v.value() for v in client.get(client.list())]))
        assert values["duration_s"] == pytest.approx(1.0, abs=0.06) and values["file"] == str(color_mp4)
        assert values["loop"] is True and values["paused"] is False
        assert client.set({"paused": True})[0].successful
        time.sleep(0.2)
        held = len(color.stamps())
        time.sleep(0.4)
        assert len(color.stamps()) == held
        # a seek while paused: from the frame at 0.5 s (its red 10 * 10) once resumed, after the last
        last = max(color.stamps())
        assert client.set({"position_s": 0.5})[0].successful
        assert client.set({"paused": False})[0].successful
        wait_until(lambda: max(color.stamps()) > last, timeout=5.0)
        first = min(s for s in color.stamps() if s > last)
        assert first - last >= 49 * MS  # a period after it, and the pause's wall time
        assert int(color.at(first).image[0, 0, 0]) == pytest.approx(100, abs=3)  # H.264's rounding
        # loop off while it plays: it ends at the end, and says so
        assert client.set({"loop": False})[0].successful
        wait_until(lambda: recorder.state == "idle", timeout=5.0)
        wait_until(lambda: any(e.get("file") == "" for e in events), timeout=5.0)
        stamps = sorted(color.stamps())
        assert stamps == sorted(set(stamps))  # never back
    finally:
        client.close()
        color.close()
        recorder.close()


def _play_videos_on_gpu(port, files, ready, stop):
    """Child (a CUDA ring cannot be opened by the process that made it): two videos played with a CUDA ring each,
    until ``stop``."""
    from xblick.session import open_session

    session = open_session(listen=[f"127.0.0.1:{port}"])
    node = recording.Recorder(session, Path(files[0]).parent, name="video_gpu")
    node.play(files, streams=["t/gpu/color", "t/gpu/depth"], loop=True, cuda_device=0)
    ready.set()
    stop.wait(60)
    node.close()
    session.close()


@pytest.mark.skipif(_core.cuda_device_count() == 0, reason="no CUDA device")
def test_videos_in_cuda_rings(videos, tmp_path):
    """play --cuda-device with videos: each stream offers a CUDA ring on that GPU, which a subscriber on the same GPU
    takes by default (local auto, device cuda:0); it keeps serving after a seek and across loops."""
    import multiprocessing as mp

    from xblick.gpu import CudaImage
    from xblick.session import open_session
    from test_points import free_port

    ctx = mp.get_context("spawn")
    port = free_port()
    ready, stop = ctx.Event(), ctx.Event()
    child = ctx.Process(target=_play_videos_on_gpu, args=(port, [str(v) for v in videos], ready, stop), daemon=True)
    child.start()
    try:
        assert ready.wait(60)
        session = open_session(connect=[f"127.0.0.1:{port}"])
        color = ImageSubscriber(session, "t/gpu/color", transport="auto", device="cuda:0")
        depth = ImageSubscriber(session, "t/gpu/depth", transport="auto", device="cuda:0")

        def frame_of(subscriber):
            end = time.monotonic() + 30
            while time.monotonic() < end:
                frame = subscriber.latest(timeout=0.5)
                if frame is not None:
                    return frame
            raise TimeoutError(subscriber.describe())

        for subscriber, shape in ((color, (H, W, 3)), (depth, (H, W))):
            frame = frame_of(subscriber)
            assert subscriber.transport == "cuda", subscriber.transport_reason
            assert isinstance(frame.image, CudaImage) and frame.image.shape == shape
            assert subscriber.transport_reason == "cuda (on this subscriber's GPU)"
        client = ParameterClient(session, "xblick/video_gpu")
        before = frame_of(color).stamp_ns
        assert client.set({"position_s": 0.0})[0].successful
        time.sleep(1.5)  # past a loop too (the videos are 1 s long)
        after = frame_of(color)
        assert isinstance(after.image, CudaImage) and after.stamp_ns > before
        client.close()
        for c in (color, depth, session):
            c.close()
    finally:
        stop_child(child, stop)


def test_play_takes_a_relative_path_with_a_directory(videos, tmp_path, monkeypatch, capsys):
    """xblick play rec/color.mp4 from the directory above rec: the file relative to the working directory, not to the
    node's directory (the file's own, rec/), which would make it rec/rec/color.mp4."""
    from xblick import cli

    color_mp4, _ = videos
    (tmp_path / "rec").mkdir()
    color_mp4.rename(tmp_path / "rec" / "color.mp4")
    monkeypatch.chdir(tmp_path)
    port = free_port()
    assert cli.main(["play", "rec/color.mp4", "--camera", "t/rel", "--exit", "--report", "0.2", "--listen",
                     f"tcp/127.0.0.1:{port}"]) == 0
    out = capsys.readouterr().out
    assert f"{tmp_path / 'rec' / 'color.mp4'} -> t/rel/color" in out, out


def test_a_looped_playback_says_its_pass(capsys):
    """xblick play --loop of a 12 s clip, a report every 10 s: the position is within the pass, so the line names the
    pass (issue #14: 10.0 s, 8.0 s, 6.0 s looked like a position going back)."""
    from xblick import cli, recording

    for position, passes, sent in ((10.0, 0, 300), (8.0, 1, 601), (6.0, 2, 901)):
        cli._report({"state": "playing", "play": {"kind": "video", "position_s": position, "passes": passes,
                                                  "loop": True, "sent": sent}}, "xblick play")
    assert capsys.readouterr().out.splitlines() == ["xblick play: playing at 10.0 s, pass 1, 300 messages",
                                                    "xblick play: playing at 8.0 s, pass 2, 601 messages",
                                                    "xblick play: playing at 6.0 s, pass 3, 901 messages"]
    # a recording's status has its length; not looped, no pass
    assert recording.position_text({"position_s": 3.0, "duration_s": 12.0, "passes": 0, "loop": False}) == \
        "at 3.0 s of 12.0 s"


def write_frames(directory, n, value=0):
    """An image sequence of ``n`` frames (W x H, rgb): frame i has the red value + i * 20."""
    from PIL import Image

    directory.mkdir()
    for i in range(n):
        Image.fromarray(np.full((H, W, 3), value + i * 20, np.uint8)).save(directory / f"{i:03d}.png")
    return directory


# a rectified stereo pair's calibration as ROS's camera_calibration writes it (ost.yaml: lists over several lines)
STEREO_YAML = """image_width: 64
image_height: 48
camera_name: {name}
camera_matrix:
  rows: 3
  cols: 3
  data: [50.5, 0., 31.25, 0., 50.75, 23.5,
         0., 0., 1.]
distortion_model: plumb_bob
distortion_coefficients:
  rows: 1
  cols: 5
  data: [-0.125, 0.0625, 0.001, -0.002, 0.]
rectification_matrix:
  rows: 3
  cols: 3
  data: [0.9998, 0.0175, 0., -0.0175, 0.9998, 0., 0., 0., 1.]
projection_matrix:
  rows: 3
  cols: 4
  data: [48., 0., 32.5, {tx}, 0., 48., 24.25, 0., 0., 0., 1., 0.]
"""


def test_a_stereo_pair_from_two_videos(sessions, tmp_path):  # noqa: F811
    """Two videos as cam/stereo/left and right, each with its calibration file and pose: the camera_info arrives
    exactly as the files have it (the baseline in the right P), in the optical frames; both poses on the one
    cam/stereo/tf_static (TransformBuffer: the right optical frame 0.12 m along the left one's x); frames of equal
    times have equal stamps."""
    from xblick.msg import CameraInfo
    from xblick.poses import TransformBuffer

    a, b = sessions
    left, right = write_frames(tmp_path / "left", 4), write_frames(tmp_path / "right", 4, 5)
    files = {}
    for name, tx in (("left", 0.0), ("right", -48.0 * 0.12)):
        files[name] = tmp_path / f"{name}.yaml"
        files[name].write_text(STEREO_YAML.format(name=name, tx=tx))
    recorder = recording.Recorder(a, tmp_path, name="video_stereo")
    subs = {n: ImageSubscriber(b, f"t/stereo/{n}", transport="raw", camera_info=True) for n in ("left", "right")}
    histories = {n: FrameHistory(s, 20) for n, s in subs.items()}
    transforms = TransformBuffer(b, "t/stereo")
    try:
        recorder.play([str(left), str(right)], streams=["t/stereo/left", "t/stereo/right"], fps=10.0,
                      camera_infos=[str(files["left"]), str(files["right"])],
                      poses=[[0, 0, 1, 0, 0, 0], [0, -0.12, 1, 0, 0, 0]], loop=True)
        for n in ("left", "right"):
            wait_until(lambda n=n: subs[n].camera_info is not None and len(histories[n].stamps()) >= 4)
            got, want = subs[n].camera_info, CameraInfo.read_file(files[n])
            assert got.header.frame_id == f"t_stereo_{n}_optical" and got.distortion_model == "plumb_bob"
            for field in ("k", "d", "r", "p"):
                np.testing.assert_array_equal(getattr(got, field), getattr(want, field))
        assert subs["right"].camera_info.p[3] == pytest.approx(-48.0 * 0.12)  # P[3] = -f b
        assert set(histories["left"].stamps()) & set(histories["right"].stamps())  # equal times, equal stamps
        stamp = max(histories["left"].stamps())
        pose = transforms.lookup("t_stereo_left_optical", "t_stereo_right_optical", stamp, 5.0)
        np.testing.assert_allclose(pose[:3, 3], [0.12, 0, 0], atol=1e-9)
        np.testing.assert_allclose(pose[:3, :3], np.eye(3), atol=1e-9)
        world = transforms.lookup("world", "t_stereo_left", stamp, 1.0)
        np.testing.assert_allclose(world[:3, 3], [0, 0, 1], atol=1e-9)
    finally:
        transforms.close()
        for h in histories.values():
            h.close()
        recorder.close()
    recorder.check()


def test_different_rates_and_lengths_looped(sessions, tmp_path):  # noqa: F811
    """Two image sequences with a rate each (--fps once per video): 10 frames at 10 frames/s and 3 at 5 frames/s.
    The short one's frames have exactly the stamps of the long one's frames of equal time; it sends nothing after
    its end until the long one has ended; looped, both start again together, the stamps on by the long one's 1.0 s."""
    a, b = sessions
    long, short = write_frames(tmp_path / "long", 10), write_frames(tmp_path / "short", 3)
    recorder = recording.Recorder(a, tmp_path, name="video_rates")
    histories = {n: FrameHistory(ImageSubscriber(b, f"t/rates/{n}", transport="raw"), 50) for n in ("long", "short")}
    try:
        recorder.play([str(long), str(short)], streams=["t/rates/long", "t/rates/short"], fps=[10.0, 5.0], loop=True)
        wait_until(lambda: len(histories["short"].stamps()) >= 6, timeout=10.0)
        longs, shorts = sorted(histories["long"].stamps()), sorted(histories["short"].stamps())
        first = longs[0]
        assert [s - first for s in shorts[:6]] == [0, 200 * MS, 400 * MS, 1000 * MS, 1200 * MS, 1400 * MS]
        assert set(shorts[:6]) <= set(longs)
    finally:
        for h in histories.values():
            h.close()
        recorder.close()
    recorder.check()


def test_the_per_video_rule(sessions, tmp_path, capsys):  # noqa: F811
    """One rule for every option of a video: once per video, or once for all (not names, frame ids, poses), or not
    at all; another count is refused with one line before anything plays. The CLI: --camera once per video."""
    from xblick import cli
    from xblick.video import camera_info_for

    a, _ = sessions
    seqs = [str(write_frames(tmp_path / f"v{i}", 2)) for i in range(3)]
    streams = ["t/r/a", "t/r/b", "t/r/c"]
    recorder = recording.Recorder(a, tmp_path, name="video_rule")
    try:
        for options, refused in (({"poses": [[0, 0, 1, 0, 0, 0]]}, "1 --pose for 3 video"),
                                 ({"frame_ids": ["x"]}, "1 --frame-id for 3 video"),
                                 ({"offsets_ms": [1.0, 2.0]}, "2 --offset-ms for 3 video"),
                                 ({"fps": [10.0, 5.0]}, "2 --fps for 3 video")):
            with pytest.raises(ValueError, match=refused):
                recorder.play(seqs, streams=streams, **{"fps": 10.0, **options})
        with pytest.raises(ValueError, match="2 video.* and 1 --stream"):
            recorder.play(seqs[:2], streams=streams[:1], fps=10.0)
        with pytest.raises(ValueError, match="give each its own --frame-id"):
            recorder.play(seqs[:2], streams=["t/r/a", "t/r/b"], fps=10.0, frame_ids=["same", "same"],
                          poses=[[0, 0, 0, 0, 0, 0]] * 2)
        player = recorder.play(seqs, streams=streams, fps=10.0, offsets_ms=5.0, hfov=[50.0, 60.0, 70.0])
        assert [s["offset_ms"] for s in player.status()["streams"]] == [5.0, 5.0, 5.0]
    finally:
        recorder.close()
    out = capsys.readouterr().out
    assert "camera_info from --hfov 70" in out
    port = free_port()
    with pytest.raises(SystemExit, match="3 video.* and 2 --stream .or --camera."):
        cli.main(["play", *seqs, "--camera", "t/x", "--camera", "t/y", "--fps", "10", "--listen",
                  f"tcp/127.0.0.1:{port}"])
    # nothing given: an assumed field of view, said plainly; the test camera's is vertical
    info, said = camera_info_for(64, 48, "f", None, None, None)
    assert said.startswith("no calibration given: camera_info from an assumed horizontal field of view of 60 degrees")
    assert info.k[0] == pytest.approx(32 / np.tan(np.radians(30)))
    info, said = camera_info_for(64, 48, "f", None, None, None, assumed=("vertical", 60.0))
    assert "assumed vertical field of view of 60 degrees" in said and info.k[0] == pytest.approx(24 / np.tan(np.radians(30)))
    # xblick recorder play sends the video options to a running node, which takes them
    command, options = recording.parse_command(b'{"command": "play", "files": ["a.mp4"], "streams": ["t/a/color"], '
                                               b'"poses": [[0, 0, 1, 0, 0, 0]], "camera_infos": ["a.yaml"]}')
    assert command == "play" and options["poses"] == [[0, 0, 1, 0, 0, 0]]


def blocks() -> np.ndarray:
    """64 x 32 pixels in eight blocks of 16 x 16, each a grey of its own: every turn and mirror gives another image."""
    greys = np.array([[0, 40, 80, 120], [160, 200, 240, 20]], np.uint8)
    return np.repeat(np.repeat(np.repeat(greys, 16, axis=0), 16, axis=1)[..., None], 3, axis=2)


@pytest.mark.parametrize("degrees, hflip", [(0, False), (90, False), (180, False), (-90, False), (0, True),
                                            (90, True), (180, True), (-90, True)])
def test_a_video_with_a_display_matrix_comes_out_upright(tmp_path, degrees, hflip):
    """A video's display matrix (a phone's portrait video: -90, turned clockwise) is applied: its frames, its size,
    and --size of the turned frame. Its meaning as PyAV writes it: turned counter-clockwise, then mirrored."""
    from xblick import video

    path = tmp_path / "turned.mp4"
    with av.open(str(path), "w") as out:
        s = out.add_stream("libx264", rate=10)
        s.width, s.height, s.pix_fmt = 64, 32, "yuv444p"
        s.options = {"qp": "0"}  # lossless: the blocks exact to the colour conversion
        s.set_display_rotation(degrees, hflip=hflip)
        for _ in range(3):
            for packet in s.encode(av.VideoFrame.from_ndarray(blocks(), format="rgb24")):
                out.mux(packet)
        for packet in s.encode():
            out.mux(packet)
    expected = np.rot90(blocks(), degrees // 90)
    expected = expected[:, ::-1] if hflip else expected
    source = video.VideoFile(path)
    assert (source.width, source.height) == (expected.shape[1], expected.shape[0])
    _, image = next(source.frames())
    assert image.shape == expected.shape and np.abs(image.astype(int) - expected).max() <= 3
    sized = video.VideoFile(path, size=(expected.shape[1] // 2, expected.shape[0] // 2))
    assert next(sized.frames())[1].shape == (expected.shape[0] // 2, expected.shape[1] // 2, 3)


def test_a_display_matrix_that_is_no_quarter_turn_is_refused():
    from types import SimpleNamespace

    from xblick import video

    one = video.FIXED_ONE
    half = round(one * np.sqrt(0.5))
    for matrix, rotation in (([half, -half, 0, half, half, 0, 0, 0, 1 << 30], 45),
                             ([2 * one, 0, 0, 0, 2 * one, 0, 0, 0, 1 << 30], 0)):  # turned by 45 degrees, scaled
        frame = SimpleNamespace(side_data={av.sidedata.sidedata.Type.DISPLAYMATRIX: np.array(matrix, np.int32).tobytes()},
                                rotation=rotation)
        with pytest.raises(video.VideoError, match=f"turns it by {rotation} degrees or scales it"):
            video.display_turn(frame, av, "clip.mp4")
    assert video.display_turn(SimpleNamespace(side_data={}), av, "clip.mp4") == (False, False, False)
