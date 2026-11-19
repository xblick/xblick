"""Photos played by xblick play (video.ImageSequence) come out upright: each of the eight EXIF orientations, from a
directory and a glob, JPEG and PNG; the stream's size and CameraInfo are those of the turned image. Generated images
only. Videos with a display matrix: test_video.py (needs PyAV)."""

import numpy as np
import pytest
from PIL import Image
from test_camera import sessions, wait_until  # noqa: F401 - sessions is a fixture

from xblick import recording, video
from xblick.camera import ImageSubscriber

W, H = 48, 32  # as stored in the file: landscape
ORIENTATION = 0x0112  # the EXIF tag


def stored() -> np.ndarray:
    """Six blocks of 16 x 16 pixels, each a colour of its own: every turn and mirror gives another image."""
    colours = np.array([[[255, 0, 0], [0, 255, 0], [0, 0, 255]], [[255, 255, 0], [0, 255, 255], [255, 255, 255]]],
                       np.uint8)
    return np.repeat(np.repeat(colours, 16, axis=0), 16, axis=1)


# what each orientation shows (EXIF 2.32, tag 274: where the stored row 0 and column 0 appear), from the stored pixels
UPRIGHT = {
    1: lambda a: a,
    2: lambda a: a[:, ::-1],                # mirrored left to right
    3: lambda a: a[::-1, ::-1],             # turned by 180 degrees
    4: lambda a: a[::-1],                   # mirrored top to bottom
    5: lambda a: a.swapaxes(0, 1),          # mirrored along the main diagonal
    6: lambda a: np.rot90(a, -1),           # turned by 90 degrees clockwise (a phone held upright)
    7: lambda a: np.rot90(a, -1)[::-1],     # mirrored along the other diagonal
    8: lambda a: np.rot90(a, 1),            # turned by 90 degrees counter-clockwise
}


def save(path, orientation: int, image=None):
    exif = Image.Exif()
    exif[ORIENTATION] = orientation
    Image.fromarray(stored() if image is None else image).save(path, exif=exif, quality=95)
    return path


@pytest.mark.parametrize("orientation", range(1, 9))
def test_each_orientation_upright(tmp_path, orientation):
    save(tmp_path / "photo.jpg", orientation)
    sequence = video.ImageSequence(str(tmp_path), fps=1.0)
    expected = UPRIGHT[orientation](stored())
    assert (sequence.width, sequence.height) == (expected.shape[1], expected.shape[0])
    [(_, image)] = list(sequence.frames())
    assert image.shape == expected.shape
    assert np.abs(image.astype(int) - expected).mean() < 10  # JPEG: close (a wrong turn: about 100 or more)


def test_a_glob_png_and_a_size(tmp_path):
    for i, orientation in enumerate((6, 6, 8)):
        save(tmp_path / f"{i}.png", orientation)
    (tmp_path / "x.txt").write_text("not an image")
    sequence = video.ImageSequence(str(tmp_path / "*.png"), fps=1.0)
    assert (sequence.width, sequence.height) == (H, W)  # turned: portrait
    images = [image for _, image in sequence.frames()]
    np.testing.assert_array_equal(images[0], UPRIGHT[6](stored()))  # PNG: exact
    # a photo turned the other way: the same size, resized to the first one's as every image of a sequence
    assert images[2].shape == (W, H, 3)
    sized = video.ImageSequence(str(tmp_path / "*.png"), fps=1.0, size=(16, 24))
    assert next(sized.frames())[1].shape == (24, 16, 3)


def test_the_stream_and_its_camera_info_are_turned(sessions, tmp_path):  # noqa: F811
    a, b = sessions
    frames = tmp_path / "frames"
    frames.mkdir()
    for i in range(3):
        save(frames / f"{i:03d}.jpg", 6)
    recorder = recording.Recorder(a, tmp_path, name="orientation_test")
    subscriber = ImageSubscriber(b, "t/photo/color", transport="raw", camera_info=True)
    try:
        player = recorder.play([str(frames)], streams=["t/photo/color"], fps=10.0, loop=True)
        assert player.status()["streams"][0]["size"] == [H, W]
        frame = subscriber.latest(timeout=5.0)
        assert frame is not None and frame.image.shape == (W, H, 3)
        wait_until(lambda: subscriber.camera_info is not None, timeout=5.0)
        info = subscriber.camera_info
        assert (info.width, info.height) == (H, W)
        assert info.k[2] == pytest.approx((H - 1) / 2) and info.k[5] == pytest.approx((W - 1) / 2)
    finally:
        subscriber.close()
        recorder.close()
    recorder.check()
