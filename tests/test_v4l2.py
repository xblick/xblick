"""The modes of a V4L2 capture node (v4l2.py) read from realistic ioctl answers (a UVC webcam with MJPG and YUYV, a
device with stepwise sizes and continuous intervals, one that reports nothing), FFmpeg's names of the pixel formats,
and the recommended mode as a table. No device is needed: a stand-in answers the ioctls."""

import errno
import struct
from fractions import Fraction as F

import pytest

from xblick import v4l2
from xblick.v4l2 import Choice, Format, Size, recommend


def code(text: str) -> int:
    return struct.unpack("<I", text.encode())[0]


class Device:
    """Answers VIDIOC_ENUM_FMT, VIDIOC_ENUM_FRAMESIZES and VIDIOC_ENUM_FRAMEINTERVALS as a driver does, from
    ``formats``: [(fourcc, description, flags, sizes)], sizes ("discrete", [(w, h, intervals)]) or ("stepwise" |
    "continuous", (min_w, max_w, step_w, min_h, max_h, step_h), intervals); intervals ("discrete", [(num, den)]) or
    ("stepwise" | "continuous", (min_num, min_den, max_num, max_den, step_num, step_den)), or None (ENOTTY: the
    driver does not know the ioctl)."""

    KIND = {"discrete": 1, "continuous": 2, "stepwise": 3}

    def __init__(self, formats):
        self.formats = formats
        self.requests = []

    def _format(self, fourcc):
        return next(f for f in self.formats if code(f[0]) == fourcc)

    def ioctl(self, fd, request, data):
        self.requests.append(request)
        if request == v4l2.VIDIOC_ENUM_FMT:
            index, buf_type, *_ = v4l2.FMTDESC.unpack(data)
            assert buf_type == v4l2.BUF_TYPE_VIDEO_CAPTURE
            if index >= len(self.formats):
                raise OSError(errno.EINVAL, "Invalid argument")
            fourcc, description, flags, _ = self.formats[index]
            return v4l2.FMTDESC.pack(index, buf_type, flags, description.encode(), code(fourcc), 0)
        if request == v4l2.VIDIOC_ENUM_FRAMESIZES:
            index, fourcc, *_ = v4l2.FRMSIZE.unpack(data)
            sizes = self._format(fourcc)[3]
            if sizes is None:
                raise OSError(errno.ENOTTY, "Inappropriate ioctl for device")
            kind, values = sizes[0], sizes[1]
            if kind == "discrete":
                if index >= len(values):
                    raise OSError(errno.EINVAL, "Invalid argument")
                w, h, _ = values[index]
                return v4l2.FRMSIZE.pack(index, fourcc, 1, w, h, 0, 0, 0, 0)
            if index > 0:
                raise OSError(errno.EINVAL, "Invalid argument")
            return v4l2.FRMSIZE.pack(index, fourcc, self.KIND[kind], *values)
        if request == v4l2.VIDIOC_ENUM_FRAMEINTERVALS:
            index, fourcc, w, h, *_ = v4l2.FRMIVAL.unpack(data)
            sizes = self._format(fourcc)[3]
            if sizes[0] == "discrete":
                intervals = next(i for sw, sh, i in sizes[1] if (sw, sh) == (w, h))
            else:
                intervals = sizes[2]
            if intervals is None:
                raise OSError(errno.ENOTTY, "Inappropriate ioctl for device")
            kind, values = intervals
            if kind == "discrete":
                if index >= len(values):
                    raise OSError(errno.EINVAL, "Invalid argument")
                return v4l2.FRMIVAL.pack(index, fourcc, w, h, 1, *values[index], 0, 0, 0, 0)
            if index > 0:
                raise OSError(errno.EINVAL, "Invalid argument")
            return v4l2.FRMIVAL.pack(index, fourcc, w, h, self.KIND[kind], *values)
        raise OSError(errno.ENOTTY, "Inappropriate ioctl for device")


# a UVC webcam as uvcvideo reports it (intervals simplified to 1/30, 2/15 ...): YUYV first, then MJPG
UVC_RATES = ("discrete", [(1, 30), (1, 24), (1, 20), (1, 15), (1, 10), (2, 15), (1, 5)])
WEBCAM = [
    ("YUYV", "YUYV 4:2:2", 0, ("discrete", [(640, 480, UVC_RATES),
                                             (1280, 720, ("discrete", [(1, 10), (2, 15), (1, 5)])),
                                             (1920, 1080, ("discrete", [(1, 5), (1, 2)]))])),
    ("MJPG", "Motion-JPEG", 1, ("discrete", [(1920, 1080, UVC_RATES), (1280, 720, ("discrete", [(1, 60), (1, 30)])),
                                              (640, 480, UVC_RATES)])),
]


def test_the_ioctl_numbers_and_layouts_are_those_of_videodev2():
    assert (v4l2.FMTDESC.size, v4l2.FRMSIZE.size, v4l2.FRMIVAL.size) == (64, 44, 52)
    assert v4l2.VIDIOC_ENUM_FMT == 0xC0405602
    assert v4l2.VIDIOC_ENUM_FRAMESIZES == 0xC02C564A
    assert v4l2.VIDIOC_ENUM_FRAMEINTERVALS == 0xC034564B


def test_a_uvc_webcam():
    device = Device(WEBCAM)
    yuyv, mjpg = v4l2.formats_of(3, device.ioctl)
    assert (yuyv.code, yuyv.description, yuyv.name, yuyv.compressed) == ("YUYV", "YUYV 4:2:2", "yuyv422", False)
    assert (mjpg.code, mjpg.name, mjpg.compressed, mjpg.sizes_from) == ("MJPG", "mjpeg", True, "discrete")
    assert [s.text() for s in mjpg.sizes] == ["1920x1080", "1280x720", "640x480"]  # the device's order
    assert mjpg.sizes[0].rates == [F(30), F(24), F(20), F(15), F(10), F(15, 2), F(5)]
    assert mjpg.sizes[0].rates_from == "discrete"
    assert yuyv.sizes[2].rates == [F(5), F(2)]
    assert recommend([yuyv, mjpg]) == Choice(mjpg, mjpg.sizes[0], F(30))
    assert v4l2.VIDIOC_ENUM_FMT in device.requests and not {r for r in device.requests} - {
        v4l2.VIDIOC_ENUM_FMT, v4l2.VIDIOC_ENUM_FRAMESIZES, v4l2.VIDIOC_ENUM_FRAMEINTERVALS}  # only enumerations


def test_ranges_offer_the_common_values_inside_them_and_their_ends():
    device = Device([
        # sizes 160x120 to 2592x1944 in steps of 16 x 8, any interval from 1/90 s to 1 s
        ("YU12", "Planar YUV 4:2:0", 0, ("stepwise", (160, 2592, 16, 120, 1944, 8),
                                         ("continuous", (1, 90, 1, 1, 1, 1)))),
        # any size from 1x1 to 1920x1080, intervals from 1/60 s to 1/15 s in steps of 1/60 s
        ("NV12", "Y/UV 4:2:0", 0, ("continuous", (1, 1920, 1, 1, 1080, 1), ("stepwise", (1, 60, 1, 15, 1, 60)))),
    ])
    stepwise, continuous = v4l2.formats_of(3, device.ioctl)
    assert stepwise.sizes_from == "stepwise"
    assert [s.text() for s in stepwise.sizes] == ["2592x1944", "2560x1440", "1920x1080", "1600x1200", "1280x960",
                                                  "1280x720", "1024x768", "800x600", "640x480", "320x240", "160x120"]
    assert stepwise.sizes[0].rates == [F(90), F(60), F(50), F(30), F(25), F(24), F(20), F(15), F(10), F(5), F(1)]
    assert stepwise.sizes[0].rates_from == "continuous"
    assert [s.text() for s in continuous.sizes] == ["1920x1080", "1280x960", "1280x720", "1024x768", "800x600",
                                                    "640x480", "320x240", "1x1"]
    assert continuous.sizes[0].rates == [F(60), F(30), F(20), F(15)]  # 1/60, 2/60, 3/60, 4/60 s: on the steps
    assert continuous.sizes[0].rates_from == "stepwise"


def test_exact_rates():
    device = Device([("MJPG", "Motion-JPEG", 1, ("discrete", [(1920, 1080, ("discrete", [(1001, 30000), (1, 25)]))]))])
    (fmt,) = v4l2.formats_of(3, device.ioctl)
    assert fmt.sizes[0].rates == [F(30000, 1001), F(25)]
    assert v4l2.rate_text(F(30000, 1001)) == "29.97" and v4l2.rate_text(F(15, 2)) == "7.5"
    assert recommend([fmt]).options() == ["--pixel-format", "mjpeg", "--size", "1920x1080", "--fps", "29.97"]


def test_a_device_that_reports_nothing():
    assert v4l2.formats_of(3, Device([]).ioctl) == []
    (fmt,) = v4l2.formats_of(3, Device([("YUYV", "YUYV 4:2:2", 0, None)]).ioctl)  # no ENUM_FRAMESIZES
    assert fmt.sizes == [] and fmt.sizes_from == ""
    assert recommend([fmt]) == Choice(fmt, None, None)
    assert recommend([fmt]).options() == ["--pixel-format", "yuyv422"]
    (fmt,) = v4l2.formats_of(3, Device([("YUYV", "YUYV", 0, ("discrete", [(640, 480, None)]))]).ioctl)
    assert fmt.sizes == [Size(640, 480, [], "")]
    assert recommend([fmt]).options() == ["--pixel-format", "yuyv422", "--size", "640x480"]


def test_names_of_the_formats():
    assert v4l2.name_format("MJPG", "Motion-JPEG", 1).name == "mjpeg"
    assert v4l2.name_format("H264", "H.264", 1).name == "h264"
    assert v4l2.name_format("GREY", "8-bit Greyscale", 0).name == "gray"
    assert v4l2.name_format("Y16 ", "16-bit Greyscale", 0).name == "gray16le"
    hevc = v4l2.name_format("HEVC", "HEVC", 1)
    assert hevc.name is None and hevc.why == "FFmpeg's V4L2 input does not read it" and hevc.compressed
    later = v4l2.name_format("YV12", "Planar YVU 4:2:0", 0)
    assert later.name is None and later.why == "--pixel-format yuv420p asks FFmpeg for YU12"
    assert v4l2.name_format("Z16 ", "16-bit Depth", 0).why == "--pixel-format gray16le asks FFmpeg for Y16"


def fmt(code_, *sizes):
    """A Format from (w, h, rates...) tuples."""
    return Format(code_, code_, [Size(w, h, [F(r) for r in rates], "discrete") for w, h, *rates in sizes],
                  "discrete", *(lambda f: (f.name, f.compressed, f.why))(v4l2.name_format(code_, code_, 0)))


@pytest.mark.parametrize("formats, expected", [
    # the owner's example: 1920x1080 at 30 in MJPG wins over the same in H.264, over 720p60 and over YUYV at 5
    ([fmt("H264", (1920, 1080, 30)), fmt("MJPG", (1920, 1080, 30), (1280, 720, 60)), fmt("YUYV", (1920, 1080, 5))],
     ("mjpeg", "1920x1080", 30)),
    # a smaller size never wins for its higher rate
    ([fmt("MJPG", (1280, 720, 60), (1920, 1080, 30))], ("mjpeg", "1920x1080", 30)),
    # 60 beats 30 at the same size, before the format's order
    ([fmt("MJPG", (1920, 1080, 30)), fmt("YUYV", (1920, 1080, 60))], ("yuyv422", "1920x1080", 60)),
    # at least 30 frames/s first: 4K at 30 beats 1080p at 15 ...
    ([fmt("MJPG", (3840, 2160, 30), (1920, 1080, 15))], ("mjpeg", "3840x2160", 30)),
    # ... and above 1920x1080 the smallest size
    ([fmt("MJPG", (3840, 2160, 30), (2560, 1440, 30))], ("mjpeg", "2560x1440", 30)),
    # 29.97 counts as 30
    ([fmt("MJPG", (3840, 2160, 30)), Format("YUYV", "", [Size(1920, 1080, [F(30000, 1001)])], "", "yuyv422")],
     ("yuyv422", "1920x1080", F(30000, 1001))),
    # nothing at 30: the largest size up to 1920x1080, at its highest rate
    ([fmt("YUYV", (1280, 720, 25, 10), (1920, 1080, 15, 5))], ("yuyv422", "1920x1080", 15)),
    # format order at the same size and rate: mjpeg, h264, other compressed, uncompressed, then the device's order
    ([fmt("YUYV", (640, 480, 30)), fmt("H264", (640, 480, 30))], ("h264", "640x480", 30)),
    ([fmt("YUYV", (640, 480, 30)), fmt("MPG4", (640, 480, 30))], ("mpeg4", "640x480", 30)),
    ([fmt("NV12", (640, 480, 30)), fmt("YUYV", (640, 480, 30))], ("nv12", "640x480", 30)),
    # unsupported formats do not count, however good their modes
    ([fmt("HEVC", (1920, 1080, 60)), fmt("YV12", (1920, 1080, 60)), fmt("YUYV", (640, 480, 30))],
     ("yuyv422", "640x480", 30)),
    ([fmt("HEVC", (1920, 1080, 60))], None),
    ([], None),
])
def test_the_recommended_mode(formats, expected):
    choice = recommend(formats)
    if expected is None:
        assert choice is None
    else:
        assert (choice.format.name, choice.size.text(), choice.rate) == expected


def test_within_a_format_or_a_size():
    mjpg = fmt("MJPG", (1920, 1080, 30, 15), (1280, 720, 60, 30))
    assert recommend([mjpg], only=mjpg.sizes[1]) == Choice(mjpg, mjpg.sizes[1], F(60))
    yuyv = fmt("YUYV", (1920, 1080, 5), (640, 480, 30))
    assert recommend([yuyv]) == Choice(yuyv, yuyv.sizes[1], F(30))  # within YUYV alone: 640x480 at 30
    assert recommend([yuyv], only=yuyv.sizes[0]) == Choice(yuyv, yuyv.sizes[0], F(5))
    # a size without rates counts only when no size has any
    partly = Format("YUYV", "", [Size(1920, 1080, [], ""), Size(640, 480, [F(15)], "discrete")], "discrete", "yuyv422")
    assert recommend([partly]) == Choice(partly, partly.sizes[1], F(15))


def test_by_id_names_and_unreadable_nodes(tmp_path):
    (tmp_path / "video2").write_bytes(b"")
    by_id = tmp_path / "by-id"
    by_id.mkdir()
    (by_id / "usb-046d_C922_Pro_Stream_Webcam_ABC-video-index0").symlink_to(tmp_path / "video2")
    (by_id / "usb-gone-video-index0").symlink_to(tmp_path / "video9")  # a dangling link is left out
    assert v4l2.by_id_names(by_id) == {str(tmp_path / "video2"): ["usb-046d_C922_Pro_Stream_Webcam_ABC-video-index0"]}
    assert v4l2.by_id_names(tmp_path / "none") == {}
    capture = v4l2.Capability("uvcvideo", "C922", "usb-1", 0x84A00001, 0x04200001)
    mplane = v4l2.Capability("rkisp", "ISP", "platform:1", 0x84201000, 0x04201000)
    listed = [v4l2.Device(str(tmp_path / "video2"), "C922", True, capability=capture),  # a plain file: no ioctl
              v4l2.Device("/dev/video3", "ISP", True, capability=mplane),
              v4l2.Device("/dev/video4", "", None, "capabilities unknown: Permission denied")]
    found = v4l2.cameras(listed, by_id)
    assert [(c.path[-6:], c.by_id, c.formats) for c in found] == [
        ("video2", ["usb-046d_C922_Pro_Stream_Webcam_ABC-video-index0"], []), ("video3", [], []), ("video4", [], [])]
    assert found[0].why == ""  # a file answers no ioctl (ENOTTY): no formats reported
    assert found[1].why == "multi-planar capture: its formats are not listed"
    assert found[2].why == "capabilities unknown: Permission denied"
    assert v4l2._reason(PermissionError(errno.EACCES, "Permission denied")) == (
        "no permission to open it (add your user to the group video)")
    assert v4l2._reason(OSError(errno.EBUSY, "Device or resource busy")) == "busy (another program uses it)"
