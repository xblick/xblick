"""The V4L2 devices of this PC, whether each captures video, and the modes it offers: ``xblick cam`` without a source
shows them in a tree (cam.py).

A webcam usually has two or more device nodes: one that captures video and one that delivers metadata (UVC). The
kernel tells them apart in the node's capabilities, which the ioctl VIDIOC_QUERYCAP returns (struct v4l2_capability,
linux/videodev2.h): with V4L2_CAP_DEVICE_CAPS set, ``device_caps`` holds the node's own capabilities, and a node
captures video when they contain V4L2_CAP_VIDEO_CAPTURE or V4L2_CAP_VIDEO_CAPTURE_MPLANE. The ioctl needs the node
opened; a node this user cannot open (no access to the group ``video``) is listed with its capabilities unknown.
The name is the ioctl's ``card``, else the node's name in sysfs (/sys/class/video4linux/<node>/name).

**Modes.** What a capture node offers comes from three more ioctls, which only read and change no setting of the
device: VIDIOC_ENUM_FMT (its pixel formats, as fourcc codes), VIDIOC_ENUM_FRAMESIZES (per format: a list of sizes,
or a stepwise or continuous range) and VIDIOC_ENUM_FRAMEINTERVALS (per format and size: a list of frame intervals,
or a range). A rate is the inverse of the interval, as an exact fraction (30000/1001 for 29.97). For a range the list
offers the common values inside it (COMMON_SIZES, COMMON_RATES; a stepwise range only those on its steps) and its two
ends.

**Names.** ``xblick cam --pixel-format`` takes FFmpeg's names (its V4L2 input's ``input_format``). FFMPEG_FORMATS maps
the fourcc codes to them. It is FFmpeg's table of the formats its V4L2 input reads (ff_fmt_conversion_table in
libavdevice/v4l2-common.c), read out of the libavdevice inside PyAV 19.0.2's wheel. A fourcc outside it is not
supported. Where two codes share a name (YU12 and YV12 are both ``yuv420p``), the name asks FFmpeg for the first one,
so the later code (LATER_CODES) is not supported either.

**The recommended mode** (:func:`recommend`), from the advertised modes only, by fixed rules:

1. Only supported formats count.
2. Modes of at least 29.97 frames/s (30000/1001, counted as 30) win, if there are any. Otherwise all modes count.
3. Among those, the size with the most pixels at or below 1920 x 1080 (2073600 pixels) wins. If every size is
   larger, the smallest one wins. A smaller size never wins for its higher rate.
4. At that size, the highest rate wins: 60 frames/s beats 30 only at the same size.
5. Then the format: mjpeg, h264, hevc, other compressed formats, uncompressed formats. Every MJPEG frame decodes on
   its own; H.264 saves bandwidth at the cost of delay.
6. Then the device's order: the format listed first, then its size and rate listed first.

A format without sizes or a size without rates counts only where nothing else is left. The same function gives the
recommended mode within one format or one size.
"""

from __future__ import annotations

import errno
import fcntl
import os
import re
import struct
from dataclasses import dataclass, field
from fractions import Fraction
from pathlib import Path

# struct v4l2_capability: driver[16], card[32], bus_info[32], version, capabilities, device_caps, reserved[3]
CAPABILITY = struct.Struct("<16s32s32sIII12x")
VIDIOC_QUERYCAP = (2 << 30) | (CAPABILITY.size << 16) | (ord("V") << 8) | 0  # _IOR('V', 0, struct v4l2_capability)
CAP_VIDEO_CAPTURE = 0x00000001
CAP_VIDEO_CAPTURE_MPLANE = 0x00001000
CAP_META_CAPTURE = 0x00800000
CAP_DEVICE_CAPS = 0x80000000
SYSFS = Path("/sys/class/video4linux")


@dataclass
class Capability:
    """What VIDIOC_QUERYCAP says about a device node."""

    driver: str
    card: str
    bus_info: str
    capabilities: int
    device_caps: int

    @property
    def node_caps(self) -> int:
        """The node's own capabilities: device_caps when the driver fills it, else those of the whole device."""
        return self.device_caps if self.capabilities & CAP_DEVICE_CAPS else self.capabilities

    @property
    def captures_video(self) -> bool:
        return bool(self.node_caps & (CAP_VIDEO_CAPTURE | CAP_VIDEO_CAPTURE_MPLANE))


def parse_capability(data: bytes) -> Capability:
    """struct v4l2_capability as VIDIOC_QUERYCAP filled it."""
    if len(data) != CAPABILITY.size:
        raise ValueError(f"struct v4l2_capability: {CAPABILITY.size} bytes, not {len(data)}")
    driver, card, bus_info, _version, capabilities, device_caps = CAPABILITY.unpack(data)

    def text(raw: bytes) -> str:
        return raw.split(b"\0", 1)[0].decode("utf-8", "replace").strip()

    return Capability(text(driver), text(card), text(bus_info), capabilities, device_caps)


@dataclass
class Device:
    """A /dev/video* node: ``captures`` True or False from its capabilities, None when they could not be read
    (``why``)."""

    path: str
    name: str
    captures: bool | None
    why: str = ""
    capability: Capability | None = None

    def text(self) -> str:
        return f"{self.path}  {self.name or '(no name)'}" + (f"  ({self.why})" if self.why else "")


def _number(path: Path) -> tuple[int, str]:
    digits = re.sub(r"\D", "", path.name)
    return (int(digits) if digits else -1, path.name)


def query(path: Path) -> Capability:
    """VIDIOC_QUERYCAP of a device node (OSError when it cannot be opened or asked)."""
    fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
    try:
        return parse_capability(fcntl.ioctl(fd, VIDIOC_QUERYCAP, bytes(CAPABILITY.size)))
    finally:
        os.close(fd)


def devices(dev: Path = Path("/dev"), sysfs: Path = SYSFS) -> list[Device]:
    """Every /dev/video* node in number order, with its name and whether it captures video."""
    found = []
    for path in sorted(dev.glob("video*"), key=_number):
        name_file = sysfs / path.name / "name"
        name = name_file.read_text().strip() if name_file.is_file() else ""
        try:
            capability = query(path)
        except OSError as error:
            reason = errno.errorcode.get(error.errno, "") if error.strerror is None else error.strerror
            found.append(Device(str(path), name, None, f"capabilities unknown: {reason}"))
            continue
        found.append(Device(str(path), capability.card or name, capability.captures_video, capability=capability))
    return found


def capture_devices(listed: list[Device]) -> list[Device]:
    """The nodes that capture video, and those whose capabilities are unknown (they may)."""
    return [d for d in listed if d.captures is not False]


# ------------------------------------------------------------------------------------------------------------ modes

def _iowr(number: int, size: int) -> int:
    return (3 << 30) | (size << 16) | (ord("V") << 8) | number


# struct v4l2_fmtdesc: index, type, flags, description[32], pixelformat, mbus_code, reserved[3]
FMTDESC = struct.Struct("<III32sII12x")
VIDIOC_ENUM_FMT = _iowr(2, FMTDESC.size)
# struct v4l2_frmsizeenum: index, pixel_format, type, union {discrete: width, height; stepwise: min_width, max_width,
# step_width, min_height, max_height, step_height}, reserved[2]
FRMSIZE = struct.Struct("<III6I8x")
VIDIOC_ENUM_FRAMESIZES = _iowr(74, FRMSIZE.size)
# struct v4l2_frmivalenum: index, pixel_format, width, height, type, union {discrete: numerator, denominator;
# stepwise: min, max, step, each numerator and denominator}, reserved[2]
FRMIVAL = struct.Struct("<IIIII6I8x")
VIDIOC_ENUM_FRAMEINTERVALS = _iowr(75, FRMIVAL.size)
BUF_TYPE_VIDEO_CAPTURE = 1
FMT_FLAG_COMPRESSED = 0x0001
DISCRETE, CONTINUOUS, STEPWISE = 1, 2, 3  # V4L2_FRMSIZE_TYPE_*, V4L2_FRMIVAL_TYPE_*
KINDS = {DISCRETE: "discrete", CONTINUOUS: "continuous", STEPWISE: "stepwise"}

# FFmpeg's V4L2 input: fourcc -> (its input_format name, compressed), in the order of FFmpeg's table (module docstring)
FFMPEG_FORMATS = {
    "YU12": ("yuv420p", False), "YV12": ("yuv420p", False), "422P": ("yuv422p", False), "YUYV": ("yuyv422", False),
    "UYVY": ("uyvy422", False), "411P": ("yuv411p", False), "YUV9": ("yuv410p", False), "YVU9": ("yuv410p", False),
    "RGBO": ("rgb555le", False), "RGBQ": ("rgb555be", False), "RGBP": ("rgb565le", False),
    "RGBR": ("rgb565be", False), "BGR3": ("bgr24", False), "RGB3": ("rgb24", False), "XR24": ("bgr0", False),
    "BX24": ("0rgb", False), "AR24": ("bgra", False), "BA24": ("argb", False), "BGR4": ("bgr0", False),
    "RGB4": ("0rgb", False), "GREY": ("gray", False), "Y16 ": ("gray16le", False), "Z16 ": ("gray16le", False),
    "NV12": ("nv12", False), "NV16": ("nv16", False), "NV24": ("nv24", False), "MJPG": ("mjpeg", True),
    "JPEG": ("mjpeg", True), "H264": ("h264", True), "MPG4": ("mpeg4", True), "CPIA": ("cpia", True),
    "BA81": ("bayer_bggr8", False), "GBRG": ("bayer_gbrg8", False), "GRBG": ("bayer_grbg8", False),
    "RGGB": ("bayer_rggb8", False),
}
_first: dict[str, str] = {}
for _code, (_name, _) in FFMPEG_FORMATS.items():
    _first.setdefault(_name, _code)
LATER_CODES = {code: _first[name] for code, (name, _) in FFMPEG_FORMATS.items() if _first[name] != code}
del _code, _name, _first

COMMON_SIZES = ((3840, 2160), (2560, 1440), (1920, 1080), (1600, 1200), (1280, 960), (1280, 720), (1024, 768),
                (800, 600), (640, 480), (320, 240))  # offered inside a stepwise or continuous range of sizes
COMMON_RATES = tuple(Fraction(r) for r in (120, 90, 60, 50, 30, 25, 24, 20, 15, 10, 5))  # ... of rates
TARGET_PIXELS = 1920 * 1080
MIN_GOOD_RATE = Fraction(30000, 1001)  # at least this counts as 30 frames/s
FORMAT_ORDER = ("mjpeg", "h264", "hevc")  # then other compressed formats, then uncompressed ones


def fourcc(code: int) -> str:
    return struct.pack("<I", code).decode("latin-1")


@dataclass
class Size:
    """A frame size and its rates (frames/s, exact) as the device lists them, or the values offered from its range
    (``rates_from``: discrete, stepwise, continuous; empty when it reports none)."""

    width: int
    height: int
    rates: list[Fraction] = field(default_factory=list)
    rates_from: str = ""

    @property
    def pixels(self) -> int:
        return self.width * self.height

    def text(self) -> str:
        return f"{self.width}x{self.height}"


@dataclass
class Format:
    """A pixel format of a capture node: its fourcc code, the driver's description, its sizes (``sizes_from`` as
    Size.rates_from). ``name``: FFmpeg's name, which --pixel-format takes; None when FFmpeg's V4L2 input does not read
    it (``why``)."""

    code: str
    description: str
    sizes: list[Size] = field(default_factory=list)
    sizes_from: str = ""
    name: str | None = None
    compressed: bool = False
    why: str = ""

    @property
    def supported(self) -> bool:
        return self.name is not None


def rate_text(rate: Fraction) -> str:
    """A rate for people and for --fps: 30, 29.97, 7.5 (``:g``, six significant digits)."""
    return f"{float(rate):g}"


def name_format(code: str, description: str, flags: int) -> Format:
    """A Format with FFmpeg's name, or the reason it is not supported."""
    if code in FFMPEG_FORMATS and code not in LATER_CODES:
        name, compressed = FFMPEG_FORMATS[code]
        return Format(code, description, name=name, compressed=compressed)
    if code in LATER_CODES:
        name = FFMPEG_FORMATS[code][0]
        why = f"--pixel-format {name} asks FFmpeg for {LATER_CODES[code].strip()}"
    else:
        why = "FFmpeg's V4L2 input does not read it"
    return Format(code, description, compressed=bool(flags & FMT_FLAG_COMPRESSED), why=why)


def _enum(fd: int, ioctl, request: int, layout: struct.Struct, fields: tuple,
          kind_at: int | None = None) -> list[tuple]:
    """Every answer of an enumerating ioctl (index 0, 1, ... until EINVAL; ENOTTY, a driver that does not know the
    ioctl, ends it as well: what it has not reported is unknown). ``kind_at``: the field of the answer that says
    whether it is a list (DISCRETE) or a range, which has only index 0."""
    answers = []
    for index in range(1024):
        try:
            data = ioctl(fd, request, layout.pack(index, *fields))
        except OSError as error:
            if error.errno in (errno.EINVAL, errno.ENOTTY):
                break
            raise
        answer = layout.unpack(data)
        answers.append(answer)
        if kind_at is not None and answer[kind_at] != DISCRETE:
            break
    return answers


def _range(low: int, high: int, step: int, kind: int, value: int) -> bool:
    if not low <= value <= high:
        return False
    return kind == CONTINUOUS or step <= 0 or (value - low) % step == 0


def sizes_of(fd: int, code: int, ioctl=fcntl.ioctl) -> tuple[list[tuple[int, int]], str]:
    """The sizes of a pixel format and how the device gave them (module docstring)."""
    answers = _enum(fd, ioctl, VIDIOC_ENUM_FRAMESIZES, FRMSIZE, (code, 0, 0, 0, 0, 0, 0, 0), kind_at=2)
    if not answers:
        return [], ""
    kind = answers[0][2]
    if kind == DISCRETE:
        return [(a[3], a[4]) for a in answers], "discrete"
    if kind not in (CONTINUOUS, STEPWISE):
        return [], ""
    _, _, _, min_w, max_w, step_w, min_h, max_h, step_h = answers[0]
    inside = [(w, h) for w, h in COMMON_SIZES
              if _range(min_w, max_w, step_w, kind, w) and _range(min_h, max_h, step_h, kind, h)]
    found = sorted({(max_w, max_h), *inside, (min_w, min_h)}, key=lambda s: (-s[0] * s[1], -s[0]))
    return found, KINDS[kind]


def rates_of(fd: int, code: int, width: int, height: int, ioctl=fcntl.ioctl) -> tuple[list[Fraction], str]:
    """The rates of a pixel format at a size, frames/s as fractions, and how the device gave them."""
    answers = _enum(fd, ioctl, VIDIOC_ENUM_FRAMEINTERVALS, FRMIVAL, (code, width, height, 0, 0, 0, 0, 0, 0, 0),
                    kind_at=4)
    if not answers:
        return [], ""
    kind = answers[0][4]
    if kind == DISCRETE:
        return [Fraction(a[6], a[5]) for a in answers if a[5] and a[6]], "discrete"
    if kind not in (CONTINUOUS, STEPWISE):
        return [], ""
    _, _, _, _, _, min_n, min_d, max_n, max_d, step_n, step_d = answers[0]
    if not (min_n and min_d and max_n and max_d):
        return [], ""
    shortest, longest = Fraction(min_n, min_d), Fraction(max_n, max_d)  # intervals, seconds
    step = Fraction(step_n, step_d) if step_n and step_d else Fraction(0)

    def inside(rate: Fraction) -> bool:
        interval = 1 / rate
        if not shortest <= interval <= longest:
            return False
        return kind == CONTINUOUS or step == 0 or (interval - shortest) % step == 0

    found = sorted({1 / shortest, *(r for r in COMMON_RATES if inside(r)), 1 / longest}, reverse=True)
    return found, KINDS[kind]


def formats_of(fd: int, ioctl=fcntl.ioctl) -> list[Format]:
    """Every pixel format of an open capture node, with its sizes and their rates (module docstring)."""
    formats = []
    for _, _, flags, description, code, _ in _enum(fd, ioctl, VIDIOC_ENUM_FMT, FMTDESC,
                                                    (BUF_TYPE_VIDEO_CAPTURE, 0, b"", 0, 0)):
        text = description.split(b"\0", 1)[0].decode("utf-8", "replace").strip()
        fmt = name_format(fourcc(code), text, flags)
        sizes, fmt.sizes_from = sizes_of(fd, code, ioctl)
        for width, height in sizes:
            rates, rates_from = rates_of(fd, code, width, height, ioctl)
            fmt.sizes.append(Size(width, height, rates, rates_from))
        formats.append(fmt)
    return formats


@dataclass
class Camera:
    """A capture node for the list of ``xblick cam``: its formats (empty: none reported), its stable names in
    /dev/v4l/by-id, and ``why`` when its formats could not be read (it cannot be chosen then)."""

    path: str
    name: str
    formats: list[Format] = field(default_factory=list)
    by_id: list[str] = field(default_factory=list)
    why: str = ""


def by_id_names(by_id: Path = Path("/dev/v4l/by-id")) -> dict[str, list[str]]:
    """Device node -> its names in /dev/v4l/by-id (udev's stable links), sorted."""
    names: dict[str, list[str]] = {}
    if by_id.is_dir():
        for link in sorted(by_id.iterdir()):
            try:
                names.setdefault(str(link.resolve(strict=True)), []).append(link.name)
            except OSError:
                continue
    return names


def _reason(error: OSError) -> str:
    if error.errno in (errno.EACCES, errno.EPERM):
        return "no permission to open it (add your user to the group video)"
    if error.errno == errno.EBUSY:
        return "busy (another program uses it)"
    return error.strerror or errno.errorcode.get(error.errno, str(error))


def cameras(listed: list[Device] | None = None, by_id: Path = Path("/dev/v4l/by-id")) -> list[Camera]:
    """The capture nodes of this PC with their modes (``listed``: ``devices()``). A node that cannot be opened or
    asked keeps the reason. A multi-planar node's formats are not read (it is listed without them)."""
    links = by_id_names(by_id)
    found = []
    for device in capture_devices(devices() if listed is None else listed):
        camera = Camera(device.path, device.name, by_id=links.get(device.path, []), why=device.why)
        caps = device.capability.node_caps if device.capability is not None else 0
        if device.captures and caps & CAP_VIDEO_CAPTURE:
            try:
                fd = os.open(device.path, os.O_RDONLY | os.O_NONBLOCK)
                try:
                    camera.formats = formats_of(fd)
                finally:
                    os.close(fd)
            except OSError as error:
                camera.why = _reason(error)
        elif device.captures:
            camera.why = "multi-planar capture: its formats are not listed"
        found.append(camera)
    return found


# ------------------------------------------------------------------------------------------- the recommended mode

@dataclass(frozen=True)
class Choice:
    """A mode to start: a format and, where known, a size and a rate. ``format`` None: the camera's own mode."""

    format: Format | None = None
    size: Size | None = None
    rate: Fraction | None = None

    def options(self) -> list[str]:
        """The options of ``xblick cam`` that ask for it."""
        options = []
        if self.format is not None:
            options += ["--pixel-format", self.format.name]
        if self.size is not None:
            options += ["--size", self.size.text()]
        if self.rate is not None:
            options += ["--fps", rate_text(self.rate)]
        return options

    def text(self) -> str:
        if self.format is None:
            return "the camera's own mode"
        parts = [self.format.name] + ([self.size.text()] if self.size else [])
        return " ".join(parts) + (f" at {rate_text(self.rate)} frames/s" if self.rate is not None else "")


def _preference(fmt: Format) -> int:
    if fmt.name in FORMAT_ORDER:
        return FORMAT_ORDER.index(fmt.name)
    return len(FORMAT_ORDER) if fmt.compressed else len(FORMAT_ORDER) + 1


def recommend(formats: list[Format], only: Size | None = None) -> Choice | None:
    """The recommended mode among ``formats`` (module docstring), within the size ``only`` if given; None when no
    format is supported."""
    modes = []  # (kind, format index, size index, rate index, format, size, rate)
    for fi, fmt in enumerate(formats):
        if not fmt.supported:
            continue
        sizes = [only] if only is not None else fmt.sizes
        if not sizes:
            modes.append((2, fi, 0, 0, fmt, None, None))
        for si, size in enumerate(sizes):
            if not size.rates:
                modes.append((1, fi, si, 0, fmt, size, None))
            for ri, rate in enumerate(size.rates):
                modes.append((0, fi, si, ri, fmt, size, rate))
    if not modes:
        return None
    best_kind = min(m[0] for m in modes)
    modes = [m for m in modes if m[0] == best_kind]  # sizes and rates known, else sizes known, else formats only
    good = [m for m in modes if m[6] is not None and m[6] >= MIN_GOOD_RATE]
    modes = good or modes

    def key(m):
        _, fi, si, ri, fmt, size, rate = m
        pixels = size.pixels if size is not None else 0
        place = (0, -pixels) if pixels <= TARGET_PIXELS else (1, pixels)
        return place, -(rate if rate is not None else 0), _preference(fmt), fi, si, ri

    _, _, _, _, fmt, size, rate = min(modes, key=key)
    return Choice(fmt, size, rate)
