"""The GPU link (ros_cuda.py) against ROS 2's own programs in docker containers on this GPU: a stock Lyrical
subscriber that accepts the CUDA buffer backend (tests/ros/cuda_check, built here) gets our GPU frames as CUDA buffers
with the right pixels, with the backend's release and, given an overlay, another build of it; a reader that holds its
messages makes the pool drop rather than wait; a peer of another Linux user gets no block; and Isaac ROS's
ResizeNode reads our GPU frames as descriptors and resizes them right.

Skipped without docker or a CUDA device, and unless an environment variable names the image:

* XBLICK_ROS_CUDA_IMAGE: ROS 2 Lyrical with rmw_zenoh_cpp, cuda_buffer_backend, nvcc and colcon
  (tests/ros/lyrical_cuda.Dockerfile); XBLICK_ROS_CUDA_OVERLAY: a colcon install space with another build of the
  backend (such as its current source), mounted at its path and sourced on top;
* XBLICK_ROS_ISAAC_IMAGE: Isaac ROS 5.1 on ROS 2 Lyrical with isaac_ros_image_proc, rmw_zenoh_cpp and
  ros-lyrical-cuda-buffer-backend (the plugin, which Isaac's packages do not pull).

The containers share this host's network and pid namespace (the link must see the reader's process), run as this
user, have a /dev/shm of their own (a tmpfs: CUDA's interprocess events need one), which this process reaches as
/proc/<pid>/root/dev/shm (the readers' records and the backend's registry; never the host's), and the test's folder
at its own path (the socket files). The Isaac container runs alone: a wrong descriptor aborts it."""

import os
import shutil
import socket
import subprocess
import time
import uuid

import numpy as np
import pytest

from xblick import _core, ros_cuda, session
from xblick.camera import CameraPublisher, ImageSubscriber
from xblick.gpu import CudaImage
from xblick.msg import CameraInfo

IMAGE = os.environ.get("XBLICK_ROS_CUDA_IMAGE", "")
OVERLAY = os.environ.get("XBLICK_ROS_CUDA_OVERLAY", "")
ISAAC = os.environ.get("XBLICK_ROS_ISAAC_IMAGE", "")
HERE = os.path.dirname(os.path.abspath(__file__))
DOMAIN = 79
LYRICAL = "/opt/ros/lyrical"

pytestmark = pytest.mark.skipif(shutil.which("docker") is None or _core.cuda_device_count() == 0,
                                reason="needs docker and a CUDA device")
needs_image = pytest.mark.skipif(not IMAGE,
                                 reason="XBLICK_ROS_CUDA_IMAGE (Lyrical with cuda_buffer_backend) is not set")
needs_isaac = pytest.mark.skipif(not ISAAC, reason="XBLICK_ROS_ISAAC_IMAGE (Isaac ROS 5.1 on Lyrical) is not set")


def free_port():
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def gpu_option() -> str:
    """docker's --gpus for the GPU this process calls 0 (by UUID: CUDA_VISIBLE_DEVICES numbers otherwise)."""
    h = _core.cuda_device_uuid(0)
    return f'"device=GPU-{h[:8]}-{h[8:12]}-{h[12:16]}-{h[16:20]}-{h[20:]}"'


class Box:
    """A container on this host's network and pid namespace, as this user, with a /dev/shm of its own (``shm``, as
    this process reaches it) and ``tmp``
    at its own path: ``start`` a program in it (its output into ``<tmp>/<name>.log``), ``run`` one to its end."""

    def __init__(self, image: str, tmp, port: int, user: str | None = None):
        self.tmp = tmp
        self.name = f"xblick-test-gpu-{uuid.uuid4().hex[:8]}"
        mounts = ["--shm-size", "256m", "-v", f"{tmp}:{tmp}", "-v", f"{HERE}:{HERE}:ro"]
        if OVERLAY:
            mounts += ["-v", f"{OVERLAY}:{OVERLAY}:ro"]
        subprocess.run(["docker", "run", "-d", "--rm", "--name", self.name, "--network", "host", "--pid", "host",
                        "--user", user or f"{os.getuid()}:{os.getgid()}", "--gpus", gpu_option(), *mounts,
                        "-e", "HOME=/tmp", "-e", "ROS_LOG_DIR=/tmp/roslog", "-e", f"ROS_DOMAIN_ID={DOMAIN}",
                        "-e", "RMW_IMPLEMENTATION=rmw_zenoh_cpp", "-e", "RCUTILS_LOGGING_BUFFERED_STREAM=0",
                        image, "sleep", "infinity"], check=True, capture_output=True, timeout=120)
        self.port = port
        pid = subprocess.run(["docker", "inspect", "-f", "{{.State.Pid}}", self.name], check=True, capture_output=True,
                             text=True, timeout=30).stdout.strip()
        self.shm = f"/proc/{pid}/root/dev/shm"

    def env(self, overlay: str = "", router: bool = False) -> str:
        endpoint = "listen" if router else "connect"
        text = (f"source {LYRICAL}/setup.bash && "
                + (f"source {overlay}/setup.bash && " if overlay else "")
                + f"export ZENOH_CONFIG_OVERRIDE='{endpoint}/endpoints=[\"tcp/127.0.0.1:{self.port}\"];"
                  "scouting/multicast/enabled=false' && ")
        return text

    def run(self, command: str, timeout: float, overlay: str = "") -> subprocess.CompletedProcess:
        return subprocess.run(["docker", "exec", self.name, "bash", "-c", self.env(overlay) + command],
                              capture_output=True, text=True, timeout=timeout)

    def start(self, command: str, log: str, overlay: str = "", router: bool = False):
        subprocess.run(["docker", "exec", "-d", self.name, "bash", "-c",
                        self.env(overlay, router) + f"exec {command} > {self.tmp}/{log} 2>&1"], check=True, timeout=30)

    def log(self, name: str) -> str:
        path = self.tmp / name
        return path.read_text(errors="replace") if path.exists() else ""

    def close(self):
        subprocess.run(["docker", "rm", "-f", self.name], capture_output=True, timeout=60)


def wait_for_port(port: int, timeout: float = 30):
    end = time.monotonic() + timeout
    while True:
        try:
            socket.create_connection(("127.0.0.1", port), timeout=0.2).close()
            return
        except OSError:
            if time.monotonic() > end:
                raise
            time.sleep(0.1)


@pytest.fixture
def tmp(tmp_path, monkeypatch):
    monkeypatch.setenv(ros_cuda.SOCKETS_ENV, str(tmp_path))
    return tmp_path


def box_with_router(image, tmp, monkeypatch):
    """A box with rmw_zenohd; the link's records go to its /dev/shm."""
    port = free_port()
    box = Box(image, tmp, port)
    monkeypatch.setenv(ros_cuda.SHM_ENV, box.shm)
    box.start("ros2 run rmw_zenoh_cpp rmw_zenohd", "router.log", router=True)
    wait_for_port(port)
    return box


def pattern(n: int, seed: int) -> np.ndarray:
    return ((np.arange(n, dtype=np.uint64) * 13 + seed) & 0xFF).astype(np.uint8)


def publish_pattern(camera, height, width, seconds, rate=20.0, until=None):
    """Frames of the test pattern from the GPU, frame id ``xb:<seed>``, for ``seconds`` or until ``until()``."""
    buffer = _core.make_device_buffer(0, height * width * 3)
    end, seed = time.monotonic() + seconds, 1000
    while time.monotonic() < end and not (until and until()):
        buffer.upload(pattern(height * width * 3, seed))
        camera.frame_id = f"xb:{seed}"
        camera.publish(CudaImage(buffer.ptr, (height, width, 3), np.uint8, 0, owner=buffer))
        seed += 1
        time.sleep(1 / rate)


def checks(text: str) -> list[dict]:
    lines = []
    for line in text.splitlines():
        if line.startswith("CUDACHECK ") and "start" not in line:
            fields = dict(p.split("=", 1) for p in line.split()[2:] if "=" in p)
            fields["ok"] = line.split()[-1] == "OK" or " OK" in line
            lines.append(fields)
    return lines


def build_check(box, overlay: str) -> str:
    """cuda_check built in the box (against the overlay's backend when given); its install space."""
    out = box.tmp / ("ws-overlay" if overlay else "ws")
    done = box.run(f"cd /tmp && colcon build --base-paths {HERE}/ros/cuda_check --build-base {out}/build "
                   f"--install-base {out}/install --event-handlers console_direct-", 600, overlay)
    assert done.returncode == 0, (done.stdout + done.stderr)[-3000:]
    return f"{out}/install"


@needs_image
@pytest.mark.parametrize("variant", ["release", "overlay"])
def test_stock_reader_gets_cuda_buffers(tmp, monkeypatch, variant):
    if variant == "overlay" and not OVERLAY:
        pytest.skip("XBLICK_ROS_CUDA_OVERLAY (another build of the backend) is not set")
    overlay = OVERLAY if variant == "overlay" else ""
    box = box_with_router(IMAGE, tmp, monkeypatch)
    camera = None
    try:
        install = build_check(box, overlay)
        with session.open_session(connect=[f"tcp/127.0.0.1:{box.port}"],
                                  ros_options=session.RosOptions(DOMAIN, "xblick_gpu_test")) as s:
            camera = CameraPublisher(s, "xbcuda/cam/color", encoding="rgb8", frame_id="xb:0", cuda_device=0)
            box.start(f"bash -c 'source {install}/setup.bash && exec ros2 run xblick_cuda_check cuda_check "
                      "--ros-args -p topic:=/xbcuda/cam/color/image_raw'", "check.log", overlay)
            on_gpu = lambda: sum(c.get("backend") == "cuda" for c in checks(box.log("check.log"))) >= 40
            publish_pattern(camera, 1080, 1920, 40, until=on_gpu)
            time.sleep(1)
            stats = camera._ros_gpu.stats()
            camera.close()
            camera = None
        lines = checks(box.log("check.log"))
        gpu = [c for c in lines if c.get("backend") == "cuda"]
        assert len(gpu) >= 40, (stats, box.log("check.log")[-3000:])
        assert all(c["ok"] for c in lines), [c for c in lines if not c["ok"]][:5]  # CPU ones before it was known too
        assert all(c["size"] == str(1920 * 1080 * 3) for c in gpu)
        assert stats["accepts"] >= 1 and stats["refused"] == 0 and stats["dropped"] == 0, stats
        assert "Dropping stale descriptor" not in box.log("check.log")
        assert [p for p in os.listdir(box.shm) if p.startswith(f"cuda_vmm_{os.getpid()}_")] == []  # ours gone
    finally:
        if camera is not None:
            camera.close()
        box.close()
    assert [p for p in os.listdir(tmp) if p.startswith("xblick_cuda_")] == []


@needs_image
def test_a_reader_that_holds_makes_the_pool_drop(tmp, monkeypatch):
    """The reader keeps each message 1 s (below the hold limit): the pool's blocks (its queue depth 10, one more, the
    spare ones) are all held soon, frames for it are dropped and counted, none waits, and every frame it gets is
    right."""
    box = box_with_router(IMAGE, tmp, monkeypatch)
    camera = None
    try:
        install = build_check(box, "")
        with session.open_session(connect=[f"tcp/127.0.0.1:{box.port}"],
                                  ros_options=session.RosOptions(DOMAIN, "xblick_gpu_test")) as s:
            camera = CameraPublisher(s, "xbhold/cam/color", encoding="rgb8", frame_id="xb:0", cuda_device=0)
            box.start(f"bash -c 'source {install}/setup.bash && exec ros2 run xblick_cuda_check cuda_check "
                      "--ros-args -p topic:=/xbhold/cam/color/image_raw -p hold_ms:=1000'", "check.log")
            publish_pattern(camera, 480, 640, 8, rate=30)
            stats = camera._ros_gpu.stats()
            camera.close()
            camera = None
        lines = checks(box.log("check.log"))
        assert stats["dropped"] > 0 and stats["expired"] == 0, stats
        assert stats["blocks"] == 10 + 1 + ros_cuda.SPARE_BLOCKS, stats
        assert sum(c.get("backend") == "cuda" for c in lines) >= 8 and all(c["ok"] for c in lines), lines[-5:]
    finally:
        if camera is not None:
            camera.close()
        box.close()


@needs_image
def test_a_peer_of_another_user_gets_no_block(tmp):
    pool = _core.RosCudaPool(0, str(tmp), str(tmp))
    descriptor = pool.publish(pool.acquire(4096, 0), 4096)
    path = descriptor["socket_path"]
    os.chmod(path, 0o666)  # so that the other user reaches it at all: the check is SO_PEERCRED's
    os.chmod(tmp, 0o755)
    script = ("import socket\n"
              f"for a in ({path!r}, '\\0' + {path!r}):\n"
              "    s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM); s.settimeout(5); s.connect(a)\n"
              "    data, fds, _, _ = socket.recv_fds(s, 1, 1)\n"
              "    print('GOT', len(data), len(fds))\n")
    box = Box(IMAGE, tmp, 0, user="65534:65534")
    try:
        done = box.run(f"python3 -c \"{script}\"", 60)
    finally:
        box.close()
    assert done.stdout.split("\n")[:2] == ["GOT 0 0", "GOT 0 0"], done.stdout + done.stderr
    assert pool.refused == 2 and pool.accepts == 0
    del pool


def blocks_image(height: int, width: int, block: int, seed: int) -> np.ndarray:
    colours = np.random.default_rng(seed).integers(0, 256, (height // block, width // block, 3), dtype=np.uint8)
    return np.repeat(np.repeat(colours, block, axis=0), block, axis=1)


@needs_isaac
def test_isaac_resize_reads_our_gpu_frames(tmp, monkeypatch):
    """Isaac ROS 5.1's ResizeNode (isaac_ros_image_proc) takes our GPU frames as descriptors (it fetched the blocks,
    and its log has no failed import) and halves them right: blocks of 16 x 16 pixels in one colour each are blocks
    of 8 x 8 of the same colour."""
    box = box_with_router(ISAAC, tmp, monkeypatch)
    camera = reader = None
    h, w = 480, 640
    k = [500.0, 0, w / 2 - 0.5, 0, 500.0, h / 2 - 0.5, 0, 0, 1]
    info = CameraInfo(height=h, width=w, distortion_model="plumb_bob", d=np.zeros(5), k=np.array(k),
                      r=np.eye(3).reshape(-1), p=np.array(k[:3] + [0] + k[3:6] + [0] + k[6:] + [0]))
    try:
        box.start("ros2 run rclcpp_components component_container --ros-args -r __node:=xbc", "container.log")
        time.sleep(4)
        load = box.run("timeout 60 ros2 component load /xbc isaac_ros_image_proc nvidia::isaac_ros::image_proc::"
                       "ResizeNode -n xbresize -r image:=/xbisaac/cam/color/image_raw -r camera_info:=/xbisaac/cam/"
                       "color/camera_info -r resize/image:=/xbisaac/out/image_raw -r resize/camera_info:=/xbisaac/out/"
                       "camera_info -p output_width:=320 -p output_height:=240", 90)
        assert load.returncode == 0, load.stdout + load.stderr + box.log("container.log")[-2000:]
        with session.open_session(connect=[f"tcp/127.0.0.1:{box.port}"],
                                  ros_options=session.RosOptions(DOMAIN, "xblick_gpu_test")) as s:
            camera = CameraPublisher(s, "xbisaac/cam/color", encoding="rgb8", frame_id="xb", camera_info=info,
                                     cuda_device=0)
            reader = ImageSubscriber(s, "xbisaac/out", transport="ros")
            buffer = _core.make_device_buffer(0, h * w * 3)
            sent, got = {}, []
            end = time.monotonic() + 60
            seed = 0
            while time.monotonic() < end and len(got) < 30:
                image = blocks_image(h, w, 16, seed)
                buffer.upload(image)
                stamp = time.time_ns()
                sent[stamp] = image
                camera.publish(CudaImage(buffer.ptr, (h, w, 3), np.uint8, 0, owner=buffer), stamp_ns=stamp)
                seed += 1
                frame = reader.latest(timeout=0.05)
                if frame is not None and frame.stamp_ns in sent and camera._ros_gpu.stats()["sent"] > 0:
                    got.append((frame, sent[frame.stamp_ns]))
                time.sleep(0.05)
            stats = camera._ros_gpu.stats()
        log = box.log("container.log")
        assert stats["sent"] > 0 and stats["accepts"] >= 1, (stats, log[-3000:])
        assert "Dropping stale descriptor" not in log and "IPC import failed" not in log, log[-3000:]
        assert len(got) >= 10, (len(got), stats, log[-2000:])
        for frame, image in got[-10:]:
            assert frame.image.shape == (240, 320, 3)
            assert np.array_equal(frame.image[4::8, 4::8], image[8::16, 8::16])  # each block's middle
    finally:
        if reader is not None:
            reader.close()
        if camera is not None:
            camera.close()
        box.close()
