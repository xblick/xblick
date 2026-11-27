"""The GPU link (ros_cuda.py) against ROS 2's own programs in docker containers on this GPU: a stock Lyrical
subscriber that accepts the CUDA buffer backend (tests/ros/cuda_check, built here) gets our GPU frames as CUDA buffers
with the right pixels, with the backend's release and, given an overlay, another build of it; a reader that holds its
messages makes the pool drop rather than wait; a peer of another Linux user gets no block; and Isaac ROS's
ResizeNode reads our GPU frames as descriptors and resizes them right. The C++ CameraPublisher (xblick-cpp-camera,
with XBLICK_CPP_BUILD) gives the stock reader its frames as the Python one does. Two stock readers at once get CUDA
buffers each; one on another GPU gets the bytes.

The other direction: a stock publisher with the backend (tests/ros/cuda_check's cuda_pattern_pub, the smallest rclcpp
program that publishes CUDA buffers: no stock node in the images does) gives an ImageSubscriber with a device (Python,
and C++ through xblick-cpp-probe) its frames as CUDA buffers on our GPU; a reader on the CPU, on another GPU or with
another /dev/shm gets the bytes, and Isaac ROS's ResizeNode, which publishes from host memory, the bytes too.

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

import json
import os
import re
import shutil
import socket
import struct
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


def gpu_option(ordinal: int = 0) -> str:
    """docker's --gpus for the GPU this process calls ``ordinal`` (by UUID: CUDA_VISIBLE_DEVICES numbers otherwise)."""
    h = _core.cuda_device_uuid(ordinal)
    return f'"device=GPU-{h[:8]}-{h[8:12]}-{h[12:16]}-{h[16:20]}-{h[20:]}"'


class Box:
    """A container on this host's network and pid namespace, as this user, with a /dev/shm of its own (``shm``, as
    this process reaches it) and ``tmp``
    at its own path: ``start`` a program in it (its output into ``<tmp>/<name>.log``), ``run`` one to its end.
    ``gpu``: the GPU it sees (this process's ordinal); ``share_tmp``: this host's /tmp as its /tmp (release 0.1.2's
    publishers put their socket files there), its home and ROS logs then in ``tmp``; ``ipc_host``: this host's
    /dev/shm (a ROS publisher's CUDA events are there, which our reader opens)."""

    def __init__(self, image: str, tmp, port: int, user: str | None = None, gpu: int = 0, share_tmp: bool = False,
                 ipc_host: bool = False):
        self.tmp = tmp
        self.name = f"xblick-test-gpu-{uuid.uuid4().hex[:8]}"
        mounts = (["--ipc", "host"] if ipc_host else ["--shm-size", "256m"]) + ["-v", f"{tmp}:{tmp}", "-v",
                                                                                 f"{HERE}:{HERE}:ro"]
        if OVERLAY:
            mounts += ["-v", f"{OVERLAY}:{OVERLAY}:ro"]
        home = "/tmp"
        if share_tmp:
            mounts += ["-v", "/tmp:/tmp"]
            home = str(tmp)
        subprocess.run(["docker", "run", "-d", "--rm", "--name", self.name, "--network", "host", "--pid", "host",
                        "--user", user or f"{os.getuid()}:{os.getgid()}", "--gpus", gpu_option(gpu), *mounts,
                        "-e", f"HOME={home}", "-e", f"ROS_LOG_DIR={home}/roslog", "-e", f"ROS_DOMAIN_ID={DOMAIN}",
                        "-e", "RMW_IMPLEMENTATION=rmw_zenoh_cpp", "-e", "RCUTILS_LOGGING_BUFFERED_STREAM=0",
                        image, "sleep", "infinity"], check=True, capture_output=True, timeout=120)
        self.port = port
        pid = subprocess.run(["docker", "inspect", "-f", "{{.State.Pid}}", self.name], check=True, capture_output=True,
                             text=True, timeout=30).stdout.strip()
        self.shm = "/dev/shm" if ipc_host else f"/proc/{pid}/root/dev/shm"

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
        """Starts `command`; its pid (this host's: the box shares the pid namespace) in <log>.pid."""
        subprocess.run(["docker", "exec", "-d", self.name, "bash", "-c",
                        self.env(overlay, router) + f"echo $$ > {self.tmp}/{log}.pid; "
                        f"exec {command} > {self.tmp}/{log} 2>&1"], check=True, timeout=30)
        self.logs = getattr(self, "logs", []) + [log]

    def log(self, name: str) -> str:
        path = self.tmp / name
        return path.read_text(errors="replace") if path.exists() else ""

    def close(self):
        """Ends the programs it started by SIGINT first (ROS nodes then remove the backend's files they made, also in
        a /dev/shm shared with this host), at most 10 s, then the box."""
        pids = []
        for log in getattr(self, "logs", []):
            path = self.tmp / f"{log}.pid"
            if path.exists() and os.path.exists(f"/proc/{int(path.read_text())}"):
                pids.append(int(path.read_text()))
        for pid in pids:
            try:
                os.kill(pid, 2)
            except ProcessLookupError:
                pass
        end = time.monotonic() + 10
        while any(os.path.exists(f"/proc/{pid}") for pid in pids) and time.monotonic() < end:
            time.sleep(0.1)
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


def box_with_router(image, tmp, monkeypatch, gpu: int = 0, share_tmp: bool = False, ipc_host: bool = False):
    """A box with rmw_zenohd; the link's records go to its /dev/shm (and are read there)."""
    port = free_port()
    box = Box(image, tmp, port, gpu=gpu, share_tmp=share_tmp, ipc_host=ipc_host)
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
    done = box.run(f"cd {box.tmp} && colcon build --base-paths {HERE}/ros/cuda_check --build-base {out}/build "
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
def test_stock_reader_gets_cuda_buffers_from_the_cpp_camera(tmp, monkeypatch):
    """The C++ CameraPublisher (xblick-cpp-camera --check-pattern, device frames) to the stock reader: the same as
    from the Python one. Needs XBLICK_CPP_BUILD (as tests/test_cpp_interop.py)."""
    from test_cpp_interop import BUILD, tool
    if not BUILD:
        pytest.skip("XBLICK_CPP_BUILD (the C++ build directory) is not set")
    box = box_with_router(IMAGE, tmp, monkeypatch)
    camera = None
    w, h = 1280, 720
    try:
        install = build_check(box, "")
        camera = subprocess.Popen([tool("xblick-cpp-camera"), "--connect", f"127.0.0.1:{box.port}", "--base",
                                   "xbcpp/cam/color", "--encoding", "rgb8", "--size", str(w), str(h), "--fps", "20",
                                   "--seconds", "60", "--cuda-device", "0", "--device-frames", "--check-pattern",
                                   "--ros-domain", str(DOMAIN)],
                                  stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, env=dict(os.environ))
        assert camera.stdout.readline().strip() == "ready"
        box.start(f"bash -c 'source {install}/setup.bash && exec ros2 run xblick_cuda_check cuda_check "
                  "--ros-args -p topic:=/xbcpp/cam/color/image_raw'", "check.log")
        end = time.monotonic() + 40
        while time.monotonic() < end and sum(c.get("backend") == "cuda" for c in checks(box.log("check.log"))) < 40:
            time.sleep(0.2)
        camera.terminate()
        _, err = camera.communicate(timeout=20)
        lines = checks(box.log("check.log"))
        gpu = [c for c in lines if c.get("backend") == "cuda"]
        assert camera.returncode == 0, err
        stats = json.loads(err.split("xblick-cpp-camera: gpu link ")[1].splitlines()[0])
        assert len(gpu) >= 40, (stats, box.log("check.log")[-3000:])
        assert all(c["ok"] for c in lines), [c for c in lines if not c["ok"]][:5]
        assert all(c["size"] == str(w * h * 3) for c in gpu)
        assert stats["accepts"] >= 1 and stats["refused"] == 0 and stats["dropped"] == 0, stats
        assert "Dropping stale descriptor" not in box.log("check.log")
        assert [p for p in os.listdir(box.shm) if p.startswith(f"cuda_vmm_{camera.pid}_")] == []  # its records gone
    finally:
        if camera is not None and camera.poll() is None:
            camera.kill()
            camera.communicate(timeout=10)
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
@pytest.mark.parametrize("device", [None, 0], ids=["cpu-reader", "gpu-reader"])
def test_isaac_resize_reads_our_gpu_frames(tmp, monkeypatch, device):
    """Isaac ROS 5.1's ResizeNode (isaac_ros_image_proc) takes our GPU frames as descriptors (it fetched the blocks,
    and its log has no failed import) and halves them right: blocks of 16 x 16 pixels in one colour each are blocks
    of 8 x 8 of the same colour. Its output comes as bytes, to our reader on the CPU and to one with a device."""
    box = box_with_router(ISAAC, tmp, monkeypatch, ipc_host=device is not None)  # its events, for our GPU reader
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
            reader = ImageSubscriber(s, "xbisaac/out", transport="ros", device=device)
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
            received = reader.ros_gpu_stats()
            reader.close()  # with the session open
            camera.close()
            reader = camera = None
        log = box.log("container.log")
        assert stats["sent"] > 0 and stats["accepts"] >= 1, (stats, log[-3000:])
        assert "Dropping stale descriptor" not in log and "IPC import failed" not in log, log[-3000:]
        assert len(got) >= 10, (len(got), stats, received, log[-2000:])
        # the ResizeNode of Isaac ROS 5.1 publishes its output from host memory: the bytes, to a reader with a device
        # as well (entered in the registry as one that takes CUDA buffers, nothing refused)
        assert not any(isinstance(f.image, CudaImage) for f, _ in got)
        if device is None:
            assert received is None
        else:
            assert received["registered"] and received["refused"] == 0 and received["copied"] == 0, received
        for frame, image in got[-10:]:
            out = frame.image.numpy() if isinstance(frame.image, CudaImage) else frame.image
            assert out.shape == (240, 320, 3)
            assert np.array_equal(out[4::8, 4::8], image[8::16, 8::16])  # each block's middle
    finally:
        if reader is not None:
            reader.close()
        if camera is not None:
            camera.close()
        box.close()


needs_two_gpus = pytest.mark.skipif(_core.cuda_device_count() < 2, reason="needs two CUDA devices")


def ros_session(box):
    return session.open_session(connect=[f"tcp/127.0.0.1:{box.port}"],
                                ros_options=session.RosOptions(DOMAIN, "xblick_gpu_test"))


def start_checks(box, install, topic, names):
    """A stock CUDA reader (cuda_check) per name, each a node of its own; their logs check_<name>.log."""
    for name in names:
        box.start(f"bash -c 'source {install}/setup.bash && exec ros2 run xblick_cuda_check cuda_check "
                  f"--ros-args -r __node:=cuda_check_{name} -p topic:={topic}'", f"check_{name}.log")


def start_pattern_publisher(box, install, topic, overlay="", name="pub", width=1280, height=720) -> int:
    """A stock CUDA publisher (cuda_pattern_pub) on `topic`, its log <name>.log; its pid (this host's)."""
    box.start(f"bash -c 'source {install}/setup.bash && exec ros2 run xblick_cuda_check cuda_pattern_pub --ros-args "
              f"-p topic:={topic} -p width:={width} -p height:={height} -p rate_ms:=50'", f"{name}.log", overlay)
    end = time.monotonic() + 30
    while (found := re.search(r"CUDAPUB start pid=(\d+)", box.log(f"{name}.log"))) is None:
        assert time.monotonic() < end, box.log(f"{name}.log")[-2000:]
        time.sleep(0.2)
    return int(found.group(1))


def stop_publisher(box, pid: int) -> list[str]:
    """Ends the publisher (SIGINT: it removes its blocks' files), then removes what it left, named after its pid: socket
    files of release 0.1.2 in this host's /tmp, block records in the box's /dev/shm. The records it left."""
    try:
        os.kill(pid, 2)  # a process this test started (in the box, which shares this host's pids)
    except ProcessLookupError:
        pass
    end = time.monotonic() + 10
    while os.path.exists(f"/proc/{pid}") and time.monotonic() < end:
        time.sleep(0.1)
    for name in os.listdir("/tmp"):
        if name.startswith(f"cuda_vmm_{pid}_") and name.endswith(".sock"):
            os.unlink(f"/tmp/{name}")
    left = [name for name in os.listdir(box.shm) if name.startswith(f"cuda_vmm_{pid}_")]
    for name in left:
        os.unlink(f"{box.shm}/{name}")
    return left


def active_entries(registry: str) -> list:
    """(pid, entity type, name) of the active entries of the backend's registry file (for a failure's message)."""
    try:
        with open(registry, "rb") as f:
            data = f.read()
    except OSError as error:
        return [str(error)]
    out = []
    for i in range(min(int.from_bytes(data[264:268], "little"), 512)):
        instance, _, kind, _, _, active, _, name = struct.unpack_from("<Q16sB3xiI??2x256s", data, 272 + i * 296)
        if active:
            out.append((instance >> 32, kind, name.split(b"\0")[0].decode(errors="replace")))
    return out


def mismatches(image, frame_id: str) -> int:
    flat = np.asarray(image).reshape(-1)
    return int(np.count_nonzero(flat != pattern(flat.size, int(frame_id.split(":")[1]))))


@needs_image
@pytest.mark.parametrize("variant", ["release", "overlay"])
def test_our_gpu_reader_takes_the_cuda_buffers_of_a_stock_publisher(tmp, monkeypatch, variant):
    """A stock publisher with the backend (cuda_pattern_pub: allocate_buffer, a kernel on its stream) to an
    ImageSubscriber of its topic with a device: its frames come as CudaImage on our GPU with the right pixels, read
    under the backend's hold (the publisher's pool stays small: blocks come back), and a subscriber on the CPU of the
    same topic gets the bytes. Our registry entry goes inactive with the subscriber. Release 0.1.2 serves its blocks
    by socket files in /tmp (shared with the box here), the current source (the overlay) by abstract sockets."""
    if variant == "overlay" and not OVERLAY:
        pytest.skip("XBLICK_ROS_CUDA_OVERLAY (another build of the backend) is not set")
    overlay = OVERLAY if variant == "overlay" else ""
    registry = f"/dev/shm/{_core.ros_cuda_registry_name()}"
    there_before = os.path.exists(registry)
    box = box_with_router(IMAGE, tmp, monkeypatch, share_tmp=variant == "release", ipc_host=True)
    pub = gpu = cpu = None
    try:
        install = build_check(box, overlay)
        with ros_session(box) as s:
            gpu = ImageSubscriber(s, "/xbrev/image", transport="auto", device=0)
            cpu = ImageSubscriber(s, "/xbrev/image", transport="auto")
            pub = start_pattern_publisher(box, install, "/xbrev/image", overlay)
            on_gpu, on_cpu, wrong = 0, 0, []
            end = time.monotonic() + 60
            while time.monotonic() < end and (on_gpu < 40 or on_cpu < 5):
                frame = gpu.latest(timeout=0.05)
                if frame is not None:
                    is_gpu = isinstance(frame.image, CudaImage)
                    on_gpu += is_gpu
                    bad = mismatches(frame.image.numpy() if is_gpu else frame.image, frame.header.frame_id)
                    if bad:
                        wrong.append((frame.header.frame_id, is_gpu, bad))
                frame = cpu.latest()
                if frame is not None:
                    assert not isinstance(frame.image, CudaImage)
                    on_cpu += 1
                    if mismatches(frame.image, frame.header.frame_id):
                        wrong.append((frame.header.frame_id, "cpu"))
            stats = gpu.ros_gpu_stats()
            blocks = [p for p in os.listdir(box.shm) if p.startswith(f"cuda_vmm_{pub}_")]
            gid = gpu._subscriber.gid
            registered = _core.ros_cuda_registry_lookup(box.shm, gid)
            gpu.close()
            cpu.close()
            gpu = cpu = None
            assert _core.ros_cuda_registry_lookup(box.shm, gid) is None
        log = box.log("pub.log")
        assert on_gpu >= 40 and on_cpu >= 5, (on_gpu, on_cpu, stats, log[-2000:])
        assert wrong == [], wrong[:5]
        assert stats["refused"] == 0 and stats["registered"] and stats["copied"] >= 40, stats
        assert registered is not None and registered["pid"] == os.getpid() and registered["entity_type"] == 1
        assert len(blocks) <= 10, blocks  # released after each copy: the stock pool reuses its blocks
        assert "backend=cuda" in log
        stopped, pub = pub, None
        left = stop_publisher(box, stopped)
        assert left == [], (left, box.log("pub.log")[-1000:])  # its pool's records went with it (no count held)
        # none of the registry's active entries is ours; release 0.1.2 leaves its publisher's entry active at its
        # exit, so a registry this test's publisher made is removed here when that is all it holds
        active = active_entries(registry) if os.path.exists(registry) else []
        assert all(entry[0] != os.getpid() for entry in active), active
        if not there_before and active and all(entry[0] == stopped for entry in active):
            for path in (registry, f"/dev/shm/sem.{_core.ros_cuda_registry_name()}_mtx"):
                os.unlink(path)
    finally:
        for sub in (gpu, cpu):
            if sub is not None:
                sub.close()
        if pub is not None:
            stop_publisher(box, pub)
        box.close()


@needs_image
def test_our_gpu_reader_with_another_dev_shm_gets_a_stock_publishers_bytes(tmp, monkeypatch, caplog):
    """A stock publisher in a container with a /dev/shm of its own: the CUDA events of its buffers cannot be opened
    here, so our reader does not enter the registry, gets the bytes (all right) and says why in one line."""
    box = box_with_router(IMAGE, tmp, monkeypatch)
    pub = gpu = None
    try:
        install = build_check(box, "")
        with caplog.at_level("INFO", logger="xblick"), ros_session(box) as s:
            gpu = ImageSubscriber(s, "/xbshm/image", transport="auto", device=0)
            pub = start_pattern_publisher(box, install, "/xbshm/image", width=640, height=480)
            frames, end = [], time.monotonic() + 40
            while len(frames) < 20 and time.monotonic() < end:
                frame = gpu.latest(timeout=0.1)
                if frame is not None:
                    frames.append((isinstance(frame.image, CudaImage), mismatches(frame.image, frame.header.frame_id)))
            stats = gpu.ros_gpu_stats()
            entry = _core.ros_cuda_registry_lookup(box.shm, gpu._subscriber.gid)
            gpu.close()
            gpu = None
        assert len(frames) >= 20 and all(not on_gpu and bad == 0 for on_gpu, bad in frames), frames[:5]
        assert stats["registered"] is False and stats["copied"] == 0, stats
        assert entry is not None and entry["ipc_capable"] is False  # in the box's registry: "the CPU form, please"
        assert "it has another /dev/shm than this program" in caplog.text, caplog.text[-2000:]
    finally:
        if gpu is not None:
            gpu.close()
        if pub is not None:
            stop_publisher(box, pub)
        box.close()


@needs_image
def test_cpp_gpu_reader_takes_the_cuda_buffers_of_a_stock_publisher(tmp, monkeypatch):
    """The C++ ImageSubscriber with a cuda_device (xblick-cpp-probe --cuda-device 0) as the Python one: the stock
    publisher's frames on our GPU with the right pixels. Needs XBLICK_CPP_BUILD."""
    from test_cpp_interop import BUILD, tool
    if not BUILD:
        pytest.skip("XBLICK_CPP_BUILD (the C++ build directory) is not set")
    box = box_with_router(IMAGE, tmp, monkeypatch, share_tmp=True, ipc_host=True)
    pub = None
    try:
        install = build_check(box, "")
        pub = start_pattern_publisher(box, install, "/xbcpprev/image")
        done = subprocess.run([tool("xblick-cpp-probe"), "--connect", f"127.0.0.1:{box.port}", "--base",
                               "/xbcpprev/image", "--cuda-device", "0", "--check-pattern", "--frames", "80",
                               "--timeout", "60", "--ros-domain", str(DOMAIN)],
                              check=False, capture_output=True, text=True, timeout=120, env=dict(os.environ))
        assert done.returncode == 0, done.stderr[-3000:]
        lines = [json.loads(line) for line in done.stdout.splitlines()]
        on_gpu = [line for line in lines if line["device"] >= 0]
        stats = json.loads(done.stderr.split("xblick-cpp-probe: ros gpu ")[1].splitlines()[0])
        assert len(on_gpu) >= 40, (len(on_gpu), stats, done.stderr[-2000:])
        assert all(line["mismatches"] == 0 for line in lines), [line for line in lines if line["mismatches"]][:5]
        assert stats["refused"] == 0 and stats["registered"], stats
    finally:
        if pub is not None:
            stop_publisher(box, pub)
        box.close()


@needs_image
def test_two_stock_readers_get_our_cuda_buffers_at_once(tmp, monkeypatch):
    """Two stock CUDA readers of one topic (two processes): each gets our GPU frames as CUDA buffers, all right."""
    box = box_with_router(IMAGE, tmp, monkeypatch)
    camera = None
    try:
        install = build_check(box, "")
        with ros_session(box) as s:
            camera = CameraPublisher(s, "xbtwo/cam/color", encoding="rgb8", frame_id="xb:0", cuda_device=0)
            start_checks(box, install, "/xbtwo/cam/color/image_raw", ["a", "b"])
            on_gpu = lambda: all(sum(c.get("backend") == "cuda" for c in checks(box.log(f"check_{n}.log"))) >= 40
                                 for n in "ab")
            publish_pattern(camera, 720, 1280, 60, until=on_gpu)
            time.sleep(1)
            stats = camera._ros_gpu.stats()
            camera.close()
            camera = None
        for name in "ab":
            lines = checks(box.log(f"check_{name}.log"))
            assert sum(c.get("backend") == "cuda" for c in lines) >= 40, (name, stats, lines[-3:])
            assert all(c["ok"] for c in lines), [c for c in lines if not c["ok"]][:5]
            assert "Dropping stale descriptor" not in box.log(f"check_{name}.log")
        assert stats["refused"] == 0 and stats["dropped"] == 0 and stats["accepts"] >= 2, stats
    finally:
        if camera is not None:
            camera.close()
        box.close()


@needs_image
@needs_two_gpus
def test_a_stock_reader_on_another_gpu_gets_our_frames_as_bytes(tmp, monkeypatch, caplog):
    """A stock CUDA reader that sees another GPU than our publisher's: the CPU form, all right, and one line that
    says why; no descriptor that it could not import."""
    box = box_with_router(IMAGE, tmp, monkeypatch, gpu=1)
    camera = None
    try:
        install = build_check(box, "")
        with caplog.at_level("INFO", logger="xblick"), ros_session(box) as s:
            camera = CameraPublisher(s, "xbother/cam/color", encoding="rgb8", frame_id="xb:0", cuda_device=0)
            start_checks(box, install, "/xbother/cam/color/image_raw", ["a"])
            enough = lambda: len(checks(box.log("check_a.log"))) >= 40
            publish_pattern(camera, 480, 640, 40, until=enough)
            stats = camera._ros_gpu.stats()
            camera.close()
            camera = None
        lines = checks(box.log("check_a.log"))
        assert len(lines) >= 40 and all(c["ok"] and c.get("backend") == "cpu" for c in lines), lines[-3:]
        assert "IPC import failed" not in box.log("check_a.log")
        assert stats["sent"] == 0 and stats["blocks"] == 0, stats
        assert "gets the CPU form: it uses GPU /dev/nvidia" in caplog.text, caplog.text[-2000:]
    finally:
        if camera is not None:
            camera.close()
        box.close()


@needs_image
@needs_two_gpus
def test_our_gpu_reader_on_another_gpu_gets_a_stock_publishers_bytes(tmp, monkeypatch, caplog):
    """A stock CUDA publisher on another GPU than our reader's: our reader is not entered in the registry, gets the
    CPU form (arrays in host memory), all right, and one line says why."""
    box = box_with_router(IMAGE, tmp, monkeypatch, gpu=1, ipc_host=True)
    pub = gpu = None
    try:
        install = build_check(box, "")
        with caplog.at_level("INFO", logger="xblick"), ros_session(box) as s:
            gpu = ImageSubscriber(s, "/xbother/image", transport="auto", device=0)
            pub = start_pattern_publisher(box, install, "/xbother/image", width=640, height=480)
            frames, end = [], time.monotonic() + 40
            while len(frames) < 20 and time.monotonic() < end:
                frame = gpu.latest(timeout=0.1)
                if frame is not None:
                    frames.append((frame.header.frame_id, isinstance(frame.image, CudaImage),
                                   mismatches(frame.image, frame.header.frame_id)))
            stats = gpu.ros_gpu_stats()
            gpu.close()
            gpu = None
        assert len(frames) >= 20 and all(not on_gpu and bad == 0 for _, on_gpu, bad in frames), frames[:5]
        assert stats["registered"] is False and stats["copied"] == 0, stats
        assert "sends the CPU form: it uses GPU /dev/nvidia" in caplog.text, caplog.text[-2000:]
    finally:
        if gpu is not None:
            gpu.close()
        if pub is not None:
            stop_publisher(box, pub)
        box.close()
