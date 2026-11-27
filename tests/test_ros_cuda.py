"""The GPU link to ROS 2 readers without ROS (ros_cuda.py, cpp/include/xblick/ros_cuda.hpp): the descriptor message
byte for byte against one a stock Lyrical publisher sent, the backend's registry read, the export pool with a reader
that holds a block (its record written here), the sockets, the removal of what a crashed process left, and the routing
of a CameraPublisher with a stand-in reader in this process. The stock subscriber and Isaac ROS: test_ros_cuda_live.py.
"""

import json
import mmap
import os
import socket
import struct
import subprocess
import sys
import time

import numpy as np
import pytest

from xblick import _core, ros, ros_cuda, session
from xblick.camera import CameraPublisher
from xblick.gpu import CudaImage

needs_cuda = pytest.mark.skipif(_core.cuda_device_count() == 0, reason="no CUDA device")

# sensor_msgs/Image with a CUDA buffer descriptor as cuda_buffer_backend 0.1.2 (ROS 2 Lyrical, rmw_zenoh_cpp 0.10.6)
# sent it on <key>/_buf/<gid>: 64 x 48 rgb8, frame "s0:16", block 0 of 2 MiB of pid 4081145, its socket in /tmp
STOCK = bytes.fromhex(
    "00010000c8f4076bcbb52c040600000073303a31360000003000000040000000050000007267623800000000c0000000ffffffff"
    "32534f52050000006375646100000000002400000000000002000000680000000000000001000000f9453e000000000000002000"
    "000000001d0000002f746d702f637564615f766d6d5f343038313134355f302e736f636b0000000000000000d42f82926d34419e"
    "f9453e00000000000100000000000000ff0f00000000000000000000000000000000000000000000000000000000000000000000"
    "000000000000000000000000")

REGISTRY_HEAD = struct.Struct("<I256sII4x")       # version, hostname, max_entries, num_entries (entries at 272)
REGISTRY_ENTRY = struct.Struct("<Q16sB3xiI??2x256s")  # instance id, gid, type, device, uid, active, ipc, name


def test_descriptor_message_is_the_stock_publishers():
    descriptor = {"size": 9216, "device_id": 0, "use_ipc": True, "pid": 4081145, "block_id": 0,
                  "block_size": 2 * 1024 * 1024, "socket_path": "/tmp/cuda_vmm_4081145_0.sock",
                  "uid": int.from_bytes(bytes.fromhex("d42f82926d34419e"), "little"), "event": STOCK[-64:]}
    payload = _core.ros_cuda_image_payload(0x6B07F4C8, 0x042CB5CB, "s0:16", 48, 64, "rgb8", 0, 192, descriptor)
    assert payload == STOCK
    del descriptor["event"]  # without an event: all zero, the rest the same
    payload = _core.ros_cuda_image_payload(0x6B07F4C8, 0x042CB5CB, "s0:16", 48, 64, "rgb8", 0, 192, descriptor)
    assert payload[:-64] == STOCK[:-64] and payload[-64:] == bytes(64)


def write_registry(path, entries, version=1):
    data = bytearray(272 + 512 * REGISTRY_ENTRY.size)
    REGISTRY_HEAD.pack_into(data, 0, version, b"host", 512, len(entries))
    for i, e in enumerate(entries):
        REGISTRY_ENTRY.pack_into(data, 272 + i * REGISTRY_ENTRY.size, *e)
    path.write_bytes(bytes(data))


def test_registry_lookup(tmp_path):
    name = _core.ros_cuda_registry_name()
    assert name.startswith("ros2_hem_d0_h")
    assert _core.ros_cuda_registry_lookup(str(tmp_path), bytes(16)) is None  # no registry: nobody local
    gid, other, gone = bytes(range(16)), bytes(range(1, 17)), bytes(range(2, 18))
    write_registry(tmp_path / name, [((4242 << 32) | 7, other, 0, 1, 1000, True, True, b"pub"),
                                     ((4243 << 32) | 1, gone, 1, 0, 1000, False, True, b"gone"),
                                     ((4244 << 32) | 3, gid, 1, 2, 1001, True, False, b"sensor_msgs/msg/Image")])
    entry = _core.ros_cuda_registry_lookup(str(tmp_path), gid)
    assert entry == {"pid": 4244, "instance_id": (4244 << 32) | 3, "entity_type": 1, "device_id": 2, "uid": 1001,
                     "ipc_capable": False, "name": "sensor_msgs/msg/Image"}
    assert _core.ros_cuda_registry_lookup(str(tmp_path), gone) is None  # inactive
    assert _core.ros_cuda_registry_lookup(str(tmp_path), bytes(16)) is None
    write_registry(tmp_path / name, [], version=2)
    with pytest.raises(_core.RingError, match="version 2"):
        _core.ros_cuda_registry_lookup(str(tmp_path), gid)
    (tmp_path / name).write_bytes(b"\0" * 100)
    with pytest.raises(_core.RingError, match="another backend version"):
        _core.ros_cuda_registry_lookup(str(tmp_path), gid)


def test_pool_needs_its_directories(tmp_path):
    with pytest.raises(_core.RingError, match="no such directory"):
        _core.RosCudaPool(0, str(tmp_path / "missing"), str(tmp_path))
    pool = _core.RosCudaPool(0, str(tmp_path), str(tmp_path))  # nothing allocated, no file, before acquire
    assert pool.blocks == 0 and os.listdir(tmp_path) == []
    del pool


def record_of(shm, descriptor):
    return shm / f"cuda_vmm_{descriptor['pid']}_{descriptor['block_id']}"


class Holder:
    """A reader as the backend's: one count in the block's record while it holds it."""

    def __init__(self, path):
        self.file = open(path, "r+b")
        self.map = mmap.mmap(self.file.fileno(), 24)

    @property
    def uid(self):
        return struct.unpack_from("<Q", self.map, 8)[0]

    def count(self, delta):
        struct.pack_into("<i", self.map, 0, struct.unpack_from("<i", self.map, 0)[0] + delta)

    def close(self):
        self.map.close()
        self.file.close()


@needs_cuda
def test_pool_blocks_holds_and_drops(tmp_path):
    """Blocks of whole 2 MiB steps; one comes back only after as many frames as the readers' queues hold (here 2) and
    while nobody holds it; at most max_blocks (3) of a size: then the frame is dropped; a hold past the limit ends."""
    sockets, shm = tmp_path / "sockets", tmp_path / "shm"
    sockets.mkdir()
    shm.mkdir()
    pool = _core.RosCudaPool(0, str(sockets), str(shm), max_blocks=3, hold_limit_s=0.3, min_reuse_s=0.0)
    frame = 1920 * 1080 * 3
    assert pool.acquire(frame, 2) == 0 and pool.blocks == 1
    first = pool.publish(0, frame)
    assert first["size"] == frame and first["block_size"] == 6 * 1024 * 1024
    assert first["pid"] == os.getpid() and first["uid"] != 0 and first["device_id"] == 0
    assert first["socket_path"] == f"{sockets}/xblick_cuda_{os.getpid()}_{first['block_id']}.sock"
    holder = Holder(record_of(shm, first))
    assert holder.uid == first["uid"] and oct(os.stat(record_of(shm, first)).st_mode & 0o777) == "0o600"
    assert pool.acquire(frame, 2) == 1  # block 0 may still be in a reader's queue: a second block
    second = pool.publish(1, frame)
    assert pool.acquire(frame, 2) == 2
    pool.publish(2, frame)
    assert pool.acquire(frame, 2) == 0 and holder.uid == 0  # two frames later: block 0 again, uid 0 while written
    again = pool.publish(0, frame)
    assert again["block_id"] == first["block_id"] and again["uid"] not in (0, first["uid"])
    assert holder.uid == again["uid"]

    holder.count(+1)  # a reader takes the message of block 0
    holder2 = Holder(record_of(shm, second))
    assert pool.acquire(frame, 2) == 1
    pool.publish(1, frame)
    holder2.count(+1)
    assert pool.acquire(frame, 2) == 2
    pool.publish(2, frame)
    assert pool.acquire(frame, 2) == -1 and pool.dropped == 1 and pool.blocks == 3  # held, held, queued: dropped
    assert holder.uid == again["uid"]  # left as it is
    time.sleep(0.35)  # block 0 and 1 held past the hold limit
    assert pool.acquire(frame, 2) == 0 and pool.expired == 1  # taken although its count is up
    pool.publish(0, frame)

    # another frame size: new blocks with new ids; the old ones go once nobody holds or queues them
    holder.count(-1)
    holder2.count(-1)
    ids = set()
    for _ in range(3):
        index = pool.acquire(640 * 480 * 3, 2)
        small = pool.publish(index, 640 * 480 * 3)
        assert small["block_size"] == 2 * 1024 * 1024
        ids.add(small["block_id"])
    assert not ids & {first["block_id"], second["block_id"]}
    assert pool.blocks == len(ids) and not record_of(shm, first).exists() and not record_of(shm, second).exists()
    holder.close()
    holder2.close()
    del pool
    assert os.listdir(sockets) == [] and os.listdir(shm) == []  # everything removed on close


@needs_cuda
def test_pool_grows_to_the_queue_depth_and_spares(tmp_path):
    pool = _core.RosCudaPool(0, str(tmp_path), str(tmp_path), min_reuse_s=0.0)
    for i in range(30):  # a reader that keeps 10 messages: 11 blocks in turn, each again 10 frames later
        index = pool.acquire(1000, 10)
        assert index == i % 11
        pool.publish(index, 1000)
    assert pool.blocks == 11 and pool.dropped == 0
    holders = [Holder(tmp_path / f"cuda_vmm_{os.getpid()}_{d}")
               for d in sorted(int(p.split("_")[-1]) for p in os.listdir(tmp_path) if p.startswith("cuda_vmm_"))]
    for h in holders:
        h.count(+1)  # all held: up to 10 + 1 + spare blocks, then dropped
    for _ in range(ros_cuda.SPARE_BLOCKS):
        pool.publish(pool.acquire(1000, 10), 1000)
    assert pool.blocks == 11 + ros_cuda.SPARE_BLOCKS and pool.acquire(1000, 10) == -1
    for h in holders:
        h.close()
    del pool


@needs_cuda
def test_pool_hands_the_block_to_readers_of_this_user_by_both_sockets(tmp_path):
    pool = _core.RosCudaPool(0, str(tmp_path), str(tmp_path))
    index = pool.acquire(1000, 0)
    descriptor = pool.publish(index, 1000)
    path = descriptor["socket_path"]
    assert oct(os.stat(path).st_mode & 0o777) == "0o600"
    for address in (path, "\0" + path):  # release 0.1.2's file socket, the current source's abstract one
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as s:
            s.settimeout(5)
            s.connect(address)
            data, fds, _, _ = socket.recv_fds(s, 1, 1)
        assert data == b"X" and len(fds) == 1
        os.close(fds[0])
    # the server thread counts an accept after its send returned, which can be after the fd arrived here
    end = time.monotonic() + 5
    while pool.accepts < 2 and time.monotonic() < end:
        time.sleep(0.01)
    assert pool.accepts == 2 and pool.refused == 0
    del pool
    assert not os.path.exists(path)


def test_files_of_a_crashed_pool_are_removed_and_only_those(tmp_path):
    sockets, shm = tmp_path / "sockets", tmp_path / "shm"
    sockets.mkdir()
    shm.mkdir()
    dead = subprocess.Popen([sys.executable, "-c", "pass"])
    dead.wait()  # its pid is free now (not taken again within the test, as pids count up)
    ns = os.readlink("/proc/self/ns/pid")

    def leave(pid, start):
        boot = _core.host_id()
        listed = [sockets / f"xblick_cuda_{pid}_7.sock", shm / f"cuda_vmm_{pid}_7"]
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as s:
            s.bind(str(listed[0]))
        listed[1].write_bytes(bytes(24))
        (sockets / f"xblick_cuda_{pid}.files").write_text(
            f"{boot}\n{ns}\n{pid}\n{start}\nsocket {listed[0]}\nrecord {listed[1]}\nrecord {shm / 'foreign_7'}\n")
        return listed

    gone = leave(dead.pid, 12345)
    alive = leave(os.getppid(), _core.process_start_time(os.getppid()))
    foreign = shm / "foreign_7"  # listed, but no record of that pid: kept
    foreign.write_bytes(bytes(24))
    unlisted = shm / f"cuda_vmm_{dead.pid}_8"  # a record of that pid, not listed (a stock ROS program's): kept
    unlisted.write_bytes(bytes(24))
    pool = _core.RosCudaPool(0, str(sockets), str(shm))
    assert not any(p.exists() for p in gone) and not (sockets / f"xblick_cuda_{dead.pid}.files").exists()
    assert all(p.exists() for p in alive) and (sockets / f"xblick_cuda_{os.getppid()}.files").exists()
    assert foreign.exists() and unlisted.exists()
    del pool


@pytest.fixture
def ros_session():
    with session.open_session(listen=["tcp/127.0.0.1:0"], declare=False,
                              ros_options=session.RosOptions(91, "gpu_link_test")) as s:
        yield s


def test_backends_part():
    token = ros.parse_token("@ros2_lv/0/z/1/2/MS/%/%/n/%t/sensor_msgs::msg::dds_::Image_/RIHS01_x/::,10:,:,:,,/"
                            "backends:cpu:;cuda:")
    assert ros_cuda.backends_of(token) == {"cpu": "", "cuda": ""}
    assert ros_cuda.backends_of(ros.Token(0, "z", "1", "2", "MS", "/", "/", "n")) is None


@needs_cuda
def test_camera_routes_gpu_frames_to_a_cuda_reader(tmp_path, ros_session, monkeypatch):
    """A stand-in for a Lyrical reader that accepts CUDA buffers (its token, its registry entry with this process as
    its pid, which maps the registry and has this GPU open), a Lyrical CPU reader and a Jazzy one: the descriptor goes
    to the first, the CPU form to _buf_cpu, nothing on the key; with the Jazzy reader all on the key in CPU form."""
    base = "gpulink/cam/color"
    topic = "/" + base + "/image_raw"
    s = ros_session
    zid = str(s.info.zid())
    hash_ = ros.hash_of("sensor_msgs/msg/Image")
    key = ros.data_key(91, topic, "sensor_msgs/msg/Image", hash_)

    def token(entity, node, backends):
        parts = ["@ros2_lv", "91", zid, "90", str(entity), "MS", "%", "%", node, ros.mangle(topic),
                 "sensor_msgs::msg::dds_::Image_", hash_, "::,10:,:,:,,"] + ([backends] if backends else [])
        return "/".join(parts)

    cuda_token, cpu_token, jazzy_token = (token(91, "gpu", "backends:cuda:"), token(92, "cpu", "backends:cpu:"),
                                          token(93, "old", None))
    cuda_gid = ros.gid(cuda_token)
    registry = tmp_path / _core.ros_cuda_registry_name()
    write_registry(registry, [((os.getpid() << 32) | 1, cuda_gid, 1, 0, os.getuid(), True, True, b"image")])
    mapped = open(registry, "rb")
    view = mmap.mmap(mapped.fileno(), 0, prot=mmap.PROT_READ)  # the process "maps the registry", as a ROS one
    got = {"desc": [], "cpu": [], "key": []}
    subs = [s.declare_subscriber(f"{key}/_buf/{cuda_gid.hex()}", lambda x: got["desc"].append(x.payload.to_bytes())),
            s.declare_subscriber(f"{key}/_buf_cpu", lambda x: got["cpu"].append(x.payload.to_bytes())),
            s.declare_subscriber(key, lambda x: got["key"].append(x.payload.to_bytes()))]
    monkeypatch.setenv(ros_cuda.SOCKETS_ENV, str(tmp_path))
    monkeypatch.setenv(ros_cuda.SHM_ENV, str(tmp_path))
    camera = None
    try:
        camera = CameraPublisher(s, base, encoding="rgb8", frame_id="f", cuda_device=0)
        live = [s.liveliness().declare_token(cuda_token), s.liveliness().declare_token(cpu_token)]
        buffer = _core.make_device_buffer(0, 48 * 64 * 3)
        buffer.upload(np.arange(48 * 64 * 3, dtype=np.uint8))
        frame = CudaImage(buffer.ptr, (48, 64, 3), np.uint8, 0, owner=buffer)
        end = time.monotonic() + 10
        while not got["desc"] and time.monotonic() < end:
            camera.publish(frame)
            time.sleep(0.05)
        assert got["desc"], camera._ros_gpu.stats()
        time.sleep(0.3)
        for x in got.values():
            x.clear()
        for _ in range(5):
            camera.publish(frame)
        time.sleep(0.5)
        assert len(got["desc"]) == 5 and len(got["cpu"]) == 5 and got["key"] == []
        payload = got["desc"][-1]
        assert b"\xff\xff\xff\xff2SOR" in payload and len(payload) < 300  # the marker words in place of the data
        assert len(got["cpu"][-1]) > 48 * 64 * 3
        stats = camera._ros_gpu.stats()
        assert stats["blocks"] >= 1 and stats["sent"] >= 5

        live.append(s.liveliness().declare_token(jazzy_token))  # reads the key only: all on the key
        time.sleep(0.5)
        for x in got.values():
            x.clear()
        for _ in range(3):
            camera.publish(frame)
        time.sleep(0.5)
        assert got["desc"] == [] and got["cpu"] == [] and len(got["key"]) == 3
        for t in live:
            t.undeclare()
    finally:
        if camera is not None:
            camera.close()
        for sub in subs:
            sub.undeclare()
        view.close()
        mapped.close()
    assert sorted(os.listdir(tmp_path)) == [registry.name]  # the pool's files are gone with the camera


def test_buffer_form_is_parsed_and_the_cpu_form_is_not():
    parsed = _core.ros_cuda_parse_image(STOCK)
    assert {k: parsed[k] for k in ("sec", "nanosec", "frame_id", "height", "width", "encoding", "step")} == {
        "sec": 0x6B07F4C8, "nanosec": 0x042CB5CB, "frame_id": "s0:16", "height": 48, "width": 64, "encoding": "rgb8",
        "step": 192}
    d = parsed["descriptor"]
    assert (d["size"], d["pid"], d["block_id"], d["block_size"], d["socket_path"], d["event"]) == (
        9216, 4081145, 0, 2 * 1024 * 1024, "/tmp/cuda_vmm_4081145_0.sock", STOCK[-64:])
    from xblick.msg import Header, Image, Time
    cpu = bytes(Image(header=Header(stamp=Time(1, 2), frame_id="f"), height=1, width=2, encoding="mono8", step=2,
                      data=b"ab").serialize())
    assert _core.ros_cuda_parse_image(cpu) is None
    other = STOCK.replace(b"\x05\x00\x00\x00cuda\x00", b"\x05\x00\x00\x00qcom\x00")
    with pytest.raises(_core.RingError, match="backend 'qcom'"):
        _core.ros_cuda_parse_image(other)


# a stand-in for a ROS publisher with the CUDA buffer backend (its pool is ours, its process a child): it maps the
# registry as a ROS process does, and for each seed on stdin writes the test pattern into a block and prints the
# block's descriptor
PUBLISHER = r"""
import json, mmap, sys
import numpy as np
from xblick import _core
shm, n = sys.argv[1], int(sys.argv[2])
f = open(shm + "/" + _core.ros_cuda_registry_name(), "rb")
view = mmap.mmap(f.fileno(), 0, prot=mmap.PROT_READ)
pool = _core.RosCudaPool(0, shm, shm, min_reuse_s=0.0)
ring = _core.CudaRingWriter(0, 2, n)
print("ready", flush=True)
for line in sys.stdin:
    seed = int(line)
    slot = ring.write_host(((np.arange(n, dtype=np.uint64) * 13 + seed) & 0xFF).astype(np.uint8))[0]
    index = pool.acquire(n, 0)
    ring.copy_slot(slot, pool.block_ptr(index), n)
    print(json.dumps(pool.publish(index, n)), flush=True)
"""


def fake_backend(shm):
    """The backend's registry (no entries) and its semaphore as glibc's sem_open leaves it (value 1, shared)."""
    write_registry(shm / _core.ros_cuda_registry_name(), [])
    (shm / f"sem.{_core.ros_cuda_registry_name()}_mtx").write_bytes(b"\x01" + bytes(31))


def add_entry(shm, entry):
    path = shm / _core.ros_cuda_registry_name()
    with open(path, "r+b") as f, mmap.mmap(f.fileno(), 0) as m:
        count = struct.unpack_from("<I", m, 264)[0]
        REGISTRY_ENTRY.pack_into(m, 272 + count * REGISTRY_ENTRY.size, *entry)
        struct.pack_into("<I", m, 264, count + 1)


def pattern(n, seed):
    return ((np.arange(n, dtype=np.uint64) * 13 + seed) & 0xFF).astype(np.uint8)


@pytest.fixture
def stand_in_publisher(tmp_path):
    """(shm, child, its gid, next(seed) -> descriptor) of a stand-in ROS publisher on GPU 0 in the registry."""
    shm = tmp_path / "shm"
    shm.mkdir()
    fake_backend(shm)
    n = 640 * 480 * 3
    child = subprocess.Popen([sys.executable, "-c", PUBLISHER, str(shm), str(n)], stdin=subprocess.PIPE,
                             stdout=subprocess.PIPE, text=True)
    assert child.stdout.readline().strip() == "ready"
    gid = bytes(range(16))
    add_entry(shm, ((child.pid << 32) | 1, gid, 0, 0, os.getuid(), True, True, b"sensor_msgs/msg/Image"))

    def publish(seed):
        child.stdin.write(f"{seed}\n")
        child.stdin.flush()
        return json.loads(child.stdout.readline())

    yield shm, child, gid, publish, n
    child.stdin.close()
    child.wait(timeout=30)


@needs_cuda
def test_receiver_takes_a_publishers_block_under_its_hold(stand_in_publisher):
    """The receiver enters the registry once the publisher there is decided (on its GPU: "can import"), copies a
    block's frame out under the record's count (back to 0 after), finds a block rewritten after its descriptor stale,
    refuses a descriptor of a process that is no decided publisher, and unmaps the blocks when the publisher's token
    goes."""
    shm, _, publisher_gid, publish, n = stand_in_publisher
    mine = bytes(range(1, 17))
    receiver = _core.RosCudaReceiver("/rev/image", 0, str(shm), mine)
    assert _core.ros_cuda_registry_lookup(str(shm), mine) is None  # nothing before a publisher is known
    receiver.add_publisher("pub-token", publisher_gid, "/stand_in")
    receiver.refresh()
    notes = receiver.notes()
    assert any("uses this GPU" in text for _, text in notes), notes
    entry = _core.ros_cuda_registry_lookup(str(shm), mine)
    assert entry is not None and (entry["pid"], entry["entity_type"], entry["device_id"], entry["uid"],
                                  entry["ipc_capable"], entry["name"]) == (os.getpid(), 1, 0, os.getuid(), True,
                                                                           "/rev/image")
    first = publish(1)
    buffer = receiver.take(first)
    host = np.empty(n, np.uint8)
    buffer.download(host)
    assert buffer.device == 0 and np.array_equal(host, pattern(n, 1))
    holder = Holder(record_of(shm, first))
    assert struct.unpack_from("<i", holder.map, 0)[0] == 0  # the hold ended with the copy
    second = publish(2)  # the same block again (no reader holds it), a new uid
    assert second["block_id"] == first["block_id"] and second["uid"] != first["uid"]
    assert receiver.take(first) is None and receiver.stats()["stale"] == 1
    buffer = receiver.take(second)
    buffer.download(host)
    assert np.array_equal(host, pattern(n, 2))
    assert receiver.stats() == {"copied": 2, "stale": 1, "refused": 0, "blocks": 1, "registered": True}
    assert receiver.take({**second, "pid": 1}) is None and receiver.stats()["refused"] == 1
    assert any("process 1," in text for _, text in receiver.notes())
    receiver.remove_publisher("pub-token")
    assert receiver.stats()["blocks"] == 0
    assert receiver.take(second) is None and receiver.stats()["stale"] == 2  # its publisher went
    holder.close()
    del receiver
    assert _core.ros_cuda_registry_lookup(str(shm), mine) is None  # the entry inactive


@needs_cuda
def test_receiver_leaves_a_publisher_on_another_gpu_or_of_another_user_the_cpu_form(stand_in_publisher):
    """A publisher of another user decided first: entered as "cannot import", which stays when one on this GPU comes;
    entered before any publisher: as a stock subscription; on another GPU: its descriptors are refused."""
    shm, child, publisher_gid, publish, _ = stand_in_publisher
    other_user = bytes(range(2, 18))
    add_entry(shm, ((child.pid << 32) | 2, other_user, 0, 0, os.getuid() + 1, True, True, b"t"))
    receiver = _core.RosCudaReceiver("/rev/image", 0, str(shm), bytes(range(1, 17)))
    receiver.add_publisher("other-user", other_user, "/other")
    receiver.refresh()
    assert receiver.stats()["registered"] is False
    assert any("sends the CPU form: it runs as Linux user" in text for _, text in receiver.notes())
    # entered as "cannot import": a publisher that found no entry would send descriptors (release 0.1.2)
    assert _core.ros_cuda_registry_lookup(str(shm), bytes(range(1, 17)))["ipc_capable"] is False
    receiver.add_publisher("pub", publisher_gid, "/stand_in")  # one on this GPU comes later: the entry stays
    receiver.refresh()
    assert receiver.stats()["registered"] is False
    assert _core.ros_cuda_registry_lookup(str(shm), bytes(range(1, 17)))["ipc_capable"] is False
    assert any("written when it started, says it cannot import" in text for _, text in receiver.notes())
    del receiver
    # entered before any publisher is known: as a stock subscription ("can import", this program's ordinal)
    early = _core.RosCudaReceiver("/rev/image", 0, str(shm), bytes(range(4, 20)))
    early.enter()
    entry = _core.ros_cuda_registry_lookup(str(shm), bytes(range(4, 20)))
    assert entry["ipc_capable"] is True and entry["device_id"] == 0 and early.stats()["registered"]
    del early
    if _core.cuda_device_count() > 1:  # this program on GPU 1, the publisher on GPU 0 (same numbering)
        on_other = _core.RosCudaReceiver("/rev/image", 1, str(shm), bytes(range(3, 19)))
        on_other.add_publisher("pub", publisher_gid, "/stand_in")
        on_other.refresh()
        assert on_other.stats()["registered"] is False
        assert any("uses CUDA device 0, this program 1" in text for _, text in on_other.notes())
        assert on_other.take(publish(3)) is None and on_other.stats()["refused"] == 1
        del on_other


@needs_cuda
def test_image_subscriber_with_a_device_reads_a_ros_topics_cuda_buffers(stand_in_publisher, ros_session,
                                                                        monkeypatch):
    """ImageSubscriber of a ROS topic with a device: its token says backends:cuda:, it reads <key>/_buf/<its gid>,
    and a descriptor there becomes a CudaImage on its GPU with the publisher's pixels; a subscriber on the CPU says
    backends:cpu: and is not entered in the registry."""
    shm, _, _, publish, n = stand_in_publisher
    from xblick.camera import ImageSubscriber
    monkeypatch.setenv(ros_cuda.SHM_ENV, str(shm))
    s = ros_session
    topic = "/rev/cam/image"
    hash_ = ros.hash_of("sensor_msgs/msg/Image")
    key = ros.data_key(91, topic, "sensor_msgs/msg/Image", hash_)
    token = "/".join(["@ros2_lv", "91", "0123456789abcdef0123456789abcdef", "90", "91", "MP", "%", "%", "pub",
                      ros.mangle(topic), "sensor_msgs::msg::dds_::Image_", hash_, "::,10:,:,:,,", "backends:cpu:;cuda:"])
    # the stand-in's registry entry has the gid of the token
    with open(shm / _core.ros_cuda_registry_name(), "r+b") as f, mmap.mmap(f.fileno(), 0) as m:
        m[280:296] = ros.gid(token)
    live = s.liveliness().declare_token(token)
    gpu = cpu = None
    try:
        gpu = ImageSubscriber(s, topic, transport="ros", device=0)
        cpu = ImageSubscriber(s, topic, transport="ros")
        ours = gpu._subscriber
        assert ours.token.backends == ros.CUDA_BACKENDS and cpu._subscriber.token.backends == ros.CPU_BACKENDS
        end = time.monotonic() + 10
        while not gpu.ros_gpu_stats()["registered"] and time.monotonic() < end:
            time.sleep(0.1)  # the publisher's token came and was decided
        assert gpu.ros_gpu_stats()["registered"]
        assert _core.ros_cuda_registry_lookup(str(shm), cpu._subscriber.gid) is None
        out = s.declare_publisher(ros.buffer_key(key, ours.gid))
        d = publish(7)
        payload = _core.ros_cuda_image_payload(5, 6, "xb:7", 480, 640, "rgb8", 0, 640 * 3, d)
        out.put(payload, attachment=ros.encode_attachment(1, time.time_ns(), ros.gid(token)))
        frame = gpu.latest(timeout=5)
        assert frame is not None and isinstance(frame.image, CudaImage) and frame.image.shape == (480, 640, 3)
        assert frame.header.frame_id == "xb:7" and frame.stamp_ns == 5 * 10**9 + 6
        assert np.array_equal(frame.image.numpy().reshape(-1), pattern(n, 7))
        assert gpu.ros_gpu_stats()["copied"] == 1
        out.undeclare()
    finally:
        live.undeclare()
        for sub in (gpu, cpu):
            if sub is not None:
                sub.close()
    assert _core.ros_cuda_registry_lookup(str(shm), ours.gid) is None


@needs_cuda
def test_reader_of_another_user_or_gpu_gets_the_cpu_form(tmp_path, ros_session):
    publisher = ros_session.ros.node.publisher("/gpulink/other/image_raw", "sensor_msgs/msg/Image")
    link = ros_cuda.GpuLink(publisher, 0, socket_dir=str(tmp_path), shm_dir=str(tmp_path))
    writer = _core.CudaRingWriter(0, 2, 1024)  # this process has the GPU open, as a reader would
    registry = tmp_path / _core.ros_cuda_registry_name()
    write_registry(registry, [])
    with open(registry, "rb") as f, mmap.mmap(f.fileno(), 0, prot=mmap.PROT_READ):
        mine = {"entity_type": 1, "uid": os.getuid(), "ipc_capable": True, "pid": os.getpid(), "device_id": 0}
        assert link._importable(mine) == (True, "")
        assert link._importable({**mine, "uid": os.getuid() + 1})[0] is False
        assert link._importable({**mine, "ipc_capable": False})[0] is False
        assert link._importable({**mine, "pid": os.getppid()})[0] is False  # does not map the registry
        if len(ros_cuda.gpu_minors("self")) > 1:
            assert "device 1" in link._importable({**mine, "device_id": 1})[1]
    del writer
    link.close()
    publisher.close()
