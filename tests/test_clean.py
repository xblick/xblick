"""``xblick clean`` and the creator record of a ring (ring.hpp): who made a ring, whether it lives, the lock.

Every test names its own rings to the command, so it never judges or touches another program's ring in /dev/shm.
The publishers are child processes that create a ring through xblick._core; a SIGKILL leaves its ring behind as a
crash does."""

import io
import os
import signal
import struct
import subprocess
import sys
import time

import numpy as np
import pytest

from xblick import _core, clean

needs_cuda = pytest.mark.skipif(_core.cuda_device_count() == 0, reason="no CUDA device")

# a publisher: one ring, one frame, its name on stdout; ends (removing the ring) when stdin closes
PUBLISHER = """
import sys
import numpy as np
from xblick import _core
ring = _core.HostRingWriter(4, 4096)
ring.write(np.arange(4096, dtype=np.uint8))
print(ring.name, flush=True)
sys.stdin.read()
"""


class Publisher:
    """A child process holding one host ring."""

    def __init__(self):
        self.process = subprocess.Popen([sys.executable, "-c", PUBLISHER], stdin=subprocess.PIPE,
                                        stdout=subprocess.PIPE, text=True)
        self.name = self.process.stdout.readline().strip().lstrip("/")
        assert clean.NAME.fullmatch(self.name), self.name

    def end(self):
        """Ends it as a program ends: it removes its ring."""
        self.process.stdin.close()
        assert self.process.wait(10) == 0

    def kill(self):
        self.process.kill()
        self.process.wait(10)


@pytest.fixture
def made():
    """Ring names a test created; any left at the end are removed (only these)."""
    names: list[str] = []
    yield names
    for name in names:
        try:
            os.unlink(clean.SHM_DIR / name)
        except FileNotFoundError:
            pass


def listing(*names, remove=False, force=False):
    out = io.StringIO()
    code = clean.run(list(names), remove, force, out=out)
    return code, out.getvalue()


def test_creator_record_written_and_read(made):
    ring = _core.HostRingWriter(4, 4096)
    name = ring.name.lstrip("/")
    made.append(name)
    header = (clean.SHM_DIR / name).read_bytes()[:64]
    magic, _, _, _, _, kind, pid, start, last, reserved = clean.HEADER.unpack(header)
    assert (magic, kind, pid, last, reserved) == (b"DBLKRING", 0, os.getpid(), 0, bytes(8))
    assert start == _core.process_start_time(os.getpid())
    before = time.time_ns()
    ring.write(np.zeros(16, np.uint8))
    last = struct.unpack_from("<Q", (clean.SHM_DIR / name).read_bytes(), 48)[0]
    assert before <= last <= time.time_ns()
    verdict = clean.judge(name, {})
    assert (verdict.state, verdict.creator_pid, verdict.kind) == (clean.LIVING, os.getpid(), "host")
    del ring
    assert not (clean.SHM_DIR / name).exists()


def test_the_lock_keeps_no_reader_out(made):
    """The creator's lock is advisory: readers in this process and in another one open and read its ring."""
    publisher = Publisher()
    made.append(publisher.name)
    try:
        here = _core.HostRingReader("/" + publisher.name)
        out = np.empty(4096, np.uint8)
        assert here.read(0, 2, out) and (out == np.arange(4096, dtype=np.uint8)).all()
        reader = ("import sys, numpy as np; from xblick import _core; r = _core.HostRingReader(sys.argv[1]); "
                  "o = np.empty(4096, np.uint8); print(r.read(0, 2, o) and int(o[100]))")
        other = subprocess.run([sys.executable, "-c", reader, "/" + publisher.name], capture_output=True, text=True,
                               timeout=30, check=False)
        assert other.stdout.strip() == "100", other.stderr
        assert clean.judge(publisher.name, {}).state == clean.LIVING
    finally:
        publisher.end()


def test_killed_publisher_is_removed_living_ones_stay(made):
    """Two publishers of the same program live side by side; the killed one's ring goes, the living one's stays."""
    living, killed, second = Publisher(), Publisher(), Publisher()
    made += [living.name, killed.name, second.name]
    try:
        killed.kill()
        assert (clean.SHM_DIR / killed.name).exists()  # a SIGKILL leaves the ring behind
        code, text = listing(living.name, killed.name, second.name)
        assert code == 0
        lines = {line.split()[0]: line for line in text.splitlines()[1:-1]}
        assert "creator living" in lines[living.name] and "would remove" not in lines[living.name]
        assert "creator living" in lines[second.name]
        assert "would remove  creator gone" in lines[killed.name]
        assert (clean.SHM_DIR / killed.name).exists()  # listing removes nothing
        code, text = listing(living.name, killed.name, second.name, remove=True)
        assert code == 0 and "removed 1 of 3 rings" in text, text
        assert not (clean.SHM_DIR / killed.name).exists()
        assert (clean.SHM_DIR / living.name).exists() and (clean.SHM_DIR / second.name).exists()
        code, text = listing(living.name, remove=True, force=True)  # --force never removes a living one
        assert (clean.SHM_DIR / living.name).exists() and "removed 0 of 1" in text
    finally:
        living.end()
        second.end()


def test_ring_without_record_is_unknown(made):
    """A ring as xblick 0.5.11 and older write it: no creator record, no lock. Only --force with its name removes it."""
    name = "xblick-" + os.urandom(8).hex()
    made.append(name)
    header = struct.pack("<8sIIQQI28s", b"DBLKRING", 1, 4, 4096, 4096, 0, bytes(28))
    (clean.SHM_DIR / name).write_bytes(header.ljust(4096 + 4 * 4096, b"\0"))
    os.chmod(clean.SHM_DIR / name, 0o600)
    code, text = listing(name)
    assert code == 0 and "unknown" in text and "would remove" not in text and "nothing to remove" in text
    listing(name, remove=True)
    assert (clean.SHM_DIR / name).exists()
    code, text = listing(name, remove=True, force=True)
    assert code == 0 and "removed 1 of 1" in text and not (clean.SHM_DIR / name).exists()


def test_cli_refuses_force_without_names_and_foreign_names(capsys):
    from xblick import cli

    assert cli.main(["clean", "--remove", "--force"]) == 1
    assert cli.main(["clean", "not-a-ring"]) == 1
    assert cli.main(["clean", "xblick-" + os.urandom(8).hex()]) == 1
    assert "no such ring" in capsys.readouterr().err


@needs_cuda
def test_killed_cuda_publisher_leaves_its_control_block(made):
    """A CUDA ring's control block is a file in /dev/shm as well; killed, its publisher leaves it behind."""
    before = {e.name for e in os.scandir(clean.SHM_DIR) if clean.NAME.fullmatch(e.name)}
    code = ("import sys, os, numpy as np; from xblick import _core; r = _core.CudaRingWriter(0, 4, 4096); "
            "print('ready', flush=True); sys.stdin.read()")
    child = subprocess.Popen([sys.executable, "-c", code], stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True)
    try:
        assert child.stdout.readline().strip() == "ready"
        mine = [n for n in clean.mappers().items() if child.pid in n[1] and n[0] not in before]
        assert len(mine) == 1, mine  # the child maps exactly its own control block
        name = mine[0][0]
        made.append(name)
        assert clean.judge(name, {}).kind == "cuda"
    finally:
        child.send_signal(signal.SIGKILL)
        child.wait(10)
    code, text = listing(name, remove=True)
    assert "removed 1 of 1" in text and not (clean.SHM_DIR / name).exists()
