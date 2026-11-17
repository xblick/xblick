"""A node's parameters on ROS 2's keys (``--ros``) without a ROS install: our ParameterClient with ``ros`` against
our Parameters' ROS services, the set path among them (sent once, without node or deadline). tests/test_ros_live.py
checks the same against ROS 2's own programs."""

import socket
import time

import pytest

from xblick import session
from xblick.msg import ParameterEvent
from xblick.parameters import ParameterClient, Parameters

DOMAIN = 92


def free_port():
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def wait_until(condition, timeout=5.0):
    end = time.monotonic() + timeout
    while not condition():
        if time.monotonic() > end:
            raise TimeoutError
        time.sleep(0.02)


class Network:
    """Two sessions with --ros connected directly, and one without."""

    def __init__(self):
        endpoint = f"tcp/127.0.0.1:{free_port()}"
        self.a = session.open_session(listen=[endpoint], ros_options=session.RosOptions(DOMAIN, "xblick_test"))
        self.b = session.open_session(connect=[endpoint], ros_options=session.RosOptions(DOMAIN, "xblick_reader"))
        self.observer = session.open_session(connect=[endpoint], declare=False)

    def close(self):
        for s in (self.b, self.a, self.observer):
            s.close()


def test_parameters_through_ros_keys():
    """A ROS client's calls (ParameterClient with ros) on our node's ROS services: read, set, refusals, events. A set
    on ROS's key carries no node or deadline: executed unchecked (last_checked False)."""
    net = Network()
    params = Parameters(net.a, "boxer", "rostest/cam")
    params.declare("threshold", 0.5, range=(0.0, 1.0))
    params.declare("model", "base", read_only=True)
    client = ParameterClient(net.b, "rostest/cam/boxer", timeout=2.0, ros=True)
    events: list[ParameterEvent] = []
    client.on_event(events.append)
    try:
        assert sorted(client.list()) == ["model", "threshold"]
        assert [v.value() for v in client.get(["threshold", "model"])] == [0.5, "base"]
        assert client.describe(["model"])[0].read_only
        (result,) = client.set({"threshold": 0.7})
        assert result.successful and not client.last_checked
        assert params["threshold"] == 0.7
        (refused,) = client.set({"model": "large"})
        assert not refused.successful and params["model"] == "base"
        assert not client.set_atomically({"threshold": 2.0}).successful and params["threshold"] == 0.7
        wait_until(lambda: any(p.name == "threshold" for e in events for p in e.changed_parameters), 3.0)
        with pytest.raises(ValueError, match="need a session with --ros"):
            ParameterClient(net.observer, "rostest/cam/boxer", ros=True)
    finally:
        client.close()
        params.close()
        net.close()


def test_a_set_to_a_ros_node_that_is_not_there_is_not_sent():
    net = Network()
    client = ParameterClient(net.b, "nobody", timeout=0.5, ros=True)
    try:
        with pytest.raises(TimeoutError, match="not sent"):
            client.set({"x": 1})
    finally:
        client.close()
        net.close()
