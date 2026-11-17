"""A node's parameters on ROS 2's keys (``--ros``) without a ROS install: our ParameterClient with ``ros`` and a bare
ROS client (one request, as a ROS tool sends it, no count) against our Parameters' ROS services. Sets through ROS are
refused unless ``--ros-params-writable``; with it they are unchecked and uncounted. tests/test_ros_live.py checks the
same against ROS 2's own programs."""

import socket
import time

import threading

import pytest

from xblick import ros, session
from xblick.msg import (Parameter, ParameterEvent, ParameterValue, SetParametersRequest,
                        SetParametersResponse)
from xblick.parameters import READ_ONLY_ACCESS, ROS_SETS_OFF, ParameterClient, Parameters
from xblick.session import ask_once, replies

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
    """Sessions with --ros connected directly (``writable``: --ros-params-writable on the serving ones, a and c), and
    one without."""

    def __init__(self, writable: bool = False):
        endpoint = f"tcp/127.0.0.1:{free_port()}"
        serving = dict(params_writable=writable)
        self.a = session.open_session(listen=[endpoint], ros_options=session.RosOptions(DOMAIN, "xblick_a", **serving))
        self.b = session.open_session(connect=[endpoint], ros_options=session.RosOptions(DOMAIN, "xblick_reader"))
        self.c = session.open_session(connect=[endpoint], ros_options=session.RosOptions(DOMAIN, "xblick_c", **serving))
        self.observer = session.open_session(connect=[endpoint], declare=False)

    def close(self):
        for s in (self.b, self.c, self.a, self.observer):
            s.close()


def ros_set(caller, node: str, name: str, value, timeout: float = 2.0) -> list[SetParametersResponse]:
    """One set_parameters request as a ROS tool sends it (rmw_zenoh's attachment, ALL_COMPLETE, no read before it,
    no count): the responses of every node that answered."""
    client = caller.ros.node.client(f"/{node}/set_parameters", "rcl_interfaces/srv/SetParameters")
    try:
        wait_until(lambda: client.matched)
        request = SetParametersRequest([Parameter(name, ParameterValue.of(value))])
        answers = replies(caller, client.key, timeout, **client.request_options(request.serialize()))
        return [SetParametersResponse.deserialize(r.ok.payload.to_bytes()) for r in answers if r.ok is not None]
    finally:
        client.close()


def test_parameters_through_ros_keys():
    """A ROS client's calls (ParameterClient with ros) on our node's ROS services, sets allowed
    (--ros-params-writable): read, set, refusals, events. A set on ROS's key carries no node or deadline: executed
    unchecked (last_checked False)."""
    net = Network(writable=True)
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


def test_ros_sets_are_refused_by_default():
    """Without --ros-params-writable a ROS client reads, and its sets are refused with the reason; nothing changes."""
    net = Network()
    params = Parameters(net.a, "boxer", "rostest/cam")
    params.declare("threshold", 0.5, range=(0.0, 1.0))
    client = ParameterClient(net.b, "rostest/cam/boxer", timeout=2.0, ros=True)
    try:
        assert client.get(["threshold"])[0].value() == 0.5
        (result,) = client.set({"threshold": 0.7})
        assert not result.successful and result.reason == ROS_SETS_OFF
        assert not client.set_atomically({"threshold": 0.7}).successful
        assert params["threshold"] == 0.5
    finally:
        client.close()
        params.close()
        net.close()


@pytest.mark.parametrize("writable", [False, True])
def test_two_nodes_of_one_name_and_a_ros_set(writable):
    """Two nodes of one name (a and c). A ROS tool's set reaches both: by default both refuse it; with
    --ros-params-writable both execute it, uncounted (a ROS client shows one of the answers). Our ParameterClient with
    ros counts first and sends nothing."""
    net = Network(writable=writable)
    nodes = [Parameters(s, "boxer", "dup/cam") for s in (net.a, net.c)]
    for p in nodes:
        p.declare("max_boxes", 10, range=(1, 100))
    ours = ParameterClient(net.b, "dup/cam/boxer", timeout=1.0, ros=True)
    try:
        time.sleep(1.0)  # both known to the caller
        answers = ros_set(net.b, "dup/cam/boxer", "max_boxes", 42)
        assert len(answers) == 2
        assert [a.results[0].successful for a in answers] == [writable, writable]
        assert [p["max_boxes"] for p in nodes] == ([42, 42] if writable else [10, 10])
        with pytest.raises(RuntimeError, match="not sent: 2 nodes answer"):
            ours.set({"max_boxes": 7})
        assert [p["max_boxes"] for p in nodes] == ([42, 42] if writable else [10, 10])
    finally:
        ours.close()
        for p in nodes:
            p.close()
        net.close()


def test_read_only_access_holds_on_the_ros_path():
    net = Network(writable=True)
    params = Parameters(net.a, "boxer", "rostest/cam", read_only_access=True)
    params.declare("threshold", 0.5, range=(0.0, 1.0))
    try:
        (answer,) = ros_set(net.b, "rostest/cam/boxer", "threshold", 0.7)
        assert not answer.results[0].successful and answer.results[0].reason == READ_ONLY_ACCESS
        assert params["threshold"] == 0.5
    finally:
        params.close()
        net.close()


def test_malformed_ros_requests_are_refused_and_the_server_goes_on():
    net = Network(writable=True)
    params = Parameters(net.a, "boxer", "rostest/cam")
    params.declare("threshold", 0.5, range=(0.0, 1.0))
    client = net.b.ros.node.client("/rostest/cam/boxer/set_parameters", "rcl_interfaces/srv/SetParameters")
    try:
        wait_until(lambda: client.matched)
        good = client.request_options(SetParametersRequest([Parameter("threshold", ParameterValue.of(0.7))])
                                      .serialize())
        no_attachment = {k: v for k, v in good.items() if k != "attachment"}
        bad_cdr = dict(client.request_options(b"\x00\x01\x00\x00\xff"))
        for options in (no_attachment, bad_cdr, {**client.request_options(b""), "payload": b""}):
            answers = replies(net.b, client.key, 2.0, **options)
            assert [r.err is not None for r in answers] == [True], answers
        assert params["threshold"] == 0.5 and params._server.is_alive()
        assert ParameterClient(net.b, "rostest/cam/boxer", timeout=2.0).get(["threshold"])[0].value() == 0.5
    finally:
        client.close()
        params.close()
        net.close()


def test_a_command_on_our_key_still_reaches_every_queryable_with_ros():
    """With --ros sessions and their complete ROS parameter services about, a command on one of our keys reaches
    both plain queryables of the key (BestMatching: none of them is complete)."""
    net = Network(writable=True)
    params = Parameters(net.a, "boxer", "rostest/cam")
    calls = []
    plain = [s.declare_queryable("rostest/cam/plain/cmd", lambda q, n=n: (calls.append(n), q.reply(q.key_expr, b"ok")))
             for n, s in enumerate((net.a, net.c))]
    try:
        time.sleep(1.0)
        answers = ask_once(net.b, "rostest/cam/plain/cmd", 2.0)
        assert sorted(calls) == [0, 1] and len([r for r in answers if r.ok is not None]) == 2
    finally:
        for q in plain:
            q.undeclare()
        params.close()
        net.close()


def test_ros_reads_are_asked_again_with_a_new_sequence_number(monkeypatch):
    """A read before the node is known is asked again; each request has a sequence number of its own (a ROS service
    refuses a second pending request of one client with the same one)."""
    net = Network()
    seqs = []
    original = ros.RosClient.request_options

    def recorded(self, payload):
        options = original(self, payload)
        seqs.append((self.topic, ros.decode_attachment(options["attachment"])[0]))
        return options

    monkeypatch.setattr(ros.RosClient, "request_options", recorded)
    client = ParameterClient(net.b, "late/cam/boxer", timeout=5.0, ros=True)
    holder = {}
    starter = threading.Timer(1.0, lambda: holder.setdefault("p", Parameters(net.a, "boxer", "late/cam")))
    starter.start()
    try:
        assert client.list() == []
        lists = [seq for topic, seq in seqs if topic.endswith("/list_parameters")]
        assert len(lists) >= 2 and len(set(lists)) == len(lists)
    finally:
        starter.join()
        client.close()
        holder["p"].close()
        net.close()


def test_a_ros_node_closed_before_its_session_leaves_our_services_running():
    net = Network()
    params = Parameters(net.a, "boxer", "rostest/cam")
    params.declare("threshold", 0.5)
    try:
        params._ros_node.close()  # as Session.close does before the session itself
        time.sleep(0.2)
        assert params._server.is_alive()
        assert ParameterClient(net.b, "rostest/cam/boxer", timeout=2.0).get(["threshold"])[0].value() == 0.5
    finally:
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
