"""Queries of a session just opened behind a router (the default setup of every command line tool): through the
router the queryables become known to the new session some 0.1-0.3 s later, and a query sent before ends at once
without a reply. Reads are asked again until a first reply (session.ask); each test counts how often the queryable
ran."""

import threading
import time

import pytest
from test_camera import free_port
from test_tui import open_router

from xblick import cli, controller_manager
from xblick.camera import CameraPublisher, wait_for_info
from xblick.msg import ControllerState, ListControllersResponse
from xblick.session import NotReached, ask, ask_once, open_session, replies


class Network:
    """A router, a serving session behind it, and new sessions (as programs) behind it."""

    def __init__(self):
        self.port = free_port()
        self.router = open_router(self.port)
        self.endpoint = [f"tcp/127.0.0.1:{self.port}"]
        self.server = open_session(connect=self.endpoint, listen=["127.0.0.1:0"])
        self._queryables = []
        self._sessions = []
        self.calls: dict[str, int] = {}
        self._lock = threading.Lock()

    def serve(self, key: str, answer=lambda query: b"ok", delay: float = 0.0, server=None, name: str | None = None):
        """A queryable on ``key`` that counts its queries (in ``calls[name or key]``); ``answer(query)`` the reply's
        payload, None: no reply; ``server``: another serving session (a second node)."""
        name = name or key

        def handle(query):
            with self._lock:
                self.calls[name] = self.calls.get(name, 0) + 1
            if delay:
                time.sleep(delay)
            payload = answer(query)
            if payload is not None:
                query.reply(query.key_expr, payload)

        self._queryables.append((server or self.server).declare_queryable(key, handle))
        self.calls[name] = 0

    def second_server(self):
        """Another serving session behind the router (a second node, a forgotten second bridge)."""
        self._sessions.append(open_session(connect=self.endpoint, listen=["127.0.0.1:0"]))
        return self._sessions[-1]

    def new_session(self):
        # no declaration: the queries right after the start below are those of a bare session (a declaration's
        # own exchange with the router delays the first query a little)
        return open_session(connect=self.endpoint, listen=["127.0.0.1:0"], declare=False)

    def close(self):
        for q in self._queryables:
            q.undeclare()
        for extra in self._sessions:
            extra.close()
        self.server.close()
        self.router.close()


@pytest.fixture
def network():
    n = Network()
    time.sleep(0.3)
    yield n
    n.close()


def test_a_read_is_asked_again_until_it_is_answered(network):
    network.serve("t/read")
    for _ in range(5):
        with network.new_session() as s:
            assert [r.ok.payload.to_bytes() for r in ask(s, "t/read", 2.0)] == [b"ok"]
    assert network.calls["t/read"] == 5  # one answered query per session: none sent after the first reply


def test_a_single_query_right_after_the_start_finds_nothing(network):
    """The reason for ask: at least once in ten new sessions a single query ends at once without a reply (it did
    8 of 10 times on the test PC)."""
    network.serve("t/once")
    empty = 0
    for _ in range(10):
        with network.new_session() as s:
            empty += not replies(s, "t/once", 2.0)
    assert empty >= 1


def test_xblick_hz_knows_a_point_cloud_stream_from_a_new_session(network):
    publisher = CameraPublisher(network.server, "t/info/color", encoding="rgb8", frame_id="cam")
    try:
        for _ in range(3):
            with network.new_session() as s:
                (record,) = wait_for_info(s, "t/info/color", 2.0)
                assert record["kind"] == "image"
                assert not cli._is_cloud(s, "t/info/color")
    finally:
        publisher.close()


def test_the_controller_managers_reads_from_a_new_session(network):
    state = ControllerState(name="arms", state="active", type="Impedance")
    network.serve("robot/controller_manager/list_controllers",
                  lambda q: bytes(ListControllersResponse([state]).serialize()))
    for _ in range(3):
        with network.new_session() as s:
            assert controller_manager.ControllerManager(s, "robot", timeout=2.0).states() == {"arms": "active"}
    assert network.calls["robot/controller_manager/list_controllers"] == 3


# ------------------------------------------------------------------------------ commands: once per queryable
# A command (a parameter set, a controller manager's create, switch or unload, a torque step, a recorder command) is
# sent once to each queryable of its key, after the session knows one (session.ask_once, session.command), and only
# after a read of the same node had exactly one answer. Each test counts the handlers' runs.

import json  # noqa: E402

from xblick import recording  # noqa: E402
from xblick.msg import (CreateControllerResponse, ListParametersResponse, SwitchControllerResponse,  # noqa: E402
                        TorqueStepResponse, UnloadControllerResponse)
from xblick.session import NoAnswer, SeveralExecuted, command, timed_out  # noqa: E402

M = "robot/controller_manager"
STATE = ControllerState(name="arms", state="active", type="Impedance")


def manager(network, server=None, name_suffix=""):
    """A controller manager's queryables (list answering, the commands answering ok)."""
    for service, payload in (("list_controllers", ListControllersResponse([STATE])),
                             ("create_controller", CreateControllerResponse(True, "")),
                             ("switch_controller", SwitchControllerResponse(True, "switched")),
                             ("unload_controller", UnloadControllerResponse(True))):
        network.serve(f"{M}/{service}", lambda q, b=bytes(payload.serialize()): b, server=server,
                      name=f"{M}/{service}{name_suffix}")


def controller(network, answer=True):
    network.serve("robot/controller/arms/list_parameters",
                  lambda q: bytes(ListParametersResponse(["level"], []).serialize()))
    network.serve("robot/controller/arms/torque_step",
                  (lambda q: bytes(TorqueStepResponse(True, "").serialize())) if answer else (lambda q: None))


def test_an_empty_result_does_not_mean_that_nobody_got_the_query(network):
    """Why a command is not asked again: a queryable that drops the query without a reply ends it at once with no
    reply, as a session that knows no queryable does. session.ask (for reads) sends such a query again."""
    network.serve("t/silent", lambda q: None)
    with network.new_session() as s:
        time.sleep(0.5)  # the queryable known
        assert ask(s, "t/silent", 0.6) == []
    assert network.calls["t/silent"] >= 2  # a read is sent again: harmless for a read, wrong for a command


def test_zenohs_timeout_reply_is_pinned():
    """session.timed_out compares Zenoh's error reply with its text "Timeout" (eclipse-zenoh 1.10, the version in
    uv.lock): a change of that text fails here, not silently in the callers."""
    from importlib.metadata import version

    from xblick.session import open_session as open_

    port = free_port()
    a = open_(listen=[f"tcp/127.0.0.1:{port}"])
    b = open_(connect=[f"tcp/127.0.0.1:{port}"])
    q = a.declare_queryable("t/slow", lambda query: (time.sleep(0.8), query.reply(query.key_expr, b"late")))
    try:
        time.sleep(0.5)
        (reply,) = replies(b, "t/slow", 0.2)
        assert reply.err is not None and reply.err.payload.to_string() == "Timeout" and timed_out(reply), \
            f"eclipse-zenoh {version('eclipse-zenoh')}"
        assert version("eclipse-zenoh").startswith("1.10.")
    finally:
        time.sleep(1.0)
        q.undeclare()
        b.close()
        a.close()


@pytest.mark.parametrize("kind", ["fast", "silent", "slow"])
def test_a_command_is_sent_once(network, kind):
    """From sessions just opened behind the router: a handler that answers, one that gives no reply, one that answers
    after the timeout. Each ran once per command; the two last are NoAnswer."""
    answer = None if kind == "silent" else (lambda q: b"done")
    network.serve("t/command", answer, delay=1.0 if kind == "slow" else 0.0)
    for _ in range(5):
        with network.new_session() as s:
            if kind == "fast":
                assert [r.payload.to_bytes() for r in command(s, "t/command", 0.4)] == [b"done"]
            else:
                with pytest.raises(NoAnswer, match="t/command: sent once, no answer within 0.4 s: it may still be "
                                                   "executed; not sent again"):
                    command(s, "t/command", 0.4)
    deadline = time.monotonic() + 10  # the handler works the queries one after another (1 s each when slow)
    while network.calls["t/command"] < 5 and time.monotonic() < deadline:
        time.sleep(0.05)
    time.sleep(1.5)  # nothing more comes
    assert network.calls["t/command"] == 5


def test_the_two_cases_are_told_apart_by_type(network):
    """NotReached (nothing sent) is a TimeoutError, as before; NoAnswer (sent once) is not, so that a caller's
    ``except TimeoutError: try again`` cannot send a command a second time."""
    network.serve("t/silent", lambda q: None)
    with network.new_session() as s:
        try:
            command(s, "t/nobody", 0.3)
        except TimeoutError as error:
            assert isinstance(error, NotReached) and str(error).startswith("not sent: no node serves t/nobody")
        with pytest.raises(NoAnswer):
            try:
                command(s, "t/silent", 0.3)
            except TimeoutError:
                pytest.fail("NoAnswer caught as a TimeoutError")
    assert issubclass(NoAnswer, RuntimeError) and not issubclass(NoAnswer, TimeoutError)
    assert network.calls["t/silent"] == 1


def test_without_a_queryable_nothing_is_sent_within_the_timeout(network):
    network.serve("t/elsewhere")
    with network.new_session() as s:
        started = time.monotonic()
        with pytest.raises(NotReached, match="not sent: no node serves t/command"):
            ask_once(s, "t/command", 0.3)
        assert time.monotonic() - started < 0.6
    assert network.calls["t/elsewhere"] == 0


def test_the_time_a_command_takes_at_most(network):
    """The bound in ask_once's docstring: up to timeout for the queryable, then the query's own timeout (Zenoh ends
    it and adds the "Timeout" reply), so about 2 x timeout here; 3 x timeout + 1 s when Zenoh does not end it."""
    network.serve("t/slow", lambda q: b"late", delay=2.0)
    with network.new_session() as s:
        started = time.monotonic()
        with pytest.raises(NoAnswer):
            command(s, "t/slow", 0.5)
        took = time.monotonic() - started
    assert took < 2 * 0.5 + 0.3, took


def test_late_execution_after_the_caller_gave_up(network):
    """A command sent without a target (no read before it, or one answered by an older receiver) has no deadline: a
    receiver busy with an earlier query executes it after the caller got NoAnswer, as before 0.5.13. This pins the
    behaviour and its message; with a target, test_a_busy_node_does_not_execute_a_set_after_the_sender_gave_up."""
    executed = []

    def handle(query):
        if query.payload is not None and query.payload.to_bytes() == b"busy":
            time.sleep(1.5)
            return b"busy done"
        executed.append(time.monotonic())
        return b"done"

    network.serve("t/arm", handle)
    with network.new_session() as busy, network.new_session() as s:
        time.sleep(0.5)
        threading.Thread(target=lambda: replies(busy, "t/arm", 3.0, payload=b"busy"), daemon=True).start()
        time.sleep(0.1)
        with pytest.raises(NoAnswer, match="it may still be executed; not sent again"):
            command(s, "t/arm", 0.4)
        gave_up = time.monotonic()
        deadline = time.monotonic() + 5
        while not executed and time.monotonic() < deadline:
            time.sleep(0.05)
    assert executed and executed[0] > gave_up  # executed after the caller had given up


@pytest.mark.parametrize("mode", ["client", "two_routers_peer", "two_routers_client"])
def test_a_command_is_sent_once_in_other_topologies(network, mode):
    """A client session behind the router, and sessions behind a second router chained to the first."""
    network.serve("t/command", lambda q: b"done")
    extra = None
    if mode == "client":
        endpoint = network.endpoint
    else:
        import json as json_
        import zenoh

        port = free_port()
        config = zenoh.Config()
        config.insert_json5("mode", '"router"')
        config.insert_json5("listen/endpoints", json_.dumps([f"tcp/127.0.0.1:{port}"]))
        config.insert_json5("connect/endpoints", json_.dumps(network.endpoint))
        config.insert_json5("scouting/multicast/enabled", "false")
        extra = zenoh.open(config)
        endpoint = [f"tcp/127.0.0.1:{port}"]
    try:
        for _ in range(5):
            peer = mode == "two_routers_peer"
            with open_session(connect=endpoint, listen=["127.0.0.1:0"] if peer else [],
                              mode="peer" if peer else "client") as s:
                assert [r.payload.to_bytes() for r in command(s, "t/command", 2.0)] == [b"done"]
    finally:
        if extra is not None:
            extra.close()
    assert network.calls["t/command"] == 5


def test_a_parameter_set_runs_the_nodes_writer_once(network):
    """set_parameters is a command (a node may act on a set): once per set, also with a writer slower than the
    client's timeout, which then gives NoAnswer and is not sent again."""
    from xblick.parameters import ParameterClient, Parameters

    writes = []

    def write(value):
        writes.append(value)
        if value > 5:
            time.sleep(1.0)  # slower than the client's timeout

    params = Parameters(network.server, "node", "t/w")
    params.declare("x", 0.0, write=write)
    try:
        for i in range(1, 4):
            with network.new_session() as s:
                assert ParameterClient(s, "t/w/node", 2.0).set({"x": float(i)})[0].successful
        with network.new_session() as s:
            with pytest.raises(NoAnswer, match="sent once with a deadline, no answer within 0.3 s"):
                ParameterClient(s, "t/w/node", 0.3).set({"x": 9.0})
        time.sleep(1.5)
        assert writes == [1.0, 2.0, 3.0, 9.0]
    finally:
        params.close()


def test_two_nodes_on_one_key_get_no_command(network):
    """Two nodes serving one key would both execute a command: it is refused before anything is sent, where the
    protocol lets the sender count them (a read of the node with one answer). Handlers 0 and 0."""
    from xblick.parameters import ParameterClient, Parameters

    writes = {"a": [], "b": []}
    second = network.second_server()
    nodes = [Parameters(session, "node", "t/two") for session in (network.server, second)]
    for params, which in zip(nodes, "ab"):
        params.declare("x", 0.0, write=lambda v, w=which: writes[w].append(v))
    manager(network)
    manager(network, server=second, name_suffix=" (second)")
    for service in ("status", "control"):
        network.serve(f"xblick/rec/{service}", lambda q: json.dumps({"state": "idle", "play": None}).encode())
        network.serve(f"xblick/rec/{service}", lambda q: json.dumps({"state": "idle", "play": None}).encode(),
                      server=second, name=f"xblick/rec/{service} (second)")
    try:
        time.sleep(0.5)
        with network.new_session() as s:
            # a session just opened links to the serving sessions one after the other: its read before a command
            # may see one node only (the documented limit). Wait until it sees both nodes of every key.
            deadline = time.monotonic() + 10.0
            for key in ("t/two/node/list_parameters", f"{M}/list_controllers", "xblick/rec/status"):
                while len([r for r in replies(s, key, 1.0) if r.ok]) < 2:
                    assert time.monotonic() < deadline, f"the new session sees fewer than 2 nodes at {key}"
                    time.sleep(0.1)
            with pytest.raises(RuntimeError, match="not sent: 2 nodes answer t/two/node/list_parameters"):
                ParameterClient(s, "t/two/node", 2.0).set({"x": 1.0})
            for call in (lambda m: m.create("arms", "Impedance", {}), lambda m: m.switch(activate=["arms"]),
                         lambda m: m.unload("arms")):
                with pytest.raises(RuntimeError, match="not sent: 2 nodes answer robot/controller_manager/list_cont"):
                    call(controller_manager.ControllerManager(s, "robot", timeout=2.0))
            with pytest.raises(RuntimeError, match="not sent: 2 nodes answer xblick/rec/status"):
                recording.request(s, "rec", "record", timeout=2.0)
        assert writes == {"a": [], "b": []}
        for service in ("create_controller", "switch_controller", "unload_controller"):
            assert network.calls[f"{M}/{service}"] == 0 and network.calls[f"{M}/{service} (second)"] == 0
        assert network.calls["xblick/rec/control"] == 0 and network.calls["xblick/rec/control (second)"] == 0
    finally:
        for params in nodes:
            params.close()


def test_the_controller_managers_commands_run_once(network):
    manager(network)
    controller(network)
    for _ in range(3):
        with network.new_session() as s:
            controller_manager.ControllerManager(s, "robot", timeout=2.0).create("arms", "Impedance", {"a": 1})
        with network.new_session() as s:
            assert controller_manager.ControllerManager(s, "robot", timeout=2.0).switch(activate=["arms"]) == "switched"
        with network.new_session() as s:
            controller_manager.ControllerManager(s, "robot", timeout=2.0).unload("arms")
        with network.new_session() as s:
            controller_manager.torque_step(s, "robot/controller/arms", "joint1", 1.0, 0.1)
    assert {k: v for k, v in network.calls.items() if "list" not in k} == {
        f"{M}/create_controller": 3, f"{M}/switch_controller": 3, f"{M}/unload_controller": 3,
        "robot/controller/arms/torque_step": 3}


@pytest.mark.parametrize("which", ["create", "switch", "unload", "torque_step"])
def test_a_controller_command_without_an_answer_is_not_repeated(network, which):
    """A manager or a controller that takes a command and answers nothing: NoAnswer, the handler ran once."""
    network.serve(f"{M}/list_controllers", lambda q: bytes(ListControllersResponse([STATE]).serialize()))
    key = {"create": f"{M}/create_controller", "switch": f"{M}/switch_controller", "unload": f"{M}/unload_controller",
           "torque_step": "robot/controller/arms/torque_step"}[which]
    if which == "torque_step":
        controller(network, answer=False)
    else:
        network.serve(key, lambda q: None)
    with network.new_session() as s:
        m = controller_manager.ControllerManager(s, "robot", timeout=0.5)
        call = {"create": lambda: m.create("arms", "Impedance", {}, timeout=0.5),
                "switch": lambda: m.switch(activate=["arms"], timeout=0.0),
                "unload": lambda: m.unload("arms"),
                "torque_step": lambda: controller_manager.torque_step(s, "robot/controller/arms", "j", 1.0, 0.1,
                                                                      timeout=0.5)}[which]
        with pytest.raises(NoAnswer, match=f"{key}: sent once, no answer"):
            call()
    time.sleep(0.5)
    assert network.calls[key] == 1


def recorder(network, control):
    network.serve("xblick/rec/status", lambda q: json.dumps({"state": "idle", "play": None}).encode())
    network.serve("xblick/rec/control", control)


def test_a_recorder_command_runs_once(network):
    recorder(network, lambda q: json.dumps({"v": recording.REPLY_VERSION, "ok": True,
                                            "result": {"state": "recording"}}).encode())
    for _ in range(3):
        with network.new_session() as s:
            assert recording.request(s, "rec", "record", timeout=2.0) == {"state": "recording"}
    assert network.calls["xblick/rec/control"] == 3


@pytest.mark.parametrize("kind", ["silent", "slow"])
def test_a_recorder_command_without_an_answer(network, kind):
    """No answer to a record command: NoAnswer "sent once ... not sent again", the node ran it once."""
    recorder(network, (lambda q: None) if kind == "silent" else
             (lambda q: (time.sleep(1.0), json.dumps({"v": recording.REPLY_VERSION, "ok": True,
                                                      "result": {}}).encode())[1]))
    with network.new_session() as s:
        with pytest.raises(NoAnswer, match="xblick/rec/control: sent once, no answer within 0.4 s"):
            recording.request(s, "rec", "record", timeout=0.4)
    time.sleep(1.2)
    assert network.calls["xblick/rec/control"] == 1


@pytest.mark.skipif(not __import__("os").environ.get("XBLICK_CPP_BUILD"), reason="XBLICK_CPP_BUILD is not set")
@pytest.mark.parametrize("slow", [False, True])
def test_a_cpp_parameter_set_runs_the_writer_once(network, slow):
    """C++ ParameterClient (xblick-cpp-probe --params --set), each run a new session behind the router; a writer
    slower than the client's timeout (2 s): NoAnswer "sent once", the writer ran once."""
    import os
    import subprocess

    from xblick.parameters import Parameters

    writes = []

    def write(value):
        writes.append(value)
        if slow:
            time.sleep(3.0)

    params = Parameters(network.server, "node", "t/c")
    params.declare("x", 0.0, write=write)
    probe = os.path.join(os.environ["XBLICK_CPP_BUILD"], "xblick-cpp-probe")
    try:
        for i in range(1, 2 if slow else 4):
            out = subprocess.run([probe, "--connect", network.endpoint[0], "--listen", "tcp/127.0.0.1:0", "--params",
                                  "t/c/node", "--set", f"x:={i}.0"], capture_output=True, text=True, timeout=30)
            if slow:
                assert out.returncode != 0 and "set_parameters: sent once with a deadline, no answer within 2000 ms: " \
                                                "it may have been executed" in out.stderr, out.stderr
            else:
                assert out.returncode == 0, out.stderr
        time.sleep(3.5 if slow else 0.0)
        assert writes == ([1.0] if slow else [1.0, 2.0, 3.0])
    finally:
        params.close()


def test_the_command_line_says_which_case_it_was(network, capsys):
    """xblick param --set: "not sent" (two nodes) and "sent once ... it may still be executed" each as one line."""
    from xblick.parameters import Parameters

    writes = []
    params = Parameters(network.server, "node", "t/cli")
    params.declare("x", 0.0, write=lambda v: (writes.append(v), time.sleep(1.0) if v > 5 else None)[1])
    second = network.second_server()
    twins = [Parameters(session, "node", "t/twin") for session in (network.server, second)]
    for twin in twins:
        twin.declare("x", 0.0)
    endpoint = ["--connect", network.endpoint[0], "--listen", "tcp/127.0.0.1:0"]
    try:
        time.sleep(0.5)
        assert cli.main(["param", "--node", "t/cli/node", "--set", "x:=9.0", "--timeout", "0.5", *endpoint]) == 1
        err = capsys.readouterr().err
        assert err == ("xblick param: t/cli/node/set_parameters: sent once with a deadline, no answer within 0.5 s: it "
                       "may have been executed; a receiver that checks the deadline (xblick 0.5.13 or later) no longer "
                       "begins it, one that does not may still execute it; not sent again\n"), err
        assert cli.main(["param", "--node", "t/twin/node", "--set", "x:=1.0", *endpoint]) == 1
        err = capsys.readouterr().err
        assert err.startswith("xblick param: ") and "2 nodes answer" in err and err.count("\n") == 1, err
        time.sleep(1.2)
        assert writes == [9.0]
    finally:
        params.close()
        for twin in twins:
            twin.close()


# ------------------------------------------------------------------- the check for a second node: what it catches
# The check before a command (session.count) is best effort: it catches two nodes that both answer the read, and a
# node that has not finished the read at its timeout (the "Timeout" reply that the caller's session adds). It does
# not catch a second node that drops the read, that appears after it, or a second torque_step server without
# parameters: those get the command (pinned below).

def busy(params, name="x"):
    """Keeps a Parameters node busy for 3 s: a local set whose writer blocks, so that its services answer late."""
    from xblick.msg import Parameter, ParameterValue, SetParametersRequest

    threading.Thread(target=lambda: params.respond("set_parameters", SetParametersRequest(
        [Parameter(name, ParameterValue.of(1.0))])), daemon=True).start()
    time.sleep(0.2)


def knows_all(session, key: str, nodes: int, timeout: float = 5.0, read: float = 1.0):
    """Waits until a read of ``key`` (each with timeout ``read``) from ``session`` is answered by all ``nodes``
    serving sessions. A session just opened behind a router links to the serving sessions one after the other and
    learns each one's queryables over its link: until then a query reaches only some of them (seen in CI: the busy one
    alone, so that the check before a set saw only its late reply)."""
    end = time.monotonic() + timeout
    while len(got := [r for r in replies(session, key, read) if r.ok is not None]) < nodes:
        assert time.monotonic() < end, f"{key}: {len(got)} of {nodes} nodes reached within {timeout:g} s"
        time.sleep(0.05)


def test_a_busy_second_node_gets_no_set(network):
    """Two nodes of one name, the second busy: the read is still open at its timeout ("Timeout"), so the set is not
    sent; the first node's writer ran 0 times, the second's only for its own local set."""
    from xblick.parameters import ParameterClient, Parameters

    writes = {"a": [], "b": []}
    second = network.second_server()
    a = Parameters(network.server, "node", "t/busy")
    a.declare("x", 0.0, write=lambda v: writes["a"].append(v))
    b = Parameters(second, "node", "t/busy")
    b.declare("x", 0.0, write=lambda v: (writes["b"].append(v), time.sleep(3.0)))
    try:
        with network.new_session() as s:
            knows_all(s, "t/busy/node/list_parameters", 2)
            busy(b)
            with pytest.raises(RuntimeError, match="not sent: 1 node\\(s\\) answered t/busy/node/list_parameters, and "
                                                   "the query was still open at its timeout \\(a second node, or the "
                                                   "answering node"):
                ParameterClient(s, "t/busy/node", 1.0).set({"x": 2.0})
        time.sleep(3.0)
        assert writes == {"a": [], "b": [1.0]}
    finally:
        a.close()
        b.close()


def test_a_command_answered_by_one_node_while_another_is_late(network):
    """If a second node gets slow only after the read, the command's own result has one answer and the "Timeout"
    reply: PartlyAnswered (a NoAnswer, no TimeoutError), never success. Both ran it once."""
    from xblick.session import PartlyAnswered

    second = network.second_server()
    network.serve("t/both", lambda q: b"done", name="fast")
    network.serve("t/both", lambda q: b"late", delay=1.5, server=second, name="slow")
    with network.new_session() as s:
        knows_all(s, "t/both", 2, read=2.0)
        network.calls.update(fast=0, slow=0)
        with pytest.raises(PartlyAnswered, match="t/both: answered by 1 node\\(s\\), and the query was still open "
                                                 "after 0.5 s "
                                                 ".*a second node may still execute it"):
            command(s, "t/both", 0.5)
    time.sleep(1.5)
    assert network.calls == {"fast": 1, "slow": 1}
    assert issubclass(PartlyAnswered, NoAnswer) and not issubclass(PartlyAnswered, TimeoutError)


@pytest.mark.parametrize("read", ["slow", "silent"])
def test_a_second_manager_that_answers_its_read_late_or_not_at_all(network, read):
    """A second manager whose list_controllers is slower than the timeout: not sent (0 and 0). One that drops the
    read (no reply at all) is not seen by the check: it gets the switch, the caller learns it afterwards (pinned)."""
    second = network.second_server()
    manager(network)
    network.serve(f"{M}/list_controllers", (lambda q: bytes(ListControllersResponse([STATE]).serialize())) if read ==
                  "slow" else (lambda q: None), delay=3.0 if read == "slow" else 0.0, server=second,
                  name="second list")
    network.serve(f"{M}/switch_controller", lambda q: bytes(SwitchControllerResponse(True, "b").serialize()),
                  server=second, name="second switch")
    # declared last on each serving session: a session that reaches both has their managers' queryables too (one link
    # per serving session, its declarations in order); the second's list cannot tell (late, or no reply)
    for server, name in ((network.server, "known"), (second, "second known")):
        network.serve("t/known", server=server, name=name)
    with network.new_session() as s:
        knows_all(s, "t/known", 2)  # else the second manager "appears later" to the check (best effort)
        m = controller_manager.ControllerManager(s, "robot", timeout=1.0)
        if read == "slow":
            with pytest.raises(RuntimeError, match="not sent: 1 node\\(s\\) answered .*list_controllers, and the "
                                                   "query was still open at its timeout"):
                m.switch(activate=["arms"])
        else:
            with pytest.raises(SeveralExecuted, match="switch_controller: sent, and 2 nodes answered: each of them "
                                                      "executed it"):
                m.switch(activate=["arms"])
    time.sleep(0.5)
    ran = 0 if read == "slow" else 1
    assert network.calls[f"{M}/switch_controller"] == ran and network.calls["second switch"] == ran


def test_a_second_torque_step_server_without_parameters_gets_the_step(network):
    """The torque step's check counts the controller's parameter servers, not its torque_step servers: a second
    torque_step server without parameters (an older bridge) gets the step; the caller learns it afterwards."""
    controller(network)
    second = network.second_server()
    network.serve("robot/controller/arms/torque_step", lambda q: bytes(TorqueStepResponse(True, "").serialize()),
                  server=second, name="second step")
    with network.new_session() as s:
        with pytest.raises(SeveralExecuted, match="robot/controller/arms/torque_step: sent, and 2 nodes answered"):
            controller_manager.torque_step(s, "robot/controller/arms", "j", 1.0, 0.1)
    assert network.calls["robot/controller/arms/torque_step"] == 1 and network.calls["second step"] == 1


def test_a_read_slower_than_its_timeout_stops_the_command(network):
    """A manager whose read takes longer than the timeout: "not sent" after about the timeout (it went out before
    0.5.2: fails safe); its switch ran 0 times."""
    network.serve(f"{M}/list_controllers", lambda q: bytes(ListControllersResponse([STATE]).serialize()), delay=1.5)
    network.serve(f"{M}/switch_controller", lambda q: bytes(SwitchControllerResponse(True, "").serialize()))
    with network.new_session() as s:
        started = time.monotonic()
        with pytest.raises(TimeoutError, match="not sent: no node answered robot/controller_manager/"
                                               "list_controllers in time"):
            controller_manager.ControllerManager(s, "robot", timeout=1.0).switch(activate=["arms"])
        took = time.monotonic() - started
    assert took < 1.0 + 0.6, took
    time.sleep(1.0)
    assert network.calls[f"{M}/switch_controller"] == 0


def test_send_command_goes_through_request(network):
    """recording.send_command is request (checked, sent once) and then the node's status."""
    recorder(network, lambda q: json.dumps({"v": recording.REPLY_VERSION, "ok": True, "result": {}}).encode())
    with network.new_session() as s:
        assert recording.send_command(s, "rec", "record") == {"state": "idle", "play": None}
        with pytest.raises(TimeoutError, match="not sent: no node answered xblick/nobody/status"):
            recording.request(s, "nobody", "record", timeout=0.5)
    assert network.calls["xblick/rec/control"] == 1


@pytest.mark.skipif(not __import__("os").environ.get("XBLICK_CPP_BUILD"), reason="XBLICK_CPP_BUILD is not set")
def test_a_cpp_set_with_a_busy_second_node(network):
    """xblick-cpp-probe --set while a second node of the name is busy: not sent (the C++ check), writers 0 and only
    the second's own local set."""
    import os
    import subprocess

    from xblick.parameters import Parameters

    writes = {"a": [], "b": []}
    second = network.second_server()
    a = Parameters(network.server, "node", "t/cbusy")
    a.declare("x", 0.0, write=lambda v: writes["a"].append(v))
    b = Parameters(second, "node", "t/cbusy")
    b.declare("x", 0.0, write=lambda v: (writes["b"].append(v), time.sleep(5.0)))
    probe = os.path.join(os.environ["XBLICK_CPP_BUILD"], "xblick-cpp-probe")
    try:
        time.sleep(0.5)
        busy(b)
        out = subprocess.run([probe, "--connect", network.endpoint[0], "--listen", "tcp/127.0.0.1:0", "--params",
                              "t/cbusy/node", "--set", "x:=2.0"], capture_output=True, text=True, timeout=30)
        assert out.returncode != 0 and "not sent: 1 node(s) answered t/cbusy/node/list_parameters, and the query " \
                                       "was still open at its timeout" in \
            out.stderr, out.stderr
        time.sleep(5.0)
        assert writes == {"a": [], "b": [1.0]}
    finally:
        a.close()
        b.close()


def test_one_slow_node_is_not_taken_for_a_second(network):
    """A single node whose count read answers only late: "not sent: no node answered ... in time", a TimeoutError
    that names no second node; its command ran 0 times."""
    from xblick.parameters import ParameterClient, Parameters

    writes = []
    a = Parameters(network.server, "node", "t/slow1")
    a.declare("x", 0.0, write=lambda v: (writes.append(v), time.sleep(3.0)))
    try:
        time.sleep(0.5)
        busy(a)
        with network.new_session() as s:
            with pytest.raises(TimeoutError, match="not sent: no node answered t/slow1/node/list_parameters in time"):
                ParameterClient(s, "t/slow1/node", 1.0).set({"x": 2.0})
        time.sleep(3.0)
        assert writes == [1.0]  # its own local set only
    finally:
        a.close()


def test_a_node_that_holds_the_query_after_its_reply(network):
    """The receiver rule, pinned: a single manager that replies at once and then keeps working inside the query's
    callback keeps the query open until its timeout, and the caller's session adds the "Timeout" reply; the check
    takes it for a possible second node and sends nothing ("a second node, or the answering node still holding the
    query"). Its switch ran 0 times."""
    def reply_then_hold(payload):
        def handle(query):
            query.reply(query.key_expr, payload)
            time.sleep(1.5)  # still inside the callback: the query is not finalized

        return handle

    network.server.declare_queryable(f"{M}/list_controllers",
                                     reply_then_hold(bytes(ListControllersResponse([STATE]).serialize())))
    network.serve(f"{M}/switch_controller", lambda q: bytes(SwitchControllerResponse(True, "").serialize()))
    with network.new_session() as s:
        time.sleep(0.5)
        with pytest.raises(RuntimeError, match="not sent: 1 node\\(s\\) answered .*list_controllers, and the query was "
                                               "still open at its timeout \\(a second node, or the answering node "
                                               "still holding the "
                                               "query after its reply\\)"):
            controller_manager.ControllerManager(s, "robot", timeout=0.5).switch(activate=["arms"])
    time.sleep(1.5)
    assert network.calls[f"{M}/switch_controller"] == 0


def test_a_command_reaches_every_queryable_of_its_key():
    """"Sent once to each queryable of its key" holds because Zenoh's default query target (BestMatching) delivers a
    query to every matching queryable while none of them is declared complete. Pinned in the sources: no Durchblick
    queryable sets ``complete`` and no query sets a ``target``. The behaviour itself, two queryables that both get a
    command, is pinned by test_a_second_torque_step_server_without_parameters_gets_the_step."""
    import re
    from pathlib import Path

    root = Path(__file__).resolve().parents[1]
    files = [*root.glob("src/xblick/**/*.py"), *root.glob("cpp/**/*.cpp"), *root.glob("cpp/**/*.hpp")]
    assert len(files) > 20
    pattern = re.compile(r"\bcomplete\s*=(?!=)|\.complete\b|QueryTarget|query_target|Z_QUERY_TARGET|\.target\s*=(?!=)|"
                         r"\btarget\s*=\s*zenoh")
    found = [f"{f.relative_to(root)}:{n}: {line.strip()}" for f in files
             for n, line in enumerate(f.read_text().splitlines(), 1) if pattern.search(line)]
    assert found == [], "\n".join(found)


# ------------------------------------------------------------------------ commands: node and deadline (0.5.13)
# A command whose read before it was answered by a receiver of 0.5.13 names that node and carries a deadline in its
# query's attachment (session.ask_once); such a receiver refuses a command meant for another node and one whose
# deadline has passed, just before the effect (session.CommandQuery). An older receiver never looks at the
# attachment, an older sender never at a reply's: the mixed cases below use queryables that ignore them, as the
# programs of 0.5.0 to 0.5.12 do.

from xblick import session as zs  # noqa: E402
from xblick.session import CommandTarget, Expired, node_id, one_node  # noqa: E402

SET = "set_parameters"


def set_request(name: str, value) -> bytes:
    from xblick.msg import Parameter, ParameterValue, SetParametersRequest

    return bytes(SetParametersRequest([Parameter(name, ParameterValue.of(value))]).serialize())


def record_of(attachment) -> dict | None:
    return None if attachment is None else json.loads(attachment.to_bytes())


def test_the_command_names_are_the_protocols():
    """session.py's names of the attachment and of the refusals are protocol.hpp's (xblick._core.protocol)."""
    from xblick import _core

    P = _core.protocol
    assert (zs.COMMAND_FIELD, zs.COMMAND_VERSION, zs.EXPIRED, zs.NOT_TARGET, zs.INVALID_COMMAND) == (
        P.COMMAND_FIELD, P.COMMAND_VERSION, P.EXPIRED, P.NOT_TARGET, P.INVALID_COMMAND)


def test_the_target_and_its_deadline_on_the_nodes_clock():
    """target_of: the node's clock offset is its clock in the read's answer minus this program's clock when the read
    ended, the least the true offset can be; the deadline on the node's clock is never later than ours. A node far
    ahead or behind (another PC, another boot) shifts it by that much: no clock needs to be set."""
    import zenoh

    class Answer:  # a reply's ok as target_of reads it
        key_expr = "t/n/list_parameters"

        def __init__(self, record):
            self.attachment = None if record is None else zenoh.ZBytes(json.dumps(record).encode())

    now = zs.command_clock_ns()
    for ahead in (0, 5 * 10**12, -3 * 10**12):
        to = zs.target_of(Answer({zs.COMMAND_FIELD: 1, "zid": "ab12", "clock_ns": now + ahead, "checked": []}), now)
        assert to == CommandTarget("ab12", ahead)
        before = zs.command_clock_ns()
        deadline = to.deadline_ns(0.5)
        after = zs.command_clock_ns()
        assert before + ahead + 500_000_000 <= deadline <= after + ahead + 500_000_000
    assert zs.target_of(Answer(None), now) is None  # an older receiver: no target, nothing attached
    assert zs.target_of(Answer({"v": 1, "other": "program"}), now) is None  # another attachment: not ours
    for bad in ({zs.COMMAND_FIELD: 2, "zid": "ab", "clock_ns": 1}, {zs.COMMAND_FIELD: 1, "zid": "ab"}):
        with pytest.raises(RuntimeError, match="not sent: the answer of t/n/list_parameters has a xblick_command "
                                               "attachment this program cannot read"):
            zs.target_of(Answer(bad), now)


def test_a_receiver_checks_node_then_deadline():
    """CommandQuery.check on queries as a receiver gets them: nothing to check without the attachment (an older sender)
    or with another program's; refused when meant for another node, past its deadline, or unreadable."""
    import zenoh

    class Query:
        def __init__(self, attachment):
            self.attachment = None if attachment is None else zenoh.ZBytes(attachment)

    def check(attachment, zid="me"):
        q = zs.CommandQuery(Query(attachment), zid, "t/k")
        q.check()
        return q.checked

    later = zs.command_clock_ns() + 10**10
    assert check(None) == [] and check(b"\x00\x01binary") == [] and check(b'{"seq": 1}') == []
    assert check(json.dumps({zs.COMMAND_FIELD: 1, "node": "me", "deadline_ns": later}).encode()) == ["node", "deadline"]
    cases = [({zs.COMMAND_FIELD: 1, "node": "other", "deadline_ns": later}, zs.NOT_TARGET,
              "t/k: not executed here: it is meant for the node of session other, this is me"),
             ({zs.COMMAND_FIELD: 1, "node": "me", "deadline_ns": zs.command_clock_ns() - 1}, zs.EXPIRED,
              "t/k: not executed: its deadline had passed 0.0"),
             ({zs.COMMAND_FIELD: 2, "node": "me", "deadline_ns": later}, zs.INVALID_COMMAND,
              "t/k: not executed: its xblick_command attachment is of version 2"),
             ({zs.COMMAND_FIELD: 1, "node": "me"}, zs.INVALID_COMMAND, "t/k: not executed: its xblick_command"),
             (b'{"xblick_command": 1, "node": "me", "deadline', zs.INVALID_COMMAND,
              "t/k: not executed: its xblick_command attachment is no JSON this node reads")]
    for record, schema, text in cases:
        with pytest.raises(zs.CommandRefused) as refused:
            check(record if isinstance(record, bytes) else json.dumps(record).encode())
        assert refused.value.schema == schema and str(refused.value).startswith(text), str(refused.value)


def test_a_set_names_its_node_and_deadline(network):
    """A set to a Parameters node of 0.5.13: its read answer names the serving session and its clock, the set's query
    names that node and a deadline no later than the end of the client's wait, and the answer says both were
    checked (ParameterClient.last_checked). A second queryable on the set key in the same session (a spy that
    replies nothing and drops the query) sees the attachment."""
    from xblick.parameters import ParameterClient, Parameters

    seen = []
    params = Parameters(network.server, "node", "t/named")
    params.declare("x", 0.0)
    spy = network.server.declare_queryable(f"t/named/node/{SET}", lambda q: seen.append(record_of(q.attachment)))
    try:
        with network.new_session() as s:
            knows_all(s, "t/named/node/list_parameters", 1)
            (read,) = [r.ok for r in replies(s, "t/named/node/list_parameters", 1.0) if r.ok is not None]
            answer = record_of(read.attachment)
            assert answer[zs.COMMAND_FIELD] == 1 and answer["zid"] == node_id(network.server)
            assert answer["checked"] == [] and isinstance(answer["clock_ns"], int)
            client = ParameterClient(s, "t/named/node", 2.0)
            before = zs.command_clock_ns()
            assert client.set({"x": 1.0})[0].successful and client.last_checked
            after = zs.command_clock_ns()
        (sent,) = seen
        assert sent["node"] == node_id(network.server)
        # the same PC: the node's clock is this one; the deadline at most the client's own (2 s after sending)
        assert before + 2 * 10**9 - 10**9 < sent["deadline_ns"] <= after + 2 * 10**9
        assert params["x"] == 1.0
    finally:
        spy.undeclare()
        params.close()


def test_an_older_receiver_gets_todays_command(network):
    """New sender, older receiver: a read answered without the attachment (a queryable that knows nothing of it) gives
    no target, the command goes out without any attachment, exactly as before 0.5.13, and the caller does not claim
    a check (last_checked False)."""
    seen = []
    network.serve(f"{M}/list_controllers", lambda q: bytes(ListControllersResponse([STATE]).serialize()))
    network.serve(f"{M}/switch_controller", lambda q: (seen.append(q.attachment),
                                                        bytes(SwitchControllerResponse(True, "ok").serialize()))[1])
    with network.new_session() as s:
        m = controller_manager.ControllerManager(s, "robot", timeout=2.0)
        assert m.switch(activate=["arms"]) == "ok" and not m.last_checked
    assert seen == [None] and network.calls[f"{M}/switch_controller"] == 1


def test_an_older_receiver_ignores_the_attachment(network):
    """New sender, a read answered by a receiver of 0.5.13 and the command served by a queryable that never looks at
    its attachment (an older torque_step server beside new parameters): executed once, success as before, and the
    caller does not claim a check. The case "an older receiver ignores the attachment"."""
    zid = node_id(network.server)
    network._queryables.append(network.server.declare_queryable(  # the read answered as by a receiver of 0.5.13
        "robot/controller/arms/list_parameters",
        lambda q: q.reply(q.key_expr, bytes(ListParametersResponse(["level"], []).serialize()),
                          attachment=zs.answer_attachment(zid))))
    attachments = []
    network.serve("robot/controller/arms/torque_step",
                  lambda q: (attachments.append(record_of(q.attachment)),
                             bytes(TorqueStepResponse(True, "").serialize()))[1])
    with network.new_session() as s:
        controller_manager.torque_step(s, "robot/controller/arms", "j", 1.0, 0.1)
        to = one_node(s, "robot/controller/arms/list_parameters", 2.0)
        ok = command(s, "robot/controller/arms/torque_step", 2.0, to, payload=b"x")
        assert not zs.checked(ok[0], to)
    assert network.calls["robot/controller/arms/torque_step"] == 2
    assert [a["node"] for a in attachments] == [zid, zid]  # it got the attachment and executed all the same


def test_an_older_sender_is_served_as_before(network):
    """Older sender, new receiver: a set without the attachment (as 0.5.0 to 0.5.12 send it: no target) is applied as
    before, and the reply's attachment (which such a sender never reads) says nothing was checked."""
    from xblick.parameters import Parameters

    params = Parameters(network.server, "node", "t/old")
    params.declare("x", 0.0)
    try:
        with network.new_session() as s:
            knows_all(s, "t/old/node/list_parameters", 1)
            (ok,) = command(s, f"t/old/node/{SET}", 2.0, payload=set_request("x", 4.0))
            assert record_of(ok.attachment)["checked"] == [] and not zs.checked(ok, None)
            (raw,) = [r for r in replies(s, f"t/old/node/{SET}", 2.0, payload=set_request("x", 5.0),
                                         attachment=b"\x01not json")]  # another program's attachment
            assert raw.ok is not None
        assert params["x"] == 5.0
    finally:
        params.close()


@pytest.mark.parametrize("server", ["parameters", "recorder"])
def test_an_expired_command_is_not_executed(network, server, tmp_path):
    """A command whose deadline has passed when its node gets to it: Expired (a RuntimeError, no TimeoutError, no
    NoAnswer), and the handler was never called: the writer did not run (the value is unchanged), the recorder did not
    start (idle, no file). The target is the node itself, its clock moved back 10 s so that the deadline lies in the
    past on arrival."""
    writes = []
    if server == "parameters":
        from xblick.parameters import Parameters

        node = Parameters(network.server, "node", "t/exp")
        node.declare("x", 0.0, write=writes.append)
        read_key, key, payload = "t/exp/node/list_parameters", f"t/exp/node/{SET}", set_request("x", 1.0)
    else:
        node = recording.Recorder(network.server, tmp_path, name="exp")
        read_key, key, payload = "xblick/exp/status", "xblick/exp/control", json.dumps({"command": "record"})
    try:
        with network.new_session() as s:
            knows_all(s, read_key, 1)
            to = one_node(s, read_key, 2.0)
            assert to.node == node_id(network.server)
            late = CommandTarget(to.node, to.offset_ns - 10 * 10**9)
            with pytest.raises(Expired, match=f"{key}: refused as expired: {key}: not executed: its deadline had "
                                              "passed 8\\.") as error:
                command(s, key, 2.0, late, payload=payload)
            assert not isinstance(error.value, (TimeoutError, NoAnswer))
            if server == "parameters":
                assert node["x"] == 0.0 and writes == []
            else:
                assert recording.query_status(s, "exp")["state"] == "idle"
        assert list(tmp_path.iterdir()) == []
    finally:
        node.close()


def test_a_busy_node_does_not_execute_a_set_after_the_sender_gave_up(network, caplog):
    """Issue #1's case, with a check as late as possible: the node gets the set in time, but a step before its effect
    (here the parameter's reader, which the set calls first) takes until after the sender gave up (NoAnswer). The
    check right before the writer finds the deadline passed: the writer never runs. Before 0.5.13 it ran after the
    caller's NoAnswer (test_late_execution_after_the_caller_gave_up)."""
    import logging

    from xblick.parameters import ParameterClient, Parameters

    writes, entered, release = [], threading.Event(), threading.Event()
    armed = threading.Event()

    def read():
        if armed.is_set():
            entered.set()
            release.wait(10.0)
        return 0.0

    params = Parameters(network.server, "node", "t/late")
    params.declare("x", 0.0, read=read, write=writes.append)
    try:
        with network.new_session() as s, caplog.at_level(logging.WARNING, logger="xblick"):
            knows_all(s, "t/late/node/list_parameters", 1)
            armed.set()
            with pytest.raises(NoAnswer, match="t/late/node/set_parameters: sent once with a deadline, no answer "
                                               "within 0.5 s: it may have been executed; a receiver that checks the "
                                               "deadline"):
                ParameterClient(s, "t/late/node", 0.5).set({"x": 1.0})
            assert entered.is_set()  # the node had the set before the sender gave up
            armed.clear()
            release.set()
            # the node answers queries one after another: a get answered means the set's handling has ended
            assert ParameterClient(s, "t/late/node", 5.0).get(["x"])[0].value() == 0.0
        assert writes == []
        assert any("t/late/node/set_parameters: not executed: its deadline had passed" in r.getMessage()
                   for r in caplog.records)
    finally:
        release.set()
        params.close()


@pytest.mark.parametrize("second", ["new", "old"])
def test_a_second_node_the_read_did_not_see(network, second, caplog):
    """Two nodes on one key, the read before the set answered by one only: here the second starts after the read
    (made so, not by timing: the read and the set are sent by hand, the second node declared between them, and the
    session waits until it reaches it). A second node of 0.5.13 refuses the set (not meant for it): only the named node
    applies it, the caller gets its answer and a warning. A second node that does not check (older) applies it too:
    SeveralExecuted, as before 0.5.13."""
    import logging

    from xblick.parameters import Parameters

    writes = {"a": [], "b": []}
    a = Parameters(network.server, "node", "t/late2")
    a.declare("x", 0.0, write=writes["a"].append)
    b = None
    try:
        with network.new_session() as s, caplog.at_level(logging.WARNING, logger="xblick"):
            knows_all(s, "t/late2/node/list_parameters", 1)
            to = one_node(s, "t/late2/node/list_parameters", 2.0)
            other = network.second_server()
            if second == "new":
                b = Parameters(other, "node", "t/late2")
                b.declare("x", 0.0, write=writes["b"].append)
            else:
                from xblick.msg import SetParametersResponse

                network.serve("t/late2/node/list_parameters",
                              lambda q: bytes(ListParametersResponse(["x"], []).serialize()), server=other,
                              name="old list")
                network.serve(f"t/late2/node/{SET}", lambda q: (writes["b"].append(1.0), bytes(
                    SetParametersResponse([]).serialize()))[1], server=other, name="old set")
            knows_all(s, "t/late2/node/list_parameters", 2)
            if second == "new":
                (ok,) = command(s, f"t/late2/node/{SET}", 2.0, to, payload=set_request("x", 1.0))
                assert zs.checked(ok, to)
                assert any("t/late2/node/set_parameters: executed by its node; 1 other node(s) on the key refused it "
                           "as not meant for them" in r.getMessage() for r in caplog.records)
            else:
                with pytest.raises(SeveralExecuted, match="sent, and 2 nodes answered: each of them executed it"):
                    command(s, f"t/late2/node/{SET}", 2.0, to, payload=set_request("x", 1.0))
        assert writes == {"a": [1.0], "b": [] if second == "new" else [1.0]}
    finally:
        a.close()
        if b is not None:
            b.close()


def test_a_command_meant_for_a_node_that_is_gone(network):
    """The named node gone (here: a target that names no serving session), another node of 0.5.13 on the key: it
    refuses, nothing is executed, and the caller gets NoAnswer (the named node's answer is missing) that names the
    refusal."""
    from xblick.parameters import Parameters

    writes = []
    params = Parameters(network.server, "node", "t/gone")
    params.declare("x", 0.0, write=writes.append)
    try:
        with network.new_session() as s:
            knows_all(s, "t/gone/node/list_parameters", 1)
            with pytest.raises(NoAnswer, match="no answer within 1 s: .*; 1 other node\\(s\\) refused it as not meant "
                                               "for them"):
                command(s, f"t/gone/node/{SET}", 1.0, CommandTarget("0" * 32, 0), payload=set_request("x", 1.0))
        assert writes == [] and params["x"] == 0.0
    finally:
        params.close()


@pytest.mark.skipif(not __import__("os").environ.get("XBLICK_CPP_BUILD"), reason="XBLICK_CPP_BUILD is not set")
@pytest.mark.parametrize("case", ["checked", "late", "second"])
def test_a_cpp_set_with_node_and_deadline(network, case):
    """C++ ParameterClient (xblick-cpp-probe --set) against Python nodes: "checked" (the set names its node and a
    deadline, and the answer confirms the check), "late" (the node's reader holds the set until the probe gave up: the
    writer never runs, the probe says "sent once with a deadline"), "second" (a second node of 0.5.13 on the set key
    that the read cannot see, as it serves no list_parameters: it refuses, only the named node applies it). The second
    node is the router's own session: the probe knows its queryables from its first link on, before it can reach
    the named node, so it surely gets the set."""
    import os
    import subprocess

    from xblick.parameters import ParameterClient, Parameters

    writes, seen, release, held = [], [], threading.Event(), threading.Event()

    def read():
        if held.is_set():
            release.wait(10.0)
        return 0.0

    params = Parameters(network.server, "node", "t/cpp13")
    params.declare("x", 0.0, write=writes.append, read=read if case == "late" else None)
    extra = []
    if case == "checked":  # a spy beside the node (same session, no reply): it sees the attachment
        extra.append(network.server.declare_queryable(f"t/cpp13/node/{SET}",
                                                      lambda q: seen.append(record_of(q.attachment))))
    if case == "second":
        zid = node_id(network.router)

        def second(query):
            received = zs.CommandQuery(query, zid, f"t/cpp13/node/{SET}")
            try:
                received.check()
            except zs.CommandRefused as refused:
                seen.append(refused.schema)
                zs.refuse(query, refused)
                return
            writes.append("second")
            query.reply(query.key_expr, b"")

        extra.append(network.router.declare_queryable(f"t/cpp13/node/{SET}", second))
    probe = os.path.join(os.environ["XBLICK_CPP_BUILD"], "xblick-cpp-probe")
    try:
        if case == "late":
            held.set()
        out = subprocess.run([probe, "--connect", network.endpoint[0], "--listen", "tcp/127.0.0.1:0", "--params",
                              "t/cpp13/node", "--set", "x:=1.0"], capture_output=True, text=True, timeout=30)
        if case == "late":
            assert out.returncode != 0 and ("set_parameters: sent once with a deadline, no answer within 2000 ms"
                                            in out.stderr), out.stderr
            held.clear()
            release.set()
            with network.new_session() as s:  # answered after the set's handling ended (one query at a time)
                assert ParameterClient(s, "t/cpp13/node", 5.0).get(["x"])[0].value() == 0.0
            assert writes == []
        else:
            assert out.returncode == 0, out.stderr
            assert json.loads(out.stdout.splitlines()[0])["checked"] is True
            assert writes == [1.0]
            if case == "checked":
                assert seen[0]["node"] == node_id(network.server) and isinstance(seen[0]["deadline_ns"], int)
            else:
                assert seen == [zs.NOT_TARGET] and "1 other node(s) on the key refused it" in out.stderr, out.stderr
    finally:
        release.set()
        for q in extra:
            q.undeclare()
        params.close()
