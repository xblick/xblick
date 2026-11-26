"""Opening a Zenoh session the way Durchblick uses it.

Default: peer mode, multicast scouting off, the PC's Zenoh router. Every PC that runs Durchblick runs one router
(zenohd) on port 7447 (docs/concepts/sessions.md, "Zenoh routers"): without endpoints, a session connects to it at
127.0.0.1:7447 and listens on a free loopback port. Through the router the sessions on one PC find each other and
connect directly (images stay in shared memory or on the GPU, only their descriptors pass Zenoh); between PCs the
routers carry the data. On the robot network nothing is found by multicast by accident. Explicit endpoints
(``--connect``/``--listen``) replace the router (two programs connected directly, a test); a Zenoh JSON5
configuration file replaces everything (``config_file``).
"""

from __future__ import annotations

import dataclasses
import ipaddress
import json
import logging
import re
import socket
import sys
import threading
import time
import weakref
from collections.abc import Callable, Sequence
from pathlib import Path

import zenoh

DEFAULT_PORT = 7447
ROUTER = f"tcp/127.0.0.1:{DEFAULT_PORT}"  # the PC's router
LOCAL_LISTEN = "tcp/127.0.0.1:0"         # a free loopback port: the PC's other sessions connect to it directly


def endpoint(address: str, port: int = DEFAULT_PORT) -> str:
    """A Zenoh endpoint from what a person types: ``host`` and ``host:port`` take ``tcp/``, and an address without a
    port takes ``port`` (``robot-pc`` and ``tcp/robot-pc`` are ``tcp/robot-pc:7447``; ``[::1]`` is
    ``tcp/[::1]:7447``). A locator with a port and a protocol without host and port (``unixsock-stream/...``) are
    returned unchanged; so is anything after ``#`` (Zenoh's endpoint configuration)."""
    address = address.strip()
    if not address:
        raise ValueError("an empty Zenoh endpoint")
    locator, hash_, config = address.partition("#")
    protocol, slash, rest = locator.partition("/")
    if not slash:
        protocol, rest = "tcp", locator
    if protocol not in ("tcp", "udp", "tls", "quic", "ws"):
        return address  # unixsock-stream/<path>, serial/...: no port
    if rest.startswith("["):  # IPv6: [addr] or [addr]:port
        has_port = "]:" in rest
    else:
        if rest.count(":") > 1:
            raise ValueError(f"Zenoh endpoint {address!r}: an IPv6 address goes in brackets, e.g. tcp/[::1]:7447")
        has_port = ":" in rest
    if not rest or rest.endswith(":"):
        raise ValueError(f"Zenoh endpoint {address!r}: no host or no port after ':'")
    return f"{protocol}/{rest}{'' if has_port else f':{port}'}{hash_}{config}"


def make_config(connect: Sequence[str] = (), listen: Sequence[str] = (), mode: str = "peer",
                multicast_scouting: bool = False, config_file: str | None = None,
                local_router: bool = True) -> zenoh.Config:
    """Neither ``connect`` nor ``listen`` (nor a config file), with ``local_router``: the PC's router (``ROUTER``),
    listening on ``LOCAL_LISTEN``. A peer with ``connect`` and no ``listen`` listens on ``LOCAL_LISTEN`` as well.
    ``local_router=False`` without endpoints: a session that reaches nobody."""
    if config_file:
        if connect or listen:
            raise ValueError("give either a Zenoh config file or connect/listen endpoints, not both")
        return zenoh.Config.from_file(config_file)
    if mode not in ("peer", "client"):
        raise ValueError(f"mode {mode!r}: peer or client")
    if local_router and not connect and not listen:
        connect, listen = [ROUTER], [LOCAL_LISTEN] if mode == "peer" else []
    elif mode == "peer" and connect and not listen:
        # A peer always listens: behind a Zenoh 1.10 router (zenohd or xblick router) peers reach each other only by
        # connecting directly (gossip scouting; the peers of a router form a clique), the router does not route
        # between them; a peer that only connected found nothing (before 0.5.3).
        listen = [LOCAL_LISTEN]
    if mode == "client" and not connect:
        raise ValueError("client mode needs an endpoint to connect to (a router or a peer)")
    if mode == "client" and listen:
        raise ValueError("client mode does not listen: give connect endpoints only (or peer mode)")
    config = zenoh.Config()
    config.insert_json5("mode", json.dumps(mode))
    config.insert_json5("scouting/multicast/enabled", json.dumps(bool(multicast_scouting)))
    config.insert_json5("connect/endpoints", json.dumps([endpoint(e) for e in connect]))
    if mode == "peer":
        config.insert_json5("listen/endpoints", json.dumps([endpoint(e) for e in listen]))
    return config


def open_session(connect: Sequence[str] = (), listen: Sequence[str] = (), mode: str = "peer",
                 multicast_scouting: bool = False, config_file: str | None = None,
                 local_router: bool = True, program: str | None = None, declare: bool = True,
                 ros_options: RosOptions | bool | None = None) -> Session:
    """A session (``make_config``). It keeps the endpoints of ``across_pcs`` for ``why_nothing``. OpenError (a
    zenoh.ZError) when Zenoh cannot open it, with a message that says why in one line.

    With ``declare`` (the default) the session declares its program on the network (declaration.py): ``program``
    (default ``program_name()``), its version (``declaration.program_version``), host, command line, process id and
    start time. ``declare=False``: a session that does not (a test standing in for an older program).

    The session takes part in ROS 2 as well (``Session.ros``, ros.py): a ROS node, and the publishers, subscribers
    and parameters made with the session offer themselves to ROS too (docs/integrations/ros2.md). ``ros_options``:
    None (the default) as ``XBLICK_ROS`` says (0: no part; 1 or unset: ``default_ros_options`` of ``program``),
    True those defaults, a RosOptions those, False no part: nothing of ROS is sent. RosError (a ValueError) for an
    ``XBLICK_ROS`` or ``ROS_DOMAIN_ID`` that is not understood."""
    from . import declaration, ros

    if ros_options is None:
        ros_options = ros.enabled_by_environment()
    if ros_options is True:
        ros_options = default_ros_options(program or program_name())
    config = make_config(connect, listen, mode, multicast_scouting, config_file, local_router)
    try:
        opened = zenoh.open(config)
    except zenoh.ZError as error:
        raise OpenError(open_error_text(str(error), connect, mode)) from error
    declared = None
    if declare:
        program = program or program_name()
        try:
            declared = declaration.Declaration(opened, program, declaration.program_version(program))
        except BaseException:
            close(opened)
            raise
    taking_part = None
    if ros_options:
        try:
            taking_part = ros.Ros(opened, ros_options.domain, ros_options.node, ros_options.namespace,
                                  params_writable=ros_options.params_writable,
                                  follow_parameters=ros_options.follow_parameters, clock=ros_options.clock)
        except BaseException:
            if declared is not None:
                declared.close()
            close(opened)
            raise
    wrapper = Session(opened, across_pcs(connect, listen, mode, config_file), declared, taking_part)
    if taking_part is not None:
        # ROS's daemon threads hold the Zenoh session, which a dropped Session would otherwise keep open until the
        # program ends; without ROS nothing holds it, and Zenoh closes it when it is collected. Not at exit: what ends
        # a program is not changed here
        finalizer = weakref.finalize(wrapper, _release, opened, declared, taking_part)
        finalizer.atexit = False
        object.__setattr__(wrapper, "_finalizer", finalizer)
    return wrapper


def _release(raw: zenoh.Session, declared, taking_part):
    """A Session in ROS 2 that its program dropped without closing it, closed as ``close`` closes one."""
    _close_parts(raw, declared, taking_part, CLOSE_TIMEOUT_S)


class OpenError(zenoh.ZError):
    """Zenoh could not open a session (``open_session``). A zenoh.ZError, as zenoh.open raises."""


def open_error_text(zenoh_text: str, connect: Sequence[str], mode: str) -> str:
    """One line for a session that Zenoh could not open: in client mode, that none of the ``connect`` endpoints
    answered and that a client needs its router there (Zenoh's "Unable to connect to any of ..."); otherwise Zenoh's
    own text, without the source locations it appends (`` at /.../file.rs:123.``)."""
    if mode == "client" and zenoh_text.startswith("Unable to connect"):
        return (f"could not connect to {' '.join(endpoint(e) for e in connect)}: a session in client mode needs its "
                "router there. Start xblick router (or zenohd) on that PC, or check the address and the port")
    text = re.sub(r"\s+at \S+\.rs:\d+\.?", "", zenoh_text).strip()
    return f"could not open the Zenoh session: {text}"


# ---------------------------------------------------------------------------------------------- across PCs

IP_PROTOCOLS = ("tcp", "udp", "tls", "quic", "ws")


def host_of(locator: str) -> str | None:
    """The host of an endpoint (after ``endpoint``), IPv6 without brackets; None for a locator without a host
    (``unixsock-stream/<path>``, ``serial/...``)."""
    protocol, _, rest = endpoint(locator).partition("#")[0].partition("/")
    if protocol not in IP_PROTOCOLS:
        return None
    if rest.startswith("["):
        return rest[1:rest.index("]")]
    return rest.rpartition(":")[0]


def _addresses(host: str) -> list[tuple]:
    """(family, sockaddr) of each address ``host`` resolves to; none when it does not resolve."""
    try:
        return [(family, sockaddr) for family, _, _, _, sockaddr in socket.getaddrinfo(host, None,
                                                                                      type=socket.SOCK_STREAM)]
    except (socket.gaierror, UnicodeError):
        return []


def _named_loopback(host: str) -> bool:
    return host.lower() == "localhost" or host.lower().endswith(".localhost")


def this_pc(host: str) -> bool:
    """True when ``host`` is this PC: ``localhost``, or a name or address that resolves to a loopback address, the
    unspecified address (0.0.0.0, ::) or an address of one of this PC's interfaces. An address is this PC's when a
    socket can bind to it, which sends nothing over the network. A name that does not resolve is not this PC."""
    if _named_loopback(host):
        return True
    for family, sockaddr in _addresses(host):
        ip = ipaddress.ip_address(sockaddr[0].split("%")[0])
        if ip.is_loopback or ip.is_unspecified:
            return True
        with socket.socket(family, socket.SOCK_STREAM) as s:
            try:
                s.bind((sockaddr[0], 0, *sockaddr[2:]))
                return True
            except OSError:
                pass
    return False


def loopback(host: str) -> bool:
    """True when ``host`` is ``localhost`` or every address it resolves to is a loopback address: a session that
    listens there is reachable from this PC only."""
    if _named_loopback(host):
        return True
    addresses = _addresses(host)
    return bool(addresses) and all(ipaddress.ip_address(a[0].split("%")[0]).is_loopback for _, a in addresses)


def across_pcs(connect: Sequence[str], listen: Sequence[str], mode: str, config_file: str | None = None,
               local: Callable[[str], bool] = this_pc, loopback_only: Callable[[str], bool] = loopback) -> list[str]:
    """The ``connect`` endpoints (as ``endpoint`` writes them) when the session is a peer that connects only to other
    PCs and listens on loopback only, else none. Behind a Zenoh 1.10 router peers reach each other only directly, so
    the programs on the other PCs usually cannot reach this one. They can when they listen on network addresses, which
    the options of this session do not show. Decided from the options alone: no option with this PC's own router (a
    ``connect`` endpoint on this PC, ``local``), no ``listen`` on a network address (``loopback_only``), no config
    file, peer mode. A locator without a host (``unixsock-stream/...``) counts as this PC."""
    if config_file or mode != "peer" or not connect:
        return []
    endpoints = [endpoint(e) for e in connect]
    hosts = [host_of(e) for e in endpoints]
    if any(h is None or local(h) for h in hosts):
        return []
    if any(h is not None and not loopback_only(h) for h in map(host_of, listen)):
        return []
    return endpoints


def across_pcs_text(endpoints: Sequence[str], prefix: str = "") -> str:
    """Why such a session (``across_pcs``) usually finds nothing, and the three ways out, with its endpoints: client
    mode, a router on this PC, or listening on the network (the peers there learn its address through their router and
    connect to it)."""
    given = " ".join(endpoints)
    return (f"connecting as a peer to another PC ({given}): programs there usually cannot reach this one, which "
            f"listens on loopback only. Use --{prefix}zenoh-mode client, or start a router on this PC with: xblick "
            f"router --connect {given}, and this program without --{prefix}connect, or make this program reachable from there "
            f"with --{prefix}listen tcp/0.0.0.0:0 (any program on the network can then connect to it)")


def program_name() -> str:
    """The program's name for its messages: the name it was started by (a node's console script)."""
    return Path(sys.argv[0]).name if sys.argv and sys.argv[0] else "python"


def add_arguments(parser, prefix: str = ""):
    """``--<prefix>connect``, ``--<prefix>listen``, ``--<prefix>zenoh-mode``, ``--<prefix>config`` for a script's
    argparse parser."""
    p = f"--{prefix}"
    parser.add_argument(f"{p}connect", nargs="*", default=[], metavar="ENDPOINT",
                        help="Zenoh endpoints to connect to, like tcp/192.0.2.10:7447; a host alone means "
                             "tcp/<host>:7447, host:port tcp/<host>:<port> (default, without --listen: this PC's "
                             "router, 127.0.0.1:7447)")
    parser.add_argument(f"{p}listen", nargs="*", default=[], metavar="ENDPOINT",
                        help="Zenoh endpoints to listen on, e.g. 0.0.0.0:7448 (default for a peer: a free loopback "
                             "port, so that the PC's other sessions connect to it directly)")
    parser.add_argument(f"{p}zenoh-mode", choices=("peer", "client"), default="peer",
                        help="peer (default): listens, and the PC's sessions connect to it directly; client: everything "
                             "through the router --connect names, which a session needs that connects to another PC's "
                             "router (a router does not route between peers)")
    parser.add_argument(f"{p}config", default=None, metavar="FILE",
                        help="a Zenoh JSON5 configuration file instead of the endpoints")
    parser.add_argument(f"{p}ros", action="store_true",
                        help="accepted and does nothing: every program takes part in ROS 2 by default (rmw_zenoh, "
                             f"ROS 2 Jazzy and Lyrical); --{prefix}no-ros turns it off")
    parser.add_argument(f"{p}no-ros", action="store_true",
                        help="take no part in ROS 2: no ROS node, nothing on ROS's keys (default: a part, as a ROS "
                             "node, beside our own keys, which stay as they are; XBLICK_ROS=0 turns it off too, "
                             "XBLICK_ROS=1 or unset leaves it on; docs/integrations/ros2.md)")
    parser.add_argument(f"{p}ros-domain", type=int, default=None, metavar="ID",
                        help="the ROS domain id (default: ROS_DOMAIN_ID, else 0)")
    parser.add_argument(f"{p}ros-node", default=None, metavar="NAME",
                        help="the program's ROS node, NAME or /namespace/NAME (default: the name of the program's "
                             "parameters when it has some, e.g. /test/camera/yolox; else the program's name in the "
                             "namespace of its --camera, every character other than a letter, digit or _ written as "
                             "_, e.g. /test/camera/xblick_test_camera)")
    parser.add_argument(f"{p}ros-params-writable", action="store_true",
                        help="ROS clients may set this program's parameters (default: they read them, "
                             "sets are refused). A ROS set is unchecked and uncounted: no node check, no deadline, "
                             "and two nodes of one name both execute it while the ROS client shows one success")
    parser.add_argument(f"{p}ros-clock", default=None, metavar="KEY",
                        help="a clock key this program publishes (rosgraph_msgs/Clock, e.g. sim/clock) goes to ROS's "
                             "global /clock instead of /KEY, for ROS nodes with use_sim_time (default: none)")
    # the program's name as the parser sees it (its prog): the default ROS node name (ros_options_from_args)
    parser.set_defaults(**{f"{prefix.replace('-', '_')}ros_program": parser.prog})


@dataclasses.dataclass(frozen=True)
class RosOptions:
    """How a session takes part in ROS 2 (``--ros-domain``, ``--ros-node``, ``--ros-params-writable``,
    ``--ros-clock``): its node ``<namespace>/<node>``; ``follow_parameters``: that name is a default, and the
    program's first parameters give the node theirs (ros.Ros.parameters_node)."""

    domain: int
    node: str
    params_writable: bool = False  # --ros-params-writable: ROS clients may set parameters
    namespace: str = "/"
    follow_parameters: bool = False
    clock: str | None = None       # --ros-clock: the key of ours that goes to ROS's /clock


def default_ros_options(program: str, camera: str | None = None) -> RosOptions:
    """A program's part in ROS 2 when nothing is given: the domain of ``ROS_DOMAIN_ID`` (else 0), the node the
    program's name (ros.node_name_of) in the namespace of ``camera`` (ros.namespace_of; ``/`` without), renamed after
    the program's first parameters."""
    from . import ros
    return RosOptions(ros.domain_from(None), ros.node_name_of(program),
                      namespace=ros.namespace_of(camera) if camera else "/", follow_parameters=True)


def ros_options_from_args(args, prefix: str = "", prog: str | None = None,
                          camera: str | None = None) -> RosOptions | None:
    """The ROS options of a script's arguments (``add_arguments``); None with ``--no-ros`` or ``XBLICK_ROS=0``.
    ``--ros`` changes nothing. The node name defaults to ``prog``, else the parser's prog (as the parser sees the
    program: a node started from ``python -c`` or a test gets its parser's name), else ``program_name()``, in the
    namespace of ``camera``, else of ``args.camera`` when that is one name (a node's ``--camera``), else ``/``; such a
    default name gives way to the name of the program's first parameters. ValueError (ros.RosError) for a domain or
    node name ROS does not take, an ``XBLICK_ROS`` other than 0 and 1, or for ``--ros-domain``, ``--ros-node``,
    ``--ros-params-writable`` or ``--ros-clock`` while ROS is off."""
    from . import ros
    key = prefix.replace("-", "_")
    domain, node = getattr(args, f"{key}ros_domain", None), getattr(args, f"{key}ros_node", None)
    writable = getattr(args, f"{key}ros_params_writable", False)
    clock = getattr(args, f"{key}ros_clock", None)
    off = getattr(args, f"{key}no_ros", False)
    if off or not ros.enabled_by_environment():
        if domain is not None or node is not None or writable or clock is not None:
            raise ros.RosError(f"--{prefix}ros-domain, --{prefix}ros-node, --{prefix}ros-params-writable and "
                               f"--{prefix}ros-clock need ROS 2, which "
                               + (f"--{prefix}no-ros" if off else f"{ros.ENABLE_ENV}=0") + " turns off")
        return None
    if clock is not None:
        from .topics import check_name
        check_name(clock.strip("/"))
    if node is not None:
        namespace, name = ros.split_node(node) if node.startswith("/") else ("/", ros.check_node_name(node))
        return RosOptions(ros.domain_from(domain), name, bool(writable), namespace, False, clock)
    program = prog or getattr(args, f"{key}ros_program", None) or program_name()
    if camera is None and isinstance(getattr(args, "camera", None), str):
        camera = args.camera
    default = default_ros_options(program, camera)
    return dataclasses.replace(default, domain=ros.domain_from(domain), params_writable=bool(writable), clock=clock)


def open_from_args(args, prefix: str = "", prog: str | None = None, camera: str | None = None) -> Session:
    """The session of a script's options (``add_arguments``), declaring the program ``prog`` (default:
    ``program_name()``, a node's console script) on the network (``open_session``), in ROS 2 as well unless
    ``--no-ros`` or ``XBLICK_ROS=0`` (``ros_options_from_args``; ``camera``: the camera the program serves, for the
    node's namespace, by default ``args.camera``). A peer that connects only to other PCs and listens on loopback
    only (``across_pcs``) is said on stderr first, as ``<prog>: ...``. Only a notice: the session opens as given. A
    session that Zenoh cannot open (OpenError) ends the program with one line, ``<prog>: ...``, and exit code 1."""
    key = prefix.replace("-", "_")
    mode = getattr(args, f"{key}zenoh_mode")
    connect, listen, config_file = getattr(args, f"{key}connect"), getattr(args, f"{key}listen"), getattr(
        args, f"{key}config")
    if config_file and mode != "peer":
        raise ValueError("give the mode in the Zenoh config file, not --zenoh-mode")
    given = prog
    prog = prog or program_name()
    try:
        ros_options = ros_options_from_args(args, prefix, given, camera)
    except ValueError as error:
        raise SystemExit(f"{prog}: {error}") from None
    say_across_pcs(connect, listen, mode, config_file, prog, prefix)
    try:
        session = open_session(connect=connect, listen=listen, mode=mode, config_file=config_file, program=prog,
                               ros_options=ros_options or False)
    except OpenError as error:
        raise SystemExit(f"{prog}: {error}") from None
    if session.ros is not None:
        session.ros.prog = prog
        clock = f"; {session.ros.clock} on /clock" if session.ros.clock else ""
        print(f"{prog}: in ROS 2 as node {session.ros.node.fully_qualified} on domain {session.ros.domain} "
              f"(rmw_zenoh keys besides our own{clock}; --no-ros: none)", file=sys.stderr, flush=True)
    return session


def say_across_pcs(connect: Sequence[str], listen: Sequence[str], mode: str, config_file: str | None, prog: str,
                   prefix: str = "") -> bool:
    """The notice of ``across_pcs`` on stderr, one line, when it applies. True when said."""
    endpoints = across_pcs(connect, listen, mode, config_file)
    if endpoints:
        print(f"{prog}: {across_pcs_text(endpoints, prefix)}", file=sys.stderr, flush=True)
    return bool(endpoints)


def why_nothing(session) -> str:
    """Why a session that found nothing may have found nothing: it is a peer that connects only to other PCs
    (``across_pcs``, kept by ``open_session``), or it reached no router and no other session (``alone``). Empty when
    neither holds."""
    endpoints = getattr(session, "across_pcs", ())
    return f"this session is {across_pcs_text(endpoints)}" if endpoints else alone(session)


def alone(session) -> str:
    """Why a session that found nothing may have found nothing: it reached no Zenoh router and no other session.
    Empty when it reached one."""
    if list(session.info.routers_zid()) or list(session.info.peers_zid()):
        return ""
    return ("this session reached no Zenoh router and no other session: start a router on this PC with xblick router "
            f"(or zenohd; a session without --connect looks for it at {ROUTER}), or name the router or session with "
            "--connect")


# ---------------------------------------------------------------------------------------------- queries

log = logging.getLogger("xblick")
not_finalized: dict[str, int] = {}  # selector -> queries Zenoh did not finalize by their deadline (logged once each)
_not_finalized_lock = threading.Lock()


def deadline_of(timeout: float) -> float:
    """How long a query of ``timeout`` s is waited for at most: its timeout and ``max(0.5, timeout)`` more."""
    return timeout + max(0.5, timeout)


def collect(handler, what: str, timeout: float, cancel=None) -> list:
    """The replies of a query's handler (a get's FIFO channel), read to its end by a short daemon thread: Zenoh's
    channel ends once it finalized the query. Waited for at most ``deadline_of(timeout)``: Zenoh does not always
    finalize a query by its timeout (seen once in CI: a get with timeout=1.0 that iterated for 5 min), and iterating
    its replies would then block for good. At the deadline it cancels the query (``cancel``, the get's
    CancellationToken: Session.close() waits for good for a query Zenoh did not finalize, seen in the tests), goes on
    with the replies that came, counts the query in ``not_finalized`` and logs it once per selector. The thread reads in
    Rust without the GIL, and no Python callback runs per reply: a query under GIL load costs what iterating the get
    costs. Error replies (``r.ok`` None) are kept as they came."""
    got: list = []

    def drain():
        for reply in handler:
            got.append(reply)

    reader = threading.Thread(target=drain, name=f"replies {what}", daemon=True)
    reader.start()
    reader.join(deadline_of(timeout))
    if reader.is_alive() and cancel is not None:
        cancel.cancel()  # the query ended by us: a query Zenoh did not finalize makes Session.close() wait for good
        reader.join(1.0)
    if reader.is_alive() or (cancel is not None and cancel.is_cancelled):
        with _not_finalized_lock:
            not_finalized[what] = not_finalized.get(what, 0) + 1
            first = not_finalized[what] == 1
        if first:
            log.warning(f"query {what} not finalized by Zenoh after {deadline_of(timeout):.1f} s (its timeout "
                        f"{timeout:g} s): going on with the {len(got)} replies that came")
        return list(got)
    return got


def replies(session: zenoh.Session, selector: str, timeout: float, liveliness: bool = False, **get_options) -> list:
    """The replies to a query (``session.get``, or ``session.liveliness().get`` with ``liveliness``; its other
    options in ``get_options``: payload, ...), within a deadline (``collect``). Use it instead of iterating a get's
    replies (CLAUDE.md, "Traps")."""
    getter = session.liveliness().get if liveliness else session.get
    if not liveliness:
        # every reply, not one per key (Zenoh's default consolidation): several publishers answer a stream's info key
        # (a source and its relays), and a caller that expects one node per service key counts the answers
        get_options.setdefault("consolidation", zenoh.ConsolidationMode.NONE)
    cancel = zenoh.CancellationToken()
    return collect(getter(selector, timeout=timeout, cancellation_token=cancel, **get_options), selector, timeout,
                   cancel)


ASK_PERIOD = 0.25  # s: how often ask() sends a read again while nothing answered


def ask(session: zenoh.Session, selector: str, timeout: float, **get_options) -> list:
    """The replies to a query that only reads (an info record, a status, a node's parameter values, rays), asked
    again every ASK_PERIOD seconds until a first reply comes or ``timeout`` passed.

    A session just opened knows no queryable yet: through a router their declarations arrive some 0.1-0.3 s later,
    and a query sent before ends at once without a reply. Asking again is right only for reads: an empty result
    does not prove that no queryable got the query (a queryable that drops it without a reply also gives an empty
    result at once), so a command would run twice. Commands go through ``ask_once``."""
    end = time.monotonic() + timeout
    while True:
        answers = replies(session, selector, max(end - time.monotonic(), 0.01), **get_options)
        if answers or time.monotonic() >= end:
            return answers
        time.sleep(min(ASK_PERIOD, max(0.0, end - time.monotonic())))


def timed_out(reply) -> bool:
    """The error reply "Timeout" (eclipse-zenoh 1.10; tests/test_queries.py pins the text). A queryable ends its part
    of a query when it drops the query object. At the query's timeout, the querying session ends a query that a
    queryable has not finished yet and adds this reply itself: nothing comes over the network for it. So a queryable
    received the query and had not dropped it in time. It may still be working on it."""
    return reply.err is not None and reply.err.payload.to_string() == "Timeout"


class NotReached(TimeoutError):
    """A command was not sent: no queryable for its key became known to the session within the timeout. Safe to try
    again (a TimeoutError, as before)."""


class NoAnswer(RuntimeError):
    """A command was sent once and no answer came within the timeout: it may have been executed, and it was not sent
    again. A receiver that does not check the command's deadline (any receiver of a command sent without a target,
    see ``ask_once``) may still execute it later, when it gets it (busy, briefly unreachable). Deliberately no
    TimeoutError, so that ``except TimeoutError: try again`` does not send it a second time; a RuntimeError, as a
    refusal, which callers do not repeat."""


class PartlyAnswered(NoAnswer):
    """A command was answered by some node(s), and the query was still open at its timeout (the "Timeout" reply,
    ``timed_out``): another node may have got it and may still execute it, or the answering node held the query after
    its reply. Not sent again. A NoAnswer, so no TimeoutError."""


class SeveralExecuted(NoAnswer):
    """A command was sent and answered by more than one node (two nodes on one key, which the check before the
    command did not see): each that answered with success executed it (one of them may have refused it as expired,
    said in the text). Not sent again. A NoAnswer, so no TimeoutError."""


class Expired(RuntimeError):
    """A command was refused by the node it was meant for, a receiver that checks the deadline (``CommandQuery``): its
    deadline, the time the sender stops waiting for the answer, had passed when the node came to execute it. That
    node did not execute it. Not sent again (a RuntimeError, as a refusal)."""


HOLDING = "a second node, or the answering node still holding the query after its reply"  # who keeps a query open


# ---------------------------------------------------------------------------------------------- command checks
# A command carries the node it is meant for and a deadline in its query's attachment, when the read before it was
# answered by a receiver that takes part. Such a receiver checks the node on receipt and the deadline just before the
# command takes effect (docs/reference/python-api.md, "Node and deadline"). xblick's Parameters and record and play
# node take part; another receiver only if its own code uses CommandQuery. An addition older programs ignore: an
# older receiver never looks at a query's attachment, an older sender never at a reply's. protocol.hpp has the format,
# command.hpp the rules of reading it, which _parse_attachment follows (C++ parse_command_attachment; one table of
# inputs runs through both in tests/test_queries.py).

COMMAND_FIELD = "xblick_command"  # the key that marks the attachment as a command's (protocol.hpp kCommandField)
COMMAND_VERSION = 1                # its value (kCommandVersion)
EXPIRED = "xblick-expired"         # the error reply's encoding schema: refused, past the deadline (kExpired)
NOT_TARGET = "xblick-not-target"   # refused, meant for another node (kNotTarget)
INVALID_COMMAND = "xblick-invalid-command"  # refused, an attachment it cannot read (kInvalidCommand)
CHECKS = ("node", "deadline")      # what a receiver confirms it checked (a reply's "checked")
MAX_DEPTH = 16                     # nesting of a command's attachment, at most (kCommandMaxDepth)
MAX_BYTES = 65536                  # its length, at most (kCommandMaxBytes)
MAX_AHEAD_NS = 86400 * 10**9       # a deadline at most 24 h ahead (kCommandMaxAheadNs)
_INT64 = (-2**63, 2**63 - 1)
_CLOCK_BOOTTIME = 7  # Linux's CLOCK_BOOTTIME (time.CLOCK_BOOTTIME is missing from some Python builds; Linux only)


def command_clock_ns() -> int:
    """The clock of a command's deadline: CLOCK_BOOTTIME, ns since the PC started, also counting a suspend, never set,
    only slewed by NTP (C++ ``xblick::command_clock_ns``). Each PC has its own: a sender converts its deadline into
    the receiver's clock (``CommandTarget``), so that the clocks of two PCs need not agree."""
    return time.clock_gettime_ns(_CLOCK_BOOTTIME)


def node_id(session: zenoh.Session) -> str:
    """The session's Zenoh id as text: the name a command gives the node it is meant for. Random, unless a Zenoh
    configuration sets ``id``: two programs given the same id are one node to the check."""
    return str(session.info.zid())


class _Other:
    """A JSON value of a kind a command's attachment does not use, or a number that is no int64: only skipped."""


_OTHER = _Other()


def _int_token(text: str):
    # as C++: more than 19 digits is no int64 (and Python's int() would refuse very long ones by itself)
    return int(text) if len(text.lstrip("-")) <= 19 else _OTHER


def _no_constant(name: str):
    raise ValueError(f"{name} is no JSON")


def _depth(value) -> int:
    """How deeply ``value`` nests (an object or a list: 1 more than its deepest member), without recursion."""
    deepest, stack = 0, [(value, 1)]
    while stack:
        v, d = stack.pop()
        if isinstance(v, dict | list):
            deepest = max(deepest, d)
            stack.extend((m, d + 1) for m in (v.values() if isinstance(v, dict) else v))
    return deepest


def _field(record: dict, key: str, kind: type):
    v = record.get(key)
    if kind is int:
        return v if type(v) is int and _INT64[0] <= v <= _INT64[1] else None
    if kind is list:
        return v if type(v) is list and all(type(s) is str for s in v) else None
    return v if type(v) is kind else None


def _parse_attachment(attachment) -> tuple[str, str, dict]:
    """``(kind, problem, fields)`` of a query's or reply's attachment by the rules of command.hpp, never raising:
    kind ``none`` (no attachment, or another program's: no bytes ``"xblick_command"``), ``invalid`` (problem says
    why) or ``ours``, with the known fields of their types (``version``, ``node``, ``deadline_ns``, ``zid``,
    ``clock_ns``, ``checked``; a field of another type is None). Unknown fields are ignored."""
    try:
        data = attachment.to_bytes() if attachment is not None else b""
    except Exception as error:  # noqa: BLE001 - no input may end a receiver
        return "invalid", f"its attachment cannot be read: {error}", {}
    if f'"{COMMAND_FIELD}"'.encode() not in data:
        return "none", "", {}
    what = f"its {COMMAND_FIELD} attachment "
    if len(data) > MAX_BYTES:
        return "invalid", what + f"is longer than {MAX_BYTES} bytes", {}
    try:
        text = data.decode("utf-8")
    except UnicodeDecodeError:
        return "invalid", what + "is no UTF-8", {}
    if text.lstrip(" \t\n\r")[:1] != "{":
        return "invalid", what + "is no JSON object", {}
    try:
        record = json.loads(text, strict=False, parse_int=_int_token, parse_float=lambda s: _OTHER,
                            parse_constant=_no_constant)
    except Exception:  # noqa: BLE001 - a malformed input, deep nesting (RecursionError) and anything else: refused
        return "invalid", what + "is no valid JSON", {}
    if _depth(record) > MAX_DEPTH:
        return "invalid", what + f"is nested deeper than {MAX_DEPTH}", {}
    if COMMAND_FIELD not in record:
        return "invalid", what + f"names {COMMAND_FIELD} below its top level", {}
    fields = {"version": _field(record, COMMAND_FIELD, int), "node": _field(record, "node", str),
              "deadline_ns": _field(record, "deadline_ns", int), "zid": _field(record, "zid", str),
              "clock_ns": _field(record, "clock_ns", int), "checked": _field(record, "checked", list)}
    return "ours", "", fields


def refuse_on_receipt(kind: str, problem: str, fields: dict, zid: str) -> tuple[str, str] | None:
    """The first check of a command (C++ ``refuse_on_receipt``): ``(schema, reason)`` for an attachment it cannot
    read (invalid, a version other than 1, no node or deadline_ns, a negative deadline) or one meant for another node
    than ``zid``; None to go on (also for an attachment of kind none: nothing to check)."""
    what = f"its {COMMAND_FIELD} attachment "
    if kind == "none":
        return None
    if kind == "invalid":
        return INVALID_COMMAND, problem
    if fields["version"] != COMMAND_VERSION:
        return INVALID_COMMAND, what + f"is not of version {COMMAND_VERSION}, the version this node reads"
    if not fields["node"] or fields["deadline_ns"] is None or fields["deadline_ns"] < 0:
        return INVALID_COMMAND, what + "has no node and deadline_ns"
    if fields["node"] != zid:
        return NOT_TARGET, f"it is meant for the node of session {fields['node']}, this is {zid}"
    return None


def refuse_at_effect(kind: str, fields: dict, now_ns: int) -> tuple[str, str] | None:
    """The second check, just before the effect, on this PC's command clock ``now_ns`` (C++ ``refuse_at_effect``):
    the deadline reached, or one more than 24 h ahead (no sender's). None to go on. Only after ``refuse_on_receipt``
    gave none."""
    if kind != "ours" or fields["deadline_ns"] is None:
        return None
    deadline = fields["deadline_ns"]
    if now_ns >= deadline:
        return EXPIRED, f"its deadline had passed {(now_ns - deadline) / 1e9:.3f} s before this node came to execute it"
    if deadline - now_ns > MAX_AHEAD_NS:
        return INVALID_COMMAND, f"its deadline lies more than {MAX_AHEAD_NS // 10**9} s ahead"
    return None


def answer_attachment(zid: str, checked=()) -> str:
    """The attachment of every reply of a receiver that takes part, to a command and to the read before it: its
    session's Zenoh id ``zid`` (``node_id``), its clock now (``command_clock_ns``) and what it checked before executing
    the command (``CHECKS``; none for a read or a command without a target). Older senders do not look at it."""
    return json.dumps({COMMAND_FIELD: COMMAND_VERSION, "zid": zid, "clock_ns": command_clock_ns(),
                       "checked": list(checked)}, separators=(",", ":"))


class CommandRefused(Exception):
    """A receiver's refusal of a command before its effect (``CommandQuery``): ``schema`` names the case (EXPIRED,
    NOT_TARGET, INVALID_COMMAND), the text is the error reply's. Deliberately no RuntimeError or ValueError, which
    receivers catch as the command's own failure."""

    def __init__(self, schema: str, text: str):
        super().__init__(text)
        self.schema = schema


class CommandQuery:
    """A command as its receiver got it (``query``, on the key ``what``; ``zid``: the receiver's session, ``node_id``):
    the node it is meant for and its deadline, as the sender attached them. A query without them (an older sender, a
    command sent without a target) has nothing to check. It never raises on any input; build it inside the code that
    answers the query with an error reply on any exception. Call ``check_node()`` on receipt, before anything else,
    and ``check_deadline()`` as late as possible before the command takes effect; reply with
    ``answer_attachment(zid, checked)``; a refusal goes back with ``refuse``. ``checked``: what was checked
    (``CHECKS``), empty before ``check_deadline`` or without a target."""

    def __init__(self, query: zenoh.Query, zid: str, what: str):
        self.what, self.zid = what, zid
        self.checked: list[str] = []
        try:
            attachment = query.attachment
        except Exception as error:  # noqa: BLE001 - no input may end a receiver
            self.kind, self.problem, self.fields = "invalid", f"its attachment cannot be read: {error}", {}
            return
        self.kind, self.problem, self.fields = _parse_attachment(attachment)

    @property
    def node(self) -> str | None:
        return self.fields.get("node") if self.kind == "ours" else None

    @property
    def deadline_ns(self) -> int | None:
        return self.fields.get("deadline_ns") if self.kind == "ours" else None

    def check_node(self):
        """CommandRefused (INVALID_COMMAND, NOT_TARGET) when this node must not execute it."""
        refusal = refuse_on_receipt(self.kind, self.problem, self.fields, self.zid)
        if refusal is not None:
            schema, reason = refusal
            raise CommandRefused(schema, f"{self.what}: not executed{' here' if schema == NOT_TARGET else ''}: "
                                         f"{reason}")

    def check_deadline(self):
        """CommandRefused (EXPIRED; INVALID_COMMAND for a deadline more than 24 h ahead) when the deadline has passed
        on this PC's ``command_clock_ns``; runs ``check_node`` first. Afterwards ``checked`` names both checks."""
        self.check_node()
        refusal = refuse_at_effect(self.kind, self.fields, command_clock_ns())
        if refusal is not None:
            raise CommandRefused(refusal[0], f"{self.what}: not executed: {refusal[1]}")
        if self.kind == "ours":
            self.checked = list(CHECKS)


def refuse(query: zenoh.Query, refused: CommandRefused):
    """The error reply of a refused command: its text, its case in the encoding's schema (``text/plain;<schema>``),
    which a sender of 0.5.13 tells apart and an older one shows as the refusal's text."""
    query.reply_err(str(refused), encoding=zenoh.Encoding(f"text/plain;{refused.schema}"))


@dataclasses.dataclass(frozen=True)
class CommandTarget:
    """The node a command is meant for (``target_of``): ``node`` its session's Zenoh id, ``offset_ns`` the least its
    clock (``command_clock_ns``) can be ahead of this program's."""

    node: str
    offset_ns: int

    def deadline_ns(self, timeout: float) -> int:
        """``timeout`` s from now on the node's clock, at the earliest that can be: late only by the clocks' rate
        difference (at most 0.05 % with NTP) over the time from the read to the node's check. ValueError for a
        timeout above 24 h (a receiver refuses a deadline further ahead); RuntimeError ("not sent") for a deadline
        out of range."""
        budget = round(timeout * 1e9)
        if not 0 <= budget <= MAX_AHEAD_NS:
            raise ValueError(f"a command's timeout is between 0 and 24 h, not {timeout:g} s (a receiver refuses a "
                             "deadline further ahead)")
        deadline = command_clock_ns() + budget + self.offset_ns
        if not 0 <= deadline <= _INT64[1]:
            raise RuntimeError(f"not sent: the deadline on the node of session {self.node} is out of range (its clock "
                               "reading cannot be right)")
        return deadline


def target_of(sample, answered_ns: int) -> CommandTarget | None:
    """The target of a command from the one answer of the read before it (``sample``, the reply's ok) and the time
    the read ended (``answered_ns``, ``command_clock_ns()`` once it returned). None when the answer carries no
    ``answer_attachment``: its receiver does not take part (whatever its version), and nothing new is sent. The node
    read its clock before the read ended, so ``clock_ns - answered_ns`` is at most its clock's true offset: the
    deadline on its clock comes out early by at most the read's round trip, late only by the clocks' rate difference
    (``CommandTarget.deadline_ns``). RuntimeError ("not sent") for an attachment it cannot read, of a version other
    than 1 among them: a receiver keeps answering version 1."""
    kind, _, fields = _parse_attachment(sample.attachment)
    if kind == "none":
        return None
    if (kind != "ours" or fields["version"] != COMMAND_VERSION or not fields["zid"] or fields["clock_ns"] is None
            or fields["clock_ns"] < 0):
        raise RuntimeError(f"not sent: the answer of {sample.key_expr} has a {COMMAND_FIELD} attachment this program "
                           "cannot read")
    return CommandTarget(fields["zid"], fields["clock_ns"] - answered_ns)


def one_node(session: zenoh.Session, selector: str, timeout: float, **get_options) -> CommandTarget | None:
    """The read before a command (``ask``, then ``count``: best effort, exactly one node must answer it in time) and
    the command's target from its answer (``target_of``; None when that node does not take part). Raises as
    ``count``."""
    answers = ask(session, selector, timeout, **get_options)
    answered = command_clock_ns()
    (sample,) = count(answers, selector)
    return target_of(sample, answered)


def checked(sample, to: CommandTarget | None) -> bool:
    """True when the answer ``sample`` to a command sent to ``to`` says that the node checked the node and the
    deadline before executing it (``answer_attachment``). False otherwise; a sender cannot know it in advance, nor
    after a command without an answer."""
    kind, _, fields = _parse_attachment(sample.attachment)
    return (to is not None and kind == "ours" and fields["version"] == COMMAND_VERSION and fields["zid"] == to.node
            and set(CHECKS) <= set(fields["checked"] or ()))


def _schema(reply) -> str:
    """The schema of an error reply's encoding (``text/plain;<schema>``), empty for none."""
    return str(reply.err.encoding).partition(";")[2]


def ask_once(session: zenoh.Session, selector: str, timeout: float, to: CommandTarget | None = None,
             **get_options) -> list:
    """The replies to a command (a parameter set, a controller switch, a torque step, a recorder command): sent once
    to each queryable of ``selector``, after the session knows one. NotReached when none became known within
    ``timeout`` (nothing sent). The replies as they came, possibly none; ``command`` interprets them.

    ``to`` (``one_node``): the node it is meant for. The query's attachment then names it and the deadline, ``timeout``
    from the moment it is sent (the time Zenoh ends the query), converted into the node's clock (``CommandTarget``).
    A receiver that checks (``CommandQuery``) and is not that node does not execute it, nor does that node once the
    deadline has passed. A receiver that does not check, an older one or one whose code does not, ignores the
    attachment and executes it as before; this program cannot tell which it is before it has an answer
    (``checked``). Without ``to`` nothing is attached, as before 0.5.13.

    Once per queryable: two queryables on the key (two nodes, two bridges) both get it and both may execute it. This
    holds because Zenoh's default query target (BestMatching) reaches every matching queryable while none is declared
    complete, and no Durchblick queryable on our keys is (tests/test_queries.py pins both). The ROS 2 services of a
    session with ``--ros`` (ros.py) are complete, as rmw_zenoh's, and a ROS request goes with target ALL_COMPLETE; their
    keys start with the ROS domain id and never match one of ours. The
    callers check first, as far as the protocol lets them, that one node serves the key (``count``): a best-effort
    check that catches two nodes that both answer a read, not a guarantee.

    Why the command is never sent again: Zenoh ends a query without any reply in cases that the sender cannot tell
    apart. The session knew no queryable, so nothing went out (the case of a session just opened). Or a queryable
    got the query and dropped it without a reply, which also ends it at once. Or a connection on the way was lost.
    A command sent again in the later cases would run twice. Instead, ask_once waits until a querier on the key
    reports a matching queryable (Zenoh's own routing knowledge, also through a router) and then sends the command
    once. A queryable that has not dropped the query by its timeout gives the error reply "Timeout" (``timed_out``),
    which the session adds itself.

    Time: up to ``timeout`` for the queryable, then the query's own timeout ``timeout``; when Zenoh does not end the
    query by then, ``collect`` waits up to ``deadline_of(timeout)`` (``timeout + max(0.5, timeout)``) and 1 s more
    for the cancellation. At most about 3 x timeout + 1 s in all (measured in tests/test_queries.py)."""
    if to is not None and "attachment" in get_options:
        raise ValueError("a command to a target carries its own attachment (its node and deadline)")
    end = time.monotonic() + timeout
    querier = session.declare_querier(selector)
    try:
        while not querier.matching_status.matching:
            if time.monotonic() >= end:
                raise NotReached(f"not sent: no node serves {selector} (none became known within {timeout:g} s)")
            time.sleep(0.01)
    finally:
        querier.undeclare()
    if to is not None:  # taken last: the deadline is never later than the time Zenoh ends the query
        get_options["attachment"] = json.dumps({COMMAND_FIELD: COMMAND_VERSION, "node": to.node,
                                                "deadline_ns": to.deadline_ns(timeout)}, separators=(",", ":"))
    return replies(session, selector, timeout, **get_options)


def no_answer_text(selector: str, timeout: float, to: CommandTarget | None) -> str:
    """What a NoAnswer says: without a target as before 0.5.13; with one, that it may still be executed unless its
    receiver checks the deadline (which has passed when the sender gives up), and that this program cannot tell
    whether it does: a receiver checks only if its own code does (``CommandQuery``)."""
    if to is None:
        return f"{selector}: sent once, no answer within {timeout:g} s: it may still be executed; not sent again"
    return (f"{selector}: sent once with a deadline, no answer within {timeout:g} s: it may have been executed, and "
            "it may still be executed unless its receiver checks the deadline, which this program cannot tell; not "
            "sent again")


def command(session: zenoh.Session, selector: str, timeout: float, to: CommandTarget | None = None,
            **get_options) -> list:
    """A command sent with ``ask_once`` (``to``: the node it is meant for) and its answers checked: the successful
    replies (one or more), else NotReached (not sent), RuntimeError with the first error reply's text (an error reply
    other than Zenoh's own timeout and the refusals below), Expired (refused by its node: its deadline had passed, not
    executed there), NoAnswer (sent once, no answer within ``timeout``: it may have been executed; not sent again),
    PartlyAnswered (answered, and the query was still open at its timeout: another node may still execute it), or
    SeveralExecuted (answered by several nodes: each that answered with success executed it). A refusal by a node that
    the command was not meant for (NOT_TARGET) means that node did not execute it: beside one successful answer it is
    a warning in the log (a second node on the key, which the read before missed), alone it is a NoAnswer (the target
    gave no answer). ``checked`` tells whether a successful answer's node checked node and deadline.

    A receiver replies and drops the query at once: one that keeps the query after its reply keeps it open until the
    timeout, and the "Timeout" reply that the session then adds looks like a second node."""
    answers = ask_once(session, selector, timeout, to, **get_options)
    errors = [r for r in answers if r.err is not None and not timed_out(r)]
    plain = [r.err.payload.to_string() for r in errors if _schema(r) not in (EXPIRED, NOT_TARGET)]
    if plain:
        raise RuntimeError(plain[0])
    expired = [r.err.payload.to_string() for r in errors if _schema(r) == EXPIRED]
    others = len([r for r in errors if _schema(r) == NOT_TARGET])
    still_open = any(timed_out(r) for r in answers)
    ok = [r.ok for r in answers if r.ok is not None]
    refused_by_others = f"; {others} other node(s) refused it as not meant for them" if others else ""
    if not ok:
        if expired and not still_open:
            raise Expired(f"{selector}: refused as expired: {expired[0]}{refused_by_others}; not sent again")
        if expired:
            raise PartlyAnswered(f"{selector}: refused as expired by its node ({expired[0]}), and the query was still "
                                 f"open after {timeout:g} s ({HOLDING}): a second node may still execute it; not sent "
                                 "again")
        raise NoAnswer(no_answer_text(selector, timeout, to) + refused_by_others)
    if still_open:
        raise PartlyAnswered(f"{selector}: answered by {len(ok)} node(s), and the query was still open after "
                             f"{timeout:g} s ({HOLDING}): a second node may still execute it; not sent again")
    if len(ok) > 1 or expired:
        refused = f" and {len(expired)} refused it as expired" if expired else ""
        raise SeveralExecuted(f"{selector}: sent, and {len(ok) + len(expired)} nodes answered: {len(ok)} executed it"
                              f"{refused}{refused_by_others}; not sent again (one key, one node)"
                              if expired else
                              f"{selector}: sent, and {len(ok)} nodes answered: each of them executed it"
                              f"{refused_by_others}; not sent again (one key, one node)")
    if others:
        log.warning(f"{selector}: executed by its node; {others} other node(s) on the key refused it as not meant for "
                    "them (one key, one node: the read before the command did not see them)")
    return ok


def count(answers: list, selector: str) -> list:
    """The successful replies of a read that counts the nodes of a command's key before the command (best effort):
    RuntimeError, "not sent", unless exactly one node answered and the query ended before its timeout (no "Timeout"
    reply, ``timed_out``). A query still open at its timeout means a queryable got it and had not finished: a node that
    exists but did not answer in time is a second node, not none, or a single node keeps the query after its reply.
    TimeoutError when none answered, in time or at all. A session just opened links to the serving sessions one after
    the other: for a short time the read can miss a second node (the command then reaches it too, if known by then;
    one that checks refuses a command meant for another node, ``one_node``) or reach only a busy one
    (TimeoutError, although another node serves the key)."""
    ok = [r.ok for r in answers if r.ok is not None]
    still_open = any(timed_out(r) for r in answers)
    if not ok:
        raise TimeoutError(f"not sent: no node answered {selector}" + (" in time" if still_open else ""))
    if still_open:
        raise RuntimeError(f"not sent: {len(ok)} node(s) answered {selector}, and the query was still open at its "
                           f"timeout ({HOLDING})")
    if len(ok) > 1:
        raise RuntimeError(f"not sent: {len(ok)} nodes answer {selector}: one key, one node")
    return ok


CLOSE_TIMEOUT_S = 5.0  # how long a close waits for Zenoh (Session, close)


def close(session, timeout: float = CLOSE_TIMEOUT_S) -> bool:
    """Closes ``session`` (a zenoh.Session or a Session), waiting at most ``timeout`` s: Zenoh's close blocked for
    good after a query it did not finalize (seen in the tests, also with that query cancelled), and a program must
    still end. True when closed; False: it is left closing in a daemon thread, logged."""
    raw = session._session if isinstance(session, Session) else session
    declared = taking_part = None
    if isinstance(session, Session):
        declared, taking_part = session.declaration, session.ros
        object.__setattr__(session, "declaration", None)
        object.__setattr__(session, "ros", None)
        if session._finalizer is not None:
            session._finalizer.detach()
    return _close_parts(raw, declared, taking_part, timeout)


def _close_parts(raw: zenoh.Session, declared, taking_part, timeout: float) -> bool:
    """``close`` of a Zenoh session with its declaration and its part in ROS 2 (either may be None)."""
    def end():
        try:
            if taking_part is not None:  # its tokens gone first, so that ROS's graph drops the node at once
                taking_part.close()
            if declared is not None:  # gone with the session too; undeclared first, while the session still sends
                declared.close()
        except zenoh.ZError:  # the session was closed already (by zenoh directly): nothing left to undeclare
            pass
        raw.close()

    closer = threading.Thread(target=end, name="zenoh close", daemon=True)
    closer.start()
    closer.join(timeout)
    if closer.is_alive():
        log.warning(f"the Zenoh session {raw.info.zid()} did not close within {timeout:g} s: going on without "
                    "waiting for it")
        return False
    return True


class Session:
    """A zenoh.Session (``open_session``, ``open_from_args``) whose close, also at the end of a ``with`` block, waits
    at most CLOSE_TIMEOUT_S (``close``). Everything else is the session's own: attributes go to it, and
    ``isinstance(s, zenoh.Session)`` holds (zenoh.Session cannot be subclassed). ``across_pcs``: the endpoints of
    ``across_pcs`` for its options, or none; ``declaration``: its program's (declaration.py), None when it does not
    declare; ``ros``: its part in ROS 2 (ros.Ros, ``--ros``), None when it takes none. A Session in ROS 2 that its
    program drops without closing it is closed when it is collected, as Zenoh closes a dropped session without ROS
    (``_finalizer``; not at the program's exit)."""

    __slots__ = ("_session", "across_pcs", "declaration", "ros", "_finalizer", "__weakref__")

    def __init__(self, session: zenoh.Session, across_pcs: Sequence[str] = (), declaration=None, ros=None):
        object.__setattr__(self, "_session", session)
        object.__setattr__(self, "across_pcs", list(across_pcs))
        object.__setattr__(self, "declaration", declaration)
        object.__setattr__(self, "ros", ros)
        object.__setattr__(self, "_finalizer", None)

    def __getattr__(self, name):
        return getattr(self._session, name)

    @property
    def __class__(self):  # isinstance(s, zenoh.Session); type(s) is still Session
        return zenoh.Session

    def close(self, timeout: float = CLOSE_TIMEOUT_S) -> bool:
        return close(self, timeout)

    def __enter__(self) -> Session:
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    def __repr__(self) -> str:
        return f"xblick Session({self._session!r})"
