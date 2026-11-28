"""A node's parameters, as ROS 2's: declared with a ParameterDescriptor each (rcl_interfaces), read and changed
over Zenoh through ROS 2's parameter services, and set at the start from ``--param name:=value`` and ROS params
files. docs/reference/parameters.md has the whole of it; C++ ``xblick::Parameters`` (parameters.hpp) is the same.

    params = node.Parameters(session, "boxer", "test/camera")      # the node <namespace>/<name>
    params.configure(args)                                     # --param, --params-file, --dump-params, ...
    params.declare("threshold", 0.3, description="a box's score at least", range=(0.0, 1.0, 0.05),
                   write=lambda value: None)                   # None: taken; a string: refused, the reason
    params.declare("model", "base", choices=["base", "large"], read_only=True)   # applies at the next start
    params.declare_options(parser, args, live={"max_boxes": {"range": (1, 100)}})   # every other option, read-only
    params.check_overrides()                                   # all declared: unknown names given at the start refused
    params["threshold"]                                        # the value now (changed over Zenoh meanwhile)

Keys (protocol.hpp), the node's key ``<namespace>/<name>`` (its fully qualified name ``/<namespace>/<name>``):

* ``<node>/list_parameters``, ``describe_parameters``, ``get_parameters``, ``get_parameter_types``,
  ``set_parameters``, ``set_parameters_atomically``: queryables, the query's payload the service's request in CDR
  (``rcl_interfaces/srv/<Service>_Request``), the reply its response;
* ``parameter_events``: every node's ``rcl_interfaces/msg/ParameterEvent`` (one key, as ROS's ``/parameter_events``;
  ``node`` the node's fully qualified name), after each change and at each declaration;
* the liveliness token ``<node>/parameters`` while it offers them (``find_nodes``).

With a session that takes part in ROS 2 (the default, ros.py) the program's ROS node serves them as well: the same
six services on rmw_zenoh's keys (``<node>/get_parameters``, ...) and its events on ROS's ``/parameter_events``, so
that ``ros2 param`` works on it. That node is ``/<namespace>/<name>`` (each part as ros.node_of writes it) unless the
program was given a node name (``--ros-node``); a program's further Parameters are ROS nodes of their own
(ros.Ros.parameters_node). By default its ROS services answer reads only and refuse
sets. With ``--ros-params-writable`` a ROS client may set them: such a set is unchecked and uncounted (no node check,
no deadline; two nodes of one name both execute it, and the ROS client shows one success; docs/integrations/ros2.md).
Our own callers keep using our keys with the check. ``ParameterClient(..., ros=True)`` reads and sets a ROS node's
parameters through ROS's keys.
"""

from __future__ import annotations

import argparse
import dataclasses
import json
import math
import re
import threading
import time
from collections.abc import Callable
from pathlib import Path

import zenoh

from . import _core, ros, topics
from .camera import cdr_encoding
from .msg import (DescribeParametersRequest, DescribeParametersResponse, FloatingPointRange, GetParametersRequest,
                  GetParametersResponse, GetParameterTypesRequest, GetParameterTypesResponse, IntegerRange,
                  ListParametersRequest, ListParametersResponse, Parameter, ParameterDescriptor, ParameterEvent,
                  ParameterType, ParameterValue, SetParametersAtomicallyRequest, SetParametersAtomicallyResponse,
                  SetParametersRequest, SetParametersResponse, SetParametersResult, Time)
from .paths import expand_path
from .ros import NEEDS_ROS, check_ros_name
from .session import (ASK_PERIOD, CommandQuery, CommandRefused, answer_attachment, ask, checked, close_at_exit, command,
                      count, forget_at_exit, log, node_id, one_node, refuse, replies, timed_out)

P = _core.protocol
SERVICES = {P.LIST_PARAMETERS: (ListParametersRequest, ListParametersResponse),
            P.DESCRIBE_PARAMETERS: (DescribeParametersRequest, DescribeParametersResponse),
            P.GET_PARAMETERS: (GetParametersRequest, GetParametersResponse),
            P.GET_PARAMETER_TYPES: (GetParameterTypesRequest, GetParameterTypesResponse),
            P.SET_PARAMETERS: (SetParametersRequest, SetParametersResponse),
            P.SET_PARAMETERS_ATOMICALLY: (SetParametersAtomicallyRequest, SetParametersAtomicallyResponse)}
# the ROS 2 service type of each (rcl_interfaces), for its rmw_zenoh key and token
ROS_SERVICES = {P.LIST_PARAMETERS: "rcl_interfaces/srv/ListParameters",
                P.DESCRIBE_PARAMETERS: "rcl_interfaces/srv/DescribeParameters",
                P.GET_PARAMETERS: "rcl_interfaces/srv/GetParameters",
                P.GET_PARAMETER_TYPES: "rcl_interfaces/srv/GetParameterTypes",
                P.SET_PARAMETERS: "rcl_interfaces/srv/SetParameters",
                P.SET_PARAMETERS_ATOMICALLY: "rcl_interfaces/srv/SetParametersAtomically"}
SETS = (P.SET_PARAMETERS, P.SET_PARAMETERS_ATOMICALLY)  # the services that change something: commands
READ_ONLY_ACCESS = "parameters are read only on this node (--params-read-only)"
RESTART = "read-only: applies at the next start"
SERVE_PERIOD_S = 0.01  # how often the services look for queries: a parameter call waits at most this much longer
NONE_SERVE_PERIOD_S = 0.05  # the same for a program node without parameters (none_on): every program has one
# ROS calls answered per ROS service and round (_serve): a ROS client that floods a node does not hold up our own
# (checked) calls, which are all answered every round
ROS_CALLS_PER_ROUND = 4
ROS_SETS_OFF = ("sets through ROS 2 are off on this node: its ROS parameter services answer reads only (start it with "
                "--ros-params-writable to allow them)")
_CLOSED = object()  # Parameters._next: a queryable's channel closed with its session
# options declare_options leaves out: argparse's, the parameters' own, the Zenoh connection's (session.add_arguments)
OWN_OPTIONS = {"help", "param", "params_file", "dump_params", "params_read_only", "connect", "listen", "zenoh_mode",
               "config"}
_NAME = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*(\.[A-Za-z_][A-Za-z0-9_]*)*$")


class ParameterError(ValueError):
    """A parameter, its declaration or a value at the start that does not fit."""


def node_key(namespace: str, name: str) -> str:
    """``<namespace>/<name>``: the key the node's services are below."""
    return topics.check_name(f"{namespace.strip('/')}/{name}" if namespace.strip("/") else name)


def fully_qualified(key: str) -> str:
    return "/" + key.strip("/")


def text(value) -> str:
    """A value as the constraints and the dumps write it (YAML's scalar)."""
    return _yaml_scalar(value)


def constraints_text(unit: str = "", choices=None) -> str:
    """``additional_constraints``: ``unit: us; choices: 5, 15, 30`` (either part may be left out)."""
    parts = ([f"unit: {unit}"] if unit else []) + ([f"choices: {', '.join(text(c) for c in choices)}"]
                                                     if choices else [])
    return "; ".join(parts)


def constraints(descriptor: ParameterDescriptor) -> dict:
    """``additional_constraints`` read back: ``unit`` (text) and ``choices`` (values of the parameter's type), those
    there are; text it does not know is left out."""
    out = {}
    for part in descriptor.additional_constraints.split("; "):
        key, sep, rest = part.partition(": ")
        if sep and key == "unit":
            out["unit"] = rest
        elif sep and key == "choices":
            try:
                out["choices"] = [from_text(descriptor, c, check_type=False).value()
                                  for c in _split_flow(rest)]
            except (TypeError, ValueError):
                pass
    return out


def check(descriptor: ParameterDescriptor, value: ParameterValue, choices=None) -> str | None:
    """Why ``value`` does not fit ``descriptor`` (type, range and step, ``choices``: by default those of its
    additional_constraints), or None."""
    T = ParameterType
    if value.type != descriptor.type and not descriptor.dynamic_typing:
        return (f"{T.NAMES[value.type] if 0 <= value.type < 10 else value.type} given, the parameter is "
                f"{T.NAMES[descriptor.type]}")
    v = value.value()
    if descriptor.integer_range and value.type == T.INTEGER:
        r = descriptor.integer_range[0]
        if not r.from_value <= v <= r.to_value:
            return f"{v} is outside {r.from_value} to {r.to_value}"
        if r.step and v != r.to_value and (v - r.from_value) % r.step:
            return f"{v} is not {r.from_value} plus a multiple of {r.step}"
    if descriptor.floating_point_range and value.type == T.DOUBLE:
        r = descriptor.floating_point_range[0]
        tolerance = 1e-9 * max(1.0, abs(r.from_value), abs(r.to_value))
        if not (r.from_value - tolerance <= v <= r.to_value + tolerance) or math.isnan(v):
            return f"{v:g} is outside {r.from_value:g} to {r.to_value:g}"
        if r.step and abs(v - r.to_value) > tolerance:
            k = round((v - r.from_value) / r.step)
            if abs(r.from_value + k * r.step - v) > tolerance:
                return f"{v:g} is not {r.from_value:g} plus a multiple of {r.step:g}"
    choices = constraints(descriptor).get("choices") if choices is None else choices
    if choices and v not in choices:
        return f"{text(v)} is none of {', '.join(text(c) for c in choices)}"
    return None


def from_text(descriptor: ParameterDescriptor, given, check_type: bool = True) -> ParameterValue:
    """A value given as text (``--param``) or parsed from YAML, for a parameter: an integer for a double parameter
    and anything for a string parameter (its text) are taken as such."""
    T = ParameterType
    value = parse_scalar(given) if isinstance(given, str) else given
    if descriptor.type == T.DOUBLE and isinstance(value, int) and not isinstance(value, bool):
        value = float(value)
    if descriptor.type == T.STRING and not isinstance(value, str):
        value = given.strip() if isinstance(given, str) else text(value)
    if descriptor.type == T.DOUBLE_ARRAY and isinstance(value, list):
        value = [float(v) if isinstance(v, int) and not isinstance(v, bool) else v for v in value]
    return ParameterValue.of(value)


# ------------------------------------------------------------------------------------------------ the node's side


@dataclasses.dataclass
class _Declared:
    descriptor: ParameterDescriptor
    value: ParameterValue  # the stored one
    choices: list
    read: Callable | None
    write: Callable | None


class Parameters:
    """The parameters of one node ``<namespace>/<name>`` (module docstring), answering ROS 2's parameter services.
    ``session`` None: no services (a node's own use, tests). A session in ROS 2: the services on ROS's keys too,
    on the program's ROS node (module docstring)."""

    def __init__(self, session: zenoh.Session | None, name: str, namespace: str = "", read_only_access: bool = False):
        self.key = node_key(namespace, name)
        self.name = fully_qualified(self.key)
        self.read_only_access = read_only_access
        self._declared: dict[str, _Declared] = {}
        self._overrides: dict[str, tuple[object, str]] = {}  # name -> (value as given, where from)
        self._dump: Path | None = None
        self._lock = threading.RLock()
        self._events = self._queryables = self._token = self._server = None
        self._ros_node = self._ros_events = None
        self._ros_own_node = False  # a ROS node of their own (not the program's: ros.Ros.parameters_node)
        self._ros_none = False      # none_on: a program node's services while it has no parameters
        self._ros_name = self.name  # the ROS node's fully qualified name
        self._ros_services: list[tuple[str, ros.RosService]] = []
        self._stop = threading.Event()
        self._zid = node_id(session) if session is not None else ""  # names this node in its replies (commands)
        if session is not None:
            self._events = session.declare_publisher(P.PARAMETER_EVENTS, encoding=cdr_encoding(ParameterEvent))
            # channels, answered by a daemon thread of the node's own: a Python callback would run in zenoh-python's
            # own thread, which is no daemon, and a node that ends with an exception before close() (an interrupt
            # while its model loads) would then wait for it for good
            self._queryables = [(service, session.declare_queryable(f"{self.key}/{service}")) for service in SERVICES]
            taking_part = getattr(session, "ros", None)
            if taking_part is not None:  # the program's ROS node (ros.Ros.parameters_node): the services, the events
                self._ros_node, self._ros_own_node = taking_part.parameters_node(self.name)
                try:
                    self._ros_name = self._ros_node.fully_qualified
                    for service in SERVICES:
                        self._ros_services.append(
                            (service, self._ros_node.service(f"{self._ros_name}/{service}", ROS_SERVICES[service])))
                    self._ros_events = self._ros_node.publisher("/parameter_events", ParameterEvent.TYPE,
                                                                ros.PARAMETERS_QOS)
                except BaseException:
                    self._undo_ros()
                    for _, q in self._queryables:
                        q.undeclare()
                    self._events.undeclare()
                    raise
            self._server = threading.Thread(target=self._serve, name=f"parameters {self.key}", daemon=True)
            self._server.start()
            self._token = session.liveliness().declare_token(f"{self.key}/{P.PARAMETERS_TOKEN}")
            close_at_exit(self, f"the parameters of {self.key}")  # their thread answers queries: stopped at the end

    # ---------------------------------------------------------------------------------------------- at the start

    @staticmethod
    def add_arguments(parser):
        """``--param NAME:=VALUE`` (repeat), ``--params-file FILE`` (repeat; the later win, ``--param`` over all),
        ``--dump-params FILE``, ``--params-read-only``."""
        parser.add_argument("--param", action="append", default=[], metavar="NAME:=VALUE",
                            help="a parameter's value at the start, as ROS's -p (repeat for more)")
        parser.add_argument("--params-file", action="append", default=[], metavar="FILE",
                            help="a ROS params file (<node>: ros__parameters: ..., /** for every node)")
        parser.add_argument("--dump-params", default=None, metavar="FILE",
                            help="write the parameters in effect at the start to this params file")
        parser.add_argument("--params-read-only", action="store_true",
                            help="parameters can be read over Zenoh, not changed")

    def configure(self, args):
        """The values given at the start (``add_arguments``' options in ``args``): params files, then ``--param``;
        they replace the defaults of ``declare``."""
        for path in getattr(args, "params_file", []) or []:
            for name, value in read_params_file(expand_path(path), self.name).items():
                self._overrides[name] = (value, str(path))
        for item in getattr(args, "param", []) or []:
            self.add_override(item)
        if getattr(args, "dump_params", None):
            self._dump = expand_path(args.dump_params)
        if getattr(args, "params_read_only", False):
            self.read_only_access = True

    def add_override(self, assignment: str, where: str = "--param"):
        """A value at the start, ``name:=value`` (a YAML scalar, as ROS's ``-p``)."""
        name, sep, given = assignment.partition(":=")
        if not sep or not _NAME.match(name):
            raise ParameterError(f"{where} {assignment!r}: NAME:=VALUE expected")
        self._overrides[name] = (given, where)

    def declare(self, name: str, default, *, description: str = "", unit: str = "", choices=None, range=None,
                read_only: bool = False, read: Callable | None = None, write: Callable | None = None):
        """A parameter, its type that of ``default`` (bool, int, float, str, bytes, a list of one of them), its value
        the one given at the start or ``default``. ``range``: (from, to) or (from, to, step) for a number; ``choices``:
        the values it takes; ``unit`` (into additional_constraints); ``read_only``: applies at the next start.
        ``write(value)``: applies a checked change, returns None or why it failed (refused then, nothing changes);
        ``read()``: the value in effect (a device's, which may change by itself), else the stored one. Returns the
        value; ParameterError for a declaration that does not fit (a value at the start outside the range, too)."""
        if not _NAME.match(name):
            raise ParameterError(f"parameter {name!r}: names of letters, digits and _, parts joined by dots")
        value = ParameterValue.of(default)
        T = ParameterType
        descriptor = ParameterDescriptor(name, value.type, description, constraints_text(unit, choices), read_only)
        if range is not None:
            lo, hi, *step = range
            if lo > hi:
                raise ParameterError(f"{name}: range from {lo} to {hi}")
            if value.type == T.INTEGER:
                descriptor.integer_range = [IntegerRange(int(lo), int(hi), int(step[0]) if step else 0)]
            elif value.type == T.DOUBLE:
                descriptor.floating_point_range = [FloatingPointRange(float(lo), float(hi),
                                                                      float(step[0]) if step else 0.0)]
            else:
                raise ParameterError(f"{name}: a range is for integer and double parameters")
        choices = list(choices or [])
        problem = check(descriptor, value, choices)
        if problem:
            raise ParameterError(f"{name}: its default: {problem}")
        with self._lock:
            if name in self._declared:
                raise ParameterError(f"{name} is declared already")
            if name in self._overrides:
                given, where = self._overrides[name]
                try:
                    value = from_text(descriptor, given)
                except (TypeError, ValueError) as error:
                    raise ParameterError(f"{name} := {given!r} ({where}): {error}") from None
                problem = check(descriptor, value, choices)
                if problem:
                    raise ParameterError(f"{name} := {given!r} ({where}): {problem}")
            self._declared[name] = _Declared(descriptor, value, choices, read, write)
            self._publish_event(new=[Parameter(name, self._current(self._declared[name]))])
        return value.value()

    def check_overrides(self):
        """Every parameter declared: a value given at the start for a parameter the node does not have is an error
        (a typo; ROS would ignore it); ``--dump-params`` gets the parameters in effect."""
        with self._lock:
            unknown = sorted(set(self._overrides) - set(self._declared))
        if unknown:
            raise ParameterError(f"{self.name} has no parameter " + ", ".join(
                f"{n} (given by {self._overrides[n][1]})" for n in unknown) + "; it has " +
                (", ".join(sorted(self._declared)) or "none"))
        if self._dump is not None:
            self._dump.write_text(self.dump())

    def apply_all(self):
        """Every writable parameter's writer called with its value (at the start: a device keeps its last values
        otherwise), in the order they were declared: a setting declared before its ``auto_*`` bool ends as the bool
        says (writing the value switches to manual). ParameterError with the reasons of those that failed."""
        with self._lock:
            failed = []
            for name, d in self._declared.items():  # as declared: a setting's auto_* after it, as a device wants
                if d.write is not None and not d.descriptor.read_only:
                    reason = self._call_write(d, d.value)
                    if reason:
                        failed.append(f"{name}: {reason}")
        if failed:
            raise ParameterError(f"{self.name}: " + "; ".join(failed))

    # ------------------------------------------------------------------------------------------- a node's options

    def declare_options(self, parser, args, live: dict | None = None, leave_out=()) -> list[str]:
        """Every option of a node's argparse ``parser`` as a parameter, so that the terminal interface and
        ``xblick param`` show how the node runs, and ``--param``/``--params-file`` set any option at the start.
        Call it after ``parse_args`` and ``configure``, before the options are used: a value given at the start is
        written back into ``args`` (converted with the option's ``type``).

        The parameters are read-only (they apply at the next start), except those in ``live``: option name (its
        ``dest``) -> the keyword arguments of ``declare`` (``range``, ``unit``, ``choices``, ``write``, ``read``, and
        ``default`` for an option whose value is None). A live parameter changes while the node runs; the node reads
        it where it applies the value (``params["threshold"]``) or applies it in ``write``.

        Left out: ``--help``, the parameters' own options, the Zenoh connection's (``--connect``, ``--listen``,
        ``--zenoh-mode``, ``--config``), options in ``leave_out`` and parameters declared already. An option whose
        value is None is a read-only string, empty when not given. Returns the names declared."""
        live = dict(live or {})
        dests = {a.dest for a in parser._actions}  # argparse keeps no public list of its options
        unknown = sorted(set(live) - dests)
        if unknown:
            raise ParameterError(f"{self.name}: live parameters {unknown} are no options of {parser.prog}")
        declared = []
        for action in parser._actions:
            dest = action.dest
            if (dest in OWN_OPTIONS or dest in leave_out or dest == argparse.SUPPRESS or action.help == argparse.SUPPRESS
                    or not hasattr(args, dest)):
                continue
            with self._lock:
                if dest in self._declared:
                    continue
            value = getattr(args, dest)
            spec = dict(live.get(dest) or {})
            default = spec.pop("default", value)
            unset = default is None
            if unset:
                if dest in live:
                    raise ParameterError(f"{self.name}: live parameter {dest} is None: give its type with a 'default' "
                                         "in its spec")
                default = ""
            if isinstance(default, Path):
                default = str(default)
            elif isinstance(default, tuple):
                default = list(default)
            if not isinstance(default, (bool, int, float, str, list)):
                default = str(default)
            if (action.choices is not None and not unset and not isinstance(default, list)
                    and "choices" not in spec):
                spec["choices"] = [str(c) if isinstance(default, str) else c for c in action.choices]
            self.declare(dest, default, description=_option_help(parser, action), read_only=dest not in live, **spec)
            declared.append(dest)
            given = self[dest]
            if unset:
                given = None if given == "" else (action.type(given) if callable(action.type) else given)
            elif value is None and given == default:  # a live option's stand-in for None (--max-hz 0: every frame)
                given = None
            elif isinstance(value, Path):
                given = Path(given)
            elif isinstance(value, tuple):
                given = tuple(given)
            setattr(args, dest, given)
        return declared

    # ------------------------------------------------------------------------------------------------- values

    def __getitem__(self, name: str):
        return self.get(name)

    def get(self, name: str):
        """The value in effect (its reader's, else the stored one)."""
        with self._lock:
            if name not in self._declared:
                raise KeyError(f"{self.name} has no parameter {name!r}")
            return self._current(self._declared[name]).value()

    def overridden(self, name: str) -> bool:
        """A value for it was given at the start."""
        return name in self._overrides

    def descriptor(self, name: str) -> ParameterDescriptor:
        with self._lock:
            return dataclasses.replace(self._declared[name].descriptor)

    def names(self) -> list[str]:
        with self._lock:
            return sorted(self._declared)

    def set(self, name: str, value) -> SetParametersResult:
        """A change by the node itself (as one over Zenoh, but read-only access does not apply)."""
        return self._set([Parameter(name, ParameterValue.of(value))], atomic=False, local=True)[0]

    def refresh(self):
        """The values that changed by themselves (their readers': a device's, a playback's) stored and announced
        in one ParameterEvent; call it when the node's state may have changed (a node polls it, e.g. once a second)."""
        with self._lock:
            changed = []
            for name, d in self._declared.items():
                if d.read is None:
                    continue
                now = self._current(d)
                if now != d.value:
                    d.value = now
                    changed.append(Parameter(name, now))
            if changed:
                self._publish_event(changed=changed)

    def dump(self) -> str:
        """The parameters now as a params file of this node."""
        with self._lock:
            values = {n: self._current(d).value() for n, d in sorted(self._declared.items())}
        return dump_params_file({self.name: values})

    def _current(self, d: _Declared) -> ParameterValue:
        if d.read is None:
            return d.value
        got = d.read()
        return got if isinstance(got, ParameterValue) else ParameterValue.of(got)

    @staticmethod
    def _call_write(d: _Declared, value: ParameterValue) -> str | None:
        try:
            return d.write(value.value()) or None
        except Exception as error:  # noqa: BLE001 - the node's writer failed: the change refused, with why
            return f"the node failed to take it: {type(error).__name__}: {error}"

    def _refusal(self, p: Parameter, local: bool) -> str | None:
        declared = self._declared.get(p.name)
        if declared is None:
            return f"no parameter {p.name!r}"
        if self.read_only_access and not local:
            return READ_ONLY_ACCESS
        if declared.descriptor.read_only:
            return RESTART
        return check(declared.descriptor, p.value, declared.choices)

    def _set(self, parameters: list[Parameter], atomic: bool, local: bool = False,
             gate: Callable | None = None) -> list[SetParametersResult]:
        """``gate``: called just before the first change (a command's CommandQuery.check_deadline), after everything
        that may take time but changes nothing (the lock, the readers); its CommandRefused leaves everything as it
        was."""
        with self._lock:
            reasons = [self._refusal(p, local) for p in parameters]
            if atomic and any(reasons):
                return [SetParametersResult(False, "; ".join(f"{p.name}: {r}" for p, r in zip(parameters, reasons)
                                                             if r))]
            before = {n: self._current(d) for n, d in self._declared.items()}
            if gate is not None:
                gate()
            results = []
            for p, reason in zip(parameters, reasons):
                if reason is None:
                    d = self._declared[p.name]
                    if d.write is not None:
                        reason = self._call_write(d, p.value)
                    if reason is None:
                        d.value = p.value
                results.append(SetParametersResult(reason is None, reason or ""))
            # every value that differs now: a write may change others (a control's value switching its auto mode off);
            # those stored as they are now, so that apply_all writes what the device was left with
            after = {n: self._current(d) for n, d in self._declared.items()}
            changed = [Parameter(n, v) for n, v in sorted(after.items()) if v != before[n]]
            for p in changed:
                self._declared[p.name].value = p.value
            if changed:
                self._publish_event(changed=changed)
            if atomic:
                failed = [f"{p.name}: {r.reason}" for p, r in zip(parameters, results) if not r.successful]
                return [SetParametersResult(not failed, "; ".join(failed))]
            return results

    def _publish_event(self, new=(), changed=()):
        if self._events is not None:
            event = ParameterEvent(Time.from_ns(time.time_ns()), self.name, list(new), list(changed))
            payload = bytes(event.serialize())
            self._events.put(payload)
            if self._ros_events is not None and self._ros_events.matched:
                if self._ros_name != self.name:  # ROS's event names the ROS node
                    event.node = self._ros_name
                    payload = bytes(event.serialize())
                self._ros_events.put(payload)

    # ---------------------------------------------------------------------------------------------- the services

    def _serve(self, period: float = SERVE_PERIOD_S):
        """The services' queries, answered as they come (looked for every ``period`` seconds), until close(), or
        until the session closed (its queryables' channels close with it: a node that closed its session first)."""
        while not self._stop.is_set():
            for service, queryable in self._queryables:
                while (query := self._next(queryable)) is not None:
                    if query is _CLOSED:
                        return
                    self._answer(service, query)
            for service, server in self._ros_services:
                for _ in range(ROS_CALLS_PER_ROUND):  # at most so many: our own calls come first each round
                    query = self._next_ros(server)
                    if query is None:
                        break
                    self._answer_ros(service, server, query)
            self._stop.wait(period)

    @staticmethod
    def _next_ros(server: ros.RosService):
        """A ROS service's next query; None when there is none now, also when the service or its channel closed (its
        ROS node closed before the session: Session.close does so; Zenoh then says "Undeclared queryable" or
        "closed"). Our own services go on; the session's end ends them."""
        if server.closed:
            return None
        try:
            return server.try_recv()
        except (zenoh.ZError, AttributeError) as error:  # AttributeError: closed between the check and the call
            if isinstance(error, AttributeError) or "closed" in str(error) or "Undeclared" in str(error):
                return None
            raise

    @staticmethod
    def _next(queryable):
        """The queryable's next query, None when there is none now, _CLOSED when its channel closed (its session
        closed; Zenoh says so by ZError "channel is empty and closed", the only error of try_recv)."""
        try:
            return queryable.try_recv()
        except zenoh.ZError as error:
            if "closed" in str(error):
                return _CLOSED
            raise

    def _answer(self, service: str, query: zenoh.Query):
        """A service's query answered, whatever it holds (no input ends the server thread). A set (a command) is
        checked twice (session.CommandQuery): the node it is meant for on receipt, before anything else, so that a node
        it is not meant for never answers in its own name; the deadline just before its first change (``_set``).
        Every reply names this node and its clock (session.answer_attachment), so that a sender can address a command
        to it."""
        request_type, _ = SERVICES[service]
        received = None
        try:
            if service in (P.SET_PARAMETERS, P.SET_PARAMETERS_ATOMICALLY):
                received = CommandQuery(query, self._zid, f"{self.key}/{service}")
                received.check_node()
            payload = query.payload
            request = request_type.deserialize(payload.to_bytes()) if payload is not None else request_type()
            response = self._respond(service, request, received.check_deadline if received is not None else None)
        except CommandRefused as refused:
            log.warning(str(refused))
            refuse(query, refused)
            return
        except Exception as error:  # noqa: BLE001 - the asker gets the reason
            query.reply_err(f"{self.name}/{service}: {type(error).__name__}: {error}")
            return
        query.reply(query.key_expr, bytes(response.serialize()), encoding=cdr_encoding(type(response)),
                    attachment=answer_attachment(self._zid, received.checked if received is not None else ()))

    def _answer_ros(self, service: str, server: ros.RosService, query: zenoh.Query):
        """A ROS client's call on ROS's key, whatever it holds (no input ends the server thread). It carries ROS's
        attachment (sequence number, client gid), no node and no deadline. A set is refused, every parameter with
        ROS_SETS_OFF, unless the session was started with ``--ros-params-writable``. Then it is executed without a
        node check and without a deadline (``gate`` None), and uncounted: every node of the name gets it from a ROS
        client and executes it. Read-only access and each parameter's checks apply as always. A set whose writer
        blocks holds the parameters' lock, for our own calls too, as any set does. The reply carries the request's
        sequence number and client gid, as rmw_zenoh's. Replied to and dropped at once (``query`` goes out of scope
        here)."""
        request_type, _ = SERVICES[service]
        try:
            try:
                request_id = server.request_of(query)
            except ros.RosError as error:  # not a ROS client's: refused, nothing done
                query.reply_err(f"{self.name}/{service}: {error}")
                return
            try:
                payload = query.payload
                request = request_type.deserialize(payload.to_bytes()) if payload is not None else request_type()
                if service in SETS and not self._ros_node.ros.params_writable:
                    response = self._ros_sets_off(service, request)
                else:
                    response = self._respond(service, request)
                if service == P.DESCRIBE_PARAMETERS and not self._ros_node.ros.params_writable:
                    for descriptor in response.descriptors:  # a ROS tool shows no field editable that refuses every set
                        descriptor.read_only = True
            except Exception as error:  # noqa: BLE001 - the asker gets the reason
                query.reply_err(f"{self.name}/{service}: {type(error).__name__}: {error}")
                return
            server.reply(query, response.serialize(), request_id)
        except Exception as error:  # noqa: BLE001 - a reply that cannot go out (the session closing): said, not raised
            log.warning(f"{self.name}/{service}: a ROS call not answered: {type(error).__name__}: {error}")

    @staticmethod
    def _ros_sets_off(service: str, request):
        """The answer to a ROS client's set while sets through ROS are off: refused, nothing changed."""
        if service == P.SET_PARAMETERS:
            return SetParametersResponse([SetParametersResult(False, ROS_SETS_OFF) for _ in request.parameters])
        return SetParametersAtomicallyResponse(SetParametersResult(False, ROS_SETS_OFF))

    def respond(self, service: str, request):
        """What the service answers to the request (the queryables' work)."""
        return self._respond(service, request)

    def _respond(self, service: str, request, gate: Callable | None = None):
        """``respond``, a set's ``gate`` called just before its first change (``_set``)."""
        with self._lock:
            declared = dict(self._declared)
            if service == P.LIST_PARAMETERS:
                names = [n for n in sorted(declared) if _under(n, request.prefixes, request.depth)]
                prefixes = sorted({n.rsplit(".", 1)[0] for n in names if "." in n})
                return ListParametersResponse(names, prefixes)
            if service == P.DESCRIBE_PARAMETERS:
                out = []
                for n in request.names:
                    d = dataclasses.replace(declared[n].descriptor) if n in declared else ParameterDescriptor(n)
                    d.read_only = d.read_only or self.read_only_access
                    out.append(d)
                return DescribeParametersResponse(out)
            if service == P.GET_PARAMETERS:
                return GetParametersResponse([self._current(declared[n]) if n in declared else ParameterValue()
                                              for n in request.names])
            if service == P.GET_PARAMETER_TYPES:
                return GetParameterTypesResponse([declared[n].descriptor.type if n in declared else 0
                                                  for n in request.names])
        if service == P.SET_PARAMETERS:
            return SetParametersResponse(self._set(request.parameters, atomic=False, gate=gate))
        return SetParametersAtomicallyResponse(self._set(request.parameters, atomic=True, gate=gate)[0])

    @classmethod
    def none_on(cls, ros_node: ros.RosNode) -> Parameters:
        """No parameters, as the six services of ``ros_node`` on ROS's keys only: a program's ROS node until its
        first Parameters (ros.Ros), so that ROS's parameter tools get an answer from every node, as from ROS's own.
        Nothing on our keys. Every set is refused (no such parameter)."""
        p = cls(None, ros_node.name, ros_node.namespace.strip("/"))
        p._ros_node, p._ros_name, p._ros_none = ros_node, ros_node.fully_qualified, True
        p._queryables = []
        p._ros_services = [(service, ros_node.service(f"{p._ros_name}/{service}", ROS_SERVICES[service]))
                           for service in SERVICES]
        p._server = threading.Thread(target=p._serve, args=(NONE_SERVE_PERIOD_S,), name=f"parameters {p._ros_name}",
                                     daemon=True)
        p._server.start()
        return p

    def close(self):
        """Ends the services (again: nothing). A node calls it at its end; without it they are ended when the program
        ends (session.close_at_exit)."""
        forget_at_exit(self)
        if self._server is None:
            return
        if self._token is not None:
            self._token.undeclare()
        self._stop.set()
        self._server.join(timeout=2.0)
        self._server = None
        for _, q in self._queryables or []:
            q.undeclare()
        if self._events is not None:
            self._events.undeclare()
        self._undo_ros()
        self._token = None

    def _undo_ros(self):
        """What these parameters put on their ROS node closed (close; a constructor that failed after
        ros.Ros.parameters_node): a node of their own closed, else their services and events publisher, and the
        program's node released (ros.Ros.release_parameters: its services without parameters back, the next
        Parameters on it)."""
        if self._ros_node is None:
            return
        if self._ros_own_node:
            self._ros_node.close()  # its services, its events publisher and its token
        else:  # the program's node: only what these parameters put on it
            try:
                for _, server in self._ros_services:
                    server.close()
                if self._ros_events is not None:
                    self._ros_events.close()
            finally:
                if not self._ros_none:
                    self._ros_node.ros.release_parameters()
        self._ros_node = None


def _option_help(parser, action) -> str:
    """An option's help as ``--help`` prints it (``%(default)s`` filled in), with its flags first."""
    text = action.help or ""
    try:
        text = text % {**vars(action), "prog": parser.prog}
    except (KeyError, TypeError, ValueError):
        pass
    flags = "/".join(action.option_strings) or action.dest
    return f"{flags}: {text}" if text else flags


def _under(name: str, prefixes: list[str], depth: int) -> bool:
    """ROS's list_parameters: below one of the prefixes (or any, without), at most ``depth`` levels (0: any)."""
    if not prefixes:
        return depth == 0 or name.count(".") < depth
    for prefix in prefixes:
        if name == prefix or name.startswith(prefix + "."):
            rest = name[len(prefix):].lstrip(".")
            return depth == 0 or rest.count(".") < depth
    return False


# ------------------------------------------------------------------------------------------------ the asker's side


class ParameterClient:
    """Another node's parameters over Zenoh (``key``: its ``<namespace>/<name>``): ROS 2's services, and its events
    (``on_event``). Reads: TimeoutError when it does not answer within ``timeout``, RuntimeError when several do.
    Sets: not sent unless exactly one node answers first (``_call``); then sent once, to that node with a deadline
    when that node takes part (its answer names it, session.one_node): NotReached (a TimeoutError, nothing sent),
    Expired (refused by the node, past its deadline: not applied), NoAnswer (no TimeoutError: sent once, no answer;
    not sent again). ``last_checked``: the node of the last set said that it checked node and deadline before
    applying it (session.checked).

    ``ros`` (a session in ROS 2): the ROS node ``/<key>``'s parameters through ROS 2's keys (any ROS node's, or
    a Durchblick node's). Reads as above. A set: not sent unless exactly one node answered a
    list_parameters read first, then sent once, without node or deadline (ROS has neither: the receiver cannot
    refuse a late set, ``last_checked`` stays False); NotReached, NoAnswer, PartlyAnswered and SeveralExecuted as
    above. ``on_event``: ROS's ``/parameter_events``."""

    def __init__(self, session: zenoh.Session, key: str, timeout: float = 2.0, ros: bool = False):
        self.key = topics.check_name(key.strip("/"))
        self.name = fully_qualified(self.key)
        self.timeout = timeout
        self.last_checked = False
        self._session = session
        self._subscribers = []
        self._ros_clients: dict[str, ros.RosClient] | None = None
        if ros:
            if getattr(session, "ros", None) is None:
                raise ValueError(f"{self.name}: a ROS node's parameters need {NEEDS_ROS}")
            check_ros_name(self.name, "node")
            self._ros_clients = {service: session.ros.node.client(f"{self.name}/{service}", ROS_SERVICES[service])
                                 for service in SERVICES}

    def _call(self, service: str, request):
        """One service call: a read (list, describe, get, types) asked again while nothing answers (session.ask);
        a set (a command: a node may act on it) sent once (session.command), and only after a read (list_parameters)
        had exactly one answer and no "Timeout" reply (session.one_node: a best-effort check of two nodes on the key,
        which would both apply it), to the node that answered it (with a deadline, when that node takes part).
        NotReached, Expired, NoAnswer, PartlyAnswered: as session.command. Ordinary reads keep the one answer that came
        and ignore a "Timeout" reply, which says that another node had not finished the read: a read changes
        nothing."""
        if self._ros_clients is not None:
            return self._call_ros(service, request)
        _, response_type = SERVICES[service]
        key = f"{self.key}/{service}"
        if service in (P.SET_PARAMETERS, P.SET_PARAMETERS_ATOMICALLY):
            list_key = f"{self.key}/{P.LIST_PARAMETERS}"
            self.last_checked = False
            try:
                to = one_node(self._session, list_key, self.timeout,
                              payload=bytes(ListParametersRequest([], 0).serialize()))
            except RuntimeError as error:
                raise RuntimeError(f"{error} (the set to {key})") from None
            except TimeoutError as error:
                raise TimeoutError(f"{error} (the set to {key})") from None
            ok = command(self._session, key, self.timeout, to, payload=bytes(request.serialize()))
            self.last_checked = checked(ok[0], to)
            return response_type.deserialize(ok[0].payload.to_bytes())
        answers = ask(self._session, key, self.timeout, payload=bytes(request.serialize()))
        errors = [reply.err.payload.to_string() for reply in answers if reply.err is not None and not timed_out(reply)]
        if errors:
            raise RuntimeError(errors[0])
        ok = [reply.ok for reply in answers if reply.ok is not None]
        if not ok:
            raise TimeoutError(f"no answer from {key} within {self.timeout:g} s")
        if len(ok) > 1:
            raise RuntimeError(f"{len(ok)} nodes answer {key}: one name, one node")
        return response_type.deserialize(ok[0].payload.to_bytes())

    def _read_ros(self, service: str, request) -> list:
        """A read of a ROS node's service: one request (its own sequence number) after the other until a reply came
        or the timeout passed (as session.ask)."""
        client = self._ros_clients[service]
        end = time.monotonic() + self.timeout
        while True:
            answers = replies(self._session, client.key, max(end - time.monotonic(), 0.01),
                              **client.request_options(request.serialize()))
            if answers or time.monotonic() >= end:
                return answers
            time.sleep(min(ASK_PERIOD, max(0.0, end - time.monotonic())))

    def _call_ros(self, service: str, request):
        """``_call`` on ROS's keys. A set (a command) only after a list_parameters read had exactly one answer and no
        "Timeout" reply (session.count, best effort), then sent once (session.command without a target: ROS's
        services take no node or deadline). Replies of a ROS service carry no answer attachment of ours, so nothing is
        checked. session.command waits for the key's queryable with Zenoh's default target, while the set goes with
        ALL_COMPLETE: a non-complete queryable on the key (a storage on ``**``) can then make it a NoAnswer although
        no ROS service got it (on the safe side: "may have run")."""
        _, response_type = SERVICES[service]
        client = self._ros_clients[service]
        if service in SETS:
            self.last_checked = False
            lister = self._ros_clients[P.LIST_PARAMETERS]
            try:
                count(self._read_ros(P.LIST_PARAMETERS, ListParametersRequest([], 0)), lister.topic)
            except RuntimeError as error:
                raise RuntimeError(f"{error} (the set to {client.topic})") from None
            except TimeoutError as error:
                raise TimeoutError(f"{error} (the set to {client.topic})") from None
            ok = command(self._session, client.key, self.timeout, None, **client.request_options(request.serialize()))
            return response_type.deserialize(ok[0].payload.to_bytes())
        answers = self._read_ros(service, request)
        errors = [reply.err.payload.to_string() for reply in answers if reply.err is not None and not timed_out(reply)]
        if errors:
            raise RuntimeError(errors[0])
        ok = [reply.ok for reply in answers if reply.ok is not None]
        if not ok:
            raise TimeoutError(f"no answer from {client.topic} within {self.timeout:g} s")
        if len(ok) > 1:
            raise RuntimeError(f"{len(ok)} nodes answer {client.topic}: one name, one node")
        return response_type.deserialize(ok[0].payload.to_bytes())

    def list(self, prefixes=(), depth: int = 0) -> list[str]:
        return self._call(P.LIST_PARAMETERS, ListParametersRequest(list(prefixes), depth)).names

    def describe(self, names: list[str]) -> list[ParameterDescriptor]:
        return self._call(P.DESCRIBE_PARAMETERS, DescribeParametersRequest(list(names))).descriptors

    def get(self, names: list[str]) -> list[ParameterValue]:
        return self._call(P.GET_PARAMETERS, GetParametersRequest(list(names))).values

    def types(self, names: list[str]) -> list[int]:
        return self._call(P.GET_PARAMETER_TYPES, GetParameterTypesRequest(list(names))).types

    def set(self, values: dict) -> list[SetParametersResult]:
        """Each separately (ParameterValue or a Python value per name)."""
        return self._call(P.SET_PARAMETERS, SetParametersRequest(_parameters(values))).results

    def set_atomically(self, values: dict) -> SetParametersResult:
        return self._call(P.SET_PARAMETERS_ATOMICALLY, SetParametersAtomicallyRequest(_parameters(values))).result

    def all(self) -> tuple[list[ParameterDescriptor], list[ParameterValue]]:
        """Every parameter's descriptor and value."""
        names = self.list()
        return self.describe(names), self.get(names)

    def on_event(self, callback: Callable[[ParameterEvent], None]):
        """``callback(event)`` for each of its ParameterEvents (of this node only), in Zenoh's thread."""
        def receive(sample: zenoh.Sample):
            take(sample.payload.to_bytes())

        def take(payload: bytes, *_):
            event = ParameterEvent.deserialize(payload)
            if event.node == self.name:
                callback(event)

        if self._ros_clients is not None:
            self._subscribers.append(self._session.ros.node.subscriber("/parameter_events", ParameterEvent.TYPE, take,
                                                                       ros.PARAMETERS_QOS))
        else:
            self._subscribers.append(self._session.declare_subscriber(P.PARAMETER_EVENTS, receive))

    def close(self):
        for s in self._subscribers:
            s.close() if isinstance(s, ros.RosSubscriber) else s.undeclare()
        self._subscribers = []
        for client in (self._ros_clients or {}).values():
            client.close()
        self._ros_clients = None if self._ros_clients is None else {}


def _parameters(values: dict) -> list[Parameter]:
    return [Parameter(n, v if isinstance(v, ParameterValue) else ParameterValue.of(v)) for n, v in values.items()]


def find_nodes(session: zenoh.Session, timeout: float = 1.0) -> list[str]:
    """The keys (``<namespace>/<name>``) of the nodes offering parameters now (their liveliness tokens), asked
    again every ASK_PERIOD seconds until ``timeout``: a session just opened knows no token yet (session.ask)."""
    suffix = "/" + P.PARAMETERS_TOKEN
    found = set()
    end = time.monotonic() + timeout
    while (remaining := end - time.monotonic()) > 0:
        found.update(str(r.ok.key_expr)[:-len(suffix)]
                     for r in replies(session, "**" + suffix, remaining, liveliness=True) if r.ok is not None)
        time.sleep(max(0.0, min(ASK_PERIOD, end - time.monotonic())))
    return sorted(found)


# --------------------------------------------------------------------------------------------------- params files


def parse_scalar(text: str):
    """A YAML scalar as a params file has it: true/false, an integer, a float, a flow list ``[...]``, a quoted or
    plain string."""
    s = text.strip()
    if s.startswith("[") and s.endswith("]"):
        inner = s[1:-1].strip()
        return [parse_scalar(part) for part in _split_flow(inner)] if inner else []
    if len(s) >= 2 and s[0] == s[-1] and s[0] in "'\"":
        return json.loads(s) if s[0] == '"' else s[1:-1].replace("''", "'")
    lower = s.lower()
    if lower in ("true", "false"):
        return lower == "true"
    if lower in ("~", "null"):
        return None
    if re.fullmatch(r"[-+]?(0|[1-9][0-9]*)", s):
        return int(s)
    if re.fullmatch(r"[-+]?(\d+\.\d*|\.\d+|\d+)([eE][-+]?\d+)?|[-+]?\.inf|\.nan", lower):
        return float(lower.replace(".inf", "inf").replace(".nan", "nan"))
    return s


def _split_flow(text: str) -> list[str]:
    parts, depth, quote, start = [], 0, None, 0
    for i, c in enumerate(text):
        if quote:
            if c == quote:
                quote = None
        elif c in "'\"":
            quote = c
        elif c == "[":
            depth += 1
        elif c == "]":
            depth -= 1
        elif c == "," and depth == 0:
            parts.append(text[start:i])
            start = i + 1
    parts.append(text[start:])
    return [p for p in (p.strip() for p in parts)]


def parse_yaml(text: str, where: str = "params file") -> dict:
    """The block mappings of a params file (indentation by spaces, ``key: value``, ``key:`` before a deeper block,
    ``#`` comments); ParameterError for anything else."""
    root: dict = {}
    stack = [(-1, root)]
    for number, raw in enumerate(text.splitlines(), 1):
        line = _strip_comment(raw).rstrip()
        if not line.strip() or line.strip() == "---":
            continue
        indent = len(line) - len(line.lstrip(" "))
        if "\t" in line[:indent + 1]:
            raise ParameterError(f"{where}:{number}: tabs for indentation")
        key, sep, rest = line.strip().partition(":")
        if not sep or (rest and not rest.startswith(" ")):
            raise ParameterError(f"{where}:{number}: 'key: value' expected, not {raw.strip()!r}")
        key = parse_scalar(key) if key.strip()[:1] in "'\"" else key.strip()
        while stack and indent <= stack[-1][0]:
            stack.pop()
        if not stack:
            raise ParameterError(f"{where}:{number}: indentation")
        parent = stack[-1][1]
        if not isinstance(parent, dict):
            raise ParameterError(f"{where}:{number}: a mapping under a value")
        if rest.strip():
            parent[key] = parse_scalar(rest)
        else:
            parent[key] = {}
            stack.append((indent, parent[key]))
    return root


def _strip_comment(line: str) -> str:
    quote = None
    for i, c in enumerate(line):
        if quote:
            if c == quote:
                quote = None
        elif c in "'\"":
            quote = c
        elif c == "#" and (i == 0 or line[i - 1] in " \t"):
            return line[:i]
    return line


def _flatten(mapping: dict, prefix: str = "") -> dict:
    out = {}
    for k, v in mapping.items():
        name = f"{prefix}.{k}" if prefix else str(k)
        if isinstance(v, dict):
            out.update(_flatten(v, name))
        else:
            out[name] = v
    return out


def _matches(pattern: str, node: str) -> bool:
    """A params file's node key against a fully qualified node name: ``/<namespace>/<name>`` (the ``/`` optional),
    ``<name>`` alone (any namespace), ``**`` any levels, ``*`` one level (``/**`` every node)."""
    p = [s for s in pattern.strip("/").split("/") if s]
    n = [s for s in node.strip("/").split("/") if s]
    if len(p) == 1 and not pattern.startswith("/") and p[0] not in ("*", "**"):
        return bool(n) and n[-1] == p[0]

    def match(i: int, j: int) -> bool:
        if i == len(p):
            return j == len(n)
        if p[i] == "**":
            return any(match(i + 1, k) for k in range(j, len(n) + 1))
        return j < len(n) and p[i] in ("*", n[j]) and match(i + 1, j + 1)

    return match(0, 0)


def read_params_file(path: Path, node: str) -> dict:
    """The values a params file gives the node ``node`` (fully qualified), in file order (later win)."""
    data = parse_yaml(path.read_text(), str(path))
    values = {}
    for pattern, section in data.items():
        if not isinstance(section, dict) or "ros__parameters" not in section:
            raise ParameterError(f"{path}: {pattern}: 'ros__parameters:' expected below the node")
        if _matches(str(pattern), node):
            values.update(_flatten(section["ros__parameters"]))
    return values


def _yaml_scalar(value) -> str:
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, float):
        if math.isnan(value):
            return ".nan"
        return repr(value) if math.isfinite(value) else (".inf" if value > 0 else "-.inf")
    if isinstance(value, int):
        return str(value)
    if isinstance(value, (bytes, bytearray)):
        return "[" + ", ".join(str(b) for b in value) + "]"
    if isinstance(value, list):
        return "[" + ", ".join(_yaml_scalar(v) for v in value) + "]"
    text = str(value)
    if text == "" or parse_scalar(text) != text or any(c in text for c in ":#[]{},'\"") or text != text.strip():
        return json.dumps(text)
    return text


def dump_params_file(nodes: dict[str, dict]) -> str:
    """A params file of nodes (fully qualified name -> {parameter: value}), dotted names nested."""
    lines = []
    for node, values in nodes.items():
        lines.extend([f"{node}:", "  ros__parameters:"])
        tree: dict = {}
        for name, value in sorted(values.items()):
            parts = name.split(".")
            at = tree
            for part in parts[:-1]:
                at = at.setdefault(part, {})
            at[parts[-1]] = value

        def write(mapping: dict, indent: int):
            for k, v in mapping.items():
                if isinstance(v, dict):
                    lines.append(" " * indent + f"{k}:")
                    write(v, indent + 2)
                else:
                    lines.append(" " * indent + f"{k}: {_yaml_scalar(v)}")

        write(tree, 4)
    return "\n".join(lines) + "\n"
