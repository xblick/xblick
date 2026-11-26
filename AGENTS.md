# AGENTS.md

Guidance for coding agents in this repository. Human documentation: `README.md` (entry) and `docs/` (concepts,
guides, reference). Do not duplicate it here. Style rules for documentation: `docs/project/writing-docs.md`.

## What this is

Durchblick: camera streams and perception results between programs, over Zenoh, as ROS 2 message types (CDR),
without ROS. C++20 core in `cpp/` (Python bindings `xblick._core`), Python package `xblick` in `src/xblick/`
(command `xblick`). Each perception method is a node in its own repository (`xblick-<name>`) that depends on this
package. Robot integrations live in repositories of their own. The core never depends on a robot framework.
Zenoh 1.10 docs: zenoh.io/docs (concepts, deployment), API docs.rs/zenoh/1.10.1, config keys DEFAULT_CONFIG.json5 at
tag 1.10.1 (github.com/eclipse-zenoh/zenoh).

Names: every name in code, on the wire and on disk is `xblick` (since 0.5.0): Python package, distribution and
command, C++ namespace, include directory, CMake package and targets, C++ programs, `XBLICK_*` variables, Zenoh keys,
ring names, message types (`xblick_msgs`), directories. Only the prose name of the project is "Durchblick". A new
name follows this rule. Renaming a wire or on-disk name breaks every older program: only with a minor version.

## Skills

Project skills live in `.agents/skills/` (open Agent Skills format); `.claude/skills` links to it. Rules for writing
one: `.agents/skills/README.md`.

| Skill | For |
| --- | --- |
| `xblick-build-node` | a new vision node, or making a node faster or leaner |
| `xblick-run-nodes` | several nodes for an experiment or a demo, on one or more GPUs and PCs |
| `xblick-benchmark-nodes` | throughput, latency, GPU load and memory of nodes and setups |
| `xblick-develop-tui` | the terminal interface (`xblick`): tabs, dialogs, layout, its tests and screenshots |
| `xblick-release-version` | a release: the gate before a push, version, CHANGELOG entry, GitHub release, PyPI, follow-up |

- Before a task, look for a matching skill and follow it. Read its `SKILL.md` in full.
- After a complex task that is likely to recur and can be abstracted (no batch names, dates or results of one
  experiment), propose a new skill or a fix of an existing one in two or three lines. Write it only after the user's
  yes. Prefer improving a skill over adding one.
- Rules for every task stay in this file; a rule that must always hold belongs in a hook; a skill holds a procedure.
- Node repositories have no skills of their own. An agent working in one uses the skills of this repository.

## Commands

```bash
uv sync --all-extras                      # builds the C++ core (scikit-build-core, C++20); rebuilds when cpp/ changes
uv sync --reinstall-package xblick         # force the rebuild
CUDA_VISIBLE_DEVICES=0 uv run pytest -q   # ~5 min, ~633 tests (2026-11-13); CUDA tests skip without a GPU
uv run pytest -q tests/test_recording.py  # one file
ruff check --line-length 120 src tests    # lint; ruff is not a project dependency
```

C++ library, tools, point cloud node, C++ against Python: `docs/project/development.md`.
CI: `.github/workflows/ci.yml` (reference check, Python 3.10 and 3.12, C++ tests, interop, manylinux wheels).
PyPI files: `tools/build_wheels.sh <out> <work>` in quay.io/pypa/manylinux_2_28_x86_64 (CI job wheels, `.github/workflows/publish.yml`: tag v<version>, trusted publishing, environment pypi).
Reference check: `tools/check_references.sh` (no internal or robot-framework names outside its allowlist).

## Rules

- Fail fast. A wrong configuration, a missing input or a protocol mismatch raises with a message that says what to
  do. Add no default that hides a misconfiguration.
- Protocol: a change of a Durchblick message's fields raises its version in `cpp/include/xblick/protocol.hpp`.
  A ring layout change raises `kVersion` in `ring.hpp`. Either one also raises the package's minor version.
- Version: only in `CMakeLists.txt` (`project(xblick VERSION x.y.z)`). Every user-visible change: patch bump,
  entry in `CHANGELOG.md`, status line in `README.md`. Release: tag `vx.y.z`, CI builds the wheels.
- CHANGELOG entry = the release notes: heading `## x.y.z`, one summary sentence, `### Added` / `### Changed` /
  `### Fixed` (`### Known limit` only if needed), one-line bullets from the user's view, safety fix first and bold,
  no test internals or code file names, one compatibility line, 10 to 20 lines. Details into `docs/`. Cut out with
  `tools/release_notes.sh x.y.z`. Rule: `docs/project/development.md#release-notes`.
- Python and C++ stay equivalent. A feature of the camera, points or detections API exists in both.
  `tests/test_cpp_interop.py` checks it.
- Documentation changes in the same commit as the code. New topic or option: update `docs/reference/`. History goes
  to `CHANGELOG.md`, never into a page.
- A node stamps a result with the capture stamp of its input, never with the processing time.
- Git: only new commits on a branch that equals the public `main`. Never amend, rebase or reset what is pushed; never
  force-push. Write the message to a file and `git commit -F` it (subject, empty line, body).
- Public repository: `tools/check_references.sh` exits 0; nothing internal in files or commit messages.
- Within a minor line (0.5.x) nothing on the wire changes and no API breaks. An addition that older programs ignore
  needs the maintainer's word, a new minor version his approval. A changed default or a new command is a patch,
  named under `### Changed`.
- Nothing chooses silently: fixed, documented defaults; convenience in the `xblick` command (nodes: "Node
  repositories" below).
- Code that sends commands to a robot's controllers (`session.command`, `ask_once`, the controller manager,
  parameter sets, a torque step) gets an independent adversarial review before a release. State its guarantees as
  they are: best effort or guaranteed.
- Tests: never kill by name pattern; leave no process and no ring in `/dev/shm`; never rely on default ports being
  free or on how the network treats an address that does not answer.
- A problem found now and fixed later becomes a GitHub issue; name it by number in the report.
- Code style: match the surrounding code. Docstrings say what, comments say why. 120 columns. No new dependency
  without need.

## Traps

| Symptom | Cause and fix |
| --- | --- |
| Programs find nothing | A Zenoh 1.10 router does not route between peers (Zenoh 1.9+: peers of a router form a clique; failover brokering removed); peers connect to each other's listen addresses (locators). A peer with `--connect` and without `--listen` listens on a free loopback port (session.py, C++ `open_session`), which peers on other PCs or behind a tunnel cannot reach. Across PCs: `--zenoh-mode client`, a router per PC, or `--listen tcp/0.0.0.0:0` (open to the whole network). Commands, nodes (`open_from_args`) and the C++ tools (`SessionOptions::program`) print one line at the start when a peer connects only to other PCs and listens on loopback only (`session.across_pcs`, C++ `across_pcs`); `session.why_nothing` adds it to the empty-result messages. The terminal interface refuses a peer that only connects. |
| A query finds nothing right after start | A new session needs time to learn of queryables. Reads: `session.ask` (C++ `xblick::ask`), asked again until a reply. Commands (a set, a controller switch, a recorder command): `session.command` / `ask_once` (C++ `xblick::ask_once`), sent once to each queryable after one is known (Zenoh's default query target BestMatching reaches all while none is declared complete: never set `complete` on a queryable or a `target` on these queries, tests/test_queries.py pins it; the one exception is ROS 2's keys in `ros.py`, C++ `ros.cpp` and `Parameters::serve_none` (every program, by default): ROS services are complete and ROS requests go with ALL_COMPLETE, as rmw_zenoh's, on keys that start with the domain id and never match ours); first a read of the same node must have exactly one answer and no "Timeout" reply, else not sent (best effort: a node that drops the read, starts after it or serves a broad key still gets it; a session just opened links to the serving sessions one after the other, so its read can miss a second node or reach only a busy one: "no node answered in time"; tests wait until a read reaches all, `knows_all` in tests/test_queries.py). The command names the session that answered the read and carries a deadline (`session.one_node`, `CommandTarget`, attachment `xblick_command`, protocol.hpp), when that answer carries the reply attachment; else nothing is attached, as before. Never send a command again after an empty result: a queryable that dropped it also gives one. `NotReached` (a TimeoutError): not sent. `Expired`: refused by its node, past its deadline, not executed. `NoAnswer` (no TimeoutError): sent once, may have run; a receiver that does not check may still run it. `PartlyAnswered` (a NoAnswer): one answered, the query still open at its timeout. `SeveralExecuted` (a NoAnswer): several answered and executed. |
| A command or set is refused with "the query was still open at its timeout" although one node serves it | The node keeps the query after its reply (works inside the queryable's callback, or holds the Query): the query stays open, and at its timeout the caller's own session adds the error reply "Timeout" (nothing comes over the network), taken for a second node. A receiver replies and drops the query at once; long work outside the callback. xblick's Parameters and recorder do; check external receivers (the robot's bridge). |
| A command runs after the caller gave up | The receiver does not check the deadline: its own code (a robot's bridge) does not use `session.CommandQuery` / C++ `xblick::CommandQuery`, or it is older than 0.5.13, or the command went without a target (the read's answer had no attachment). Such a receiver executes it when it gets it, also after `NoAnswer`; the sender cannot tell. A receiver builds `CommandQuery` inside its error handling, calls `check_node()` on receipt and `check_deadline()` just before the effect, replies with `answer_attachment`, refuses with `refuse` (docs/reference/python-api.md#node-and-deadline). |
| A loop over query replies hangs for minutes | Zenoh does not always finalize a query at its timeout. Never `for r in session.get(...)` or `liveliness().get`. Use `session.replies(session, selector, timeout, ...)` (C++ `xblick::replies`). |
| `Session.close()` never returns | Happens after a query Zenoh did not finalize. Open sessions with `open_session` or `open_from_args` (close waits at most 5 s). Wrap an own `zenoh.open` in `session.Session(...)`. |
| A subscriber gets JPEG on the publisher's host | `ImageSubscriber` defaults to `transport="compressed"`. Pass `transport="auto"`. |
| A test cannot open its own CUDA ring | The creating process cannot open a CUDA ring. Read it from a `spawn`ed child process. |
| Device frames arrive wrong or late | `publish()` of a device array needs `stream=` (for PyTorch `torch.cuda.current_stream()`). Without it the publisher waits for all GPU work of the process. |
| Nodes misbehave on a looped playback | `xblick play --loop` moves all stamps forward on each pass. Nodes rely on stamps that only grow. |
| A test or harness never ends | Zenoh threads keep a process alive. Close sessions. Run long checks in the background with `timeout`. A shell script starts background jobs with SIGINT ignored: `xblick` commands, `node.start` (`node.end_on_signals`) and the C++ tools end on it anyway, other programs and nodes on an older xblick may not. Stop by PID with SIGTERM. Never kill by name pattern. |
| Large writes fail | NFS. Build and write large outputs on a local disk. |
| A test sees extra keys, tokens, queryables or a first output line it did not expect | Every session takes part in ROS 2 by default (ros.py): rmw_zenoh keys (they start with the domain id), `@ros2_lv` tokens, six parameter services per program, a start line on stderr. A test of native behaviour turns it off explicitly: `ros_options=False`, `--no-ros`, `XBLICK_ROS=0` (C++ `.ros = std::nullopt`). A dropped Session in ROS 2 is closed by a finalizer when it is collected (session.py `_release`; not at exit); still close every session. |

## Layout

| Path | Content |
| --- | --- |
| `cpp/include/xblick/protocol.hpp` | wire protocol: topics, message types, versions |
| `cpp/include/xblick/ring.hpp`, `cuda_ring.hpp` | host ring and CUDA ring |
| `cpp/include/xblick/camera.hpp`, `points.hpp`, `msg.hpp`, `parameters.hpp` | C++ API |
| `cpp/include/xblick/rectify.hpp`, `src/xblick/rectify.py` | rectification map from a CameraInfo (R, P), CPU remap; same in C++ and Python |
| `cpp/include/xblick/pairing.hpp` | `partner_of`: partner sample of a primary frame by stamp; Python's `PartnerHistory` calls it via `_core` |
| `cpp/include/xblick/paths.hpp`, `src/xblick/paths.py` | cache, data and config directories; same rule in C++ and Python (`tests/test_paths.py`) |
| `cpp/points/` | point cloud node `xblick-points` (CUDA) |
| `cpp/python/bindings.cpp` | `xblick._core` |
| `src/xblick/camera.py`, `points.py`, `detections.py`, `session.py` | Python API: streams, sessions |
| `src/xblick/node.py`, `history.py`, `tensors.py`, `rgbd.py`, `parameters.py` | helpers for nodes |
| `src/xblick/declaration.py`, `cpp/src/declaration.cpp` | every program's declaration (`xblick/sessions/<zid>/program`), opened with the session |
| `src/xblick/cli.py`, `tui.py`, `tui_nodes.py`, `tui_graph.py`, `rerun_bridge.py`, `recording.py`, `video.py`, `cam.py`, `v4l2.py`, `pick.py` | tools |
| `src/xblick/robot.py`, `sim.py`, `controller.py`, `poses.py`, `mps.py` | robot state, simulation, controller link, poses, MPS |
| `tests/`, `cpp/tests/` | Python and C++ tests |
| `tools/` | `check_references.sh` (+ `reference_allowlist.txt`), `install_zenoh.sh`, `gen_msgdefs.py`, `setup_python.sh`, `tui_screenshots.py`, `release_notes.sh` |
| `.github/workflows/ci.yml` | CI |
| `docs/` | `getting-started/`, `concepts/`, `guides/`, `reference/`, `integrations/`, `project/` |
| `.agents/skills/` | agent skills; `.claude/skills` links here |

## Defaults

- Router: `xblick router` or `zenohd` 1.10, one per PC, listening on `tcp/0.0.0.0:7447`. Sessions connect to
  `127.0.0.1:7447`. Start: `xblick router`, or `zenohd --no-multicast-scouting -l tcp/0.0.0.0:7447`.
- Files: cache `~/.cache/xblick/<node>/`, data `~/.local/share/xblick/<node>/`, config `~/.config/xblick/`;
  `XBLICK_HOME` moves cache and data (docs/reference/files.md).
- Stream name: `<namespace>/<camera>/<stream>`. Node results: `<camera>/<node>/<name>`. Diagnostics:
  `<camera>/diagnostics`.
- Transports: `auto`, `compressed`, `raw`, `shm`, `cuda`, `ros`. Nodes and the CLI use `auto`. In ROS 2 `auto` reads
  a stream that only a ROS node publishes from `/<base>/image_raw` (never our programs' ROS copies, `Graph.by_xblick`).
  A subscriber name with a leading `/` is a ROS topic of any name (camera.py `Subscription`).
- Image ring: 32 slots. Point cloud ring: 4 slots.
- Test without a camera: `xblick test-camera` publishes `test/camera/color` (640 x 480, 15 Hz).
  `xblick play <file.mcap> --loop` replays a recording.
- Inspect: `xblick list`, `xblick hz <stream>`, `xblick diagnostics <namespace>`, `xblick rerun`, `xblick`
  (terminal interface).
- Two programs without a router (tests): one with `--listen tcp/127.0.0.1:<port>`, the other with
  `--connect tcp/127.0.0.1:<port>`.

## Node repositories

Rules every node keeps (docs/guides/writing-nodes.md, "Rules every node keeps"): the name identifies the model
(`xblick-yolox`, not `xblick-detect`); `--camera` explicit and required, nothing guessed; fixed, documented defaults
only (nothing that depends on the machine, the installed packages or a measurement; convenience goes into the `xblick`
command); weights downloaded at the start through `xblick.weights` with `--download-only`, `--no-download` and
`--weights-dir`, the terms notice before a download; weights and built files (engines, kernels) in
`xblick.paths.cache_dir(<node>)` (`~/.cache/xblick/<node>/`), what a user made in `data_dir(<node>)`, never in the
checkout, `<node>` the short name (`yolox`), former places named in `Store(former=...)` (docs/reference/files.md); a
queryable replies and drops the query at once; drawn results only with `--overlay` (off by default, a start option):
`<camera>/<node>/overlay`, rgb8, the colour frame's size, stamp, frame id and camera_info, results published first,
drawn by `xblick.overlay.Overlay` only while subscribed; the session from `session.open_from_args(args)`, which
declares the program (console script name, its distribution's version, host, whole command line, pid) on the network,
and `node.Reporter` adds `<camera>/<node>` to it (since 0.5.9: no code of its own, a node gets it with its pin
raised). The command line is visible on the network: no secret as an argument.

Same pattern in each: a uv project pinned to a version of `xblick` (`pyproject.toml`, `uv.lock`), `tools/setup.sh`
for `.venv` and checkpoints, a console script `xblick-<name>`, outputs below `<camera>/<node>/`, diagnostics on
`<camera>/diagnostics`, its own `AGENTS.md`. A change here that nodes need: release it, then raise their pin and
run `uv lock`.
