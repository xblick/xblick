---
name: xblick-release-version
description: >
  Use this skill when the user wants to release a version of Durchblick's core package `xblick` (a patch such as
  0.5.8, or a new minor version), also when they only say "ship it", "put it on PyPI", "tag the release", "make a
  release candidate", "is this ready to push", "bump the version" or "write the release notes". It runs the gate
  before a push (GPU suite with the C++ build, ctest, a CI-like pass, manual tries), sets the version in every place,
  writes the CHANGELOG entry that becomes the release notes, and walks through push, green CI, GitHub release, the
  PyPI upload and the follow-up in node repositories. Not for writing a node (use xblick-build-node), not for
  running or measuring nodes (xblick-run-nodes, xblick-benchmark-nodes), not for the terminal interface
  (xblick-develop-tui).
compatibility: Durchblick 0.5.x, uv, git, the GitHub CLI gh for the release, a GPU and the C++ toolchain for the gate
---

# Release a version

Done means: the version's commit is on `main`, CI is green on it, the GitHub release `v<version>` exists on that
commit with the CHANGELOG entry as its text, the publish workflow uploaded the wheels and the sdist to PyPI, and the
issues and node repositories that the release concerns are updated.

Background: [Development](../../../docs/project/development.md) (build, CI, releases, the release-notes rule),
`AGENTS.md` (the rules every change keeps).

## Progress

- [ ] 1. The kind of version decided
- [ ] 2. The version set in every place
- [ ] 3. The CHANGELOG entry written
- [ ] 4. Gate: GPU suite with the C++ build, and ctest
- [ ] 5. Gate: CI-like pass, also on Python 3.10
- [ ] 6. Gate: the changed commands tried by hand
- [ ] 7. Pushed, CI green
- [ ] 8. GitHub release created, publish workflow green
- [ ] 9. PyPI checked, issues and node repositories updated

Steps 7 to 9 act on the public repository and on PyPI. Do them only on the maintainer's word for this release.

## 1. Decide the kind of version

- **Patch** (0.5.x): nothing on the wire changes and no API breaks. A changed default or a new command is a patch,
  named under `### Changed`. An addition that older programs ignore needs the maintainer's word.
- **Minor** (0.x.0): a change of a message's fields, of the ring layout or of a name on the wire or on disk. It needs
  the maintainer's approval, and `AGENTS.md` says what it raises (`protocol.hpp`, `kVersion`).

## 2. Set the version

The version lives only in `CMakeLists.txt` (`project(xblick VERSION x.y.z)`). Python reads it from there
(`pyproject.toml` has `dynamic = ["version"]`), and the tag must equal it: `ci.yml` and `publish.yml` stop a tag
that names another version. Then update the places that print it:

```bash
grep -rn "<old version>" --exclude-dir=.git --exclude-dir=.venv --exclude-dir=build --exclude=CHANGELOG.md \
     --exclude=uv.lock .
```

In 0.5.x these are the status line in `README.md` ("Version x.y.z.") and the example output of `xblick list` in
`docs/getting-started/first-stream.md`.

## 3. Write the CHANGELOG entry

Follow the rule in [Release notes](../../../docs/project/development.md#release-notes): a heading `## x.y.z` above
the previous version, one summary sentence, `### Added` / `### Changed` / `### Fixed` (`### Known limit` only when
needed), one-line bullets from the user's view, a safety fix first and in bold, no test internals, one line on
compatibility, about 10 to 20 lines. Then check the text that the release will carry:

```bash
tools/release_notes.sh x.y.z
```

Commit the version, the status lines and the entry in a commit of their own.

## 4. Gate: GPU suite with the C++ build

Build the C++ library as in [Development](../../../docs/project/development.md#build-the-c-library), then run the
full suite on a free GPU with the build connected, in the background with a log:

```bash
CUDA_VISIBLE_DEVICES=<free gpu> XBLICK_CPP_BUILD=<build dir> timeout 1800 uv run pytest -q -rs tests > gpu.log 2>&1
ctest --test-dir <build dir> --output-on-failure
```

Read the skip reasons (`-rs`). A block of skips "XBLICK_CPP_BUILD (the C++ build directory) is not set" means the
build was not connected: fix that and run again. Expected skips are the extras that are not installed.

## 5. Gate: a CI-like pass

Two release candidates once passed locally and failed on GitHub, so repeat what `ci.yml` does: an environment from
the lock file only, no GPU, few cores, the same command. Use an environment directory of its own, because
`uv sync --locked` removes the extras from the working environment.

```bash
UV_PROJECT_ENVIRONMENT=<dir>/ci-venv uv sync --locked
UV_PROJECT_ENVIRONMENT=<dir>/ci-venv CUDA_VISIBLE_DEVICES= taskset -c 0-3 timeout 1500 uv run --locked pytest -q tests
UV_PROJECT_ENVIRONMENT=<dir>/ci-310 uv sync --locked --python 3.10      # the oldest supported Python
UV_PROJECT_ENVIRONMENT=<dir>/ci-310 CUDA_VISIBLE_DEVICES= taskset -c 0-3 timeout 1500 uv run --locked pytest -q tests
```

Also `tools/check_references.sh` (exit 0) and `ruff check --line-length 120` on the files you touched.

## 6. Gate: try the changed commands by hand

Start what changed as separate processes, as a user does: a router (`xblick router`), a camera
(`xblick test-camera`) and the tool that changed. A fault in `xblick router` once passed the whole suite and showed
at its first manual start. Stop each process by its PID.

## 7. Push and wait for CI

Push the new commits to `main` with your identity. Wait until every job of `ci.yml` is green on that commit
(`gh run list --branch main`, `gh run watch <id>`). Never release on a red CI. A version whose CI failed is not
released: fix forward with the next patch number (0.5.6 was never released).

## 8. Create the GitHub release

```bash
tools/release_notes.sh x.y.z > notes.md
gh release create vx.y.z --target <full commit id> --title vx.y.z --notes-file notes.md
```

The tag starts `publish.yml`: it checks that the tag names the version, builds the wheels for CPython 3.10 to 3.13
(manylinux_2_28) and the sdist with `tools/build_wheels.sh`, and uploads them to PyPI through trusted publishing
(environment `pypi`, no token). Watch it until it is green (`gh run list --workflow publish.yml`).

## 9. After the upload

- Check that PyPI lists `xblick` x.y.z with four wheels and the sdist (pypi.org/project/xblick/x.y.z/).
- Update or close the issues that the release addresses, with the version in the comment.
- Node repositories pin `xblick` (`pyproject.toml`, `uv.lock`). For a node that needs the release: raise its pin,
  run `uv lock`, run its tests, push.

## Report

```
Version x.y.z (patch | minor), commit <id>
Gate: GPU suite <passed/failed/skipped> (skips: <reasons>), ctest <n/n>; CI-like 3.12 <numbers>, 3.10 <numbers>;
      manual tries: <what was started>
Release: CI <green | link>, release v<x.y.z> <link>, publish <green>, PyPI <files seen>
Follow-up: issues <numbers>, node repositories <names and pins>
Open: <anything not done, and why>
```

## Gotchas

- **The extra `xblick[relay]` must resolve.** `uv lock` resolves every extra, so a new minor version of
  `xblick-relay` is released first, then `uv lock` runs here, then the core is released.
- **A known stall (issue #6).** About one full pass in six stalls inside Zenoh in the pytest process from the first
  test of `tests/test_tui.py` on: a cascade of failures and "did not close within 5 s". Keep the log and repeat the
  pass once. Any other failure is a real one.
- **PyPI and the shields lag.** PyPI's "latest" pointer and the version shields of the README can show the old
  version for a while after the upload; the shields up to six hours, through two caches. Check the version's own
  PyPI page instead.
- **A message built with `$(printf ...)` once lost its empty second line.** Write the commit message to a file and
  use `git commit -F`.
- **Never amend, rebase or reset what is pushed, and never force-push.** The branch equals the public `main`.
- **A test must not depend on default ports or on the network.** It takes free ports and does not wait on an address
  that does not answer, because CI drops such packets and a close then takes more than 5 s.

## When this skill was wrong

If a step failed or misled, propose a fix of this skill to the user after the task.
