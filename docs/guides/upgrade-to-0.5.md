# Upgrade from 0.4 to 0.5

This page lists what a program or a node of Durchblick 0.4 changes to run with 0.5. Version 0.5.0 renamed every name
in code, on the wire and on disk to `xblick`. Only the prose name of the project stays Durchblick.

## Every program of a setup runs 0.5

Programs of 0.5 do not talk to programs of 0.3 or 0.4. Their message types, info records, ring names and keys
differ. A reader of 0.5 refuses an info record of 0.4, whose version is 4 now. Only samples of ROS types, for example
`image_raw`, `image_raw/compressed` and `camera_info`, keep their bytes. Update every program and node of a setup at
once.

The versions of the message types (`@1`) and of the rings (`kVersion`) stay, because their fields and layouts did not
change. The ring magics `DBLKRING` and `DBLKCUDA` stay as well.

Recordings of 0.4 still play. Their channels lack the info record `xblick_info`, so `xblick play` serves their
streams as plain topics, as it does for other MCAP files.

The repository is `github.com/xblick/xblick`. Node repositories are `xblick-<name>`, and the relay is
`xblick-relay`.

## Names

| What | Old (0.4) | New (0.5) |
| --- | --- | --- |
| Python package, extension | `import blick`, `blick._core` | `import xblick`, `xblick._core` |
| Distribution, extras | `xblick`, `xblick[relay]` with `blick-relay` 0.3 | unchanged, `xblick[relay]` with `xblick-relay` 0.4 |
| Command | `blick` (`blick list`, `blick relay`, `blick mps`, ...) | `xblick` (`xblick list`, `xblick relay`, ...) |
| Logger | `blick` | `xblick` |
| Terminal interface class | `tui.DurchblickApp` | `tui.XblickApp` |
| Relay package and program | `blick_relay`, `durchblick-relay` | `xblick_relay`, `xblick-relay` |
| C++ namespace | `durchblick::` | `xblick::` |
| C++ headers | `#include <durchblick/camera.hpp>`, `<durchblick/version.hpp>` | `#include <xblick/camera.hpp>`, `<xblick/version.hpp>` |
| Version macros | `DURCHBLICK_VERSION`, `DURCHBLICK_VERSION_MAJOR/MINOR/PATCH` | `XBLICK_VERSION`, `XBLICK_VERSION_MAJOR/MINOR/PATCH` |
| CMake package | `find_package(durchblick 0.4)`, `lib/cmake/durchblick` | `find_package(xblick 0.5)`, `lib/cmake/xblick` |
| CMake targets | `durchblick::core`, `durchblick::durchblick`, `durchblick::points` | `xblick::core`, `xblick::xblick`, `xblick::points` |
| CMake options and variables | `DURCHBLICK_CAMERA`, `DURCHBLICK_POINTS`, `DURCHBLICK_CUDA_INCLUDE_DIR`, `durchblick_points_FOUND` | `XBLICK_CAMERA`, `XBLICK_POINTS`, `XBLICK_CUDA_INCLUDE_DIR`, `xblick_points_FOUND` |
| Libraries | `libdurchblick_core.a`, `libdurchblick.a`, `libdurchblick_points.a` | `libxblick_core.a`, `libxblick.a`, `libxblick_points.a` |
| Programs | `durchblick-points`, `durchblick-cpp-camera`, `durchblick-cpp-probe` | `xblick-points`, `xblick-cpp-camera`, `xblick-cpp-probe` |
| Test variables | `DURCHBLICK_CPP_BUILD`, `DURCHBLICK_TUI_SCREENS` | `XBLICK_CPP_BUILD`, `XBLICK_TUI_SCREENS` |
| Message types | `durchblick_msgs/msg/...`, `durchblick_msgs/srv/...` | `xblick_msgs/msg/...`, `xblick_msgs/srv/...` |
| Info record (`<base>/info`, detections' info) | `{"v":3, ..., "durchblick":"<version>"}` | `{"v":4, ..., "xblick":"<version>"}` |
| Shared-memory rings | `/dev/shm/durchblick-<16 hex>` | `/dev/shm/xblick-<16 hex>` |
| CUDA ring handshake socket | `\0durchblick-<16 hex>` | `\0xblick-<16 hex>` |
| Record and play node | `durchblick/<name>/control`, `durchblick/<name>/status`, parameters of `durchblick/<name>` | `xblick/<name>/control`, `xblick/<name>/status`, parameters of `xblick/<name>` |
| MPS daemon | token `durchblick/mps/<name>`, `.../status` `{"v":1, ..., "durchblick":...}`, line `durchblick mps: ready` | token `xblick/mps/<name>`, `.../status` `{"v":2, ..., "xblick":...}`, line `xblick mps: ready` |
| MCAP files | library `durchblick <version>`, channel metadata `durchblick_info`, record `durchblick_recording` | library `xblick <version>`, `xblick_info`, `xblick_recording` |
| Rerun application id | `durchblick` | `xblick` |
| Presets of the terminal interface | `$XDG_CONFIG_HOME/durchblick/rerun_presets.json` | `$XDG_CONFIG_HOME/xblick/rerun_presets.json` |
| Test pattern's JPEG comment | `durchblick-test-jpeg <seq>` | `xblick-test-jpeg <seq>` |
| Agent skills | `durchblick-build-node`, `durchblick-run-nodes`, `durchblick-benchmark-nodes`, `durchblick-develop-tui` | `xblick-build-node`, `xblick-run-nodes`, `xblick-benchmark-nodes`, `xblick-develop-tui` |

## The test camera's stream

`xblick test-camera` publishes `test/camera/color`. Before 0.5.1 it published `robot/test_camera/color`.
`xblick-cpp-camera` and `xblick-cpp-probe` default to `test/camera/color` and `test/camera/points`. A script that
subscribes to the old name needs the new one, or start the test camera with `--namespace robot --camera
test_camera`.
