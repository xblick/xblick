#!/usr/bin/env bash
# Setup of a C++ build of this repository. First asks every question (build directory and type, camera API, point
# cloud node and its CUDA toolkit, Python environment, build and tests, missing system packages), then runs without
# asking: installs missing system packages (apt-get), downloads zenoh-c and zenoh-cpp into .zenoh/ of the checkout
# when not there (tools/install_zenoh.sh, as the CI), syncs .venv, configures, builds and tests. Prints the cmake
# command it ran, to repeat it by hand. Usage: tools/setup.sh
set -euo pipefail
cd "$(dirname "$0")/.."

if [[ ! -t 0 ]]; then
  echo "tools/setup.sh asks questions: run it in a terminal" >&2
  exit 1
fi

ask() {  # ask <question> <default>: prints the answer, the default on an empty line
  local answer
  read -r -p "$1 [$2]: " answer
  echo "${answer:-$2}"
}
yes_no() {  # yes_no <question> <y|n>: true on yes
  local answer
  while true; do
    answer=$(ask "$1 (y/n)" "$2")
    case "$answer" in
      y|Y|yes) return 0 ;;
      n|N|no) return 1 ;;
      *) echo "answer y or n" >&2 ;;
    esac
  done
}
fail() { echo "error: $*" >&2; exit 1; }

ZENOH_VERSION="${ZENOH_VERSION:-1.10.1}"
ZENOH_PREFIX="$PWD/.zenoh"
cmake_args=(-G Ninja)

# ================================================================================================ questions

build_dir=$(ask "build directory" build)
build_type=$(ask "build type (Release, RelWithDebInfo, Debug)" RelWithDebInfo)
cmake_args+=(-DCMAKE_BUILD_TYPE="$build_type")

build_camera=n
yes_no "build the C++ camera API (zenoh-c and zenoh-cpp ${ZENOH_VERSION} are downloaded into .zenoh/)?" y &&
  build_camera=y

toolkit=""
if [[ $build_camera == y ]]; then
  # default: the point cloud node with the newest toolkit found; none found and none named: without the node
  found=()
  for dir in "$HOME"/.local/opt/cuda/* /usr/local/cuda-* /usr/local/cuda; do
    [[ -x "$dir/bin/nvcc" ]] && found+=("$dir")
  done
  (( ${#found[@]} )) && echo "CUDA toolkits found: ${found[*]}"
  toolkit=$(ask "CUDA toolkit for the point cloud node xblick-points (the directory with bin/nvcc; empty: without it)" \
                "${found[-1]:-}")
  toolkit="${toolkit/#\~/$HOME}"
  toolkit="${toolkit%/}"
  [[ -z "$toolkit" || -x "$toolkit/bin/nvcc" ]] || fail "no nvcc in $toolkit/bin: name a CUDA toolkit directory"
  [[ -n "$toolkit" ]] || echo "no CUDA toolkit: building without the point cloud node"
fi

python_env=n
yes_no "set up the Python environment .venv (uv sync --all-extras: rerun, record, video, cam, relay and the test
tools; builds the C++ core into it)?" y && python_env=y
[[ -n "$toolkit" || $python_env == y ]] ||
  fail "without a CUDA toolkit, cuda.h comes from the Python environment (pip package nvidia-cuda-runtime): answer y"

build_now=n run_ctest=n run_interop=n
if yes_no "build after configuring?" y; then
  build_now=y
  yes_no "then run the C++ tests (ctest)?" y && run_ctest=y
  [[ $python_env == y && $build_camera == y ]] && yes_no "then run the C++ against Python tests?" y && run_interop=y
fi

missing_pkgs=()
command -v cmake > /dev/null || missing_pkgs+=(cmake)
command -v ninja > /dev/null || missing_pkgs+=(ninja-build)
if [[ $build_camera == y ]]; then
  command -v curl > /dev/null || missing_pkgs+=(curl)
  [[ -e /usr/include/jpeglib.h || -e /usr/include/x86_64-linux-gnu/jpeglib.h ]] || missing_pkgs+=(libjpeg-turbo8-dev)
  [[ -e /usr/include/png.h ]] || missing_pkgs+=(libpng-dev)
fi
if (( ${#missing_pkgs[@]} )); then
  echo "missing: ${missing_pkgs[*]}"
  yes_no "install them with 'sudo apt-get install ${missing_pkgs[*]}'?" y ||
    fail "install ${missing_pkgs[*]} (or the equivalent packages of your system), then run tools/setup.sh again"
fi
[[ $python_env == n ]] || command -v uv > /dev/null ||
  fail "uv not found: install it (docs.astral.sh/uv), then run tools/setup.sh again"

# ================================================================================================ work, no questions

if (( ${#missing_pkgs[@]} )); then
  sudo apt-get update -qq
  sudo apt-get install -y "${missing_pkgs[@]}"
fi

if [[ $build_camera == y ]]; then
  have_zenoh() { [[ -n "$(find "$ZENOH_PREFIX" -name zenohcConfig.cmake 2>/dev/null | head -1)" &&
                    -n "$(find "$ZENOH_PREFIX" -name zenohcxxConfig.cmake 2>/dev/null | head -1)" ]]; }
  if ! have_zenoh; then
    ZENOH_VERSION="$ZENOH_VERSION" tools/install_zenoh.sh "$ZENOH_PREFIX"
    have_zenoh || fail "tools/install_zenoh.sh ran, but .zenoh/ has no zenohcConfig.cmake and zenohcxxConfig.cmake"
  fi
  cmake_args+=(-DCMAKE_PREFIX_PATH="$ZENOH_PREFIX")
else
  cmake_args+=(-DXBLICK_CAMERA=OFF)
fi

[[ $python_env == n ]] || uv sync --all-extras

if [[ -n "$toolkit" ]]; then
  cmake_args+=(-DXBLICK_POINTS=ON -DCUDAToolkit_ROOT="$toolkit" -DCMAKE_CUDA_COMPILER="$toolkit/bin/nvcc")
else
  # only cuda.h is needed; without a toolkit it comes from the pip package nvidia-cuda-runtime in .venv
  cuda_include=$(uv run python -c "import nvidia, os; print(os.path.join(list(nvidia.__path__)[0], 'cu13', 'include'))")
  [[ -e "$cuda_include/cuda.h" ]] || fail "no cuda.h in $cuda_include (pip package nvidia-cuda-runtime)"
  cmake_args+=(-DXBLICK_CUDA_INCLUDE_DIR="$cuda_include")
fi

echo "cmake -S . -B $build_dir ${cmake_args[*]}"
cmake -S . -B "$build_dir" "${cmake_args[@]}"
[[ $build_now == n ]] || cmake --build "$build_dir"
[[ $run_ctest == n ]] || ctest --test-dir "$build_dir" --output-on-failure
[[ $run_interop == n ]] || XBLICK_CPP_BUILD="$build_dir" uv run pytest -q tests/test_cpp_interop.py
echo "done: $build_dir"
