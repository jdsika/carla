# Copilot Instructions for CARLA Simulator

## Overview

CARLA is an open-source autonomous driving simulator built on Unreal Engine 5.5. The primary development branch is `ue5-dev`. The codebase has three main layers: a C++ core library (LibCarla), Python bindings (PythonAPI), and an Unreal Engine project (Unreal/CarlaUnreal).

## Architecture

### LibCarla (`LibCarla/source/carla/`)

The core C++ library, built as two separate static libraries:

- **carla-server** — Linked into the UE5 plugin. Contains road network parsing (OpenDRIVE), geometry, RPC server, sensor serialization, streaming infrastructure, and multi-GPU support. Does _not_ include client, traffic manager, or navigation code.
- **carla-client** — Linked into the Python API. Contains everything in carla-server plus the client SDK (`carla/client/`), traffic manager (`carla/trafficmanager/`), navigation (`carla/nav/`), image processing (`carla/image/`), and sensor data deserialization (`carla/sensor/data/`).

Communication between client and server uses `rpclib` for commands and a custom Boost.Asio TCP streaming layer (`carla/streaming/`) for sensor data.

### Python API (`PythonAPI/`)

- `PythonAPI/carla/src/` — Boost.Python bindings (one `.cpp` per domain: Actor, Client, Sensor, World, etc.) that wrap LibCarla client classes.
- `PythonAPI/carla/agents/` — Pure-Python driving agents (navigation, tools) importable as `carla.agents`.
- `PythonAPI/examples/` — Standalone example scripts (manual_control, generate_traffic, etc.).
- `PythonAPI/test/smoke/` — Smoke tests that require a running CARLA server.

### Unreal Engine Project (`Unreal/CarlaUnreal/`)

The UE5 project with three plugins under `Plugins/`:

- **Carla** — The main plugin. Contains all gameplay C++ code organized by domain: `Actor/`, `Sensor/`, `Traffic/`, `Vehicle/`, `Walker/`, `Weather/`, `Server/`, `Recorder/`, etc.
- **CarlaTools** — Editor-only tooling and utilities.
- **CarlaExporter** — Mesh/map export functionality.

The CMake build generates `.def` files (Includes.def, Libraries.def, Options.def, Definitions.def) that the Unreal Build Tool consumes to link against LibCarla and its dependencies.

### ROS2 Native (`Ros2Native/`)

Optional ROS2 bridge using Fast-DDS. Enabled with `-DENABLE_ROS2=ON`.

## Build System

CARLA uses CMake (≥3.27.2) with the Ninja generator. On Linux, a custom toolchain (`CMake/Toolchain.cmake`) uses the Clang/libc++ bundled with Unreal Engine.

### First-time setup

```sh
# Linux (interactive — installs prerequisites, downloads UE5 + content, builds everything):
./CarlaSetup.sh --interactive

# Windows (from x64 Native Tools Command Prompt for VS 2022):
CarlaSetup.bat
```

### Subsequent builds

```sh
# Configure (Linux — add -DENABLE_ROS2=ON if needed):
cmake -G Ninja -S . -B Build --toolchain=$PWD/CMake/Toolchain.cmake -DCMAKE_BUILD_TYPE=Release

# Configure (Windows):
cmake -G Ninja -S . -B Build --toolchain=CMake/Toolchain.cmake -DCMAKE_BUILD_TYPE=Release

# Or use a preset:
cmake --preset Release          # also: Development, Debug

# Build all:
cmake --build Build

# Build specific targets:
cmake --build Build --target carla-client
cmake --build Build --target carla-server
cmake --build Build --target carla-python-api
cmake --build Build --target carla-python-api-install   # pip install -e
cmake --build Build --target launch                      # build + open UE editor
cmake --build Build --target launch-only                 # open UE editor without rebuild
cmake --build Build --target package                     # create shipping package
```

### Key CMake options (`CMake/Options.cmake`)

| Option | Default | Description |
|---|---|---|
| `BUILD_CARLA_CLIENT` | ON | Build client library |
| `BUILD_CARLA_SERVER` | ON | Build server library |
| `BUILD_PYTHON_API` | ON | Build Python wheel |
| `BUILD_LIBCARLA_TESTS` | ON | Build GTest suites |
| `BUILD_CARLA_UNREAL` | auto | Build UE project (auto-detected from `CARLA_UNREAL_ENGINE_PATH`) |
| `ENABLE_ROS2` | OFF | Enable ROS2 native bridge |
| `ENABLE_RSS` | OFF | Enable RSS (ad-rss-lib) |
| `ENABLE_RTTI` | ON | C++ RTTI |
| `ENABLE_EXCEPTIONS` | ON | C++ Exceptions |

### Environment variables

- `CARLA_UNREAL_ENGINE_PATH` — Must point to the CARLA fork of UE 5.5.

## Testing

### LibCarla C++ tests (Google Test)

```sh
# Build tests:
cmake --build Build --target libcarla_test_server
cmake --build Build --target libcarla_test_client

# Fetch test content (OpenDRIVE files):
git clone --depth 1 -b 0.1.4 https://github.com/carla-simulator/opendrive-test-files.git Build/test-content

# Run all tests:
./Build/LibCarla/libcarla_test_server
./Build/LibCarla/libcarla_test_client

# Run a single test (GTest filter):
./Build/LibCarla/libcarla_test_client --gtest_filter="*OpenDrive*"
```

### Python smoke tests

Require a running CARLA server:

```sh
python -m pytest PythonAPI/test/smoke/test_blueprint.py     # single test file
python -m pytest PythonAPI/test/smoke/                       # all smoke tests
```

## Coding Conventions

### C++

- **Standard:** C++20 (CMake enforced). C files use C11.
- **LibCarla style:** Variation of Google C++ Style Guide.
- **Unreal plugin style:** Unreal Engine Coding Standard, but with spaces instead of tabs.
- **Exceptions:** Use `carla::throw_exception` instead of `throw`. Server-side `try-catch` blocks must be guarded with `#ifndef LIBCARLA_NO_EXCEPTIONS`.
- **Line length:** Comments ≤80 columns; code may slightly exceed in rare cases for clarity.
- **Windows compatibility:** `LibCarla/source/util/winnt-macros.txt` lists Windows macros that are `#undef`'d via generated header guards when including UE headers. Add new conflicting macros there.

### Python

- PEP 8 with max line length 120 (configured in `.pep8`).
- Pylint with max line length 120 (configured in `PythonAPI/.pylintrc`).
- Comments ≤80 columns, code ≤120 columns.

### CMake

- The project defines wrapper macros in `CMake/Util.cmake`: `carla_add_library`, `carla_add_executable`, `carla_add_custom_target`, `carla_option`, `carla_string_option`. Use these instead of raw CMake commands for new targets/options so they appear in the auto-generated help.
- Dependencies are managed via `FetchContent` in `CMake/Dependencies.cmake`. Version pins live in `CMake/Options.cmake`.

### Git branching

- Main development branch: `ue5-dev`.
- Feature branches: `username/feature_name` off `ue5-dev`.
- PRs target `ue5-dev`, not `master`.

## Dependencies (fetched at build time)

Boost 1.84, Eigen 3.4, Google Test 1.14, libpng 1.6.40, zlib 1.3.1, PROJ 9.7, Recast Navigation, rpclib, Xerces-C 3.3, SQLite, LunaSVG. ROS2 mode adds Fast-DDS 2.11.2, Fast-CDR, foonathan_memory_vendor.
