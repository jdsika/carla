#[[

  Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
  de Barcelona (UAB).
  
  This work is licensed under the terms of the MIT license.
  For a copy, see <https://opensource.org/licenses/MIT>.

  OSI (Open Simulation Interface) two-phase build — Phase B consumption.

  Phase A: Build and install asam-osi-utilities + deps via vcpkg into a prefix.
    See scripts/build-osi-deps.sh (Linux/macOS) or scripts/build-osi-deps.bat (Windows).

  Phase B (this file): Consume the installed artifacts via find_package().

  Required cache variables (set via CMakePresets.json or command line):
    CARLA_OSI_PREFIX          — Install prefix from Phase A (e.g., Build/osi-deps)
    CARLA_OSI_VCPKG_INSTALLED — vcpkg_installed/<triplet> from Phase A build tree

]]

carla_message ("Configuring OSI support (two-phase build, Phase B)")

# --------------------------------------------------------------------------
# Validate required paths
# --------------------------------------------------------------------------

if (NOT CARLA_OSI_PREFIX)
  carla_error (
    "ENABLE_OSI=ON but CARLA_OSI_PREFIX is not set.\n"
    "Run Phase A first: scripts/build-osi-deps.sh (or .bat)\n"
    "Then set CARLA_OSI_PREFIX to the install prefix (e.g., Build/osi-deps)."
  )
endif ()

if (NOT IS_ABSOLUTE "${CARLA_OSI_PREFIX}")
  get_filename_component (CARLA_OSI_PREFIX "${CARLA_OSI_PREFIX}" ABSOLUTE BASE_DIR "${CMAKE_SOURCE_DIR}")
endif ()

if (NOT EXISTS "${CARLA_OSI_PREFIX}")
  carla_error (
    "CARLA_OSI_PREFIX='${CARLA_OSI_PREFIX}' does not exist.\n"
    "Run Phase A first to build and install OSI dependencies."
  )
endif ()

# --------------------------------------------------------------------------
# Build CMAKE_PREFIX_PATH for find_package
# --------------------------------------------------------------------------

# The OSI install prefix contains OSIUtilities and open_simulation_interface
# CMake config files.
list (PREPEND CMAKE_PREFIX_PATH "${CARLA_OSI_PREFIX}")

# The vcpkg_installed directory contains protobuf, lz4, zstd. If provided,
# add it to the prefix path so find_package(Protobuf CONFIG) works.
if (CARLA_OSI_VCPKG_INSTALLED)
  if (NOT IS_ABSOLUTE "${CARLA_OSI_VCPKG_INSTALLED}")
    get_filename_component (
      CARLA_OSI_VCPKG_INSTALLED "${CARLA_OSI_VCPKG_INSTALLED}"
      ABSOLUTE BASE_DIR "${CMAKE_SOURCE_DIR}"
    )
  endif ()
  list (PREPEND CMAKE_PREFIX_PATH "${CARLA_OSI_VCPKG_INSTALLED}")
  carla_message ("  vcpkg_installed: ${CARLA_OSI_VCPKG_INSTALLED}")
endif ()

carla_message ("  OSI prefix: ${CARLA_OSI_PREFIX}")
carla_message ("  CMAKE_PREFIX_PATH: ${CMAKE_PREFIX_PATH}")

# --------------------------------------------------------------------------
# Find packages — order matters (protobuf before OSIUtilities)
# --------------------------------------------------------------------------

# Locate protoc — may be in the vcpkg tools directory.
if (CARLA_OSI_VCPKG_INSTALLED)
  find_program (
    _CARLA_PROTOC protoc
    PATHS "${CARLA_OSI_VCPKG_INSTALLED}/tools/protobuf"
    NO_DEFAULT_PATH
  )
  if (_CARLA_PROTOC)
    set (Protobuf_PROTOC_EXECUTABLE "${_CARLA_PROTOC}" CACHE FILEPATH "Protobuf compiler" FORCE)
    carla_message ("  protoc: ${_CARLA_PROTOC}")
  endif ()
endif ()

# Protobuf must be found in CONFIG mode to resolve abseil transitive deps.
# The vcpkg-installed protobuf provides protobuf-config.cmake.
find_package (Protobuf CONFIG REQUIRED)
carla_message ("  Protobuf version: ${Protobuf_VERSION}")
carla_message ("  Protobuf libraries: ${Protobuf_LIBRARIES}")

# OSIUtilities bundles open_simulation_interface and the MCAP writer.
find_package (OSIUtilities REQUIRED)
carla_message ("  OSIUtilities found at: ${OSIUtilities_DIR}")

# --------------------------------------------------------------------------
# Suppress compiler warnings from OSI/protobuf generated code
# --------------------------------------------------------------------------

# Create an INTERFACE target that downstream OSI targets can link to silence
# warnings from generated protobuf headers.
if (NOT TARGET carla-osi-external-warnings)
  add_library (carla-osi-external-warnings INTERFACE)

  if (CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
    target_compile_options (carla-osi-external-warnings INTERFACE
      -Wno-unused-parameter
      -Wno-unused-variable
      -Wno-deprecated-declarations
      -Wno-sign-compare
    )
  elseif (MSVC)
    target_compile_options (carla-osi-external-warnings INTERFACE
      /wd4100  # unreferenced formal parameter
      /wd4267  # conversion from 'size_t'
      /wd4244  # conversion, possible loss of data
      /wd4996  # deprecated declarations
    )
  endif ()
endif ()
