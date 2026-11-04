# midi-smooth-cc/CMakeLists.txt

# =============================================================================
# MIDI Smooth CC - LV2 MIDI Processing Plugin
# Master Software Development Specification v1.0
# CMake build system for Linux x86_64
# =============================================================================
cmake_minimum_required(VERSION 3.16)

project(midi_smooth_cc
    VERSION 1.0.0
    DESCRIPTION "Real-time LV2 MIDI CC smoothing plugin for breath controllers and EWI"
    LANGUAGES C CXX
)

# -----------------------------------------------------------------------------
# C++17 standard, no extensions, no fallback
# -----------------------------------------------------------------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Default to Release unless user overrides
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
        Debug Release RelWithDebInfo MinSizeRel)
endif()

# Shared library: PIC, no "lib" prefix, .so suffix
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# -----------------------------------------------------------------------------
# Build options
# -----------------------------------------------------------------------------
option(MIDI_SMOOTH_CC_BUILD_TESTS "Build Catch2 unit tests" ON)
option(MIDI_SMOOTH_CC_INSTALL_BUNDLE "Install the LV2 bundle on 'make install'" ON)

# Install destination defaults to the user-local LV2 path scanned by Carla.
# Override with -DLV2_INSTALL_DIR=/some/path if needed.
if(NOT LV2_INSTALL_DIR)
    if(DEFINED ENV{HOME})
        set(LV2_INSTALL_DIR "$ENV{HOME}/.lv2" CACHE PATH "LV2 user bundle install directory")
    else()
        set(LV2_INSTALL_DIR "${CMAKE_INSTALL_PREFIX}/lib/lv2" CACHE PATH "LV2 bundle install directory")
    endif()
endif()
message(STATUS "MIDI Smooth CC: LV2 install directory = ${LV2_INSTALL_DIR}")

# -----------------------------------------------------------------------------
# Locate LV2 headers (system package on Arch/Manjaro: pacman -S lv2)
# -----------------------------------------------------------------------------
find_path(LV2_INCLUDE_DIR
    NAMES lv2/lv2plug.in/ns/lv2core/lv2.h
    PATHS
        /usr/include
        /usr/local/include
        /opt/local/include
    DOC "Path to the LV2 C headers (provides lv2/lv2plug.in/...)"
)
if(NOT LV2_INCLUDE_DIR)
    message(FATAL_ERROR
        "LV2 headers not found. Install the 'lv2' package on your distribution:\n"
        "  Arch / Manjaro : sudo pacman -S lv2\n"
        "  Debian / Ubuntu: sudo apt install lv2-dev\n"
        "  Fedora         : sudo dnf install lv2-devel")
endif()
message(STATUS "MIDI Smooth CC: LV2 headers found at ${LV2_INCLUDE_DIR}")

# -----------------------------------------------------------------------------
# Plugin shared library target
#
# Output name must match the binary referenced in manifest.ttl.in
# (lv2:binary <midi-smooth-cc.so>)
# -----------------------------------------------------------------------------
add_library(midi_smooth_cc MODULE
    src/plugin.cpp
    src/midi_processor.cpp
    src/smoothing.cpp
)

# LV2 shared libraries must have no "lib" prefix and use the .so extension.
set_target_properties(midi_smooth_cc PROPERTIES
    PREFIX ""
    OUTPUT_NAME "midi-smooth-cc"
    SUFFIX ".so"
    CXX_VISIBILITY_PRESET hidden
    VISIBILITY_INLINES_HIDDEN ON
)

# Output the .so directly into the build-tree bundle directory so the bundle
# is self-contained and testable in place.
set(LV2_BUNDLE_NAME "midi-smooth-cc.lv2")
set(LV2_BUNDLE_BUILD_DIR "${CMAKE_BINARY_DIR}/${LV2_BUNDLE_NAME}")
set_target_properties(midi_smooth_cc PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${LV2_BUNDLE_BUILD_DIR}"
)

target_include_directories(midi_smooth_cc PRIVATE
    "${CMAKE_SOURCE_DIR}/src"
    "${LV2_INCLUDE_DIR}"
)

target_compile_features(midi_smooth_cc PRIVATE cxx_std_17)

target_compile_options(midi_smooth_cc PRIVATE
    -Wall
    -Wextra
    -Wpedantic
    -Wno-unused-parameter   # LV2 callbacks have fixed signatures
    -fvisibility=hidden
    -fvisibility-inlines-hidden
)

# Embed the plugin version string for runtime introspection.
target_compile_definitions(midi_smooth_cc PRIVATE
    MIDI_SMOOTH_CC_VERSION="${PROJECT_VERSION}"
)

# -----------------------------------------------------------------------------
# Configure TTL bundle files
#
# manifest.ttl  - declares the plugin URI, binary, and points to plugin.ttl
# plugin.ttl    - declares ports, properties, and required LV2 features
# -----------------------------------------------------------------------------
configure_file(
    "${CMAKE_SOURCE_DIR}/lv2/manifest.ttl.in"
    "${LV2_BUNDLE_BUILD_DIR}/manifest.ttl"
    @ONLY
)
configure_file(
    "${CMAKE_SOURCE_DIR}/lv2/plugin.ttl.in"
    "${LV2_BUNDLE_BUILD_DIR}/plugin.ttl"
    @ONLY
)

# -----------------------------------------------------------------------------
# Unit tests (Catch2 v3 via FetchContent)
# -----------------------------------------------------------------------------
if(MIDI_SMOOTH_CC_BUILD_TESTS)
    include(FetchContent)
    FetchContent_Declare(
        Catch2
        GIT_REPOSITORY https://github.com/catchorg/Catch2.git
        GIT_TAG        v3.5.2
        GIT_SHALLOW    TRUE
    )
    FetchContent_MakeAvailable(Catch2)

    # Helper: link a test against plugin sources + Catch2, with the same LV2
    # include path as the plugin itself. The plugin MODULE library is not
    # linkable into executables, so tests compile the plugin sources directly.
    function(midi_smooth_cc_add_test target_name test_source)
        add_executable(${target_name}
            ${test_source}
            src/smoothing.cpp
            src/midi_processor.cpp
        )
        target_include_directories(${target_name} PRIVATE
            "${CMAKE_SOURCE_DIR}/src"
            "${LV2_INCLUDE_DIR}"
        )
        target_compile_features(${target_name} PRIVATE cxx_std_17)
        target_compile_options(${target_name} PRIVATE -Wall -Wextra -Wpedantic)
        target_link_libraries(${target_name} PRIVATE Catch2::Catch2WithMain)
    endfunction()

    midi_smooth_cc_add_test(test_smoothing tests/test_smoothing.cpp)
    midi_smooth_cc_add_test(test_midi      tests/test_midi.cpp)
    midi_smooth_cc_add_test(test_channels  tests/test_channels.cpp)

    include(CTest)
    include(Catch)
    catch_discover_tests(test_smoothing)
    catch_discover_tests(test_midi)
    catch_discover_tests(test_channels)
endif()

# -----------------------------------------------------------------------------
# Install: LV2 bundle layout
#
#   ~/.lv2/midi-smooth-cc.lv2/
#       midi-smooth-cc.so
#       manifest.ttl
#       plugin.ttl
# -----------------------------------------------------------------------------
if(MIDI_SMOOTH_CC_INSTALL_BUNDLE)
    set(LV2_BUNDLE_INSTALL_DIR "${LV2_INSTALL_DIR}/${LV2_BUNDLE_NAME}")

    install(TARGETS midi_smooth_cc
        LIBRARY DESTINATION "${LV2_BUNDLE_INSTALL_DIR}"
    )

    install(FILES
        "${LV2_BUNDLE_BUILD_DIR}/manifest.ttl"
        "${LV2_BUNDLE_BUILD_DIR}/plugin.ttl"
        DESTINATION "${LV2_BUNDLE_INSTALL_DIR}"
    )

    message(STATUS "MIDI Smooth CC: install destination = ${LV2_BUNDLE_INSTALL_DIR}")
    message(STATUS "MIDI Smooth CC: run 'make install' then rescan plugins in Carla")
endif()

# -----------------------------------------------------------------------------
# Summary
# -----------------------------------------------------------------------------
message(STATUS "==========================================================")
message(STATUS "  MIDI Smooth CC  v${PROJECT_VERSION}")
message(STATUS "    C++ standard      : C++${CMAKE_CXX_STANDARD}")
message(STATUS "    Build type        : ${CMAKE_BUILD_TYPE}")
message(STATUS "    LV2 headers       : ${LV2_INCLUDE_DIR}")
message(STATUS "    Tests             : ${MIDI_SMOOTH_CC_BUILD_TESTS}")
message(STATUS "    Bundle (build)    : ${LV2_BUNDLE_BUILD_DIR}")
if(MIDI_SMOOTH_CC_INSTALL_BUNDLE)
    message(STATUS "    Bundle (install)  : ${LV2_BUNDLE_INSTALL_DIR}")
endif()
message(STATUS "==========================================================")
