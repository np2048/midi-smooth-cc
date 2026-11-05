# midi-smooth-cc/CMakeLists.txt

# =============================================================================
# MIDI Smooth CC - CMake build system
#
# Builds the LV2 MIDI effect plugin `midi-smooth-cc.so` and its TTL bundle,
# plus optional Catch2 unit tests.
#
# =============================================================================
# THE FIX (spec §16):
#
#   set(MIDI_SMOOTH_CC_URI "urn:midi-smooth-cc:plugin")
#
# This variable MUST be defined BEFORE the configure_file() calls for
# manifest.ttl.in and plugin.ttl.in. Without it, CMake substitutes
# @MIDI_SMOOTH_CC_URI@ with an empty string, producing malformed Turtle
# where the plugin subject is <> (empty URI). Carla then shows an empty
# Name, empty Maker, and 0 ports because it cannot match the manifest's
# empty subject to the plugin.ttl's empty subject.
#
# The line appears below, marked with "# THE FIX".
# =============================================================================

cmake_minimum_required(VERSION 3.16)

project(midi_smooth_cc
    VERSION 1.0.0
    DESCRIPTION "Real-time MIDI CC smoothing LV2 plugin for breath controllers and EWI"
    LANGUAGES CXX
)

# --- C++17 standard (spec: "Programming Language: C++17") ---
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Static libraries that are linked into shared libraries need PIC.
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# =============================================================================
# THE FIX: Define the plugin URI BEFORE configure_file().
#
# This value is substituted into manifest.ttl.in and plugin.ttl.in wherever
# @MIDI_SMOOTH_CC_URI@ appears, producing <urn:midi-smooth-cc:plugin> in the
# generated .ttl files. It MUST match the URI returned by lv2_descriptor()
# in src/plugin.cpp (kMidiSmoothCC_URI).
# =============================================================================
set(MIDI_SMOOTH_CC_URI "urn:midi-smooth-cc:plugin")

# =============================================================================
# Find LV2 headers via pkg-config.
#
# On Arch / Manjaro / Debian / Fedora, the LV2 SDK is installed to the
# standard include path and pkg-config provides the include flags.
# The headers are expected at: lv2/lv2plug.in/ns/ext/atom/atom.h etc.
# =============================================================================
find_package(PkgConfig REQUIRED)
pkg_check_modules(LV2 REQUIRED lv2)

# =============================================================================
# Core library: smoothing + MIDI processing logic.
#
# Compiled as a STATIC library so it can be linked into BOTH the plugin
# shared library (midi-smooth-cc.so) and the test executable. PIC is
# enabled globally above so the objects are safe to link into a .so.
# =============================================================================
add_library(midi_smooth_cc_core STATIC
    src/smoothing.cpp
    src/midi_processor.cpp
)
target_include_directories(midi_smooth_cc_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${LV2_INCLUDE_DIRS}
)
target_compile_options(midi_smooth_cc_core PRIVATE ${LV2_CFLAGS_OTHER})

# =============================================================================
# Plugin shared library: midi-smooth-cc.so
#
# The output filename MUST be exactly "midi-smooth-cc.so" (no "lib" prefix)
# because lv2/manifest.ttl.in declares:
#     lv2:binary <midi-smooth-cc.so>
# =============================================================================
add_library(midi_smooth_cc SHARED
    src/plugin.cpp
)
target_link_libraries(midi_smooth_cc PRIVATE midi_smooth_cc_core)
target_include_directories(midi_smooth_cc PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${LV2_INCLUDE_DIRS}
)
target_compile_options(midi_smooth_cc PRIVATE ${LV2_CFLAGS_OTHER})

# Remove the "lib" prefix so the output is midi-smooth-cc.so, not
# libmidi-smooth-cc.so. This MUST match the lv2:binary path in manifest.ttl.
set_target_properties(midi_smooth_cc PROPERTIES
    PREFIX ""
    OUTPUT_NAME "midi-smooth-cc"
    SUFFIX ".so"
)

# =============================================================================
# LV2 bundle directory (in the build tree).
#
# The bundle is a directory named <plugin>.lv2 containing:
#   midi-smooth-cc.so   (the shared library)
#   manifest.ttl        (discovery metadata)
#   plugin.ttl          (port and parameter metadata)
# =============================================================================
set(LV2_BUNDLE_DIR "${CMAKE_CURRENT_BINARY_DIR}/midi-smooth-cc.lv2")
file(MAKE_DIRECTORY "${LV2_BUNDLE_DIR}")

# =============================================================================
# Configure TTL files from .in templates.
#
# configure_file() with @ONLY substitutes only @VAR@ references (not ${VAR}),
# which is what we want for Turtle files. @MIDI_SMOOTH_CC_URI@ is replaced
# with the value of MIDI_SMOOTH_CC_URI defined above.
#
# These run at CMake CONFIGURE time, so the .ttl files are present in the
# bundle directory before any build step.
# =============================================================================
configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/lv2/manifest.ttl.in"
    "${LV2_BUNDLE_DIR}/manifest.ttl"
    @ONLY
)

configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/lv2/plugin.ttl.in"
    "${LV2_BUNDLE_DIR}/plugin.ttl"
    @ONLY
)

# =============================================================================
# Copy the built .so into the bundle directory after each build, so the
# bundle in the build tree is immediately loadable by Carla for testing
# without requiring `make install`.
# =============================================================================
add_custom_command(TARGET midi_smooth_cc POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy
            "$<TARGET_FILE:midi_smooth_cc>"
            "${LV2_BUNDLE_DIR}/midi-smooth-cc.so"
    COMMENT "Copying midi-smooth-cc.so to LV2 bundle: ${LV2_BUNDLE_DIR}"
)

# =============================================================================
# Installation (spec §16: "The plugin must be installable into ~/.lv2/")
#
# Default install destination is ~/.lv2/midi-smooth-cc.lv2/ . Override with:
#   cmake -DLV2_INSTALL_DIR=/usr/lib/lv2 ..
# for system-wide installation.
# =============================================================================
if(NOT LV2_INSTALL_DIR)
    set(LV2_INSTALL_DIR "$ENV{HOME}/.lv2"
        CACHE PATH "LV2 plugin installation directory")
endif()

install(TARGETS midi_smooth_cc
    LIBRARY DESTINATION "${LV2_INSTALL_DIR}/midi-smooth-cc.lv2"
)

install(FILES
    "${LV2_BUNDLE_DIR}/manifest.ttl"
    "${LV2_BUNDLE_DIR}/plugin.ttl"
    DESTINATION "${LV2_INSTALL_DIR}/midi-smooth-cc.lv2"
)

# =============================================================================
# Unit tests (spec §15: 17 required test scenarios)
#
# Uses Catch2 v3. Tries find_package(Catch2) first (system-installed); if
# not found, falls back to FetchContent (downloads from GitHub).
# =============================================================================
option(MIDI_SMOOTH_CC_BUILD_TESTS "Build Catch2 unit tests" ON)

if(MIDI_SMOOTH_CC_BUILD_TESTS)
    # --- Locate Catch2 v3 ---
    find_package(Catch2 3 QUIET)

    if(NOT Catch2_FOUND)
        include(FetchContent)
        FetchContent_Declare(
            Catch2
            GIT_REPOSITORY https://github.com/catchorg/Catch2.git
            GIT_TAG        v3.4.0
        )
        FetchContent_MakeAvailable(Catch2)
    endif()

    # --- Test sources ---
    set(TEST_SOURCES
        tests/test_smoothing.cpp
        tests/test_midi.cpp
        tests/test_channels.cpp
    )

    # test_smoothing.cpp's header comment references tests/main.cpp. If it
    # exists, link against Catch2::Catch2 (no built-in main). If it does
    # NOT exist, link against Catch2::Catch2WithMain which provides main().
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/main.cpp")
        list(APPEND TEST_SOURCES tests/main.cpp)
        set(CATCH2_LINK_LIB Catch2::Catch2)
    else()
        set(CATCH2_LINK_LIB Catch2::Catch2WithMain)
    endif()

    add_executable(midi_smooth_cc_tests ${TEST_SOURCES})
    target_link_libraries(midi_smooth_cc_tests PRIVATE
        midi_smooth_cc_core
        ${CATCH2_LINK_LIB}
    )

    # Register tests with ctest.
    # include(Catch) MUST be called before catch_discover_tests — it loads
    # the Catch.cmake module that DEFINES the catch_discover_tests command.
    # Without it, CMake fails with: "Unknown CMake command catch_discover_tests".
    include(CTest)
    include(Catch)
    catch_discover_tests(midi_smooth_cc_tests)
endif()

# =============================================================================
# Configuration status summary (printed at the end of cmake configuration).
# =============================================================================
message(STATUS "")
message(STATUS "=== MIDI Smooth CC configuration ===")
message(STATUS "  Plugin URI:       ${MIDI_SMOOTH_CC_URI}")
message(STATUS "  LV2 bundle (build): ${LV2_BUNDLE_DIR}")
message(STATUS "  LV2 install dir:    ${LV2_INSTALL_DIR}/midi-smooth-cc.lv2")
message(STATUS "  Build tests:        ${MIDI_SMOOTH_CC_BUILD_TESTS}")
message(STATUS "====================================")
message(STATUS "")
