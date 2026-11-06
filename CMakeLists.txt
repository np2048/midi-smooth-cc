# midi-smooth-cc/CMakeLists.txt

# =============================================================================
# MIDI Smooth CC - CMake build system
# =============================================================================

cmake_minimum_required(VERSION 3.16)

project(midi_smooth_cc
    VERSION 1.0.0
    DESCRIPTION "Real-time MIDI CC smoothing LV2 plugin for breath controllers and EWI"
    LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# =============================================================================
# Plugin URI — MUST be defined before configure_file().
# =============================================================================
set(MIDI_SMOOTH_CC_URI "urn:midi-smooth-cc:plugin")

# =============================================================================
# Find LV2 headers via pkg-config.
# =============================================================================
find_package(PkgConfig REQUIRED)
pkg_check_modules(LV2 REQUIRED lv2)

# =============================================================================
# Core library: smoothing + MIDI processing logic.
# =============================================================================
add_library(midi_smooth_cc_core STATIC
    src/smoothing.cpp
    src/midi_processor.cpp
)
target_include_directories(midi_smooth_cc_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${LV2_INCLUDE_DIRS}
)
target_compile_options(midi_smooth_cc_core PRIVATE ${LV2_CFLAGS_OTHER} -Wall -Wextra)

# =============================================================================
# Plugin shared library: midi-smooth-cc.so
# =============================================================================
add_library(midi_smooth_cc SHARED
    src/plugin.cpp
)
target_link_libraries(midi_smooth_cc PRIVATE midi_smooth_cc_core)
target_include_directories(midi_smooth_cc PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${LV2_INCLUDE_DIRS}
)
target_compile_options(midi_smooth_cc PRIVATE ${LV2_CFLAGS_OTHER} -Wall -Wextra)

set_target_properties(midi_smooth_cc PROPERTIES
    PREFIX ""
    OUTPUT_NAME "midi-smooth-cc"
    SUFFIX ".so"
)

# =============================================================================
# LV2 bundle directory (build tree).
# =============================================================================
set(LV2_BUNDLE_DIR "${CMAKE_CURRENT_BINARY_DIR}/midi-smooth-cc.lv2")
file(MAKE_DIRECTORY "${LV2_BUNDLE_DIR}")

# =============================================================================
# Configure TTL files from .in templates.
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
# Copy .so to bundle directory after build.
# =============================================================================
add_custom_command(TARGET midi_smooth_cc POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy
            "$<TARGET_FILE:midi_smooth_cc>"
            "${LV2_BUNDLE_DIR}/midi-smooth-cc.so"
    COMMENT "Copying midi-smooth-cc.so to LV2 bundle: ${LV2_BUNDLE_DIR}"
)

# =============================================================================
# Installation (spec §16: "installable into ~/.lv2/")
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
# Unit tests (spec §15)
# =============================================================================
option(MIDI_SMOOTH_CC_BUILD_TESTS "Build Catch2 unit tests" ON)

if(MIDI_SMOOTH_CC_BUILD_TESTS)
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
    # Tests that touch lv2_descriptor()/MidiSmoothCCPlugin need src/plugin.cpp
    # compiled into the test executable (added below).
    #   test_plugin_integration.cpp: full plugin lifecycle.
    #   test_carla_replication.cpp:  replicates Carla's runtime environment.
    #   test_host_contract.cpp:      output atom type, host-announced capacity,
    #                                connect_port(nullptr).
    set(TEST_SOURCES
        tests/test_smoothing.cpp
        tests/test_midi.cpp
        tests/test_channels.cpp
        tests/test_plugin_integration.cpp
        tests/test_carla_replication.cpp
        tests/test_host_contract.cpp
    )

    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/main.cpp")
        list(APPEND TEST_SOURCES tests/main.cpp)
        set(CATCH2_LINK_LIB Catch2::Catch2)
    else()
        set(CATCH2_LINK_LIB Catch2::Catch2WithMain)
    endif()

    add_executable(midi_smooth_cc_tests
        ${TEST_SOURCES}
        src/plugin.cpp
    )
    target_link_libraries(midi_smooth_cc_tests PRIVATE
        midi_smooth_cc_core
        ${CATCH2_LINK_LIB}
    )

    include(CTest)
    include(Catch)
    catch_discover_tests(midi_smooth_cc_tests)
endif()

# =============================================================================
# Configuration status summary.
# =============================================================================
message(STATUS "")
message(STATUS "=== MIDI Smooth CC configuration ===")
message(STATUS "  Plugin URI:         ${MIDI_SMOOTH_CC_URI}")
message(STATUS "  LV2 bundle (build): ${LV2_BUNDLE_DIR}")
message(STATUS "  LV2 install dir:    ${LV2_INSTALL_DIR}/midi-smooth-cc.lv2")
message(STATUS "  Build tests:        ${MIDI_SMOOTH_CC_BUILD_TESTS}")
message(STATUS "====================================")
message(STATUS "")
