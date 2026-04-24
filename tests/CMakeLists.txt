# Unit tests: no libcamera, no hardware. Links only the pieces that are
# pure-logic or accept abstract dependencies (FrameSink, free helpers).
set(UNIT_SOURCES
    unit/main.cpp
    unit/rate_controller_test.cpp
    unit/chunk_protocol_test.cpp
    unit/command_parser_test.cpp
    unit/frame_saver_helpers_test.cpp
    unit/frame_saver_modes_test.cpp
    unit/stream_chunking_test.cpp
    unit/checkerboard_detector_test.cpp
)

# Production sources included in the unit binary. StreamManager currently
# depends on CameraManager* (a forward-declared pointer on this path), so we
# pull in a small no-op libcamera shim to satisfy the linker without touching a
# real libcamera install. Simpler: link the camera sources too but mark the
# tests that would hit libcamera as hardware-only. We opt for the simpler path
# below and link libcamera into the unit binary as well. This is fine because
# the unit tests never actually instantiate Camera/CameraManager.
add_executable(unit_tests
    ${UNIT_SOURCES}
    ../stream_manager.cpp
    ../rate_controller.cpp
    ../frame_saver.cpp
    ../frame_saver_helpers.cpp
    ../command_parser.cpp
    ../checkerboard_detector.cpp
    ../camera.cpp
    ../camera_manager.cpp
)

target_include_directories(unit_tests PRIVATE
    ${CMAKE_SOURCE_DIR}
    ${json_SOURCE_DIR}/include
    ${LIBCAMERA_INCLUDE_DIRS}
)

target_link_libraries(unit_tests PRIVATE
    GTest::gtest
    GTest::gmock
    ${CMAKE_THREAD_LIBS_INIT}
    ${OpenCV_LIBS}
    ${LIBCAMERA_LIBRARIES}
)
target_link_directories(unit_tests PRIVATE ${LIBCAMERA_LIBRARY_DIRS})

target_compile_definitions(unit_tests PRIVATE
    CHERUPI_TEST_FIXTURES_DIR="${CMAKE_CURRENT_SOURCE_DIR}/fixtures"
    # BATCH mode uses O_DIRECT, which is unreliable on tmpfs. Give tests a
    # scratch dir under the build tree so they land on the host's on-disk fs.
    CHERUPI_TEST_SCRATCH_DIR="${CMAKE_BINARY_DIR}/test_scratch"
)

# Hardware tests: require a live IMX519.
add_executable(hw_tests
    hw/main.cpp
    hw/camera_discovery_test.cpp
    hw/camera_lifecycle_test.cpp
    hw/camera_frame_capture_test.cpp
    ../camera.cpp
    ../camera_manager.cpp
)

target_include_directories(hw_tests PRIVATE
    ${CMAKE_SOURCE_DIR}
    ${json_SOURCE_DIR}/include
    ${LIBCAMERA_INCLUDE_DIRS}
)

target_link_libraries(hw_tests PRIVATE
    GTest::gtest
    ${CMAKE_THREAD_LIBS_INIT}
    ${LIBCAMERA_LIBRARIES}
)
target_link_directories(hw_tests PRIVATE ${LIBCAMERA_LIBRARY_DIRS})

include(GoogleTest)
gtest_discover_tests(unit_tests
    PROPERTIES LABELS unit
)
gtest_discover_tests(hw_tests
    PROPERTIES LABELS hardware
    DISCOVERY_MODE PRE_TEST
)
