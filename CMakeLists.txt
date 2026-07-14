cmake_minimum_required(VERSION 3.14)
project(camera_websocket_server)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# When ON, only tools/yuv420_convert.cpp is built. That tool has no libcamera
# or RPi dependency, so this option lets it be built on any machine, skipping
# the server, its uWebSockets/libcamera dependencies, and the test suites.
option(CHERUPI_TOOLS_ONLY "Build only the standalone tools (yuv420_convert), skipping the RPi-specific server and its libcamera/uWebSockets dependencies" OFF)

# FetchContent for downloading dependencies
include(FetchContent)

# Find packages
find_package(Threads REQUIRED)
find_package(OpenCV REQUIRED)

if(NOT CHERUPI_TOOLS_ONLY)
    find_package(ZLIB REQUIRED)
    find_package(OpenSSL REQUIRED)
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(LIBCAMERA REQUIRED libcamera)

    # Set FetchContent properties
    set(FETCHCONTENT_QUIET OFF)
    set(JSON_BuildTests OFF CACHE INTERNAL "")
    set(JSON_Install OFF CACHE INTERNAL "")

    # Fetch nlohmann/json
    FetchContent_Declare(
        json
        GIT_REPOSITORY https://github.com/nlohmann/json.git
        GIT_TAG v3.11.3
        GIT_SHALLOW TRUE
    )

    # Fetch uSockets (dependency of uWebSockets)
    FetchContent_Declare(
        usockets
        GIT_REPOSITORY https://github.com/uNetworking/uSockets.git
        GIT_TAG v0.8.8
        GIT_SHALLOW TRUE
    )

    # Fetch uWebSockets
    FetchContent_Declare(
        uwebsockets
        GIT_REPOSITORY https://github.com/uNetworking/uWebSockets.git
        GIT_TAG v20.62.0
        GIT_SHALLOW TRUE
    )

    # Fetch GoogleTest for unit/integration tests (opt-in).
    option(CHERUPI_BUILD_TESTS "Build unit and hardware test suites" ON)
    option(CHERUPI_BUILD_E2E_TESTS "Register the Python end-to-end pytest suite with ctest (requires hardware)" OFF)
    if(CHERUPI_BUILD_TESTS)
        FetchContent_Declare(
            googletest
            GIT_REPOSITORY https://github.com/google/googletest.git
            GIT_TAG v1.14.0
            GIT_SHALLOW TRUE
        )
        set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    endif()

    # Make dependencies available
    FetchContent_MakeAvailable(json usockets uwebsockets)
    if(CHERUPI_BUILD_TESTS)
        FetchContent_MakeAvailable(googletest)
    endif()

    # Build uSockets as a static library
    add_library(uSockets STATIC
        ${usockets_SOURCE_DIR}/src/bsd.c
        ${usockets_SOURCE_DIR}/src/context.c
        ${usockets_SOURCE_DIR}/src/loop.c
        ${usockets_SOURCE_DIR}/src/socket.c
        ${usockets_SOURCE_DIR}/src/eventing/epoll_kqueue.c
        ${usockets_SOURCE_DIR}/src/crypto/openssl.c
        ${usockets_SOURCE_DIR}/src/crypto/sni_tree.cpp
    )

    target_include_directories(uSockets PUBLIC
        ${usockets_SOURCE_DIR}/src
    )

    target_link_libraries(uSockets
        OpenSSL::SSL
        OpenSSL::Crypto
    )

    # Enable SSL support
    target_compile_definitions(uSockets PRIVATE LIBUS_USE_OPENSSL)

    # Set C standard for uSockets C files
    set_property(TARGET uSockets PROPERTY C_STANDARD 11)

    # Include directories
    include_directories(
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${usockets_SOURCE_DIR}/src
        ${OpenCV_INCLUDE_DIRS}
    )

    # Create a proper include structure for uWebSockets
    # This allows us to use <uWS/App.h> instead of <App.h>
    set(UWS_INCLUDE_DIR ${CMAKE_CURRENT_BINARY_DIR}/uws_include)
    file(MAKE_DIRECTORY ${UWS_INCLUDE_DIR}/uWS)

    # Copy uWebSockets headers to the proper structure
    file(GLOB UWS_HEADERS ${uwebsockets_SOURCE_DIR}/src/*.h)
    foreach(HEADER ${UWS_HEADERS})
        get_filename_component(HEADER_NAME ${HEADER} NAME)
        configure_file(${HEADER} ${UWS_INCLUDE_DIR}/uWS/${HEADER_NAME} COPYONLY)
    endforeach()

    # Add the structured include directory
    include_directories(${UWS_INCLUDE_DIR})

    # Source files
    set(SOURCES
        src/main.cpp
        src/websocket_server.cpp
        src/camera.cpp
        src/camera_manager.cpp
        src/frame_saver.cpp
        src/frame_saver_helpers.cpp
        src/stream_manager.cpp
        src/rate_controller.cpp
        src/command_parser.cpp
        src/checkerboard_detector.cpp
    )

    # Header files
    set(HEADERS
        src/types.hpp
        src/websocket_server.hpp
        src/camera.hpp
        src/camera_manager.hpp
        src/frame_saver.hpp
        src/frame_saver_helpers.hpp
        src/stream_manager.hpp
        src/rate_controller.hpp
        src/command_parser.hpp
        src/frame_sink.hpp
        src/uws_frame_sink.hpp
        src/checkerboard_detector.h
    )

    # Create executable
    add_executable(camera_ws_server ${SOURCES} ${HEADERS})

    # Include nlohmann/json headers
    target_include_directories(camera_ws_server PRIVATE
        ${json_SOURCE_DIR}/include
        ${LIBCAMERA_INCLUDE_DIRS}
    )
    target_compile_options(camera_ws_server PRIVATE ${LIBCAMERA_CFLAGS_OTHER})

    # Link libraries
    target_link_libraries(camera_ws_server
        ${CMAKE_THREAD_LIBS_INIT}
        ZLIB::ZLIB
        uSockets
        nlohmann_json::nlohmann_json
        ${OpenCV_LIBS}
        ${LIBCAMERA_LIBRARIES}
    )
    target_link_directories(camera_ws_server PRIVATE ${LIBCAMERA_LIBRARY_DIRS})

    # Compiler flags - optimize for Raspberry Pi 5 (Cortex-A76)
    target_compile_options(camera_ws_server PRIVATE
        -Wall
        -Wextra
        -O3
        -march=armv8.2-a+fp16+simd
        -mtune=cortex-a76
        -ffast-math
        -funroll-loops
        -ftree-vectorize
    )
endif()

# Standalone YUV420 converter for frames saved by FrameSaver — no libcamera or
# uWebSockets dependency, so it also builds under -DCHERUPI_TOOLS_ONLY=ON.
add_executable(yuv420_convert tools/yuv420_convert.cpp)
target_compile_options(yuv420_convert PRIVATE -O2)
target_include_directories(yuv420_convert PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(yuv420_convert ${OpenCV_LIBS} Threads::Threads)

if(NOT CHERUPI_TOOLS_ONLY)
    # Installation
    install(TARGETS camera_ws_server
        RUNTIME DESTINATION bin
    )

    # Tests
    if(CHERUPI_BUILD_TESTS)
        enable_testing()
        add_subdirectory(tests)
    endif()
endif()
