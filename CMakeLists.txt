cmake_minimum_required(VERSION 3.14)
project(camera_websocket_server)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# FetchContent for downloading dependencies
include(FetchContent)

# Find packages
find_package(Threads REQUIRED)
find_package(ZLIB REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(OpenCV REQUIRED)

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

# Make dependencies available
FetchContent_MakeAvailable(json usockets uwebsockets)

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
    ${CMAKE_CURRENT_SOURCE_DIR}
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
    main.cpp
    websocket_server.cpp
    media_device.cpp
    v4l2_device.cpp
    camera.cpp
    camera_manager.cpp
    frame_saver.cpp
    stream_manager.cpp
    bayer_converter.cpp
    checkerboard_detector.cpp
)

# Header files
set(HEADERS
    types.hpp
    websocket_server.hpp
    media_device.hpp
    v4l2_device.hpp
    camera.hpp
    camera_manager.hpp
    frame_saver.hpp
    stream_manager.hpp
    bayer_converter.h
    checkerboard_detector.h
)

# Create executable
add_executable(camera_ws_server ${SOURCES} ${HEADERS})

# Include nlohmann/json headers
target_include_directories(camera_ws_server PRIVATE ${json_SOURCE_DIR}/include)

# Link libraries
target_link_libraries(camera_ws_server
    ${CMAKE_THREAD_LIBS_INIT}
    ZLIB::ZLIB
    uSockets
    nlohmann_json::nlohmann_json
    ${OpenCV_LIBS}
)

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

add_executable(v4l2_capture v4l2_capture.cpp)

# Installation
install(TARGETS camera_ws_server
    RUNTIME DESTINATION bin
)
