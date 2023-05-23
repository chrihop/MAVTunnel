cmake_minimum_required(VERSION 3.0.0 FATAL_ERROR)

add_library(pts_common
    STATIC
    pts_common.c)

add_executable(pts_tunnel
    pts_tunnel.c)

target_link_libraries(pts_tunnel
    pts_common
    util)

add_executable(pts_fork
    pts_fork.c)

target_link_libraries(pts_fork
    pts_common
    util)

include(FetchContent)

FetchContent_Declare(
    json
    GIT_REPOSITORY https://github.com/nlohmann/json.git
    GIT_TAG        v3.11.2
    )

set (JSON_BuildTests OFF CACHE INTERNAL "")
set (JSON_Install OFF CACHE INTERNAL "")

FetchContent_MakeAvailable(json)

set (JSON_INCLUDE_DIR ${json_SOURCE_DIR}/include)

message(STATUS "Json: ${json_SOURCE_DIR}")

add_executable(profile_latency_uart
    profile_latency_uart.cc)

target_include_directories(profile_latency_uart
    PRIVATE
    ${MAVLINK_INCLUDE_DIR}
    ${JSON_INCLUDE_DIR}
    )

target_link_libraries(profile_latency_uart
    PRIVATE
    nlohmann_json::nlohmann_json
)

