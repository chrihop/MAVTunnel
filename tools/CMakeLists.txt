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

add_executable(profile_latency
    profile_latency.cc)
