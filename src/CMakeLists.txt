cmake_minimum_required(VERSION 3.0.0 FATAL_ERROR)

set (MAVTUNNEL_SRC
    tunnel.c)

add_library(mavtunnel
    STATIC
    ${MAVTUNNEL_SRC})

target_include_directories(mavtunnel
    PRIVATE
    ${MAVLINK_INCLUDE_DIR}
    ${MAVTUNNEL_INCLUDE_DIR})

add_dependencies(mavtunnel
    ardupilotmega.xml-v1.0)


