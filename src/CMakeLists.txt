cmake_minimum_required(VERSION 3.0.0 FATAL_ERROR)

set (MAVTUNNEL_SRC
    tunnel.c
    codec_passthrough.c
    codec_chacha20.c)

if (MAVTUNNEL_BAREMETAL)

else()

    list(APPEND MAVTUNNEL_SRC
        endpoint_linux_uart.c
        )

endif()

add_library(mavtunnel
    STATIC
    ${MAVTUNNEL_SRC})

target_include_directories(mavtunnel
    PRIVATE
    ${MAVLINK_INCLUDE_DIR}
    ${MAVTUNNEL_INCLUDE_DIR}
    ${ATTESTATION_INCLUDE_DIR}
    ${MBEDTLS_INCLUDE_DIR})

add_dependencies(mavtunnel
    mavlink-headers
    crypto_abstract)


