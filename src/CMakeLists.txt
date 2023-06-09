cmake_minimum_required(VERSION 3.0.0 FATAL_ERROR)

set (MAVTUNNEL_SRC
    tunnel.c
    check.c
    codec_passthrough.c
    codec_chacha20.c)

if (MAVTUNNEL_BAREMETAL)
    if (BUILD_FOR STREQUAL "certikos_user")
        list(APPEND MAVTUNNEL_SRC
            endpoint_certikos_uart.c)
    endif()

else()

    list(APPEND MAVTUNNEL_SRC
        endpoint_linux_uart.c
        endpoint_linux_udp.c
        endpoint_linux_udp_client.c
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


