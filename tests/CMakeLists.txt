cmake_minimum_required(VERSION 3.0.0 FATAL_ERROR)

FetchContent_Declare(
    googlebenchmark
    GIT_REPOSITORY https://github.com/google/benchmark.git
    GIT_TAG        v1.7.1
)
set (BENCHMARK_ENABLE_TESTING OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googlebenchmark)

FetchContent_declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG        release-1.12.0
)

set (gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

enable_testing()
include(GoogleTest)

include_directories(
    ${MAVTUNNEL_INCLUDE_DIR}
    ${MAVLINK_INCLUDE_DIR}
    ${ATTESTATION_INCLUDE_DIR}
    ${MBEDTLS_INCLUDE_DIR}
    )

add_executable(test_endpoint_linux_uart
    test_endpoint_linux_uart.cc)

target_link_libraries(test_endpoint_linux_uart
    PRIVATE
    mavtunnel
    util
    GTest::gtest_main
    GTest::gmock)

add_executable(test_codec_chacha20
    test_codec_chacha20.cc)

target_link_libraries(test_codec_chacha20
    PRIVATE
    mavtunnel
    crypto_abstract
    mbedcrypto
    GTest::gtest_main
    GTest::gmock)

gtest_discover_tests(test_endpoint_linux_uart)
gtest_discover_tests(test_codec_chacha20)

add_executable(main-pts-loopback
    main-pts-loopback.c)

target_link_libraries(main-pts-loopback
    PRIVATE
    mavtunnel
    util)

add_executable(main-ttyAMA
    main-ttyAMA.c)

target_link_libraries(main-ttyAMA
    PRIVATE
    mavtunnel
    crypto_abstract
    mbedcrypto
    )

add_executable(main-ttyAMA-passthrough
    main-ttyAMA-passthrough.c)

target_link_libraries(main-ttyAMA-passthrough
    PRIVATE
    mavtunnel)
