cmake_minimum_required(VERSION 3.0.0 FATAL_ERROR)

project(MAVTunnel)

option (MAVTUNNEL_BAREMETAL "baremetal build" OFF)
option (MAVTUNNEL_PROFILING "enable profiling" ON)
option (MAVTUNNEL_TOOLS     "build tools" OFF)

set (MAVTUNNEL_C_DEFINITIONS "")

if (MAVTUNNEL_BAREMETAL)
    message(STATUS "MAVTunnel: baremetal build")
    list(APPEND MAVTUNNEL_C_DEFINITIONS MAVTUNNEL_BAREMETAL)
else ()
    message(STATUS "MAVTunnel: Linux build")
    list(APPEND MAVTUNNEL_C_DEFINITIONS MAVTUNNEL_LINUX)
endif ()

if (MAVTUNNEL_PROFILING)
    message(STATUS "MAVTunnel: profiling enabled")
    list(APPEND MAVTUNNEL_C_DEFINITIONS MAVTUNNEL_PROFILING)
else ()
    message(STATUS "MAVTunnel: profiling disabled")
endif ()

if (MAVTUNNEL_TOOLS)
    message(STATUS "MAVTunnel: build tools")
else()
    message(STATUS "MAVTunnel: tools build disabled")
endif()

# mavtunnel C definitions
add_compile_definitions(${MAVTUNNEL_C_DEFINITIONS})

string(REPLACE ";" " " MAVTUNNEL_C_DEFINITIONS_STR "${MAVTUNNEL_C_DEFINITIONS}")
message(STATUS "MAVTunnel extra C definitions: " ${MAVTUNNEL_C_DEFINITIONS_STR})

set(CMAKE_C_STANDARD 11)

include(FindPython3)
include(FetchContent)

message(STATUS "Python: " ${Python3_EXECUTABLE})

#
# MAVLINK
#
FetchContent_Declare(
    mavlink
    GIT_REPOSITORY https://github.com/mavlink/mavlink.git
    GIT_TAG master
    OVERRIDE_FIND_PACKAGE
)

set (MAVLINK_DIALECT "ardupilotmega" CACHE STRING "" FORCE)
set (MAVLINK_VERSION "2.0" CACHE STRING "" FORCE)

FetchContent_MakeAvailable(mavlink)

message(STATUS "mavlink source dir: ${mavlink_SOURCE_DIR}")
set (MAVLINK_SOURCE_DIR "${mavlink_SOURCE_DIR}" CACHE STRING "" FORCE)
set (MAVLINK_INCLUDE_DIR "${mavlink_BINARY_DIR}/include" CACHE STRING "" FORCE)

add_custom_command(
    OUTPUT ${MAVLINK_DIALECT}-${MAVLINK_VERSION}-stamp
    COMMAND PYTHONPATH=$ENV{PYTHONPATH}:${MAVLINK_SOURCE_DIR}
            ${Python3_EXECUTABLE} -m pymavlink.tools.mavgen
            --lang=C --wire-protocol=${MAVLINK_VERSION}
            --output=${MAVLINK_INCLUDE_DIR}/v${MAVLINK_VERSION}
            ${MAVLINK_SOURCE_DIR}/message_definitions/v1.0/${MAVLINK_DIALECT}.xml
            COMMAND touch ${MAVLINK_DIALECT}-${MAVLINK_VERSION}-stamp
)

add_custom_target(${MAVLINK_DIALECT}-${MAVLINK_VERSION}
            DEPENDS ${MAVLINK_DIALECT}-${MAVLINK_VERSION}-stamp)

set (MAVTUNNEL_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/inc" CACHE STRING "" FORCE)

add_custom_target(mavlink-headers
    DEPENDS ${MAVLINK_DIALECT}-${MAVLINK_VERSION})

#
# ATTESTATION
#
if (BUILD_FOR)
    set (BUILD_FOR ${BUILD_FOR} CACHE STRING "build for os target" FORCE)
endif ()

FetchContent_Declare(
    attestation
    GIT_REPOSITORY https://github.com/chrihop/attestation.git
    GIT_TAG v2
)

FetchContent_MakeAvailable(attestation)

set (ATTESTATION_INCLUDE_DIR "${attestation_SOURCE_DIR}/inc" CACHE STRING "" FORCE)
set (MBEDTLS_INCLUDE_DIR     "${attestation_SOURCE_DIR}/backend/mbedtls/include" CACHE STRING "" FORCE)
#
# --
#

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
else()
    set (MAVTUNNEL_INC_DIRS
        ${CMAKE_CURRENT_SOURCE_DIR}/inc
        ${MAVLINK_INCLUDE_DIR}
        ${ATTESTATION_INCLUDE_DIR}
        ${MBEDTLS_INCLUDE_DIR}
        PARENT_SCOPE)

    set (MAVTUNNEL_C_DEFINITIONS
        ${MAVTUNNEL_C_DEFINITIONS}
        PARENT_SCOPE)
endif()

if (MAVTUNNEL_BAREMETAL)
    if (BUILD_FOR STREQUAL "certikos_kernel")
        message(STATUS "Attestation: CertiKOS kernel build")
        add_compile_definitions(CERTIKOS_KERNEL)
    elseif (BUILD_FOR STREQUAL "certikos_user")
        message(STATUS "Attestation: CertiKOS user build")
        add_compile_definitions(CERTIKOS_USER)
    else()


    add_compile_options(-nostdlib -nostdinc -ffreestanding -fno-builtin
        -fno-common -fno-exceptions -fno-stack-protector
        -fno-unwind-tables -fno-asynchronous-unwind-tables -fno-strict-aliasing
        -fno-omit-frame-pointer -fno-strict-overflow -fno-delete-null-pointer-checks
        -fno-PIE -fno-pic -fno-pie -fno-pic -fno-stack-protector -fno-unwind-tables
        -fno-asynchronous-unwind-tables -fno-exceptions -fno-omit-frame-pointer
        -fno-delete-null-pointer-checks)

    add_link_options(
        -nostdlib -nostdinc -ffreestanding -fno-builtin
        -fno-common -fno-exceptions -fno-stack-protector
        -fno-unwind-tables -fno-asynchronous-unwind-tables -fno-strict-aliasing
        -fno-omit-frame-pointer -fno-strict-overflow -fno-delete-null-pointer-checks
        -fno-PIE -fno-pic -fno-pie -fno-pic -fno-stack-protector -fno-unwind-tables
        -fno-asynchronous-unwind-tables -fno-exceptions -fno-omit-frame-pointer
        -fno-delete-null-pointer-checks
    )

    execute_process(COMMAND ${CMAKE_C_COMPILER} -print-file-name=libgcc.a
        OUTPUT_VARIABLE C_LIBGCC_FILE
        OUTPUT_STRIP_TRAILING_WHITESPACE)

    execute_process(COMMAND ${CMAKE_C_COMPILER} -print-file-name=libm.a
        OUTPUT_VARIABLE C_LIBM_FILE
        OUTPUT_STRIP_TRAILING_WHITESPACE)

    link_libraries(
        ${C_LIBGCC_FILE}
        ${C_LIBM_FILE}
    )
endif()

else ()

if (CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_definitions(-DDEBUG)
    add_compile_options(-fsanitize=address,undefined -rdynamic)
    add_link_options(-fsanitize=address,undefined -rdynamic)
endif()

add_compile_definitions(_STD_LIBC_)
add_compile_definitions(POSIX_LIBC)

link_libraries(
    pthread
)

endif()




set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -O0 -g -Wall -Wextra -Werror -Wno-unused-parameter -Wno-unused-function")
add_compile_options(-Wno-address-of-packed-member)
add_compile_options(-Wno-missing-field-initializers)
add_compile_options(${EXTRA_C_FLAGS})

add_subdirectory(src)
add_subdirectory(tests)

if (MAVTUNNEL_TOOLS)
    add_subdirectory(tools)
endif()
