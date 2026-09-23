cmake_minimum_required(VERSION 3.21)
project(h1_sdk_python312 LANGUAGES CXX)

if(POLICY CMP0144)
    cmake_policy(SET CMP0144 NEW)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(ZERITH_SDK_ROOT "${CMAKE_CURRENT_LIST_DIR}/.." CACHE PATH "Path to H1_SDK_1.3.7 root")
set(ROBOT_SDK_ROOT "${ZERITH_SDK_ROOT}/robot_SDK")
set(THIRD_LIBS_ROOT "${ROBOT_SDK_ROOT}/3rd_party")
set(ZCM_ROOT "${THIRD_LIBS_ROOT}/zcm_static_lib")
set(SPDLOG_ROOT "${THIRD_LIBS_ROOT}/spdlog_static_lib")
set(GRPC_ROOT "${THIRD_LIBS_ROOT}/grpc_static_lib")

if(NOT EXISTS "${ROBOT_SDK_ROOT}/include/H1_Robot.hpp")
    message(FATAL_ERROR "robot_SDK not found under ZERITH_SDK_ROOT: ${ZERITH_SDK_ROOT}")
endif()

find_package(Python3 3.12 EXACT REQUIRED COMPONENTS Interpreter Development.Module)

execute_process(
    COMMAND "${Python3_EXECUTABLE}" -m pybind11 --cmakedir
    OUTPUT_VARIABLE PYBIND11_CMAKE_DIR
    ERROR_QUIET
    RESULT_VARIABLE PYBIND11_RESULT
    OUTPUT_STRIP_TRAILING_WHITESPACE
)
if(PYBIND11_RESULT EQUAL 0 AND EXISTS "${PYBIND11_CMAKE_DIR}")
    list(PREPEND CMAKE_PREFIX_PATH "${PYBIND11_CMAKE_DIR}")
endif()

set(Protobuf_DIR "${GRPC_ROOT}/lib/cmake/protobuf")
set(gRPC_DIR "${GRPC_ROOT}/lib/cmake/grpc")
list(PREPEND CMAKE_PREFIX_PATH
    "${GRPC_ROOT}"
    "${GRPC_ROOT}/lib/cmake"
    "${GRPC_ROOT}/lib/cmake/protobuf"
    "${GRPC_ROOT}/lib/cmake/grpc"
    "${SPDLOG_ROOT}"
    "${ZCM_ROOT}"
)

set(CMAKE_FIND_LIBRARY_SUFFIXES ".a;${CMAKE_FIND_LIBRARY_SUFFIXES}")

find_package(pybind11 CONFIG REQUIRED)
find_package(Protobuf CONFIG REQUIRED)
find_package(gRPC CONFIG REQUIRED)
find_package(spdlog REQUIRED CONFIG)
find_package(Threads REQUIRED)

function(assert_real_static_archive ARCHIVE_PATH ARCHIVE_LABEL)
    if(NOT ARCHIVE_PATH OR ARCHIVE_PATH MATCHES "-NOTFOUND$" OR NOT EXISTS "${ARCHIVE_PATH}")
        message(FATAL_ERROR "${ARCHIVE_LABEL} not found: ${ARCHIVE_PATH}")
    endif()

    file(READ "${ARCHIVE_PATH}" ARCHIVE_MAGIC LIMIT 8 HEX)
    if(ARCHIVE_MAGIC STREQUAL "76657273696f6e20")
        message(FATAL_ERROR
            "${ARCHIVE_LABEL} is still a Git LFS pointer, not a real static library:\n"
            "  ${ARCHIVE_PATH}\n"
            "Run git lfs pull in the SDK repository or replace it with the real .a file."
        )
    endif()

    if(NOT ARCHIVE_MAGIC STREQUAL "213c617263683e0a")
        message(FATAL_ERROR
            "${ARCHIVE_LABEL} is not a valid ar archive:\n"
            "  ${ARCHIVE_PATH}\n"
            "Header HEX: ${ARCHIVE_MAGIC}"
        )
    endif()
endfunction()

function(find_shared_system_library OUT_VAR LIB_NAME APT_PACKAGE)
    unset(${OUT_VAR} CACHE)
    set(_OLD_SUFFIXES ${CMAKE_FIND_LIBRARY_SUFFIXES})
    set(CMAKE_FIND_LIBRARY_SUFFIXES ".so")
    find_library(_FOUND_SHARED_LIB NAMES ${LIB_NAME} NO_CACHE)
    set(CMAKE_FIND_LIBRARY_SUFFIXES ${_OLD_SUFFIXES})

    if(NOT _FOUND_SHARED_LIB)
        message(FATAL_ERROR
            "System shared library lib${LIB_NAME}.so not found. "
            "Ubuntu/Debian package: ${APT_PACKAGE}"
        )
    endif()

    set(${OUT_VAR} "${_FOUND_SHARED_LIB}" PARENT_SCOPE)
endfunction()

find_library(ROBOT_CORE_LIB NAMES robot_core PATHS "${ROBOT_SDK_ROOT}/lib" NO_DEFAULT_PATH)
find_library(ZCM_LIB NAMES libzcm.a PATHS "${ZCM_ROOT}/lib" NO_DEFAULT_PATH)
find_library(ZCM_JSON_LIB NAMES libzcm_json.a PATHS "${ZCM_ROOT}/lib" NO_DEFAULT_PATH)
find_shared_system_library(ZMQ_SYSTEM_LIB zmq libzmq3-dev)
find_shared_system_library(EFIVAR_SYSTEM_LIB efivar libefivar-dev)

assert_real_static_archive("${ROBOT_CORE_LIB}" "robot_core.a")
assert_real_static_archive("${ZCM_LIB}" "libzcm.a")
assert_real_static_archive("${ZCM_JSON_LIB}" "libzcm_json.a")

set(PROTOC_EXE "${GRPC_ROOT}/bin/protoc")
set(GRPC_CPP_PLUGIN_EXE "${GRPC_ROOT}/bin/grpc_cpp_plugin")
set(PROTO_SRC_DIR "${ROBOT_SDK_ROOT}/include/GRPC_Data")
set(GEN_SRC_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated_grpc")
file(MAKE_DIRECTORY "${GEN_SRC_DIR}")

set(PROTO_FILE "${PROTO_SRC_DIR}/ip_service.proto")
add_custom_command(
    OUTPUT
        "${GEN_SRC_DIR}/ip_service.pb.cc"
        "${GEN_SRC_DIR}/ip_service.pb.h"
        "${GEN_SRC_DIR}/ip_service.grpc.pb.cc"
        "${GEN_SRC_DIR}/ip_service.grpc.pb.h"
    COMMAND "${PROTOC_EXE}"
    ARGS
        --cpp_out="${GEN_SRC_DIR}"
        --grpc_out="${GEN_SRC_DIR}"
        --plugin=protoc-gen-grpc="${GRPC_CPP_PLUGIN_EXE}"
        -I "${PROTO_SRC_DIR}"
        "${PROTO_FILE}"
    DEPENDS "${PROTO_FILE}"
    COMMENT "Generating gRPC/Protobuf sources from ${PROTO_FILE}"
)

pybind11_add_module(lib_h1_sdk_python
    src/bindings.cpp
    "${GEN_SRC_DIR}/ip_service.pb.cc"
    "${GEN_SRC_DIR}/ip_service.grpc.pb.cc"
)

target_include_directories(lib_h1_sdk_python PRIVATE
    "${ROBOT_SDK_ROOT}/include"
    "${ROBOT_SDK_ROOT}/include/ZCM_Data"
    "${ROBOT_SDK_ROOT}/include/GRPC_Data"
    "${ROBOT_SDK_ROOT}/include/log"
    "${ZCM_ROOT}/include"
    "${SPDLOG_ROOT}/include"
    "${GEN_SRC_DIR}"
)

target_link_libraries(lib_h1_sdk_python PRIVATE
    "${ROBOT_CORE_LIB}"
    -Wl,--whole-archive
    "${ZCM_LIB}"
    -Wl,--no-whole-archive
    "${ZCM_JSON_LIB}"
    gRPC::grpc++_unsecure
    gRPC::grpc_unsecure
    gRPC::gpr
    protobuf::libprotobuf
    spdlog::spdlog
    "${ZMQ_SYSTEM_LIB}"
    "${EFIVAR_SYSTEM_LIB}"
    Threads::Threads
    rt
    m
    ${CMAKE_DL_LIBS}
)

set_target_properties(lib_h1_sdk_python PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}/lib"
)
