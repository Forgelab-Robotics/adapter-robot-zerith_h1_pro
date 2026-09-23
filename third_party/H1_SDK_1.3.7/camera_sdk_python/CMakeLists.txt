cmake_minimum_required(VERSION 3.21)
project(camera_sdk_python312 LANGUAGES CXX)

if(POLICY CMP0144)
    cmake_policy(SET CMP0144 NEW)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(ZERITH_SDK_ROOT "${CMAKE_CURRENT_LIST_DIR}/.." CACHE PATH "Path to H1_SDK_1.3.7 root")
set(CAMERA_CPP_ROOT "${ZERITH_SDK_ROOT}/camera_sdk_cpp")
set(THIRD_LIBS_ROOT "${CAMERA_CPP_ROOT}/3rd_party")

set(OPENCV_ROOT "${THIRD_LIBS_ROOT}/opencv_static_lib")
set(FFMPEG_ROOT "${THIRD_LIBS_ROOT}/ffmpeg_static_lib")
set(GRPC_ROOT "${THIRD_LIBS_ROOT}/grpc_static_lib")
set(RTC_ROOT "${THIRD_LIBS_ROOT}/libdatachannel_static_lib")
set(OPENSSL_ROOT "${THIRD_LIBS_ROOT}/openssl_static_lib")

if(NOT EXISTS "${CAMERA_CPP_ROOT}/include/camera_client.h")
    message(FATAL_ERROR "camera_sdk_cpp not found under ZERITH_SDK_ROOT: ${ZERITH_SDK_ROOT}")
endif()

find_package(Python3 3.12 REQUIRED COMPONENTS Interpreter Development.Module)
execute_process(
    COMMAND "${Python3_EXECUTABLE}" -c "import sys; print(sys.prefix)"
    OUTPUT_VARIABLE PYTHON_PREFIX
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

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
    "${GRPC_ROOT}/lib/cmake/protobuf"
    "${GRPC_ROOT}/lib/cmake/grpc"
    "${OPENCV_ROOT}"
    "${FFMPEG_ROOT}"
    "${RTC_ROOT}"
    "${OPENSSL_ROOT}"
)

set(CMAKE_FIND_LIBRARY_SUFFIXES ".a;${CMAKE_FIND_LIBRARY_SUFFIXES}")

find_package(pybind11 CONFIG REQUIRED)
find_package(Protobuf CONFIG REQUIRED)
find_package(gRPC CONFIG REQUIRED)
find_package(OpenCV REQUIRED CONFIG)
find_package(LibDataChannel CONFIG REQUIRED)
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

function(find_shared_library OUT_VAR LIB_NAME HINT_ROOT APT_PACKAGE)
    unset(${OUT_VAR} CACHE)
    set(_OLD_SUFFIXES ${CMAKE_FIND_LIBRARY_SUFFIXES})
    set(CMAKE_FIND_LIBRARY_SUFFIXES ".so")
    find_library(_FOUND_SHARED_LIB NAMES ${LIB_NAME} PATHS "${HINT_ROOT}/lib" NO_DEFAULT_PATH NO_CACHE)
    if(NOT _FOUND_SHARED_LIB)
        find_library(_FOUND_SHARED_LIB NAMES ${LIB_NAME} NO_CACHE)
    endif()
    set(CMAKE_FIND_LIBRARY_SUFFIXES ${_OLD_SUFFIXES})

    if(NOT _FOUND_SHARED_LIB)
        message(FATAL_ERROR
            "Shared library lib${LIB_NAME}.so not found. "
            "Install it in the Python environment or via ${APT_PACKAGE}."
        )
    endif()

    set(${OUT_VAR} "${_FOUND_SHARED_LIB}" PARENT_SCOPE)
endfunction()

find_library(CAMERA_CLIENT_LIB NAMES camera_client PATHS "${CAMERA_CPP_ROOT}/lib" NO_DEFAULT_PATH)
find_shared_library(AVCODEC_SHARED_LIB avcodec "${PYTHON_PREFIX}" "ffmpeg/libavcodec-dev")
find_shared_library(AVUTIL_SHARED_LIB avutil "${PYTHON_PREFIX}" "ffmpeg/libavutil-dev")
find_shared_library(SWSCALE_SHARED_LIB swscale "${PYTHON_PREFIX}" "ffmpeg/libswscale-dev")
find_library(SSL_LIB NAMES libssl.a PATHS "${OPENSSL_ROOT}/lib" NO_DEFAULT_PATH)
find_library(CRYPTO_LIB NAMES libcrypto.a PATHS "${OPENSSL_ROOT}/lib" NO_DEFAULT_PATH)

assert_real_static_archive("${CAMERA_CLIENT_LIB}" "libcamera_client.a")
assert_real_static_archive("${SSL_LIB}" "libssl.a")
assert_real_static_archive("${CRYPTO_LIB}" "libcrypto.a")

find_package(PkgConfig)
if(PkgConfig_FOUND)
    pkg_check_modules(GTK2 gtk+-2.0)
endif()

pybind11_add_module(_camera_client_native
    src/camera_bindings.cpp
)

target_include_directories(_camera_client_native PRIVATE
    "${CAMERA_CPP_ROOT}/include"
    "${OPENCV_ROOT}/include/opencv4"
    "${FFMPEG_ROOT}/include"
    "${OPENSSL_ROOT}/include"
    "${RTC_ROOT}/include"
)

target_link_libraries(_camera_client_native PRIVATE
    "${CAMERA_CLIENT_LIB}"
    opencv_core
    opencv_imgproc
    opencv_imgcodecs
    LibDataChannel::LibDataChannel
    "${AVCODEC_SHARED_LIB}"
    "${AVUTIL_SHARED_LIB}"
    "${SWSCALE_SHARED_LIB}"
    gRPC::grpc++_unsecure
    gRPC::grpc_unsecure
    gRPC::gpr
    protobuf::libprotobuf
    ${GTK2_LIBRARIES}
    X11
    "${SSL_LIB}"
    "${CRYPTO_LIB}"
    z
    m
    dl
    rt
    lzma
    bz2
    Threads::Threads
)

set_target_properties(_camera_client_native PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}/lib"
    BUILD_RPATH "${PYTHON_PREFIX}/lib"
)
