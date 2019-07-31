cmake_minimum_required(VERSION 3.0)
project(Localization)

SET(EXECUTABLE_OUTPUT_PATH "${CMAKE_BINARY_DIR}/bin/")
set( CMAKE_LIBRARY_OUTPUT_DIRECTORY ${EXECUTABLE_OUTPUT_PATH}/lib/)
set( CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${EXECUTABLE_OUTPUT_PATH}/lib )
link_directories(${CMAKE_ARCHIVE_OUTPUT_DIRECTORY} ${CMAKE_SOURCE_DIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${EXECUTABLE_OUTPUT_PATH})

add_definitions(-std=c++17)

if(UNIX)
    add_definitions(-D linux)
endif(UNIX)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

link_directories(3rdparty/lib)
include_directories(3rdparty/include)
#add_library(stdmsg proto/stdmsg.cc)
#add_subdirectory(utils)

# SET(protobuf_DIR "D:/Program Files (x86)/protobuf")
# SET(glog_DIR "D:/Program Files (x86)/glog")
# SET(gflags_DIR "D:/Program Files (x86)/gflags")
find_package(Protobuf REQUIRED)
find_package(Glog REQUIRED)
find_package(Gflags REQUIRED)

if (NOT PROTOBUF_FOUND)
    message(FATAL_ERROR "can't find protobuf")
endif (NOT PROTOBUF_FOUND)
if (NOT GLOG_FOUND)
    message(FATAL_ERROR "can't find glog")
endif (NOT GLOG_FOUND)
# link_directories("${protobuf_DIR}/lib")
# link_directories("${glog_DIR}/lib")
# link_directories("${gflags_DIR}/lib")

include_directories(${CMAKE_SOURCE_DIR})

#add_subdirectory(data_sender)
#add_subdirectory(data_reader_block)

add_subdirectory(system)
add_subdirectory(data_reader)
#add_subdirectory(recoder)