cmake_minimum_required(VERSION 3.0)
project(Localization)

SET(EXECUTABLE_OUTPUT_PATH "${CMAKE_SOURCE_DIR}/../bin/")
set( CMAKE_LIBRARY_OUTPUT_DIRECTORY ${EXECUTABLE_OUTPUT_PATH}/lib/)
set( CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${EXECUTABLE_OUTPUT_PATH}/lib )
link_directories(${CMAKE_ARCHIVE_OUTPUT_DIRECTORY} ${CMAKE_SOURCE_DIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${EXECUTABLE_OUTPUT_PATH})

if(UNIX)
    add_definitions(-std=c++17)
    add_definitions(-D linux)
endif(UNIX)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

find_package(Protobuf REQUIRED)

option (USE_GLOG "whether to use glog as the logger" ON)


if(USE_GLOG)

    message(STATUS "will use glog")
    find_package(Glog REQUIRED)
    find_package(Gflags REQUIRED)

endif(USE_GLOG)



include_directories(common)
include_directories(common/proto)
include_directories(3rdparty/include)
include_directories(${Protobuf_INCLUDE_DIRS})
include_directories(${CMAKE_SOURCE_DIR})

link_directories(3rdparty/lib)

aux_source_directory(${CMAKE_SOURCE_DIR}/common/proto PROTO)
set(COMMON_SRC ${PROTO})

###please use "add_subdirectory" function to add your cmake source file

add_subdirectory(module/data_sender)
#add_subdirectory(data_reader_block)
add_subdirectory(system)

##add_subdirectory(module/module_pf_localizer)
add_subdirectory(module/data_reader)
add_subdirectory(module/sensor)
add_subdirectory(module/reflector_slam)
#add_subdirectory(recoder)
