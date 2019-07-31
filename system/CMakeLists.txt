cmake_minimum_required(VERSION 3.0)

project(System)

include_directories(${CMAKE_SOURCE_DIR})
include_directories(${CMAKE_SOURCE_DIR}/utils)
include_directories(${CMAKE_SOURCE_DIR}/proto)
include_directories(${CMAKE_SOURCE_DIR}/recoder)
include_directories(${CMAKE_SOURCE_DIR}/data_sender)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

#include processor bolcks
include_directories(${CMAKE_SOURCE_DIR}/data_reader_block)

aux_source_directory(${CMAKE_SOURCE_DIR}/utils util)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/ source)

add_executable(system ${source} ${util} ../proto/stdmsg.pb.cc)

target_link_libraries(system ${PROTOBUF_LIBRARIES} )
                             #${GFLAGS_LIBRARIES}
                             #${GLOG_LIBRARIES})
if(UNIX)
    target_link_libraries(system dl)
endif(UNIX)