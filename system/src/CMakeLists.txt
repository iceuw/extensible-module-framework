cmake_minimum_required(VERSION 3.0)

project(System)


#include processor bolcks
#include_directories(${CMAKE_SOURCE_DIR}/data_reader_block)
if(MSVC)
    include_directories("D:/Program Files (x86)/gtest/include")
    link_directories("D:/Program Files (x86)/gtest/lib")
endif(MSVC)

aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/ source)

set(source ${source} ${COMMON_SRC})

message(STATUS ${source})

add_executable(system ${source} )

add_definitions(-DZMQ_STATIC)

target_link_libraries(system ${PROTOBUF_LIBRARIES} 
                             ${GFLAGS_LIBRARIES}
                             ${GLOG_LIBRARIES}
                             debug gtestd debug gtest_maind 
                             optimized gtest optimized gtest_main
                             debug libzmqd
                             optimized libzmq
                             ws2_32.lib
                             rpcrt4.lib
                             iphlpapi.lib)
if(UNIX)
    target_link_libraries(system dl)
endif(UNIX)