cmake_minimum_required(VERSION 2.8)

project(DataSender)

add_library(DataSender SHARED data_sender.cpp node.cpp ${COMMON_SRC})

add_definitions(-DZMQ_STATIC)

target_link_libraries(DataSender ${Protobuf_LIBRARIES}
                                 ${Glog_LIBRARIES}
                                 ${Gflags_LIBRARIES}
                                 debug libzmqd
                                 optimized libzmq
                                 ws2_32.lib
                                 rpcrt4.lib
                                 iphlpapi.lib)