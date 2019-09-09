cmake_minimum_required(VERSION 2.8)

project(ReflectorSLAM)

aux_source_directory(. DIR_SRCS)

find_package( Ceres REQUIRED)

add_library(ReflectorSLAM SHARED DIR_SRCS )

add_definitions(-DZMQ_STATIC) 

target_link_libraries(ReflectorSLAM ${Protobuf_LIBRARIES}
                                 ${Glog_LIBRARIES}
                                 ${Gflags_LIBRARIES}
				 ${CERES_LIBRARIES}
                                 debug libzmqd
                                 optimized libzmq
                                 ws2_32.lib
                                 rpcrt4.lib
                                 iphlpapi.lib)