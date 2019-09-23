cmake_minimum_required(VERSION 3.0)
project(AGVToolKit)

aux_source_directory(. SOURCE)

add_library(AGVToolKit SHARED ReflectorDetector.cpp ${COMMON_SRC})

target_link_libraries(AGVToolKit
                     ${PROTOBUF_LIBRARIES} ${GLOG_LIBRARIES} ${GFLAGS_LIBRARIES})