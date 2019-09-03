cmake_minimum_required(VERSION 3.0)
project(DataReader)

add_library(DataReader SHARED data_reader.cpp ${COMMON_SRC})

target_link_libraries(DataReader ${PROTOBUF_LIBRARIES} ${GLOG_LIBRARIES} ${GFLAGS_LIBRARIES})