cmake_minimum_required(VERSION 3.0)
project(DataReaderBlock)

add_library(DataReaderBlock SHARED proto_data_reader.cpp
 ../proto/stdmsg.pb.cc ../utils/configfile.cpp)

 target_link_libraries(DataReaderBlock ${PROTOBUF_LIBRARIES} ${GLOG_LIBRARIES} ${GFLAGS_LIBRARIES})