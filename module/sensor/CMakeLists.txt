cmake_minimum_required(VERSION 3.0)

project(Sensor)

add_library(Sensor SHARED laser.cpp
 ../proto/stdmsg.pb.cc ../utils/configfile.cpp ./net/sockets.cpp)

 target_link_libraries(Sensor ${PROTOBUF_LIBRARIES} ${GLOG_LIBRARIES} ${GFLAGS_LIBRARIES})