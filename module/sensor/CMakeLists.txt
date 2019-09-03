cmake_minimum_required(VERSION 3.0)

project(Sensor)

add_library(Sensor SHARED laser.cpp odometry.cpp
 ${COMMON_SRC} ./net/sockets.cpp ./serial/serial.cc ./serial/unix_win.cc)

 target_link_libraries(Sensor ${PROTOBUF_LIBRARIES} ${GLOG_LIBRARIES} ${GFLAGS_LIBRARIES})