cmake_minimum_required(VERSION 2.8)

project(DataSender)

include_directories(${CMAKE_SOURCE_DIR})

add_library(Sender SHARED sender.cpp ../utils/configfile.cpp)

target_link_libraries(Sender optimized node debug noded optimized libzmq debug libzmqd
optimized libprotobuf debug libprotobufd)