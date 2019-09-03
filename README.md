#CPP Modular Extensible Framework(CMEF)#

This framework is designed for robot control system.

CMEF use the ros and caffe framework as reference. 

##Required lib##

* build the framework
google protobuf
glog
gflags

*build your module
google protobuf
glog(optional)
gflags(optional)

##Install Method

'''
mkdir build
cd build
cmake ..
'''

##How to write the module

1. Confirm your input message type. Write your message in the "proto/stdmsg.proto"
2. Write your CMakefile.

You can watch the other module's code as reference.

##How to Build module your module

1. Place your code in "your_work_space/src" directory.
2. Copy "agv_robot_interface", "cmake", "CMakeLists.txt" in the root dir to "your_work_space"
3. Write your cmake file. 
4. Use cmake to build your code

##How to use

write your 


