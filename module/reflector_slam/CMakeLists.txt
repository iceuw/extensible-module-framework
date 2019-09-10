cmake_minimum_required(VERSION 2.8)

project(ReflectorSLAM)

aux_source_directory(. DIR_SRCS)

include_directories("D:/Program Files (x86)/protobuf/include")
include_directories("D:/Program Files (x86)/suitesparse")
include_directories("D:/Program Files (x86)/glog/include")
include_directories("D:/Program Files (x86)/gflags/include")
include_directories("D:/Program Files (x86)/Ceres/include")

link_directories("D:/Program Files (x86)/suitesparse/lib")
link_directories("D:/Program Files (x86)/glog/lib")
link_directories("D:/Program Files (x86)/gflags/lib")
link_directories("D:/Program Files (x86)/Ceres/lib")
link_directories("D:/Program Files (x86)/suitesparse/lib/lapack_blas_windows")


add_library(ReflectorSLAM SHARED ${DIR_SRCS} )

add_definitions(-DZMQ_STATIC) 

target_link_libraries(	ReflectorSLAM 
                      	ceres-debug.lib 
			gflags_static_d.lib 
			glogd.lib 
			libamdd.lib 
			libcamdd.lib
			libcolamd.lib
			libcholmodd.lib
			libspqrd.lib 
			suitesparseconfigd.lib
			libblas.lib
			liblapack.lib
			libprotobufd.lib
		     )