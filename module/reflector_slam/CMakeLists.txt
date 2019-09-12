cmake_minimum_required(VERSION 2.8)

project(ReflectorSLAM)

aux_source_directory(. DIR_SRCS)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(Eigen REQUIRED)
find_package(SuiteSparse REQUIRED HINTS ${SuiteSparse_DIR})
find_package(Ceres REQUIRED HINTS ${Ceres_DIR})

include_directories(${CERES_INCLUDE_DIRS})
include_directories(${EIGEN_INCLUDE_DIR})

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/include)
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/lib/x86)


# include_directories("D:/Program Files (x86)/protobuf/include")
# include_directories("D:/Program Files (x86)/suitesparse")
# include_directories("D:/Program Files (x86)/glog/include")
# include_directories("D:/Program Files (x86)/gflags/include")
# include_directories("D:/Program Files (x86)/Ceres/include")
# include_directories("D:/Program Files (x86)/Eigen3/include/eigen3")

# link_directories("D:/Program Files (x86)/suitesparse/lib")
# link_directories("D:/Program Files (x86)/glog/lib")
# link_directories("D:/Program Files (x86)/gflags/lib")
# link_directories("D:/Program Files (x86)/Ceres/lib")
# link_directories("D:/Program Files (x86)/suitesparse/lib/lapack_blas_windows")

add_library(ReflectorSLAM SHARED ${DIR_SRCS} ${COMMON_SRC})

target_link_libraries(ReflectorSLAM 
						${GFLAGS_LIBARAIES}
						${GLOG_LIBRARIES}
						${PROTOBUF_LIBRARIES}
						${CERES_LIBRARIES}
						${SuiteSparse_LIBRARIES})
	

# target_link_libraries( ReflectorSLAM 
# 		  debug ceres-debug.lib 
# 			  gflags_static_d.lib 
# 			  glogd.lib 
# 			  libamdd.lib 
# 			  libcamdd.lib
# 			  libcolamdd.lib
# 		 	  libcholmodd.lib
# 		  	  libspqrd.lib 
# 			  suitesparseconfigd.lib
# 			  libblas.lib
# 			  liblapack.lib
# 			  libprotobufd.lib
# 			 )
# target_link_libraries( ReflectorSLAM 
# 		  optimized
#  			  ceres.lib 
# 			  gflags_static.lib 
# 			  glog.lib 
# 			  libamd.lib 
# 			  libcamd.lib
# 			  libcolamd.lib
# 		 	  libcholmod.lib
# 		  	  libspqr.lib 
# 			  suitesparseconfig.lib
# 			  libblas.lib
# 			  liblapack.lib
# 			  libprotobuf.lib
# 			)
