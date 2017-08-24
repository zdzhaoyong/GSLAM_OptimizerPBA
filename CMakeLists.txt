cmake_minimum_required(VERSION 2.8.6)
project(OptimizerPBA)

include(cmake/PICMake.cmake)
include_directories(src)

pi_collect_packages(VERBOSE REQUIRED System GSLAM MODULES CUDA)

set(PBA_CPU_SRCS src/pba/pba.cpp src/pba/ConfigBA.cpp src/pba/SparseBundleCPU.cpp src/pba/pba_util.cpp src/OptimizerPBA.cpp)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O2 -Wno-deprecated -pthread -march=x86-64 -mtune=generic -mfpmath=sse -fpermissive -fPIC")

if(CUDA_FOUND)
set(GENCODE -gencode=arch=compute_30,code=sm_30 -gencode=arch=compute_35,code=compute_35)  
set(GENCODE ${GENCODE} -gencode=arch=compute_20,code=sm_20)  

set(PBA_CPU_SRCS ${PBA_CPU_SRCS} src/pba/CuTexImage.cpp  src/pba/SparseBundleCU.cpp)

cuda_add_library(gslam_optimizerPBA SHARED ${PBA_CPU_SRCS} src/pba/ProgramCU.cu OPTIONS ${GENCODE})
set_property( GLOBAL APPEND PROPERTY LIBS2COMPILE  " ${CMAKE_STATIC_LIBRARY_PREFIX}pba${CMAKE_STATIC_LIBRARY_SUFFIX}")

else(CUDA_FOUND)
message("Building pba without CUDA.")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DPBA_NO_GPU")
set(CUDA_FOUND FALSE)

pi_add_target(gslam_optimizerPBA SHARED ${PBA_CPU_SRCS} REQUIRED System GSLAM)
endif()



pi_install(TARGETS gslam_optimizerPBA LIB_DESTINATION lib)
pi_report_targets()
