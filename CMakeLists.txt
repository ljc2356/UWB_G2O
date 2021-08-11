cmake_minimum_required(VERSION 3.20)
project(UWB_g2o)

set(CMAKE_CXX_STANDARD 14)
list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )


include_directories("/usr/local/include/eigen3")
include_directories("/usr/local/include/glog")
include_directories("/usr/local/include")

link_directories("/usr/local/lib")

set(CMAKE_BUILD_TYPE "Release")
find_package(G2O REQUIRED)
include_directories(${G2O_INCLUDE_DIRS})


add_executable(UWB_g2o main.cpp tools/load_cdv.h modules/include/sd.cpp modules/include/sd.h modules/LocVertex.cpp modules/LocVertex.h modules/MpcVertex.cpp modules/MpcVertex.h modules/LocUnaryEdge.cpp modules/LocUnaryEdge.h modules/MpcBinaryEdge.cpp modules/MpcBinaryEdge.h tools/UWBtools.cpp tools/UWBtools.h modules/LocMoveBinaryEdge.cpp modules/LocMoveBinaryEdge.h)
target_link_libraries(UWB_g2o ${G2O_STUFF_LIBRARY} ${G2O_CORE_LIBRARY} glog)


