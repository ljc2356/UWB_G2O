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

find_package(CSparse REQUIRED)
include_directories(${CSPARSE_INCLUDE_DIR})

SET(G2O_LIBS g2o_cli g2o_ext_freeglut_minimal g2o_simulator g2o_solver_slam2d_linear g2o_types_icp g2o_types_slam2d g2o_core g2o_interface g2o_solver_csparse g2o_solver_structure_only g2o_types_sba g2o_types_slam3d g2o_csparse_extension g2o_opengl_helper g2o_solver_dense g2o_stuff g2o_types_sclam2d g2o_viewer g2o_parser g2o_solver_pcg g2o_types_data g2o_types_sim3 cxsparse )

add_executable(UWB_g2o main.cpp tools/load_cdv.h modules/include/sd.cpp modules/include/sd.h modules/LocVertex.cpp modules/LocVertex.h modules/MpcVertex.cpp modules/MpcVertex.h modules/LocUnaryEdge.cpp modules/LocUnaryEdge.h modules/MpcBinaryEdge.cpp modules/MpcBinaryEdge.h tools/UWBtools.cpp tools/UWBtools.h modules/LocMoveBinaryEdge.cpp modules/LocMoveBinaryEdge.h)
target_link_libraries(UWB_g2o ${G2O_STUFF_LIBRARY} ${G2O_CORE_LIBRARY} glog ${CSPARSE_LIBRARY}
        ${G2O_LIBS}
        )


