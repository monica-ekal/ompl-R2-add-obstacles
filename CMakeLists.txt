cmake_minimum_required(VERSION 3.5)
project(ompl_opt)

set(CMAKE_CXX_STANDARD 11)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})
add_executable(ompl_opt RRTStarMain.cpp)

target_link_libraries(ompl_opt Eigen3::Eigen ompl ${PYTHON_LIBRARIES})