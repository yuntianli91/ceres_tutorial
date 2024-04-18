cmake_minimum_required(VERSION 3.16.0)

project(ceres_tutorial)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(Ceres_DIR "/usr/local/ceres_2.2.0/lib/cmake/Ceres")
find_package(Ceres 2.2.0 REQUIRED)

add_executable(hello_world ${PROJECT_SOURCE_DIR}/src/hello_world.cc)
target_link_libraries(hello_world Ceres::ceres)

add_executable(powell_function ${PROJECT_SOURCE_DIR}/src/powell_function.cc)
target_link_libraries(powell_function Ceres::ceres)

add_executable(curve_fitting ${PROJECT_SOURCE_DIR}/src/curve_fitting.cc)
target_link_libraries(curve_fitting Ceres::ceres)


