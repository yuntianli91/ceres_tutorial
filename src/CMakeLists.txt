cmake_minimum_required(VERSION 2.8)

include_directories("/usr/include/eigen3")
set(CMAKE_BUILD_TYPE "Debug")
find_package(Ceres REQUIRED)
find_package(OpenCV REQUIRED)


include_directories(${CERES_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

add_library(bal_problem SHARED ./bal_problem.cpp)

add_executable(ceres_tutorial ./ceres_tutorial.cpp)
add_executable(ceres_ba ./ceres_ba.cpp)
add_executable(simple_ba ./simple_bundle_adjuster.cc)

target_link_libraries(ceres_tutorial ${CERES_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(ceres_ba ${CERES_LIBRARIES} bal_problem)
target_link_libraries(simple_ba ${CERES_LIBRARIES})