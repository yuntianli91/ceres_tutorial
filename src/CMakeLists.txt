cmake_minimum_required(VERSION 3.16.0)

project(ceres_tutorial)

set(Ceres_DIR "/home/yuntian-uav/gitRepo/3rdGitRepo/ceres-solver/build")
find_package(Ceres 2.2.0 REQUIREd)
message("Ceres Version: ${CERES_VERSION}")

