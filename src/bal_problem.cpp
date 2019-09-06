/*
 * @Description: 
 * @Author: Yuntian Li
 * @Github: https://github.com/yuntinali91
 * @Date: 2019-09-03 15:25:46
 * @LastEditors: Yuntian Li
 * @LastEditTime: 2019-09-03 15:27:48
 */
#include "./bal_problem.h"
  // open dataset text file
bool BALProblem::LoadFile(const char* filename) {
    // check whether dataset file has been opened
    FILE* fptr = fopen(filename, "r");
    if (fptr == NULL) {
      return false;
    };
    // first line: num of total cameras, num of total points, num of total observations
    FscanfOrDie(fptr, "%d", &num_cameras_);
    FscanfOrDie(fptr, "%d", &num_points_);
    FscanfOrDie(fptr, "%d", &num_observations_);

    point_index_ = new int[num_observations_];// array of point index of observations
    camera_index_ = new int[num_observations_];// array of camera index of observations
    observations_ = new double[2 * num_observations_];// array of pixel observations [x,y]

    num_parameters_ = 9 * num_cameras_ + 3 * num_points_;//array of all camera poses and points
    parameters_ = new double[num_parameters_];
    // read observations [camera_index, point_index, x, y] 
    for (int i = 0; i < num_observations_; ++i) {
      FscanfOrDie(fptr, "%d", camera_index_ + i);
      FscanfOrDie(fptr, "%d", point_index_ + i);
      for (int j = 0; j < 2; ++j) {
        FscanfOrDie(fptr, "%lf", observations_ + 2*i + j);
      }
    }

    for (int i = 0; i < num_parameters_; ++i) {
      FscanfOrDie(fptr, "%lf", parameters_ + i);
    }
    return true;
  }

