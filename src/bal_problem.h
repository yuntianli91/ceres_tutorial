/*
 * @Description: 
 * @Author: Yuntian Li
 * @Github: https://github.com/yuntinali91
 * @Date: 2019-09-03 15:21:26
 * @LastEditors: Yuntian Li
 * @LastEditTime: 2019-09-03 15:44:29
 */
#ifndef BAL_PROBLEM_H_
#define BAL_PROBLEM_H_
#include <iostream>
#include <cstdio>
#include <ceres/ceres.h>

// Read a Bundle Adjustment in the Large dataset.
class BALProblem {
public:
    ~BALProblem() {
      delete[] point_index_;// release memory
      delete[] camera_index_;//release memory
      delete[] observations_;//release memory
      delete[] parameters_;//release memory
    }

    int num_observations()       const { return num_observations_;               }
    const double* observations() const { return observations_;                   }
    double* mutable_cameras()          { return parameters_;                     }// ptr to camera pose in parameters
    double* mutable_points()           { return parameters_  + 9 * num_cameras_; }// ptr to point coordinates in parameters

    double* mutable_camera_for_observation(int i) {
      return mutable_cameras() + camera_index_[i] * 9;
    }
    double* mutable_point_for_observation(int i) {
      return mutable_points() + point_index_[i] * 3;
    }

    bool LoadFile(const char* filename);
  
 private:
    template<typename T>

    void FscanfOrDie(FILE *fptr, const char *format, T *value) {
        int num_scanned = fscanf(fptr, format, value);
        if (num_scanned != 1) {
            LOG(FATAL) << "Invalid UW data file.";
        }
    }
    int num_cameras_;//num of cameras
    int num_points_;// num of points
    int num_observations_;// num of observations
    int num_parameters_;// num of camera pose and point coordinates

    int* point_index_;
    int* camera_index_;
    double* observations_;
    double* parameters_;
};
#endif