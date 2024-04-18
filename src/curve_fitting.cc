// @file        This file is part of PML-SLAM library. You can use it in compliance with GNU GPL V3 license.
// @brief   
// @author      Yuntian Li (yuntianli@pmlabs.com.cn)
// @version     1.0
// @date        Last modified at 2024-04-17
// @copyright   Copyright (c) {2024 - } Purple Mountain Lab UAV TEAM.

#include <ceres/ceres.h>

class ExpCurveCostFunc{
public:
  explicit ExpCurveCostFunc(double x, double y) : x_(x), y_(x) {}

  template<typename T>
  bool operator()(const T* const m, const T* const c, T* residual){
    residual[0] = y_ - exp(m[0] * x_ + c[0]);
    return true;
  }
private:
  const double x_;
  const double y_;
};

void dataGen(double* x_data, double* y_data){

}