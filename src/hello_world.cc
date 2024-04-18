// @file        This file is part of PML-SLAM library. You can use it in compliance with GNU GPL V3 license.
// @brief       Hello world example of ceres solver, find the x which minimum 
//              0.5 * (10 - x)^2
// @author      Yuntian Li (yuntianli@pmlabs.com.cn)
// @version     1.0
// @date        Last modified at 2024-04-17
// @copyright   Copyright (c) {2024 - } Purple Mountain Lab UAV

#include <ceres/ceres.h>

struct CostFunctor {
  template <typename T>
  bool operator()(const T* const x, T* residual) const {
    residual[0] = 10.0 - x[0];
    return true;
  }
};

int main(int argc, char** argv){
  // initial value
  double x = 5.0;

  ceres::Problem problem;

  // 注意最新版本的ceres此处调用必须传入functor指针，否则会报错
  ceres::CostFunction* cost_function =
      new ceres::AutoDiffCostFunction<CostFunctor, 1, 1>(new CostFunctor);
  problem.AddResidualBlock(cost_function, nullptr, &x);

  ceres::Solver::Options option;
  option.minimizer_progress_to_stdout = true;
  ceres::Solver::Summary summary;
  
  ceres::Solve(option, &problem, &summary);

  std::cout << summary.BriefReport() << "\n";
  std::cout << "final x: " << x << "\n";

  return 0;
}