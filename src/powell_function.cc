// @file        This file is part of PML-SLAM library. You can use it in compliance with GNU GPL V3 license.
// @brief   
// @author      Yuntian Li (yuntianli@pmlabs.com.cn)
// @version     1.0
// @date        Last modified at 2024-04-17
// @copyright   Copyright (c) {2024 - } Purple Mountain Lab UAV TEAM.

#include <ceres/ceres.h>

struct F1
{
  // f1 = x1 + 10 * x2
  template <typename T>
  bool operator()(const T* const x1, const T* const x2, T* residual) const{
    residual[0] = x1[0] + 10. * x2[0];
    return true;
  }
};

struct F2{
  template <typename T>
  bool operator()(const T* const x3, const T* const x4, T* residual) const{
    residual[0] = sqrt(5.0) * (x3[0] - x4[0]);
    return true;
  }
};

struct F3{
  template <typename T>
  bool operator()(const T* const x2, const T* const x3, T* residual) const{
    residual[0] = pow((x2[0] - 2.0 * x3[0]), 2);
    return true;
  }
};

struct F4{
  template <typename T>
  bool operator()(const T* const x1, const T* const x4, T* residual) const{
    residual[0] = sqrt(10.0) * pow((x1[0] - x4[0]), 2);
    return true;
  }
};

int main(int argc, char** argv){
  double x1 = 3.0;
  double x2 = -1.0;
  double x3 = 0.0;
  double x4 = 1.0;
  // 构建问题
  ceres::Problem powell_problem;
  powell_problem.AddResidualBlock(
      new ceres::AutoDiffCostFunction<F1, 1, 1, 1>(new F1), nullptr, &x1, &x2);
  powell_problem.AddResidualBlock(
      new ceres::AutoDiffCostFunction<F2, 1, 1, 1>(new F2), nullptr, &x3, &x4);
  powell_problem.AddResidualBlock(
      new ceres::AutoDiffCostFunction<F3, 1, 1, 1>(new F3), nullptr, &x2, &x3);
  powell_problem.AddResidualBlock(
      new ceres::AutoDiffCostFunction<F4, 1, 1, 1>(new F4), nullptr, &x1, &x4);
  // 设置求解选项
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = true;
  options.max_num_iterations = 100;
  // 求解问题
  ceres::Solver::Summary summary;
  ceres::Solve(options, &powell_problem, &summary);

  std::cout << summary.BriefReport() << "\n";
  printf("Final resulat: %f, %f, %f, %f\n", x1, x2, x3, x4);
}