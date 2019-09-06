/*
 * @Description: 
 * @Author: Yuntian Li
 * @Github: https://github.com/yuntinali91
 * @Date: 2019-09-03 08:32:31
 * @LastEditors: Yuntian Li
 * @LastEditTime: 2019-09-03 13:43:08
 */

#include <iostream>
#include <ceres/ceres.h>
#include <opencv2/opencv.hpp>

using namespace ceres;
using namespace std;

/**
 * @brief costfunction of fitting
 * 
 */
struct SimpleCostFunctor{
        template <typename T>
        bool operator()(const T* const x, T* residual) const{
            residual[0]=T(10.0)-x[0];
            return true;
        }
};
void helloword();

struct CurveFittingFunctor{
    // observation data
    CurveFittingFunctor(double x, double y):x_(x),y_(y){}
    // residual functor
    template <typename T>
    bool operator()(const T* const m,const T* const c, T* residual) const{
        residual[0] = y_ - exp(m[0] * x_ + c[0]);
        return true;
    }

    double x_;
    double y_;
};
void curvefitting();

int main(int argc, char** argv){
    helloword();
    curvefitting();
    return 0;
}
/**
 * @brief example of solver minimum square problem: y = 0.5(10 - x)^2
 *
 * 
 */
void helloword(){
    double initial_x = 5.0;
    double x = initial_x;

    // Build the problem.
    Problem problem;

    // Set up the only cost function (also known as residual). This uses
    // auto-differentiation to obtain the derivative (jacobian).
    CostFunction* cost_function =
        new AutoDiffCostFunction<SimpleCostFunctor, 1, 1>(new SimpleCostFunctor);
    problem.AddResidualBlock(cost_function, NULL, &x);

    // Run the solver!
    Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;
    Solver::Summary summary;
    Solve(options, &problem, &summary);

    std::cout << summary.BriefReport() << "\n";
    std::cout << "x : " << initial_x
            << " -> " << x << "\n";
    

}
/**
 * @brief example of curve fitting y = exp(mx + c)
 * 
 */
void curvefitting(){
    // 生成带噪声的随机数据点集
    double step = 0.1; int num = 200;
    vector<double> param = {0.3, 0.1};
    double sigma = 0.2;
    vector<double> x_data, y_data;    
    cv::RNG rng;
  
    cout << "Generating data..." << endl;
    for(int i=0; i<num; i++){
    double x = double(i * step);
    x_data.push_back(double(x));
    y_data.push_back(exp(param[0] * x + param[1]) + rng.gaussian(sigma));
    }
    cout << "Data generated !" << endl;

    double m = 0.0;
    double c = 0.0;

    ceres::Problem problem;
    for (int i=0; i < num; i ++){
        CostFunction* functor = new AutoDiffCostFunction<CurveFittingFunctor,1,1,1>(new CurveFittingFunctor(x_data[i],y_data[i]));
        problem.AddResidualBlock(functor,nullptr,&m,&c);
    }
    // 配置求解器
      ceres::Solver::Options options;
      options.linear_solver_type = ceres::DENSE_QR;
      options.minimizer_progress_to_stdout = true;

      ceres::Solver::Summary summary;
      ceres::Solve(options, &problem, &summary);

      cout << summary.BriefReport() << endl;
      cout << "Read param are " << param[0] << " and " << param[1] << endl;
      cout << "Fitting param are " << m << " and " << c << endl;

}