/*
 * @Description: Bundle adjustment with BA
 * @Author: Yuntian Li
 * @Github: https://github.com/yuntinali91
 * @Date: 2019-09-03 13:49:15
 * @LastEditors: Yuntian Li
 * @LastEditTime: 2019-09-03 17:32:59
 */
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <iostream>

#include "./bal_problem.h"

using namespace std;

struct ReprojectionError3D{

    ReprojectionError3D(double observed_u, double observed_v)
        :observed_u_(observed_u),observed_v_(observed_v){}
 
    /**
     * @brief functor for residual
     * 
     * @param : camera array of camera parameters ,they are rotation vector, translation vector, focal length, distortion parameters k1 and k2
     *      [rvec[0], rvec[1], rvec[2], tvec[0], tvec[1], tvec[2], focal, k1, k2] 
     *      pt3_c = [R|t]pt3_w.
     *      camera model used fot bal dataset is:
     *      P  =  R * X + t       (conversion from world to camera coordinates)
     *      p  = -P / P.z         (perspective division)
     *      p' =  f * r(p) * p    (conversion to pixel coordinates)
     * @param pt3_w : 3d coordinates of point in world frame
     * @param residual : residual array with 2 dimensons in image frame
     * @return true 
     * @return false 
     */
    template <typename T>
    bool operator()(const T* const camera, const T* const pt3_w, T* residual) const{
        T pt3_c[3];//3d coordinates in camera frame     
        // conversion from world to camera coordinates
        T rvec[3] = {camera[0], camera[1], camera[2]};
        ceres::AngleAxisRotatePoint(rvec, pt3_w, pt3_c);

        pt3_c[0] += camera[3]; 
        pt3_c[1] += camera[4]; 
        pt3_c[2] += camera[5];
        // perspective division (normalization)        
        T x_normalized = -pt3_c[0] / pt3_c[2];
        T y_normalized = -pt3_c[1] / pt3_c[2];

        // distortion correction and conversion to pixel coordinates
        const T &f = camera[6];
        const T &k1 = camera[7]; 
        const T &k2 = camera[8];
        T r2 = x_normalized * x_normalized + y_normalized * y_normalized;
        // calculate distortion coefficient
        T distortion = 1.0 + r2 * (k1 + r2 * k2);
        // calculate pixel coordinates
        T predicted_u = f * distortion * x_normalized;
        T predicted_v = f * distortion * y_normalized;

        residual[0] = predicted_u - observed_u_;
        residual[1] = predicted_v - observed_v_;

        return true;
    }

    static ceres::CostFunction* create(const double observed_u, const double observed_v){
        return(new ceres::AutoDiffCostFunction<ReprojectionError3D, 2, 9, 3>(
            new ReprojectionError3D(observed_u,observed_v)));
    }
    

    double observed_u_;
    double observed_v_;

};

int main(int argc, char** argv){
    google::InitGoogleLogging(argv[0]);

    if(argc!=2){
        cerr << "usage: ceres_ba dataset_file." << endl;
        return 1;
    }

    BALProblem bal_problem;
    if(!bal_problem.LoadFile(argv[1])){
        cerr << "error loading dataset file " << argv[1] << endl;
        return 1; 
    }

    const double* observations = bal_problem.observations();

    // construct bal problem
    ceres::Problem problem;
    
    for (int i = 0; i < bal_problem.num_observations(); i++){
        ceres::CostFunction* costfunctor = 
            ReprojectionError3D::create(observations[0 + 2 * i], 
                                        observations[1 + 2 * i]);
        problem.AddResidualBlock(costfunctor, 
                                 nullptr, 
                                 bal_problem.mutable_camera_for_observation(i),
                                 bal_problem.mutable_point_for_observation(i));

    }
    
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.minimizer_progress_to_stdout = true;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << "\n";

   return 0;
}