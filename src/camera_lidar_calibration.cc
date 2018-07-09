/**
 * This program used for calibrating cameras and 2D lidars.
 * data.txt format:
 * lidar point (x, y) undistort image points (u,v)
 *
 * Author: xinliangzhong@foxmail.com
 * Data: 2018.07.05
 */

#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <ceres/ceres.h>
#include "error_types.h"
#include "config.h"

using namespace std;
using namespace Eigen;
using namespace ceres;

typedef vector<Eigen::Vector3d> Vector3dPoints;
typedef vector<Eigen::Vector2d> Vector2dPoints;


bool LoadData(const string &data_path, Vector3dPoints &laser_points, Vector2dPoints &image_points);
void BuildOptimizationProblem(Vector3dPoints &laser_points,
                              const Vector2dPoints &image_points,
                              Quaterniond &q,
                              Vector3d &t,
                              ceres::Problem* problem);
bool SolveOptimizationProblem(ceres::Problem* problem, bool show_the_solver_details = false);

int main()
{
    Config::setParameterFile("../config/config.yaml");

    /// Put your initial guess here. Tcl which take a vector from laser to camera.
    Matrix3d init_R = Matrix3d::Identity();
    Quaterniond init_q(init_R);
//    Vector3d init_t(-0.165,0.528,-0.045);
    Vector3d init_t(0.1,0.5, 0.1);
    ceres::Problem problem;
    Vector3dPoints laser_points;
    Vector2dPoints image_points;
    string data_path = Config::get<string>("working.dir") + "data/data.txt";
    cout << "data.path = " << data_path << endl;
    /**
     * Load data
     */
    cout << "Before Optimization\n" << "R = \n" << init_q.matrix() << "\nt = \n" << init_t.transpose() << endl;
    if(LoadData(data_path, laser_points, image_points))
    {
        cout << "Load data suscessfully!" << endl;

        /**
         * Optimizing
         */
        BuildOptimizationProblem(laser_points,image_points,init_q,init_t,&problem);
        SolveOptimizationProblem(&problem, true);

        cout << "After Optimization:\n";
        cout << "R = \n" << init_q.matrix() << endl;
        cout << "t = \n" << init_t.transpose() << endl;

    }

    return 0;
}

bool LoadData(const string &data_path, Vector3dPoints &laser_points, Vector2dPoints &image_points)
{
    if(data_path.empty())
        return false;
    ifstream in_file;
    in_file.open(data_path);
    while(in_file.good())
    {
        Vector3d laser_point;
        Vector2d image_point;
        in_file >> laser_point(0) >> laser_point(1) \
                >> image_point(0) >> image_point(1);
        laser_point(2) = 0.;
        laser_points.push_back(laser_point);
        image_points.push_back(image_point);
        in_file.get();
    }
    in_file.close();
    return true;
}

void BuildOptimizationProblem(Vector3dPoints &laser_points,
                              const Vector2dPoints &image_points,
                              Quaterniond &q,
                              Vector3d &t,
                              ceres::Problem* problem)
{
    ceres::LossFunction* loss_function = new ceres::HuberLoss(1.0);
    ceres::LocalParameterization* quaternion_local_parameterization =
            new EigenQuaternionParameterization;

    for (int i = 0; i < laser_points.size(); ++i)
    {
        ceres::CostFunction* cost_function =
                ErrorTypes::Create(image_points[i]);
        problem->AddResidualBlock(cost_function,
                                  loss_function,
                                  q.coeffs().data(),
                                  t.data(),
                                  laser_points[i].data());
        problem->SetParameterization(q.coeffs().data(),
                                     quaternion_local_parameterization);
    }

    for (int j = 0; j < laser_points.size(); ++j)
    {
        problem->SetParameterBlockConstant(laser_points[j].data());
    }
}


bool SolveOptimizationProblem(ceres::Problem* problem, bool show_the_solver_details)
{

    ceres::Solver::Options options;
    options.max_num_iterations = 1000;
    options.linear_solver_type = ceres::DENSE_QR;

    ceres::Solver::Summary summary;
    ceres::Solve(options, problem, &summary);
    if (show_the_solver_details)
        std::cout << summary.FullReport() << '\n';

    return summary.IsSolutionUsable();
}