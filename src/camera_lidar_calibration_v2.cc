/**
 * This program used for calibrating cameras and 2D lidar.
 * *.txt format:
 * lidar point (x, y) depth (d) normal(a b c)
 *
 * Author: xinliangzhong@foxmail.com
 * Data: 2018.07.20
 */

#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <ceres/ceres.h>
#include "error_types_v2.h"
#include "config.h"

using namespace std;
using namespace Eigen;
using namespace ceres;

typedef vector<Eigen::Vector3d> Vector3dPoints;
typedef vector<Eigen::Vector2d> Vector2dPoints;
typedef vector<Eigen::Vector3d> Vector3dNormals; /// normal(a b c)
typedef vector<double>          Vector1dDepths;  /// depth


bool LoadData(const string &data_path,
              Vector3dPoints &laser_points,
              Vector1dDepths &plane_depths,
              Vector3dNormals &plane_normals);
void BuildOptimizationProblem(Vector3dPoints &laser_points,
                              Vector3dNormals &plane_normals,
                              const Vector1dDepths &plane_depths,
                              Quaterniond &q,
                              Vector3d &t,
                              ceres::Problem* problem);
bool SolveOptimizationProblem(ceres::Problem* problem, bool show_the_solver_details = false);

int main()
{
    Config::setParameterFile("../config/config.yaml");

    /// Put your initial guess here. Tcl which take a vector from laser to camera.
    Matrix3d init_R;
    init_R << -0.675611 , -0.737083 , 0.0160854 ,-0.064347 , 0.0372179 , -0.997233, 0.734445 , -0.674776 ,-0.0725739;
    Quaterniond init_q(init_R);
//    cv::Mat Rcl = (Mat_<double>(3,3) <<    -0.675611 , -0.737083 , 0.0160854 ,-0.064347 , 0.0372179 , -0.997233, 0.734445 , -0.674776 ,-0.0725739);
//    cv::Mat tcl = (Mat_<double>(3,1) << -0.186222, 0.555921, -0.0547161);
//    Vector3d init_t(-0.165,0.528,-0.045);
    Vector3d init_t(-0.186222,0.555921, -0.0547161);
    ceres::Problem problem;
    Vector3dPoints laser_points;
    Vector3dNormals plane_normals;
    Vector1dDepths plane_depths;

    string data_path = Config::get<string>("working.dir") + "data/data_v2.txt";
    cout << "data.path = " << data_path << endl;
    /**
     * Load data
     */
    cout << "Before Optimization\n" << "R = \n" << init_q.matrix() << "\nt = \n" << init_t.transpose() << endl;
    if(LoadData(data_path, laser_points, plane_depths, plane_normals))
    {
        cout << "Load data suscessfully!" << endl;

        /**
         * Optimizing
         */
        BuildOptimizationProblem(laser_points,plane_normals, plane_depths, init_q,init_t,&problem);
        SolveOptimizationProblem(&problem, false);

        cout << "After Optimization:\n";
        cout << "R = \n" << init_q.matrix() << endl;
        cout << "t = \n" << init_t.transpose() << endl;

    }

    return 0;
}

bool LoadData(const string &data_path, Vector3dPoints &laser_points, Vector1dDepths &plane_depths, Vector3dNormals &plane_normals)
{
    if(data_path.empty())
        return false;
    ifstream in_file;
    in_file.open(data_path);
    while(in_file.good())
    {
        Vector3d laser_point;
        Vector3d plane_normal;
        double plane_depth;
        in_file >> laser_point(0) >> laser_point(1) >> laser_point(2) \
                >> plane_depth \
                >> plane_normal(0) >> plane_normal(1) >> plane_normal(2);
        laser_points.push_back(laser_point);
        plane_depths.push_back(plane_depth);
        plane_normals.push_back(plane_normal);
        in_file.get();
    }
    in_file.close();
    return true;
}

void BuildOptimizationProblem(Vector3dPoints &laser_points,
                              Vector3dNormals &plane_normals,
                              const Vector1dDepths &plane_depths,
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
                ErrorTypes::Create(plane_depths[i]);
        problem->AddResidualBlock(cost_function,
                                  loss_function,
                                  q.coeffs().data(),
                                  t.data(),
                                  laser_points[i].data(),
                                  plane_normals[i].data());
        problem->SetParameterization(q.coeffs().data(),
                                     quaternion_local_parameterization);
    }

    for (int j = 0; j < laser_points.size(); ++j)
    {
        problem->SetParameterBlockConstant(laser_points[j].data()); /// Do not optimize the laser points.
        problem->SetParameterBlockConstant(plane_normals[j].data());/// Do not optimize the plane normals.
    }
//    problem->SetParameterBlockConstant(t.data());
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