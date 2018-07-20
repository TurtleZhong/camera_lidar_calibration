
#ifndef CAMERA_LIDAR_CALIBRATION_ERROR_TYPES_V2_H
#define CAMERA_LIDAR_CALIBRATION_ERROR_TYPES_V2_H



#include <iostream>
#include <vector>
#include <eigen3/Eigen/Core>
#include <Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <ceres/ceres.h>
#include "config.h"

using namespace std;


class ErrorTypes
{
public:
    ErrorTypes(double observation):observation_(observation){}

    template<typename T>
    bool operator()(const T* const quaternion,
                    const T* const translation,
                    const T* const point,
                    const T* const normal,
                    T* residuals)const{

        Eigen::Map<const Eigen::Quaternion<T>> q_cl(quaternion);
        Eigen::Map<const Eigen::Matrix<T,3,1>> t_cl(translation);

        Eigen::Map<const Eigen::Matrix<T,3,1>> p_l(point);
        Eigen::Matrix<T,3,1> p_c = q_cl*p_l + t_cl;

        Eigen::Map<const Eigen::Matrix<T,3,1>> normal_of_plane(normal);

        T depth = normal_of_plane.transpose() * p_c; /// depth < 0
        residuals[0] = depth + T(observation_);

        return true;
    }

    static ceres::CostFunction* Create(const double observation){
        return (new ceres::AutoDiffCostFunction<ErrorTypes,1,4,3,3,3>(
                new ErrorTypes(observation)));
    }
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:
    double observation_;
};




#endif //CAMERA_LIDAR_CALIBRATION_ERROR_TYPES_V2_H
