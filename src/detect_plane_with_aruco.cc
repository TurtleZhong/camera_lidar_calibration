#include <iostream>
#include <fstream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <aruco/aruco.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>

#include "config.h"

using namespace std;
using namespace cv;
using namespace Eigen;

using namespace aruco;


std::pair<Vector3d, Vector3d> best_plane_from_points(const std::vector<Vector3d> & c)
{
    // copy coordinates to  matrix in Eigen format
    size_t num_atoms = c.size();
    Eigen::Matrix< Vector3d::Scalar, Eigen::Dynamic, Eigen::Dynamic > coord(3, num_atoms);
    for (size_t i = 0; i < num_atoms; ++i) coord.col(i) = c[i];

    // calculate centroid
    Vector3d centroid(coord.row(0).mean(), coord.row(1).mean(), coord.row(2).mean());

    // subtract centroid
    coord.row(0).array() -= centroid(0); coord.row(1).array() -= centroid(1); coord.row(2).array() -= centroid(2);

    // we only need the left-singular matrix here
    auto svd = coord.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
    Vector3d plane_normal = svd.matrixU().rightCols<1>();

    if(plane_normal(2) >0.)
        plane_normal = -1.0*plane_normal;
    cout << "depth = " << centroid.transpose() * plane_normal << endl;
    cout << "normal = \n" << plane_normal << endl;
    cout << "centor = \n" << centroid << endl;
    return std::make_pair(centroid, plane_normal);
}

//template<class Vector3>
std::pair < Vector3d, Vector3d > best_line_from_points(const std::vector<Vector3d> & c)
{
    // copy coordinates to  matrix in Eigen format
    size_t num_atoms = c.size();
    Eigen::Matrix< Vector3d::Scalar, Eigen::Dynamic, Eigen::Dynamic > centers(num_atoms, 3);
    for (size_t i = 0; i < num_atoms; ++i) centers.row(i) = c[i];

    Vector3d origin = centers.colwise().mean();
    Eigen::MatrixXd centered = centers.rowwise() - origin.transpose();
    Eigen::MatrixXd cov = centered.adjoint() * centered;
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig(cov);
    Vector3d axis = eig.eigenvectors().col(2).normalized();

    return std::make_pair(origin, axis);
}



int main()
{
    Config::setParameterFile("/home/m/work/camera_lidar_calibration/config/config_logitech.yaml");

    /// Load camera parameters
    double fx = Config::get<double>("fx");
    double fy = Config::get<double>("fy");
    double cx = Config::get<double>("cx");
    double cy = Config::get<double>("cy");

    double k1 = Config::get<double>("k1");
    double k2 = Config::get<double>("k2");
    double p1 = Config::get<double>("p1");
    double p2 = Config::get<double>("p2");

    int width = Config::get<int>("width");
    int height = Config::get<int>("height");

    Size imgSize(width, height);

    cv::Mat K = (Mat_<double>(3, 3) << fx, 0., cx, 0., fy, cy, 0., 0., 1.);
    cv::Mat D = (Mat_<double>(4,1) << k1, k2, p1, p2);
    cout << "K:\n" << K << endl;
    cout << "D:\n" << D << endl;

    /// Video capture
    VideoCapture video(0);
    if(!video.isOpened())
    {
        cout << "Can not open the capture, Please check.";
        return -1;
    }

    aruco::Dictionary dict;
    bool is_calc_plane = false;

    vector<Eigen::Vector3d> points;

    std::pair<Eigen::Vector3d, Eigen::Vector3d> plane_centor_normal;

    while (1)
    {
        Mat frame;
        video >> frame;

        /// undistort the frame
        aruco::CameraParameters camParam(K, D, imgSize);
        aruco::MarkerDetector mDetector;
        mDetector.setCornerRefinementMethod(aruco::MarkerDetector::SUBPIX);

        vector<aruco::Marker> Markers;
        mDetector.setDictionary(aruco::Dictionary::ARUCO_MIP_36h12);

        mDetector.detect(frame, Markers, camParam, 0.0385);
//        mDetector.detect(frame, Markers, camParam, 0.088);


        if(Markers.size() > 10)
            is_calc_plane = true;
        else
            is_calc_plane = false;


        if(is_calc_plane)
        {
            points.clear();
            for (int i = 0; i < Markers.size(); ++i)
            {
                Markers[i].draw(frame, Scalar(0,0,255), 1);
                aruco::CvDrawingUtils::draw3dCube(frame, Markers[i], camParam);
                aruco::CvDrawingUtils::draw3dAxis(frame, Markers[i], camParam);

                points.push_back(Vector3d((double)Markers[i].Tvec.at<float>(0,0), (double)Markers[i].Tvec.at<float>(1,0), (double)Markers[i].Tvec.at<float>(2,0)));

            }

            plane_centor_normal = best_plane_from_points(points);

            Vector3d &normal = plane_centor_normal.second;
            if(normal(2) >0.)
                normal = -1.0*normal;

//            cout << "centor is: \n" << plane_centor_normal.first << endl;
//            cout << "normal is: \n" << plane_centor_normal.second << endl;



        }


        imshow("frame", frame);
        waitKey(10);
    }

    return 0;
}