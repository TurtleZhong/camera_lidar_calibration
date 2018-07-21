
#include <iostream>
#include <fstream>
#include <vector>
#include <opencv2/opencv.hpp>

#include "config.h"

using namespace std;
using namespace cv;


int main()
{
    Config::setParameterFile("/home/m/work/camera_lidar_calibration/config/config.yaml");
    /// load points
    string working_dir = Config::get<string>("working.dir");
    string points_test_path = working_dir + "data/laser_test_points_v2.txt";
    ifstream inFile;
    inFile.open(points_test_path);
    vector<cv::Mat> laser_points;
    while (inFile.good())
    {
        Mat laser_point(3,1, CV_64FC1);
        inFile >> laser_point.at<double>(0, 0) >> laser_point.at<double>(1, 0) >> laser_point.at<double>(2, 0);
        laser_points.push_back(laser_point);
    }
    inFile.close();

//    Mat laser_point = (Mat_<double>(3,1) << 2.712, -2.958, 0);
//    laser_points.push_back(laser_point);


    cout <<  "laser_points.size()= " << laser_points.size() << endl;

    Mat org = imread(working_dir+"data/image_test.png");

    double fx = Config::get<double>("fx");
    double fy = Config::get<double>("fy");
    double cx = Config::get<double>("cx");
    double cy = Config::get<double>("cy");

    double k1 = Config::get<double>("k1");
    double k2 = Config::get<double>("k2");
    double p1 = Config::get<double>("p1");
    double p2 = Config::get<double>("p2");

    cv::Mat K = (Mat_<double>(3, 3) << fx, 0., cx, 0., fy, cy, 0., 0., 1.);
    cv::Mat D = (Mat_<double>(5,1) << k1, k2, p1, p2, 0.0);
    cout << "K:\n" << K << endl;
    cout << "D:\n" << D << endl;

//    cv::Mat Rcl = (Mat_<double>(3,3) <<    -0.675611 , -0.737083 , 0.0160854 ,-0.064347 , 0.0372179 , -0.997233, 0.734445 , -0.674776 ,-0.0725739);
    cv::Mat tcl = (Mat_<double>(3,1) << -0.186222, 0.555921, -0.0547161);

    cv::Mat Rcl = (Mat_<double>(3,3) << -0.677934,-0.680351,0.278436,-0.491677,0.138072,-0.859761,0.546495,-0.719763,-0.428116);
//    cv::Mat tcl = (Mat_<double>(3,1) << -7.48245,3.70991,0.653848);

    cout << "R:\n" << Rcl << endl << "tcl:\n" << tcl << endl;


    Mat image;
    cv::undistort(org,image,K,D);

    vector<Point2d> pts_uv;
    for (int i = 0; i < laser_points.size(); ++i)
    {
        /// Reprojection

        Mat point_c = Rcl * laser_points[i] + tcl;
        if(point_c.at<double>(2,0) <= 0.)
            continue;
        cout << point_c.at<double>(0,0) << " " << point_c.at<double>(1,0) << " " << point_c.at<double>(2,0) << endl;
        point_c.at<double>(0,0) /=  point_c.at<double>(2,0);
        point_c.at<double>(1,0) /=  point_c.at<double>(2,0);
        point_c.at<double>(2,0) = 1.0;

        cout << point_c.at<double>(0,0) << " " << point_c.at<double>(1,0) << " " << point_c.at<double>(2,0) << endl;

        Mat uv = K * point_c;
        Point2d pt_uv(uv.at<double>(0,0), uv.at<double>(1,0));
        pts_uv.push_back(pt_uv);

    }
//    cvtColor(image,image,COLOR_GRAY2BGR);
    cout <<  "pts_uv.size()= " << pts_uv.size() << endl;
    ///Draw points in images
    for (int j = 0; j < pts_uv.size(); ++j) {
        cv::circle(image, pts_uv[j], 1, Scalar(0,0,255), -1);
        cout << pts_uv[j].x << " " << pts_uv[j].y << endl;
    }

    /// Show
    cv::imshow("image_with_laser_points", image);
    cv::waitKey(0);



}

