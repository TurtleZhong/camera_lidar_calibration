#include <iostream>
#include <fstream>
#include <vector>
#include <opencv2/opencv.hpp>
//#include ""
//#include <aruco/aruco.h>
#include <aruco/aruco.h>
#include <aruco/ippe.h>



#include "config.h"

using namespace std;
using namespace cv;
//using namespace aruco;


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
    VideoCapture video(1);
    if(!video.isOpened())
    {
        cout << "Can not open the capture, Please check.";
        return -1;
    }

    aruco::Dictionary dict;

    while (1)
    {
//        Mat frame = imread("/home/m/image-test.png");
        Mat frame;
        video >> frame;

        imshow("frame_origin", frame);

        /// undistort the frame
        aruco::CameraParameters camParam(K, D, imgSize);
        aruco::MarkerDetector mDetector;
        mDetector.setCornerRefinementMethod(aruco::MarkerDetector::SUBPIX);

        vector<aruco::Marker> Markers;
        mDetector.setDictionary(aruco::Dictionary::ARUCO_MIP_36h12);

        mDetector.detect(frame, Markers, camParam, 0.0385);



//        cout << "marer.size = " << Markers.size() << endl;
        for (int i = 0; i < Markers.size(); ++i)
        {
            Markers[i].draw(frame, Scalar(0,0,255), 1);
            aruco::CvDrawingUtils::draw3dCube(frame, Markers[i], camParam);
            aruco::CvDrawingUtils::draw3dAxis(frame, Markers[i], camParam);
            cout << "Tvec: \n" << Markers[i].Tvec << endl;
//            cout << Markers[i].Rvec << endl;
//            vector<Point3f> points = Markers[i].get3DPoints(0.0385);
//            for (int j = 0; j < points.size(); ++j)
//            {
//                cout << points[j] << endl;
//            }
        }

        imshow("frame", frame);
        waitKey(30);
    }

    return 0;
}