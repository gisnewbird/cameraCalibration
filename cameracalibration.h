#ifndef CAMERACALIBRATION_H
#define CAMERACALIBRATION_H
#include "mainwindow.h"
#include "ui_mainwindow.h"

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/xfeatures2d/nonfree.hpp>	//SurfFeatureDetector实际在该头文件中
#include "opencv2/features2d/features2d.hpp"	//FlannBasedMatcher实际在该头文件中
#include "opencv2/calib3d/calib3d.hpp"	//findHomography所需头文件
#include <opencv2/features2d.hpp>
#include <opencv2/stitching.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/core/affine.hpp>
//#include <openMVG/sfm/sfm.hpp>
using namespace cv;
using namespace std;
using namespace xfeatures2d;
bool refineMatchesWithHomography(const std::vector<cv::KeyPoint>& queryKeypoints, const std::vector<cv::KeyPoint>& trainKeypoints,
    float reprojectionThreshold, std::vector<cv::DMatch>& matches, cv::Mat& homography);
void cameraCalibration_triggered(QStringList files,Size square_size,Size board_size);
void SURF_triggered(QStringList);
bool find_transform(Mat& K, vector<Point2f>& p1, vector<Point2f>& p2, Mat& R, Mat& T, Mat& mask);


#endif // CAMERACALIBRATION_H
