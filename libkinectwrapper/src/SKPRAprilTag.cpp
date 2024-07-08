#include "SKPRAprilTag.h"
#include "SKPacket.h"
#include "SKConfig.h"
#include <apriltag/apriltag.h>
#include <apriltag/tagStandard41h12.h>
#include <apriltag/tag36h11.h>
#include <apriltag/apriltag_pose.h> 
#include <k4a/k4a.hpp>

using namespace cv;
using namespace std;


SKPRAprilTag::SKPRAprilTag(
    std::string image, std::string outImage, std::string resultMat, bool render, k4a::calibration calibVals) :
    _image(image),  _resultMat(resultMat), _outImage(outImage), _render(render) {
    td = apriltag_detector_create();
    tf = tag36h11_create();
    apriltag_detector_add_family(td, tf);

    // Retrieve intrinsic parameters from device calibration
    getIntrinsicParameters(calibVals);
}

// Function to retrieve intrinsic parameters
void SKPRAprilTag::getIntrinsicParameters(k4a::calibration calibration) {
    // k4a_calibration_camera_t color_camera_calibration = calibration.depth_camera_calibration;
    k4a_calibration_camera_t color_camera_calibration = calibration.color_camera_calibration;
    _fx = color_camera_calibration.intrinsics.parameters.param.fx;
    _fy = color_camera_calibration.intrinsics.parameters.param.fy;
    _cx = color_camera_calibration.intrinsics.parameters.param.cx;
    _cy = color_camera_calibration.intrinsics.parameters.param.cy;
}

// Function to draw axis for pose estimation
std::vector<cv::Point2f> SKPRAprilTag::drawPose(cv::Mat &image, const apriltag_detection_t *det, const apriltag_pose_t *pose, double axis_length) {
    cv::Mat K = (cv::Mat_<double>(3, 3) << _fx, 0, _cx,
                                           0, _fy, _cy,
                                           0, 0, 1); // Intrinsic camera matrix

    cv::Mat rvec(3, 1, CV_64F);
    cv::Mat tvec(3, 1, CV_64F);
    cv::Mat R(3, 3, CV_64F);
    for (int i = 0; i < 9; i++) {
        R.at<double>(i / 3, i % 3) = pose->R->data[i];
    }
    cv::Rodrigues(R, rvec);  // Convert rotation matrix to rotation vector
    for (int i = 0; i < 3; i++) {
        tvec.at<double>(i) = pose->t->data[i];
    }


    std::vector<cv::Point3f> axis_points;
    axis_points.push_back(cv::Point3f(0, 0, 0));
    axis_points.push_back(cv::Point3f(axis_length, 0, 0));
    axis_points.push_back(cv::Point3f(0, -axis_length, 0));
    axis_points.push_back(cv::Point3f(0, 0, -axis_length));
    

    std::vector<cv::Point2f> image_points;
    cv::projectPoints(axis_points, rvec, tvec, K, cv::noArray(), image_points);

    // Check if points are within image bounds
    bool points_within_bounds = true;
    for (const auto &pt : image_points) {
        if (pt.x < 0 || pt.x >= image.cols || pt.y < 0 || pt.y >= image.rows) {
            points_within_bounds = false;
            break;
        }
    }

    return image_points;
}


void SKPRAprilTag::receiveFrame(SKPacket &skp) {
    
    skp.allocateCVMat(skp.getCVMat(_outImage).rows, skp.getCVMat(_outImage).cols, CV_8UC3, _outImage);
    skp.copyCVMat(_image, _outImage);
    skp.getEigenMat(_resultMat) = Eigen::MatrixXd(0,0);

    Mat gray;
    cvtColor(skp.getCVMat(_image), gray, COLOR_BGR2GRAY);
    image_u8_t im = {.width = gray.cols,
                     .height = gray.rows,
                     .stride = gray.cols,
                     .buf = gray.data};

    zarray_t* tags = apriltag_detector_detect(td, &im);

    for (int i = 0; i < zarray_size(tags); i++) {
        apriltag_detection_t *det;
        zarray_get(tags, i, &det);

        std::cout << "Apil Tag " << i << ": [" << det->c[0] << ", " << det->c[1] << "]" << std::endl;

        //Draw box around edges of the AprilTag
        cv::Point p0(det->p[0][0], det->p[0][1]);
        cv::Point p1(det->p[1][0], det->p[1][1]);
        cv::Point p2(det->p[2][0], det->p[2][1]);
        cv::Point p3(det->p[3][0], det->p[3][1]);        

        cv::line(skp.getCVMat(_image), p0, p1, cv::Scalar(255, 0, 0), 5);
        cv::line(skp.getCVMat(_image), p1, p2, cv::Scalar(255, 0, 0), 5);
        cv::line(skp.getCVMat(_image), p2, p3, cv::Scalar(255, 0, 0), 5);
        cv::line(skp.getCVMat(_image), p3, p0, cv::Scalar(255, 0, 0), 5);


        apriltag_pose_t pose;
        double tagsize = 0.20;
        apriltag_detection_info_t info;
        info.det = det;
        info.tagsize = tagsize;
        info.fx = _fx;
        info.fy = _fy;
        info.cx = _cx;
        info.cy = _cy;

        //get rotation and translation matrices for apriltag
        estimate_pose_for_tag_homography(&info, &pose);

        //Use the pose object to get coordinates for points
        std::vector<cv::Point2f> imgPoints = drawPose(skp.getCVMat(_outImage), det, &pose);

        //draw lines on screen
        cv::Point pred0(imgPoints[0]);
        cv::Point pred1(imgPoints[1]);
        cv::Point pred2(imgPoints[2]);
        cv::Point pred3(imgPoints[3]);   

        cv::line(skp.getCVMat(_image), pred0, pred1, cv::Scalar(255, 0, 0), 5);
        cv::line(skp.getCVMat(_image), pred0, pred2, cv::Scalar(0, 255, 0), 5);
        cv::line(skp.getCVMat(_image), pred0, pred3, cv::Scalar(0, 0, 255), 5);
    }


    for(size_t i = 0; i < _recipients.size(); i++) {
        _recipients[i]->receiveFrame(skp);
    }
    apriltag_detections_destroy(tags);
}

void SKPRAprilTag::addRecipient(SKPRecipient *skpr) {
    _recipients.push_back(skpr);
}

