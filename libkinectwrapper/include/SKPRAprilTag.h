#ifndef SKPR_APRIL_TAG_H
#define SKPR_APRIL_TAG_H

#include "SKPRecipient.h"
#include <apriltag/apriltag.h>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <apriltag/apriltag_pose.h> 
#include <k4a/k4a.hpp>


#include <string>

class SKPRAprilTag : public SKPRecipient {
public:
    SKPRAprilTag(std::string image, std::string outImage, std::string resultMat, bool render, k4a::calibration calibVals);
    void receiveFrame(SKPacket &skp);
    
    void addRecipient(SKPRecipient *skpr);

protected:    
    std::string _image, _resultMat, _outImage;
    bool _render;

    apriltag_detector_t *td;
    apriltag_family_t *tf;

    std::vector<SKPRecipient *> _recipients;
    
    std::vector<cv::Point2f> drawPose(cv::Mat &image, const apriltag_detection_t *det, const apriltag_pose_t *pose, double axis_length = 0.1);
    


    void getIntrinsicParameters(k4a::calibration calibration);

    float _fx;
    float _fy;
    float _cx;
    float _cy;
    k4a_device_t _device;
        
};

#endif


