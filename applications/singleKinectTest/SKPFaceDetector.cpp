#include "SKPFaceDetector.h"
#include "SKPacket.h"

// #include <opencv2/opencv.hpp>

SKPFaceDetector::SKPFaceDetector() {}

void SKPFaceDetector::receiveFrame(SKPacket &skp) {
    cv::Mat &inMat = skp.getCVMat("RGB1080p");
    skp.allocateCVMat(inMat.rows, inMat.cols, CV_8UC3, "face_detections");
    cv::Mat &faceMat = skp.getCVMat("face_detections");
    inMat.copyTo(faceMat);

    for(size_t i = 0; i < _recipients.size(); i++) {
        _recipients[i]->receiveFrame(skp);
    }
}

void SKPFaceDetector::addRecipient(SKPRecipient *skpr) {
    _recipients.push_back(skpr);

}
