#include "SMColorBlob.h"
#include "SKPacket.h"

using namespace std;
using namespace cv;

SMColorBlob::SMColorBlob(
    std::string image, std::string outImage) :
    _image(image),  _outImage(outImage) {

}

void SMColorBlob::receiveFrame(SKPacket &skp) {
    // skp.allocateCVMat(skp.getCVMat(_image).rows, skp.getCVMat(_image).cols, CV_8UC3, _image);
    cv::Mat &inMat = skp.getCVMat(_image);
    skp.allocateCVMat(inMat.rows, inMat.cols, CV_8UC3, "color_blob_rgb");
    cv::Mat &cbMat = skp.getCVMat("color_blob_rgb");
    inMat.copyTo(cbMat);

    processImage(cbMat);
    // skp.copyCVMat(_image, _outImage);

    // cvtColor(skp.getCVMat(_image), gray, COLOR_BGR2GRAY);
    // image_u8_t im = {.width = gray.cols,
    //                  .height = gray.rows,
    //                  .stride = gray.cols,
    //                  .buf = gray.data};
    for(size_t i = 0; i < _recipients.size(); i++) {
        _recipients[i]->receiveFrame(skp);
    }
}

void SMColorBlob::processImage(cv::Mat img) {
    _frame = img;
    split();
    findBlue();
    findGreen();
    findRed();
    findBGR();
    // showResult();
}

void SMColorBlob::split() {
    cv::split(_frame, _chans);
}

void SMColorBlob::findBlue() {
    cv::subtract(_chans[0], _chans[2], blue_subtraction);
    cv::threshold(blue_subtraction, blue_threshold, 55, 205, cv::THRESH_BINARY);

    std::vector<cv::Mat> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(blue_threshold, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE);

    int largest_i = 0;
    for (int i = 0; i < contours.size(); i++) {
        if (cv::contourArea(contours[i]) > cv::contourArea(contours[largest_i])) {
            largest_i = i;
        }
    }

    blue_mask = cv::Mat::zeros(_chans[0].rows, _chans[0].cols, CV_8UC1);
    cv::drawContours(blue_mask, contours, largest_i, cv::Scalar(205), cv::LineTypes::FILLED, 8, hierarchy);

    cv::Mat temp;
    _frame.cv::Mat::copyTo(temp, blue_mask);
    blue_cup = temp;
    skp.allocateCVMat(skp.getCVMat(blue_cup).rows, skp.getCVMat(blue_cup).cols, CV_8UC3, "blue_cup");
    skp.copyCVMat("blue_cup", _outImage);
}

void SMColorBlob::findGreen() {
    cv::subtract(_chans[1], _chans[0], green_subtraction);
    cv::threshold(green_subtraction, green_threshold, 60, 205, cv::THRESH_BINARY);
    
    std::vector<cv::Mat> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(green_threshold, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE);

    int largest_i = 0;
    for (int i = 0; i < contours.size(); i++) {
        if (cv::contourArea(contours[i]) > cv::contourArea(contours[largest_i])) {
            largest_i = i;
        }
    }

    green_mask = cv::Mat::zeros(_chans[1].rows, _chans[1].cols, CV_8UC1);
    cv::drawContours(green_mask, contours, largest_i, cv::Scalar(205), cv::LineTypes::FILLED, 8, hierarchy);
    cv::Mat temp;
    _frame.cv::Mat::copyTo(temp, green_mask);
    green_cup = temp;
    skp.allocateCVMat(skp.getCVMat(green_cup).rows, skp.getCVMat(green_cup).cols, CV_8UC3, "green_cup");
    skp.copyCVMat("green_cup", _outImage);
}

void SMColorBlob::findRed() {
    cv::subtract(_chans[2], _chans[1], red_subtraction);
    cv::threshold(red_subtraction, red_threshold, 55, 205, cv::THRESH_BINARY);
    
    std::vector<cv::Mat> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(red_threshold, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE);

    int largest_i = 0;
    for (int i = 0; i < contours.size(); i++) {
        if (cv::contourArea(contours[i]) > cv::contourArea(contours[largest_i])) {
            largest_i = i;
        }
    }
    red_mask = cv::Mat::zeros(_chans[2].rows, _chans[2].cols, CV_8UC1);
    cv::drawContours(red_mask, contours, largest_i, cv::Scalar(205), cv::LineTypes::FILLED, 8, hierarchy);
    cv::Mat temp;
    _frame.cv::Mat::copyTo(temp, red_mask);
    red_cup = temp;
    skp.allocateCVMat(skp.getCVMat(red_cup).rows, skp.getCVMat(red_cup).cols, CV_8UC3, "red_cup");
    skp.copyCVMat("red_cup", _outImage);
}

void SMColorBlob::findBGR() {
    cv::bitwise_or(blue_mask, green_mask, final_mask);
    cv::bitwise_or(red_mask, final_mask, final_mask);

    cv::Mat temp;
    _frame.cv::Mat::copyTo(temp, final_mask);
    final_image = temp;
    skp.allocateCVMat(skp.getCVMat(bgr_cup).rows, skp.getCVMat(bgr_cup).cols, CV_8UC3, "bgr_cup");
    skp.copyCVMat("bgr_cup", _outImage);
}

void SMColorBlob::addRecipient(SKPRecipient *skpr) {
    _recipients.push_back(skpr);
}