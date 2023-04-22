#ifndef COLOR_FILTER_H
#define COLOR_FILTER_H

#include <opencv2/opencv.hpp>
#include "SKPRecipient.h"

#include <string>
#include <vector>

class SMColorBlob : public SKPRecipient {
protected:
    std::vector<SKPRecipient *> _recipients;
    SKConfig &_skconfig;

    cv::Mat _frame;
    // std::vector<cv::Mat> _chans;

    // cv::Mat blue_subtraction;
    // cv::Mat blue_threshold;
    // cv::Mat blue_mask;
    // cv::Mat blue_cup;

    // cv::Mat green_subtraction;
    // cv::Mat green_threshold;
    // cv::Mat green_mask;
    // cv::Mat green_cup;

    // cv::Mat red_subtraction;
    // cv::Mat red_threshold;
    // cv::Mat red_mask;
    // cv::Mat red_cup;

    // cv::Mat final_mask;
    // cv::Mat final_image;

public:
    SMColorBlob(std::string image, std::string outImage);

    void receiveFrame(SKPacket &skp);
    void processImage(cv::Mat img);

    void split();

    // void findBlue();
    // void findGreen();
    // void findRed();
    // void findBGR();

};

#endif