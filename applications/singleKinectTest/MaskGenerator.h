#include <python3.8/Python.h>

class MaskGenerator {
protected:
    std::vector<cv::Mat> _chans;
    cv::Mat subtraction;
    cv::Mat threshold;

    cv::Mat final_mask;
public:
    cv::Mat getModelMask(cv::Mat cbMat, String model);
    cv::Mat getColorMask(cv::Mat cbMat, String color);
    cv::Mat getRGBMask(cv::Mat cbMat);

    // void findBlue();
    // void findGreen();
    // void findRed();
    // void findBGR();
}