#include "SKPRAprilTag.h"
#include "SKPacket.h"
#include "SKConfig.h"
#include <apriltag/apriltag.h>
#include <apriltag/tagStandard41h12.h>
#include <apriltag/tag36h11.h>

using namespace cv;
using namespace std;


SKPRAprilTag::SKPRAprilTag(
    std::string image, std::string resultMat, std::string outImage, bool render) :
    _image(image),  _resultMat(resultMat), _outImage(outImage), _render(render) {
    td = apriltag_detector_create();
    tf = tag36h11_create();
    apriltag_detector_add_family(td, tf);
}

void SKPRAprilTag::receiveFrame(SKPacket &skp) {
    skp.getEigenMat(_resultMat) =
        detectCorners(skp.getCVMat(_image));

    for(size_t i = 0; i < _recipients.size(); i++) {
        _recipients[i]->receiveFrame(skp);
    }
}

void SKPRAprilTag::addRecipient(SKPRecipient *skpr) {
    _recipients.push_back(skpr);
}

Eigen::MatrixXd SKPRAprilTag::detectCorners(const cv::Mat &img){
    Mat gray;
    cvtColor(img, gray, COLOR_BGR2GRAY);
    image_u8_t im = {.width = gray.cols,
                     .height = gray.rows,
                     .stride = gray.cols,
                     .buf = gray.data};

    zarray_t* tags = apriltag_detector_detect(td, &im);

    // TODO assumes only one apriltag in frame - how do we want to handle more?
    for (int i = 0; i < zarray_size(tags); i++){
        apriltag_detection_t *det;
        zarray_get(tags, i, &det);

        Eigen::MatrixXd detectedCorners(2,4);
        for (int i = 0; i < 4; i++){
            detectedCorners(0, i) = det->p[i][0];
            detectedCorners(1, i) = det->p[i][1];
        }
        return detectedCorners;
    }
    return Eigen::MatrixXd(0,0);
}
