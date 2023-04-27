#include "SMColorBlob.h"
#include "MaskGenerator.h"
#include "SKPacket.h"

#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION // not sure if this line is needed...
#include <numpy/ndarrayobject.h>
#include <opencv2/opencv.hpp>
#include <iostream>

using namespace std;
using namespace cv;

SMColorBlob::SMColorBlob(
    std::string image, std::string outImage) :
    _image(image),  _outImage(outImage) {

}

void SMColorBlob::receiveFrame(SKPacket &skp) {
    /* Receive the input image */
    cv::Mat &inMat = skp.getCVMat(_image);
    skp.allocateCVMat(inMat.rows, inMat.cols, CV_8UC3, "color_blob_rgb");
    cv::Mat &cbMat = skp.getCVMat("color_blob_rgb");
    inMat.copyTo(cbMat);

    /* Allocate space for the outgoing image */
    skp.allocateCVMat(skp.getCVMat(_outImage).rows, skp.getCVMat(_outImage).cols, CV_8UC3, _outImage);
    
    
    /* Retrieve the mask from MaskGenerator */
    MaskGenerator mg;
    // cv::Mat mask = mg.getModelMask(cbMat, "YOLOSegmentation");
    // cv::Mat mask = mg.getColorMask(cbMat, "blue");
    cv::Mat mask = mg.getRGBMask(cbMat);


    /* Apply the mask on the original image */
    cv::Mat temp;
    cbMat.cv::Mat::copyTo(temp, mask);
    masked_image = temp;
    skp.setCVMat(masked_image, "masked_image");

    /* Convert the masked image to a k4a::image in BGRA format */
    cv::Mat bgra;
    cv::cvtColor(masked_image, bgra, cv::COLOR_RGB2BGRA);
    k4a::image k4a_masked_image = k4a::image::create(
        K4A_IMAGE_FORMAT_COLOR_BGRA32,
        bgra.cols, 
        bgra.rows,
        (int)bgra.step
    );
    memcpy(
        k4a_masked_image.get_buffer(),
        bgra.data,
        bgra.step * bgra.rows
    );
    // k4a::image k4a_masked_image = k4a::image::create_from_buffer(
    //     K4A_IMAGE_FORMAT_COLOR_BGRA32,
    //     bgra.cols,
    //     bgra.rows,
    //     (int)bgra.step,
    //     bgra.data,
    //     bgra.step * bgra.rows,
    //     nullptr,
    //     nullptr
    // );

    /* Convert the k4a::image to a depth image */
    SKWrapper *wrapper = skp.getSKWrapper();
    k4a::image depthRegistered = wrapper->transformation.color_image_to_depth_camera(skp.getDepthImage(), k4a_masked_image);
    skp.allocateCVMat(576, 640, CV_8UC4, "DEPTH_REGISTERED_640x576_BGRA");
    uint8_t *depth_buffer = depthRegistered.get_buffer();
    memcpy(skp.getCVMat("DEPTH_REGISTERED_640x576_BGRA").data, depth_buffer, depthRegistered.get_width_pixels() * depthRegistered.get_height_pixels() * 4);
    skp.allocateCVMat(576, 640, CV_8UC3, "DEPTH_REGISTERED_640x576_RGB");
    cv::cvtColor(skp.getCVMat("DEPTH_REGISTERED_640x576_BGRA"), skp.getCVMat("DEPTH_REGISTERED_640x576_RGB"), cv::COLOR_BGRA2RGB);
    
    /* Convert depth image to point cloud */
    // k4a::image point_cloud = wrapper->transformation.depth_image_to_point_cloud(skp.getDepthImage(), K4A_CALIBRATION_TYPE_DEPTH); //, &xyzPointCloud);
    // k4a::image point_cloud;
    // wrapper->transformation.depth_image_to_point_cloud(depthRegistered, K4A_CALIBRATION_TYPE_UNKNOWN);

    // ------------- SCRATCH WORK -------------

    /*
    k4a image --> get mask through generator --> get masked image --> cvMat to k4a () --> call color to depth --> call depth to point cloud on k4a --> display depth viewer
    */

    // DELETE THIS
    // uint8_t *depth_buffer = depth.get_buffer();
    // std::cout << depth.get_buffer();
    // image.get_format() == K4A_IMAGE_FORMAT_COLOR_BGRA32
    // skp.allocateCVMat(depth.get_height_pixels(), depth.get_width_pixels(), CV_16U, "DEPTHMASK");
    // memcpy(skp.getCVMat("DEPTHMASK").data, depth_buffer, depth.get_width_pixels() * depth.get_height_pixels() * 2);
    // k4a::image *depth;
    // wrapper->transformation.color_image_to_depth_camera(skp.getDepthImage(), *k4a_masked_image_ptr, depth);
    // skp.copyCVMat("DEPTHMASK", _outImage);


    // Pass in the depth camera data and the masked image to the transform class
    // k4a::image point_cloud = wrapper->transformation.depth_image_to_point_cloud(depthRegistered, K4A_CALIBRATION_TYPE_DEPTH); //, &xyzPointCloud);

   skp.copyCVMat("masked_image", _outImage);

    // Every recipient receives the (processed?) frame
    for(size_t i = 0; i < _recipients.size(); i++) {
        _recipients[i]->receiveFrame(skp);
    }
}

void SMColorBlob::addRecipient(SKPRecipient *skpr) {
    _recipients.push_back(skpr);
}