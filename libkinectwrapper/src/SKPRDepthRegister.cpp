#include "SKPRDepthRegister.h"
#include "SKPacket.h"
#include "SKWrapper.h"

SKPRDepthRegister::SKPRDepthRegister() {}

SKPRDepthRegister::~SKPRDepthRegister() {}

void SKPRDepthRegister::receiveFrame(SKPacket &skp) {
    /* Convert the k4a::image to a depth image */

    k4a::image k4a_masked_image = skp.getColorImage();

    SKWrapper *wrapper = skp.getSKWrapper();
    k4a::image depthRegistered = wrapper->transformation.color_image_to_depth_camera(skp.getDepthImage(), k4a_masked_image);
    skp.allocateCVMat(576, 640, CV_8UC4, "DEPTH_REGISTERED_640x576_BGRA");
    uint8_t *depth_buffer = depthRegistered.get_buffer();
    memcpy(skp.getCVMat("DEPTH_REGISTERED_640x576_BGRA").data, depth_buffer, depthRegistered.get_width_pixels() * depthRegistered.get_height_pixels() * 4);
    skp.allocateCVMat(576, 640, CV_8UC3, "DEPTH_REGISTERED_640x576_RGB");
    cv::cvtColor(skp.getCVMat("DEPTH_REGISTERED_640x576_BGRA"), skp.getCVMat("DEPTH_REGISTERED_640x576_RGB"), cv::COLOR_BGRA2RGB);
    

    // Every recipient receives the (processed?) frame
    for(size_t i = 0; i < _recipients.size(); i++) {
        _recipients[i]->receiveFrame(skp);
    }
}

void SKPRDepthRegister::addRecipient(SKPRecipient *skpr) {
    _recipients.push_back(skpr);
}