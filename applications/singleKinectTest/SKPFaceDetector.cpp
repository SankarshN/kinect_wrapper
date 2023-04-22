#include "SKPFaceDetector.h"
#include "SKPacket.h"
#include <python3.6/Python.h>

#include <opencv2/opencv.hpp>
#include <numpy/arrayobject.h>
#include <iostream>

using namespace std;

// #include <opencv2/opencv.hpp>

SKPFaceDetector::SKPFaceDetector() {
    Py_Initialize();
    person_find = PyImport_ImportModule("person_find");
    get_encoding = PyObject_GetAttrString(person_find, "get_encoding");
    find_person = PyObject_GetAttrString(person_find, "find_person");
    _import_array();
}

void SKPFaceDetector::receiveFrame(SKPacket &skp) {
    cv::Mat &inMat = skp.getCVMat("RGB1080p");
    skp.allocateCVMat(inMat.rows, inMat.cols, CV_8UC3, "face_detections");
    cv::Mat &faceMat = skp.getCVMat("face_detections");

    cv::cvtColor(faceMat, faceMat, cv::COLOR_BGR2RGB);
    npy_intp dims[3] = {faceMat.rows, faceMat.cols, faceMat.channels()};
    PyObject* numpy_array = PyArray_SimpleNewFromData(3, dims, NPY_UINT8, faceMat.data);
    PyObject* encoding = PyObject_CallFunctionObjArgs(get_encoding, numpy_array, nullptr);

    target_encoding = (double*) PyArray_DATA(target_encoding);
    encoding_length = PyArray_DIMS(numpy_array)[0];

    for (int i = 0; i < encoding_length; i++) {
        cout << target_encoding[i] << " ";
    }
    cout << endl;



    // // inMat.copyTo(faceMat);


    for(size_t i = 0; i < _recipients.size(); i++) {
        _recipients[i]->receiveFrame(skp);
    }
}

// void SKPFaceDetector::get3DPose(SKPacket& skp, double x, double y) {
//     SKWrapper wrapper = skp.getSKWrapper();
//     k4a::capture capture = skp.getCapture();
//     k4a::image color_image = capture.get_color_image();

//     cv::Mat color_mat(color_image.get_height_pixels, color_image.get_width_pixels,
//                         CV_8UC4, color_image.get_buffer, cv::Mat::AUTO_STEP);

//     // Convert the depth image to an OpenCV matrix
//     k4a::image depth_image = capture.get_depth_image();
//     cv::Mat depth_mat(depth_image.get_height_pixels, depth_image.get_width_pixels,
//                         CV_16UC1, depth_image.get_buffer, cv::Mat::AUTO_STEP);

//     // Convert the 2D point to a depth value
//     cv::Point2i point(x, y);
//     k4a_float2_t pnt = {x, y};
//     uint16_t depth_value = depth_mat.at<uint16_t>(point);

//     // Map the depth value to 3D world coordinates
//     k4a_float2_t point_3d;
//     k4a_calibration_t calibration;
//     wrapper.getCalibration().convert_2d_to_3d()
//     k4a_device_get_calibration(device, config.depth_mode, config.color_resolution, &calibration);
//     k4a_calibration_2d_to_3d(&calibration, &(k4a_float2_t){ point.x, point.y }, depth_value, K4A_CALIBRATION_TYPE_DEPTH,
//                                 K4A_CALIBRATION_TYPE_DEPTH, &point_3d, NULL);
// }

void SKPFaceDetector::addRecipient(SKPRecipient *skpr) {
    _recipients.push_back(skpr);

}

/*
using namespace std;
// export PYTHONPATH=`pwd`
// g++ -g test.cpp -I/usr/include/python3.10 -I/home/krishagarwal/.local/lib/python3.10/site-packages/numpy/core/include/ -lpython3.10 -lopencv_core -lopencv_highgui -lopencv_imgcodecs -lopencv_imgproc
int main(int argc, char const *argv[]) {
    Py_Initialize();
    import_array();
    PyObject* person_find = PyImport_ImportModule("person_find");
    PyObject* get_encoding = PyObject_GetAttrString(person_find, "get_encoding");
    PyObject* find_person = PyObject_GetAttrString(person_find, "find_person");
    cv::Mat target = cv::imread("leonardo.jpg");
    cv::cvtColor(target, target, cv::COLOR_BGR2RGB);
    npy_intp dims[3] = {target.rows, target.cols, target.channels()};
    PyObject* numpy_array = PyArray_SimpleNewFromData(3, dims, NPY_UINT8, target.data);
    PyObject* target_encoding = PyObject_CallFunctionObjArgs(get_encoding, numpy_array, nullptr);
    cv::Mat scene = cv::imread("leonardo3.jpg");
    cv::cvtColor(scene, scene, cv::COLOR_BGR2RGB);
    npy_intp scene_dims[3] = {scene.rows, scene.cols, scene.channels()};
    PyObject* scene_numpy_array = PyArray_SimpleNewFromData(3, scene_dims, NPY_UINT8, scene.data);
    PyObject* found = PyObject_CallFunctionObjArgs(find_person, scene_numpy_array, target_encoding, nullptr);
    PyObject* first = PyTuple_GetItem(found, 0);
    PyObject* second = PyTuple_GetItem(found, 1);
    long x = PyLong_AS_LONG(first), y = PyLong_AS_LONG(second);
    cout << x << " " << y << endl;
    Py_DECREF(person_find);
    Py_DECREF(get_encoding);
    Py_DECREF(find_person);
    Py_DECREF(numpy_array);
    Py_DECREF(target_encoding);
}
*/