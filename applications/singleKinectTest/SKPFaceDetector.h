#ifndef SKP_FACE_DETECTOR_H
#define SKP_FACE_DETECTOR_H

#include "SKPRecipient.h"
#include "SKWrapper.h"
#include <python3.6/Python.h>
// #include <k4abt.hpp>
#include <vector>

class SKPacket;

class SKPFaceDetector : public SKPRecipient {
public:
    SKPFaceDetector(SKWrapper &skw);
    void receiveFrame(SKPacket &skp);
    void addRecipient(SKPRecipient *skpr);
    void find3DTargetPose(SKPacket& skp);
    void getTargetEncoding(SKPacket& skp);

protected:
    std::vector<SKPRecipient*> _recipients;
    PyObject* person_find;
    PyObject* get_encoding;
    PyObject* find_person;
    PyObject* target_encoding;
    bool found_target;
    // k4abt::tracker track;
};

#endif