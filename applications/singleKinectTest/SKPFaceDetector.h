#ifndef SKP_FACE_DETECTOR_H
#define SKP_FACE_DETECTOR_H

#include "SKPRecipient.h"
#include <python3.6/Python.h>
#include <vector>

class SKPacket;

class SKPFaceDetector : public SKPRecipient {
public:
    SKPFaceDetector();
    void receiveFrame(SKPacket &skp);
    void addRecipient(SKPRecipient *skpr);

protected:
    std::vector<SKPRecipient *> _recipients;
    PyObject* person_find;
    PyObject* get_encoding;
    PyObject* find_person;
    double* target_encoding;
    long encoding_length;
    bool found_target;
};

#endif