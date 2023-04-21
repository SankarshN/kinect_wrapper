#ifndef SKP_FACE_DETECTOR_H
#define SKP_FACE_DETECTOR_H

#include "SKPRecipient.h"

#include <vector>

class SKPacket;

class SKPFaceDetector : public SKPRecipient {
public:
    SKPFaceDetector();
    void receiveFrame(SKPacket &skp);
    void addRecipient(SKPRecipient *skpr);

protected:
    std::vector<SKPRecipient *> _recipients;
};

#endif