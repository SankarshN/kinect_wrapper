#ifndef SKPR_DEPTH_REGISTER_H
#define SKPR_DEPTH_REGISTER_H

#include "SKPRecipient.h"

#include <vector>

class SKPRDepthRegister : public SKPRecipient {
public:
    //Display rows & columns
    SKPRDepthRegister();
    ~SKPRDepthRegister();

    void receiveFrame(SKPacket &skp);
    void addRecipient(SKPRecipient *skpr);

protected:
    std::vector<SKPRecipient *> _recipients;
};

#endif