/**
 Copyright (c) 2018 Udey Rishi. All rights reserved.
*/

#pragma once

namespace libstepper {

class DigitalSignalConsumer {
public:
    virtual void write(bool value) = 0;
};

}