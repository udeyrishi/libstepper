#pragma once

namespace libstepper {

class DigitalSignalConsumer {
public:
    virtual void write(bool value) = 0;
};

}