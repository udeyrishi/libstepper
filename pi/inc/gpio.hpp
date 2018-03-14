#pragma once

#include <signal.hpp>
#include <stdint.h>
#include <stdexcept>
#include <string>

namespace libstepper {

class GPIOSetupError : public std::runtime_error {
public:
    GPIOSetupError(const std::string &message) : std::runtime_error(message) { }
};

void setupGPIO();

class PiDigitalSignalOut : public DigitalSignalConsumer {
public:
    PiDigitalSignalOut(const uint64_t bcmPin);
    void write(bool value);

private:
    const uint64_t bcmPin;
};

}