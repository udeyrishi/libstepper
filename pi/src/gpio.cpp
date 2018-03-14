#include <gpio.hpp>
#include <wiringPi.h>
#include <unistd.h>
#include <cstring>

using namespace std;

namespace libstepper {

void setupGPIO() {
    if (getuid() != 0) {
        throw GPIOSetupError("Please run with root access, since this program accesses the GPIO.");
    }

    if (wiringPiSetupGpio() < 0) {
        throw GPIOSetupError(string("Failed to setup GPIO: ") + string(strerror(errno)));
    }
}

PiDigitalSignalOut::PiDigitalSignalOut(const uint64_t bcmPin) : bcmPin(bcmPin) {
    pinMode(bcmPin, OUTPUT);
}

void PiDigitalSignalOut::write(bool value) {
    digitalWrite(bcmPin, value ? HIGH : LOW);
}



}