/**
 Copyright (c) 2018 Udey Rishi. All rights reserved.
*/

#include <stepper.hpp>
#include <stdexcept>
#include <exception.hpp>
#include <chrono>
#include <thread>

using namespace std::this_thread;
using namespace std::chrono;
using namespace std;

#define ABS(x) (x < 0 ? -x : x)

namespace libstepper {

StepperDriverBuilder::StepperDriverBuilder() : coil1Terminal1(nullptr), coil2Terminal1(nullptr), coil1Terminal2(nullptr), coil2Terminal2(nullptr), stepsInRotation(0), initialRPM(0), maxSafeRPM(UINT64_MAX) {
}

StepperDriverBuilder &StepperDriverBuilder::setEnableTerminal(DigitalSignalConsumer &consumer) {
    enableTerminal = &consumer;
    return *this;
}

StepperDriverBuilder &StepperDriverBuilder::setCoil1Terminal1(DigitalSignalConsumer &consumer) {
    coil1Terminal1 = &consumer;
    return *this;
}

StepperDriverBuilder &StepperDriverBuilder::setCoil2Terminal1(DigitalSignalConsumer &consumer) {
    coil2Terminal1 = &consumer;
    return *this;
}

StepperDriverBuilder &StepperDriverBuilder::setCoil1Terminal2(DigitalSignalConsumer &consumer) {
    coil1Terminal2 = &consumer;
    return *this;
}

StepperDriverBuilder &StepperDriverBuilder::setCoil2Terminal2(DigitalSignalConsumer &consumer) {
    coil2Terminal2 = &consumer;
    return *this;
}

StepperDriverBuilder &StepperDriverBuilder::setRotationStepCount(const uint64_t stepsInRotation) {
    if (stepsInRotation == 0) {
        throw invalid_argument("stepsInRotation must be > 0");
    }
    this->stepsInRotation = stepsInRotation;
    return *this;
}

StepperDriverBuilder &StepperDriverBuilder::setInitialRPM(const uint64_t initialRPM) {
    this->initialRPM = initialRPM;
    return *this;
}

StepperDriverBuilder &StepperDriverBuilder::setMaxSafeRPM(const uint64_t maxSafeRPM) {
    this->maxSafeRPM = maxSafeRPM;
    return *this;
}

StepperDriver *StepperDriverBuilder::build() const {
    if (enableTerminal == nullptr || coil1Terminal1 == nullptr || coil2Terminal1 == nullptr || coil1Terminal2 == nullptr || coil2Terminal2 == nullptr || stepsInRotation == 0) {
        throw IllegalStateError("Enable terminal, all 4 coil terminals should be initialized, and the stepsInRotation for the motor must be specified before the builder can build the StepperDriver.");
    }

    if (initialRPM > maxSafeRPM) {
        throw IllegalStateError("initialRPM must be <= maxSafeRPM");
    }

    return new StepperDriver(enableTerminal, coil1Terminal1, coil2Terminal1, coil1Terminal2, coil2Terminal2, stepsInRotation, initialRPM, maxSafeRPM);
}


StepperDriver::StepperDriver(DigitalSignalConsumer *enableTerminal,
                             DigitalSignalConsumer *coil1Terminal1,
                             DigitalSignalConsumer *coil2Terminal1,
                             DigitalSignalConsumer *coil1Terminal2,
                             DigitalSignalConsumer *coil2Terminal2,
                             const uint64_t stepsInRotation,
                             const uint64_t initialRPM,
                             const uint64_t maxSafeRPM) :

    enableTerminal(enableTerminal),
    coilTerminals { coil1Terminal1, coil2Terminal1, coil1Terminal2, coil2Terminal2 },
    stepsInRotation(stepsInRotation),
    rpm(initialRPM),
    maxSafeRPM(maxSafeRPM),
    interrupted(false),
    nextWaveformStep(0),
    nextRotationStep(0) {
}

StepperDriver::~StepperDriver() {
    enableTerminal->write(false);
    for (uint8_t i = 0; i < 4; ++i) {
        coilTerminals[i]->write(false);
    }
}

void StepperDriver::interrupt() {
    unique_lock<shared_mutex> lock(interruptMutex);
    enableTerminal->write(false);
    interrupted = true;
}

bool StepperDriver::isInterrupted() {
    shared_lock<shared_mutex> lock(interruptMutex);
    return interrupted;
}

bool StepperDriver::setRPM(const uint64_t rpm) {
    if (rpm >= maxSafeRPM) {
        return false;
    }
    this->rpm = rpm;
    return true;
}

uint64_t StepperDriver::getRPM() const {
    return rpm;
}

uint64_t StepperDriver::getMaxSafeRPM() const {
    return maxSafeRPM;
}

uint64_t StepperDriver::getStepsInRotation() const {
    return stepsInRotation;
}

template <typename T>
static void moddedStepUInt(T &num, const RotationDirection direction, const T maxValue) {
    bool increment;
    switch (direction) {
        case CLOCKWISE:
            increment = false;
            break;
        case COUNTER_CLOCKWISE:
            increment = true;
            break;
        default:
            throw IllegalStateError("Unknown RotationDirection value");
            break;
    }

    if (increment) {
        num = (num + 1) % maxValue;
    } else if (num == 0) {
        num = maxValue - 1;
    } else {
        num = num - 1;
    }
}

bool StepperDriver::driveWaveform(const uint64_t steps, const RotationDirection direction) {
    static const uint8_t baseWaveform = 0b0000'1100;

    for (uint64_t i = 0; i < steps; ++i) {
        if (isInterrupted() || !adjustSpeed()) {
            return false;
        }

        // Do a right bit shift by "nextWaveformStep" steps on "baseWaveform", wrapping around only on the last 4 bits.
        const uint8_t valueToBeWritten = (baseWaveform >> nextWaveformStep) | (((baseWaveform << (8 - nextWaveformStep)) & 0b1111'0000) >> 4);

        for (uint8_t i = 0; i < 4; ++i) {
            coilTerminals[i]->write(valueToBeWritten & (0b0000'1000 >> i));
        }

        moddedStepUInt(nextWaveformStep, direction, (uint8_t)4);
        moddedStepUInt(nextRotationStep, direction, stepsInRotation);
    }

    return true;
}

bool StepperDriver::step(const uint64_t steps, const RotationDirection direction) {
    {
        unique_lock<shared_mutex> lock(interruptMutex);
        interrupted = false;
    }
    enableTerminal->write(true);
    const bool completed = driveWaveform(steps, direction);
    enableTerminal->write(false);
    return completed;
}

double StepperDriver::getPositionInDegrees() const {
    return ((double) (nextRotationStep * 360)) / ((double) stepsInRotation);
}

bool StepperDriver::rotateBy(const double angleInDegrees, const RotationDirection direction) {
    const int64_t steps = (int64_t) (angleInDegrees * stepsInRotation)/360;

    RotationDirection correctedDirection = direction;

    if (steps < 0) {
        switch (direction) {
            case CLOCKWISE:
                correctedDirection = COUNTER_CLOCKWISE;
                break;
            case COUNTER_CLOCKWISE:
                correctedDirection = CLOCKWISE;
                break;
            default:
                throw IllegalStateError("Unknown RotationDirection value");
                break;
        }
    }

    return step((uint64_t) ABS(steps), correctedDirection);
}

/*
    Let the delay be x us; stepsInRotation = s

    s steps == 360 degrees. Therefore, => 1 step == 360/s degrees ... (1)

    1 step takes x us. Therefore, by (1), 360/s degrees take x us.
    Therefore, angular velocity
        = 360/(sx) degrees/us
        == 1/(sx) rotations/us
        == (10^6) * 60/(sx) RPM

    Therefore, rpm = 60 * 1000 * 1000/(sx)
        => x = 60'000'000/(rpm*s)
*/

bool StepperDriver::adjustSpeed() {
    if (rpm == 0) {
        return false;
    }
    sleep_for(microseconds((60'000'000/(rpm * stepsInRotation))));
    return true;
}

void StepperDriver::drive(const RotationDirection direction) {
    {
        unique_lock<shared_mutex> lock(interruptMutex);
        interrupted = false;
    }
    enableTerminal->write(true);
    while (driveWaveform(1, direction)) {
    }
    enableTerminal->write(false);
}

}