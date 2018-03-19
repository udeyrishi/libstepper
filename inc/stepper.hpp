/**
 Copyright (c) 2018 Udey Rishi. All rights reserved.
*/

#pragma once

#include <stdint.h>
#include <signal.hpp>
#include <mutex>

namespace libstepper {

enum RotationDirection {
    CLOCKWISE,
    COUNTER_CLOCKWISE
};

class StepperDriverBuilder;

class StepperDriver {
public:
    ~StepperDriver();
    StepperDriver(const StepperDriver &rhs) = delete;

    bool step(const uint64_t steps, const RotationDirection direction);
    bool rotateBy(const double angleInDegrees, const RotationDirection direction);
    void drive(const RotationDirection direction);
    void interrupt();

    bool setRPM(const uint64_t rpm);
    uint64_t getRPM() const;
    uint64_t getMaxSafeRPM() const;
    uint64_t getStepsInRotation() const;
    double getPositionInDegrees() const;

    friend class StepperDriverBuilder;

private:
    StepperDriver(DigitalSignalConsumer *enableTerminal,
                  DigitalSignalConsumer *coil1Terminal1,
                  DigitalSignalConsumer *coil2Terminal1,
                  DigitalSignalConsumer *coil1Terminal2,
                  DigitalSignalConsumer *coil2Terminal2,
                  const uint64_t stepsInRotation,
                  const uint64_t initialRPM,
                  const uint64_t maxSafeRPM);

    bool driveWaveform(const uint64_t steps, const RotationDirection direction);
    bool adjustSpeed();
    bool isInterrupted();

    DigitalSignalConsumer *enableTerminal;
    //a1, b1, a2, and b2
    DigitalSignalConsumer *coilTerminals[4];
    const uint64_t stepsInRotation;
    uint64_t rpm;
    const uint64_t maxSafeRPM;
    bool interrupted;
    uint8_t nextWaveformStep;
    uint64_t nextRotationStep;
    mutable std::mutex interruptMutex;
};

class StepperDriverBuilder {
public:
    StepperDriverBuilder();
    StepperDriverBuilder &setEnableTerminal(DigitalSignalConsumer &consumer);
    StepperDriverBuilder &setCoil1Terminal1(DigitalSignalConsumer &consumer);
    StepperDriverBuilder &setCoil2Terminal1(DigitalSignalConsumer &consumer);
    StepperDriverBuilder &setCoil1Terminal2(DigitalSignalConsumer &consumer);
    StepperDriverBuilder &setCoil2Terminal2(DigitalSignalConsumer &consumer);

    StepperDriverBuilder &setRotationStepCount(const uint64_t stepsInRotation);
    StepperDriverBuilder &setInitialRPM(const uint64_t initialRPM);

    StepperDriverBuilder &setMaxSafeRPM(const uint64_t maxSafeRPM);

    StepperDriver *build() const;

private:
    DigitalSignalConsumer *enableTerminal;
    DigitalSignalConsumer *coil1Terminal1;
    DigitalSignalConsumer *coil2Terminal1;
    DigitalSignalConsumer *coil1Terminal2;
    DigitalSignalConsumer *coil2Terminal2;
    uint64_t stepsInRotation;
    uint64_t initialRPM;
    uint64_t maxSafeRPM;
};
}