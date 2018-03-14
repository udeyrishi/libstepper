#include <csignal>
#include <iostream>
#include <gpio.hpp>
#include <constants.hpp>
#include <stepper.hpp>
#include <functional>
#include <wiringPi.h>

static void sigintHandler(int32_t);

using namespace std;
using namespace libstepper;

static function<void(void)> shutdownHandler;

int32_t main() {
    signal(SIGINT, sigintHandler);

    try {
        setupGPIO();
    } catch (const GPIOSetupError &err) {
        cout << err.what() << endl;
        return -1;
    }

    PiDigitalSignalOut enableTerminal(MOTOR1_EN_OUT_PIN);
    PiDigitalSignalOut coil1Terminal1(COIL_A1_PIN);
    PiDigitalSignalOut coil1Terminal2(COIL_A2_PIN);
    PiDigitalSignalOut coil2Terminal1(COIL_B1_PIN);
    PiDigitalSignalOut coil2Terminal2(COIL_B2_PIN);

    StepperDriver *driver = StepperDriverBuilder()
        .setEnableTerminal(enableTerminal)
        .setCoil1Terminal1(coil1Terminal1)
        .setCoil1Terminal2(coil1Terminal2)
        .setCoil2Terminal1(coil2Terminal1)
        .setCoil2Terminal2(coil2Terminal2)
        .setRotationStepCount(MOTOR1_STEPS_PER_ROTATION)
        .setMaxSafeRPM(MOTOR1_MAX_SAFE_RPM)
        .setInitialRPM(200)
        .build();

    shutdownHandler = [driver]() {
        cout << "Now exiting..." << endl;
        driver->interrupt();
    };

    cout << "libstepper-demo is running! Press CTRL+C to quit." << endl;

    driver->drive(COUNTER_CLOCKWISE);
    delete driver;
    return 0;
}

static void sigintHandler(int32_t signal) {
    if (signal == SIGINT) {
        shutdownHandler();
    }
}
