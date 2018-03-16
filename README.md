# libstepper

[![Build Status](https://travis-ci.org/udeyrishi/libstepper.svg?branch=master)](https://travis-ci.org/udeyrishi/libstepper)

**libstepper** is a cross-platform, hardware agnostic, C++ driver for controlling simple stepper motors. 


## Building

libstepper is a CMake project, and requires a C++17 capable compiler. It has been tested to compile on macOS 10.3.3 (Apple clang 9.0.0), gcc 6.3.0 on Raspbian, and gcc 7.2.0 on Ubuntu. You can either compile it as a regular CMake project, or via the included build script.

```
$ ./build -h
usage: build [-h] [-d] [-c] [-a] [-r]

builds the project

optional arguments:
  -h, --help   show this help message and exit
  -d, --debug  build the debug flavour instead of release
  -c, --clean  clean the flavour's temporary files and the build artifacts
               before building
  -a, --all    also build the test exectuable, besides the static lib
  -r, --run    if the test exectuable is built, also run it
```

Running `build -a` will build the static lib in `out/linux/release/lib/libstepper.so` (for a linux target, for instance). You can then [link](https://stackoverflow.com/a/1705972) the library to your project. The test executable will be present in `out/linux/release/bin/libsteppper-test`.

Alternatively (and preferably), if you use CMake in your project, libstepper's `CMakeLists.txt` exposes the `LIB_STEPPER` build artifact to any parent projects. So you can include libstepper as a child project, and link the static library to your build targets in your `CMakeLists.txt`:

```
target_link_libraries(${MY_TARGET} ${LIB_STEPPER})
```

## Usage

Make sure to put the header files in the `inc` directory somewhere in your include path when linking the library.

* [stepper.hpp]: This is the main header file containing the `StepperDriver` driver class, and the `StepperDriverBuilder` builder class.
* [signal.hpp]: This file just contains the one, single-abstract-method class `DigitalSignalConsumer` that does exactly what the name implies--consume a digital signal. This acts as the interface that connects the `StepperDriver` to your platform's GPIO.
* [exception.hpp]: Contains the exception classes that the driver or the driver builder may throw.

The unit tests in [stepper_test.cpp] do a pretty good job of illustrating how to use the driver. They use a mock, in-memory, `DigitalSignalConsumer` called `SignalRecorder` for showing the usage, and for testing the driver behaviour. 

Your code might look something like this:

```cpp
#include "vendor/libstepper/inc/stepper.hpp"
#include "vendor/libstepper/inc/signal.hpp"
#include "vendor/libstepper/inc/exception.hpp"
#include <MyGPIOLib.hpp> // contains something like "digitalWrite" used below
#include "inc/MyPinConstants.hpp" // contains your GPIO pin constants like the ones used below
#include <thread>
#include <chrono>

using namespace std;
using namespace libstepper;
using namespace std::chrono;

// Convert your GPIO API to DigitalSignalConsumer API, so that libstepper can interface with it
class MyGPIOSignalConsumer : public DigitalSignalConsumer {
public:
    MyGPIOSignalConsumer(const int gpioPinNumber) : gpioPinNumber(gpioPinNumber) {}

    void write(bool value) {
        digitalWrite(gpioPinNumber, value ? 1 : 0);
    }

private:
    const int gpioPinNumber;
};

int main() {
    MyGPIOSignalConsumer en(ENABLE_PIN);
    MyGPIOSignalConsumer a1(MOTOR_COIL1_TERMINAL1_PIN);
    MyGPIOSignalConsumer a2(MOTOR_COIL1_TERMINAL2_PIN);
    MyGPIOSignalConsumer b1(MOTOR_COIL2_TERMINAL1_PIN);
    MyGPIOSignalConsumer b2(MOTOR_COIL2_TERMINAL2_PIN);

    StepperDriver *driver = StepperDriverBuilder()
        .setCoil1Terminal1(a1)
        .setCoil1Terminal2(a2)
        .setCoil2Terminal1(b1)
        .setCoil2Terminal2(b2)
        .setEnableTerminal(en)
        .setRotationStepCount(200) // number of steps in 1 complete rotation for your stepper
        .setInitialRPM(initialRPM) // defaults to 0
        .setMaxSafeRPM(500) // defaults to UINT64_MAX
        .build();

    driver->step(50, CLOCKWISE);
    driver->step(50, COUNTER_CLOCKWISE);

    driver->rotateBy(90, CLOCKWISE);
    driver->rotateBy(-90, COUNTER_CLOCKWISE); // Same as 90 CLOCKWISE

    // Only calculates this theoretically.
    // The actual position would depend on your OS, and its scheduling policies.
    // On 100% guaranteed CPU time (e.g. on an RTOS), this should be very close to the real position.
    double position = driver->getPositionInDegrees();
    // position would be close to 180.0 (barring any double precision errors).

    thread drivingThread([driver] {
        // Would keep spinning indefinitely, until interrupt() is called from any thread.
        driver->drive(CLOCKWISE);
    });

    bool didChange = driver->setRPM(600); // exceeds max safe value. no-op. didChange == false

    this_thread::sleep_for(seconds(1));

    // Will interrupt the movement of the motor by turning off the enable pin, 
    // and stopping the stepper waveform. 
    // Will work for drive(), step(), and rotateBy().
    driver->interrupt();
    drivingThread.join();

    delete driver;
    return 0;
}
```

[stepper.hpp]: ./inc/stepper.hpp
[signal.hpp]: ./inc/signal.hpp
[exception.hpp]: ./inc/exception.hpp
[stepper_test.cpp]: ./test/stepper_test.cpp
