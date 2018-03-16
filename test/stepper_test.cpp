/**
 Copyright (c) 2018 Udey Rishi. All rights reserved.
*/

#include <catch.hpp>
#include <stepper.hpp>
#include <exception.hpp>
#include <vector>
#include <chrono>
#include <thread>
#include <iostream>
#include <cmath>
#include <limits>
#include <functional>

using namespace std;
using namespace libstepper;
using namespace std::chrono;

class SignalRecorder : public DigitalSignalConsumer {
public:
    void write(bool value) {
        values.push_back(value);
    }

    vector<bool> values;
};

#define BUILD_DRIVER(rotationStepCount, initialRPM)                 \
    auto a1 = SignalRecorder();                                     \
    auto a2 = SignalRecorder();                                     \
    auto b1 = SignalRecorder();                                     \
    auto b2 = SignalRecorder();                                     \
    auto en = SignalRecorder();                                     \
    auto driver = StepperDriverBuilder()                            \
        .setCoil1Terminal1(a1)                                      \
        .setCoil1Terminal2(a2)                                      \
        .setCoil2Terminal1(b1)                                      \
        .setCoil2Terminal2(b2)                                      \
        .setEnableTerminal(en)                                      \
        .setRotationStepCount(rotationStepCount)                    \
        .setInitialRPM(initialRPM)                                  \
        .build();                                                   \

#define ARE_CLOSE(a, b) (abs((double)(a) - (double)(b)) < numeric_limits<double>::epsilon())

TEST_CASE("StepperDriverBuilder configures the StepperDriver correctly", "[StepperDriverBuilder]") {
    auto builder = StepperDriverBuilder();

    SECTION("Checks for mandatory members to be set when build() is called") {

        REQUIRE_THROWS_AS(builder.build(), IllegalStateError);

        auto a1 = SignalRecorder();
        builder.setCoil1Terminal1(a1);
        REQUIRE_THROWS_AS(builder.build(), IllegalStateError);

        auto a2 = SignalRecorder();
        builder.setCoil1Terminal2(a2);
        REQUIRE_THROWS_AS(builder.build(), IllegalStateError);

        auto b1 = SignalRecorder();
        builder.setCoil2Terminal1(b1);
        REQUIRE_THROWS_AS(builder.build(), IllegalStateError);

        auto b2 = SignalRecorder();
        builder.setCoil2Terminal2(b2);
        REQUIRE_THROWS_AS(builder.build(), IllegalStateError);

        auto en = SignalRecorder();
        builder.setEnableTerminal(en);
        REQUIRE_THROWS_AS(builder.build(), IllegalStateError);

        builder.setRotationStepCount(150);
        {
            StepperDriver *d = nullptr;
            REQUIRE_NOTHROW(d = builder.build());
            if (d != nullptr) {
                delete d;
            }
        }

        SECTION("Uses sane defaults for non-mandatory members") {
            auto driver = builder.build();

            REQUIRE(driver->getRPM() == 0);
            REQUIRE(driver->getMaxSafeRPM() == UINT64_MAX);
            delete driver;
        }

        SECTION("Checks for max safe RPM") {
            builder.setMaxSafeRPM(200);
            builder.setInitialRPM(250);

            REQUIRE_THROWS_AS(builder.build(), IllegalStateError);
        }

        SECTION("Configures members correctly") {
            builder.setMaxSafeRPM(200);
            builder.setInitialRPM(50);

            auto driver = builder.build();

            REQUIRE(driver->getRPM() == 50);
            REQUIRE(driver->getMaxSafeRPM() == 200);
            REQUIRE(driver->getStepsInRotation() == 150);
            delete driver;
        }
    }
}

TEST_CASE("Driver does nothing if rpm is zero", "[StepperDriver]") {
    BUILD_DRIVER(603, 0);

    SECTION("Counter clockwise step does nothing when RPM == 0") {
        REQUIRE(!driver->step(45, COUNTER_CLOCKWISE));
        REQUIRE(en.values.size() == 2);
        REQUIRE(a1.values.size() == 0);
        REQUIRE(a2.values.size() == 0);
        REQUIRE(b1.values.size() == 0);
        REQUIRE(b2.values.size() == 0);
    }

    SECTION("Clockwise step does nothing when RPM == 0") {
        REQUIRE(!driver->step(45, CLOCKWISE));
        REQUIRE(en.values.size() == 2);
        REQUIRE(a1.values.size() == 0);
        REQUIRE(a2.values.size() == 0);
        REQUIRE(b1.values.size() == 0);
        REQUIRE(b2.values.size() == 0);
    }

    SECTION("Counter clockwise rotateBy does nothing when RPM == 0") {
        REQUIRE(!driver->rotateBy(45.0, COUNTER_CLOCKWISE));
        REQUIRE(en.values.size() == 2);
        REQUIRE(a1.values.size() == 0);
        REQUIRE(a2.values.size() == 0);
        REQUIRE(b1.values.size() == 0);
        REQUIRE(b2.values.size() == 0);
    }

    SECTION("Clockwise rotateBy does nothing when RPM == 0") {
        REQUIRE(!driver->rotateBy(45.0, CLOCKWISE));
        REQUIRE(en.values.size() == 2);
        REQUIRE(a1.values.size() == 0);
        REQUIRE(a2.values.size() == 0);
        REQUIRE(b1.values.size() == 0);
        REQUIRE(b2.values.size() == 0);
    }

    SECTION("Counter clockwise drive does nothing when RPM == 0") {
        driver->drive(COUNTER_CLOCKWISE);
        REQUIRE(en.values.size() == 2);
        REQUIRE(a1.values.size() == 0);
        REQUIRE(a2.values.size() == 0);
        REQUIRE(b1.values.size() == 0);
        REQUIRE(b2.values.size() == 0);
    }

    SECTION("Clockwise drive does nothing when RPM == 0") {
        driver->drive(CLOCKWISE);
        REQUIRE(en.values.size() == 2);
        REQUIRE(a1.values.size() == 0);
        REQUIRE(a2.values.size() == 0);
        REQUIRE(b1.values.size() == 0);
        REQUIRE(b2.values.size() == 0);
    }

    delete driver;
}

TEST_CASE("StepperDriver::step drivers correct waveforms", "[StepperDriver::step]") {
    BUILD_DRIVER(603, 23);

    const size_t stepCount = 45;

    SECTION("Counter clockwise step works when RPM > 0") {
        REQUIRE(driver->step(stepCount, COUNTER_CLOCKWISE));

        REQUIRE(en.values.size() == 2);
        REQUIRE(a1.values.size() == stepCount);
        REQUIRE(a2.values.size() == stepCount);
        REQUIRE(b1.values.size() == stepCount);
        REQUIRE(b2.values.size() == stepCount);

        REQUIRE(en.values[0]);
        REQUIRE(!en.values[1]);

        for (size_t i = 0; i < stepCount; ++i) {
            REQUIRE(a1.values[i]);
            REQUIRE(b1.values[i]);
            REQUIRE(!a2.values[i]);
            REQUIRE(!b2.values[i]);

            if (++i == stepCount) break;
            REQUIRE(!a1.values[i]);
            REQUIRE(b1.values[i]);
            REQUIRE(a2.values[i]);
            REQUIRE(!b2.values[i]);

            if (++i == stepCount) break;
            REQUIRE(!a1.values[i]);
            REQUIRE(!b1.values[i]);
            REQUIRE(a2.values[i]);
            REQUIRE(b2.values[i]);

            if (++i == stepCount) break;
            REQUIRE(a1.values[i]);
            REQUIRE(!b1.values[i]);
            REQUIRE(!a2.values[i]);
            REQUIRE(b2.values[i]);
        }
    }

    SECTION("Clockwise step works when RPM > 0") {
        REQUIRE(driver->step(stepCount, CLOCKWISE));

        REQUIRE(en.values.size() == 2);
        REQUIRE(a1.values.size() == stepCount);
        REQUIRE(a2.values.size() == stepCount);
        REQUIRE(b1.values.size() == stepCount);
        REQUIRE(b2.values.size() == stepCount);

        REQUIRE(en.values[0]);
        REQUIRE(!en.values[1]);

        for (size_t i = 0; i < stepCount; ++i) {
            REQUIRE(a1.values[i]);
            REQUIRE(b1.values[i]);
            REQUIRE(!a2.values[i]);
            REQUIRE(!b2.values[i]);

            if (++i == stepCount) break;
            REQUIRE(a1.values[i]);
            REQUIRE(!b1.values[i]);
            REQUIRE(!a2.values[i]);
            REQUIRE(b2.values[i]);

            if (++i == stepCount) break;
            REQUIRE(!a1.values[i]);
            REQUIRE(!b1.values[i]);
            REQUIRE(a2.values[i]);
            REQUIRE(b2.values[i]);

            if (++i == stepCount) break;
            REQUIRE(!a1.values[i]);
            REQUIRE(b1.values[i]);
            REQUIRE(a2.values[i]);
            REQUIRE(!b2.values[i]);
        }
    }

    delete driver;
}

TEST_CASE("StepperDriver::rotateBy drives correct waveform", "[StepperDriver::rotateBy]") {
    BUILD_DRIVER(603, 23);

    const double angle = 65.0;
    const uint64_t stepCount = (uint64_t)(angle * driver->getStepsInRotation())/360;

    SECTION("Positive angle with clockwise direction works") {
        REQUIRE(driver->rotateBy(angle, CLOCKWISE));

        REQUIRE(en.values.size() == 2);
        REQUIRE(a1.values.size() == stepCount);
        REQUIRE(a2.values.size() == stepCount);
        REQUIRE(b1.values.size() == stepCount);
        REQUIRE(b2.values.size() == stepCount);

        REQUIRE(en.values[0]);
        REQUIRE(!en.values[1]);

        for (size_t i = 0; i < stepCount; ++i) {
            REQUIRE(a1.values[i]);
            REQUIRE(b1.values[i]);
            REQUIRE(!a2.values[i]);
            REQUIRE(!b2.values[i]);

            if (++i == stepCount) break;
            REQUIRE(a1.values[i]);
            REQUIRE(!b1.values[i]);
            REQUIRE(!a2.values[i]);
            REQUIRE(b2.values[i]);

            if (++i == stepCount) break;
            REQUIRE(!a1.values[i]);
            REQUIRE(!b1.values[i]);
            REQUIRE(a2.values[i]);
            REQUIRE(b2.values[i]);

            if (++i == stepCount) break;
            REQUIRE(!a1.values[i]);
            REQUIRE(b1.values[i]);
            REQUIRE(a2.values[i]);
            REQUIRE(!b2.values[i]);
        }
    }

    SECTION("Positive angle with counter-clockwise direction works") {
        REQUIRE(driver->rotateBy(angle, COUNTER_CLOCKWISE));

        REQUIRE(en.values.size() == 2);
        REQUIRE(a1.values.size() == stepCount);
        REQUIRE(a2.values.size() == stepCount);
        REQUIRE(b1.values.size() == stepCount);
        REQUIRE(b2.values.size() == stepCount);

        REQUIRE(en.values[0]);
        REQUIRE(!en.values[1]);

        for (size_t i = 0; i < stepCount; ++i) {
            REQUIRE(a1.values[i]);
            REQUIRE(b1.values[i]);
            REQUIRE(!a2.values[i]);
            REQUIRE(!b2.values[i]);

            if (++i == stepCount) break;
            REQUIRE(!a1.values[i]);
            REQUIRE(b1.values[i]);
            REQUIRE(a2.values[i]);
            REQUIRE(!b2.values[i]);

            if (++i == stepCount) break;
            REQUIRE(!a1.values[i]);
            REQUIRE(!b1.values[i]);
            REQUIRE(a2.values[i]);
            REQUIRE(b2.values[i]);

            if (++i == stepCount) break;
            REQUIRE(a1.values[i]);
            REQUIRE(!b1.values[i]);
            REQUIRE(!a2.values[i]);
            REQUIRE(b2.values[i]);
        }
    }

    SECTION("Negative angle with clockwise direction works") {
        REQUIRE(driver->rotateBy(-angle, CLOCKWISE));

        REQUIRE(en.values.size() == 2);
        REQUIRE(a1.values.size() == stepCount);
        REQUIRE(a2.values.size() == stepCount);
        REQUIRE(b1.values.size() == stepCount);
        REQUIRE(b2.values.size() == stepCount);

        REQUIRE(en.values[0]);
        REQUIRE(!en.values[1]);

        for (size_t i = 0; i < stepCount; ++i) {
            REQUIRE(a1.values[i]);
            REQUIRE(b1.values[i]);
            REQUIRE(!a2.values[i]);
            REQUIRE(!b2.values[i]);

            if (++i == stepCount) break;
            REQUIRE(!a1.values[i]);
            REQUIRE(b1.values[i]);
            REQUIRE(a2.values[i]);
            REQUIRE(!b2.values[i]);

            if (++i == stepCount) break;
            REQUIRE(!a1.values[i]);
            REQUIRE(!b1.values[i]);
            REQUIRE(a2.values[i]);
            REQUIRE(b2.values[i]);

            if (++i == stepCount) break;
            REQUIRE(a1.values[i]);
            REQUIRE(!b1.values[i]);
            REQUIRE(!a2.values[i]);
            REQUIRE(b2.values[i]);
        }
    }

    SECTION("Negative angle with counter-clockwise direction works") {
        REQUIRE(driver->rotateBy(-angle, COUNTER_CLOCKWISE));

        REQUIRE(en.values.size() == 2);
        REQUIRE(a1.values.size() == stepCount);
        REQUIRE(a2.values.size() == stepCount);
        REQUIRE(b1.values.size() == stepCount);
        REQUIRE(b2.values.size() == stepCount);

        REQUIRE(en.values[0]);
        REQUIRE(!en.values[1]);

        for (size_t i = 0; i < stepCount; ++i) {
            REQUIRE(a1.values[i]);
            REQUIRE(b1.values[i]);
            REQUIRE(!a2.values[i]);
            REQUIRE(!b2.values[i]);

            if (++i == stepCount) break;
            REQUIRE(a1.values[i]);
            REQUIRE(!b1.values[i]);
            REQUIRE(!a2.values[i]);
            REQUIRE(b2.values[i]);

            if (++i == stepCount) break;
            REQUIRE(!a1.values[i]);
            REQUIRE(!b1.values[i]);
            REQUIRE(a2.values[i]);
            REQUIRE(b2.values[i]);

            if (++i == stepCount) break;
            REQUIRE(!a1.values[i]);
            REQUIRE(b1.values[i]);
            REQUIRE(a2.values[i]);
            REQUIRE(!b2.values[i]);
        }
    }

    delete driver;
}

TEST_CASE("StepperDriver::drive works", "[StepperDriver::drive]") {
    BUILD_DRIVER(603, 23);

    SECTION("Clockwise drive works") {
        thread drivingThread([driver] {
            driver->drive(CLOCKWISE);
        });

        this_thread::sleep_for(seconds(1));
        driver->interrupt();
        drivingThread.join();

        // We can't deterministically say the number of steps that were taken, because of linux scheduling.
        REQUIRE(en.values.size() == 3);
        auto stepCount = a1.values.size();
        REQUIRE((a2.values.size() == stepCount || a2.values.size() == stepCount - 1 || a2.values.size() == stepCount + 1));
        REQUIRE((b1.values.size() == stepCount || b1.values.size() == stepCount - 1 || b1.values.size() == stepCount + 1));
        REQUIRE((b2.values.size() == stepCount || b2.values.size() == stepCount - 1 || b2.values.size() == stepCount + 1));

        // False is 2x, once by interrupt, and once by the ending drive(). The second one makes sense, because
        // drive() can also end if the RPM is zero (which doesn't do an emergency en->write(false)).
        REQUIRE(en.values[0]);
        REQUIRE(!en.values[1]);
        REQUIRE(!en.values[2]);

        for (size_t i = 0; i < stepCount; ++i) {
            REQUIRE((i >= a1.values.size() || a1.values[i]));
            REQUIRE((i >= b1.values.size() || b1.values[i]));
            REQUIRE((i >= a2.values.size() || !a2.values[i]));
            REQUIRE((i >= b2.values.size() || !b2.values[i]));

            if (++i == stepCount) break;
            REQUIRE((i >= a1.values.size() || a1.values[i]));
            REQUIRE((i >= b1.values.size() || !b1.values[i]));
            REQUIRE((i >= a2.values.size() || !a2.values[i]));
            REQUIRE((i >= b2.values.size() || b2.values[i]));

            if (++i == stepCount) break;
            REQUIRE((i >= a1.values.size() || !a1.values[i]));
            REQUIRE((i >= b1.values.size() || !b1.values[i]));
            REQUIRE((i >= a2.values.size() || a2.values[i]));
            REQUIRE((i >= b2.values.size() || b2.values[i]));

            if (++i == stepCount) break;
            REQUIRE((i >= a1.values.size() || !a1.values[i]));
            REQUIRE((i >= b1.values.size() || b1.values[i]));
            REQUIRE((i >= a2.values.size() || a2.values[i]));
            REQUIRE((i >= b2.values.size() || !b2.values[i]));
        }
    }

    SECTION("Counter-clockwise drive works") {
        thread drivingThread([driver] {
            driver->drive(COUNTER_CLOCKWISE);
        });

        this_thread::sleep_for(seconds(1));
        driver->interrupt();
        drivingThread.join();

        // We can't deterministically say the number of steps that were taken, because of linux scheduling
        REQUIRE(en.values.size() == 3);
        auto stepCount = a1.values.size();
        REQUIRE((a2.values.size() == stepCount || a2.values.size() == stepCount - 1 || a2.values.size() == stepCount + 1));
        REQUIRE((b1.values.size() == stepCount || b1.values.size() == stepCount - 1 || b1.values.size() == stepCount + 1));
        REQUIRE((b2.values.size() == stepCount || b2.values.size() == stepCount - 1 || b2.values.size() == stepCount + 1));

        // False is 2x, once by interrupt, and once by the ending drive(). The second one makes sense, because
        // drive() can also end if the RPM is zero (which doesn't do an emergency en->write(false)).
        REQUIRE(en.values[0]);
        REQUIRE(!en.values[1]);
        REQUIRE(!en.values[2]);

        for (size_t i = 0; i < stepCount; ++i) {
            REQUIRE((i >= a1.values.size() || a1.values[i]));
            REQUIRE((i >= b1.values.size() || b1.values[i]));
            REQUIRE((i >= a2.values.size() || !a2.values[i]));
            REQUIRE((i >= b2.values.size() || !b2.values[i]));

            if (++i == stepCount) break;
            REQUIRE((i >= a1.values.size() || !a1.values[i]));
            REQUIRE((i >= b1.values.size() || b1.values[i]));
            REQUIRE((i >= a2.values.size() || a2.values[i]));
            REQUIRE((i >= b2.values.size() || !b2.values[i]));

            if (++i == stepCount) break;
            REQUIRE((i >= a1.values.size() || !a1.values[i]));
            REQUIRE((i >= b1.values.size() || !b1.values[i]));
            REQUIRE((i >= a2.values.size() || a2.values[i]));
            REQUIRE((i >= b2.values.size() || b2.values[i]));

            if (++i == stepCount) break;
            REQUIRE((i >= a1.values.size() || a1.values[i]));
            REQUIRE((i >= b1.values.size() || !b1.values[i]));
            REQUIRE((i >= a2.values.size() || !a2.values[i]));
            REQUIRE((i >= b2.values.size() || b2.values[i]));
        }
    }

    delete driver;
}

/**
 * A lot of the following tests are using inequalities for getting a rough idea of correctness of the
 * virtual motor's timing vs. position. We can't get any better than this, because:
 * (1) The code isn't performance optimized yet,
 * (2) We're not using an RTOS, and are at the mercy of the whatever scheduling policy the OS is using.
 *
 * TODO: Stricten the inequalities as (1) is made better.
*/

TEST_CASE("StepperDriver::interrupt works", "[StepperDriver::interrupt]") {
    BUILD_DRIVER(200, 60);

    SECTION("interrupt works for step") {
        thread drivingThread([driver] {
            driver->step(1200, COUNTER_CLOCKWISE);
        });

        this_thread::sleep_for(seconds(1));
        driver->interrupt();
        drivingThread.join();

        REQUIRE(en.values.size() == 3);
        REQUIRE(en.values[0]);
        REQUIRE(!en.values[1]);
        REQUIRE(!en.values[2]);

        REQUIRE(a1.values.size() < 200);
        REQUIRE(b1.values.size() < 200);
        REQUIRE(a2.values.size() < 200);
        REQUIRE(b2.values.size() < 200);

        // 1200 steps = 6 rotations. velocity = 1 rotation per second. In 1 second, around 1 rotation = 200 steps.
        // Accounting for scheduling timing issues, should not have exceeded 200 steps.
    }

    SECTION("interrupt works for rotateBy") {
        thread drivingThread([driver] {
            driver->rotateBy(6*360, COUNTER_CLOCKWISE);
        });

        this_thread::sleep_for(seconds(1));
        driver->interrupt();
        drivingThread.join();

        REQUIRE(en.values.size() == 3);
        REQUIRE(en.values[0]);
        REQUIRE(!en.values[1]);
        REQUIRE(!en.values[2]);

        REQUIRE(a1.values.size() < 200);
        REQUIRE(b1.values.size() < 200);
        REQUIRE(a2.values.size() < 200);
        REQUIRE(b2.values.size() < 200);

        // 6 rotations. velocity = 1 rotation per second. In 1 second, around 1 rotation = 200 steps.
        // Accounting for scheduling timing issues, should not have exceeded 200 steps.
    }

    delete driver;

    // No need for interrupt testing for drive, because that's part of the regular drive test
}

static uint64_t timeMilliseconds(function<void(void)> runnable) {
    auto now = system_clock::now();
    runnable();
    return (uint64_t) duration_cast<milliseconds>(system_clock::now() - now).count();
}

TEST_CASE("StepperDriver::setRPM works", "[StepperDriver::setRPM]") {
    BUILD_DRIVER(200, 60);

    SECTION("Setting RPM before step uses the new RPM value") {
        auto duration = timeMilliseconds([driver] {
            driver->setRPM(120);
            driver->step(800, COUNTER_CLOCKWISE);
        });
        REQUIRE(duration < 4000);
    }

    SECTION("Setting RPM before rotateBy uses the new RPM value") {
        auto duration = timeMilliseconds([driver] {
            driver->setRPM(120);
            driver->rotateBy(360*4, COUNTER_CLOCKWISE);
        });
        REQUIRE(duration < 4000);
    }

    SECTION("Setting RPM before drive uses the new RPM value") {
        thread drivingThread([driver] {
            driver->setRPM(120);
            driver->drive(COUNTER_CLOCKWISE);
        });

        this_thread::sleep_for(milliseconds(4000));
        driver->interrupt();
        drivingThread.join();

        REQUIRE(a1.values.size() > 800);
        REQUIRE(b1.values.size() > 800);
        REQUIRE(a2.values.size() > 800);
        REQUIRE(b2.values.size() > 800);
    }

    SECTION("Setting RPM during step increases the speed") {
        auto duration = timeMilliseconds([driver] {
            thread drivingThread([driver] {
                driver->step(800, COUNTER_CLOCKWISE);
            });

            this_thread::sleep_for(milliseconds(1000));
            driver->setRPM(120);
            drivingThread.join();
        });

        REQUIRE(duration < 4000);
    }

    SECTION("Setting RPM during rotateBy increases the speed") {
        auto duration = timeMilliseconds([driver] {
            thread drivingThread([driver] {
                driver->rotateBy(4*360, COUNTER_CLOCKWISE);
            });

            this_thread::sleep_for(milliseconds(1000));
            driver->setRPM(120);
            drivingThread.join();
        });

        REQUIRE(duration < 4000);
    }

    SECTION("Setting RPM during drive increases the speed") {
        thread drivingThread([driver] {
            driver->drive(COUNTER_CLOCKWISE);
        });

        this_thread::sleep_for(milliseconds(1000));
        driver->setRPM(120);
        this_thread::sleep_for(milliseconds(3000));
        driver->interrupt();
        drivingThread.join();

        REQUIRE(a1.values.size() > 800);
        REQUIRE(b1.values.size() > 800);
        REQUIRE(a2.values.size() > 800);
        REQUIRE(b2.values.size() > 800);
    }

    delete driver;
}

TEST_CASE("StepperDriver::getPositionInDegrees works", "[StepperDriver::getPositionInDegrees]") {
    BUILD_DRIVER(200, 600);

    SECTION("getPositionInDegrees works for step") {
        REQUIRE(ARE_CLOSE(driver->getPositionInDegrees(), 0.0));
        driver->step(50, COUNTER_CLOCKWISE);
        REQUIRE(ARE_CLOSE(driver->getPositionInDegrees(), 90.0));
        driver->step(25, CLOCKWISE);
        REQUIRE(ARE_CLOSE(driver->getPositionInDegrees(), 45.0));
        driver->step(50, CLOCKWISE);
        REQUIRE(ARE_CLOSE(driver->getPositionInDegrees(), 360.0 - 45.0));
    }

    SECTION("getPositionInDegrees works for rotateBy") {
        REQUIRE(ARE_CLOSE(driver->getPositionInDegrees(), 0.0));
        driver->rotateBy(90.0, COUNTER_CLOCKWISE);
        REQUIRE(ARE_CLOSE(driver->getPositionInDegrees(), 90.0));
        driver->rotateBy(45.0, CLOCKWISE);
        REQUIRE(ARE_CLOSE(driver->getPositionInDegrees(), 45.0));
        driver->rotateBy(-90.0, COUNTER_CLOCKWISE);
        REQUIRE(ARE_CLOSE(driver->getPositionInDegrees(), 360.0 - 45.0));
    }

    SECTION("getPositionInDegrees works for drive") {
        driver->setRPM(60);

        thread drivingThread([driver] {
            driver->drive(COUNTER_CLOCKWISE);
        });

        this_thread::sleep_for(milliseconds(250));
        driver->interrupt();
        drivingThread.join();

        REQUIRE((driver->getPositionInDegrees() <= 90.0));

        const double minBound = driver->getPositionInDegrees() + 180;
        const double maxBound = driver->getPositionInDegrees();

        drivingThread = thread([driver] {
            driver->drive(CLOCKWISE);
        });

        this_thread::sleep_for(milliseconds(500));
        driver->interrupt();
        drivingThread.join();

        const double newPosition = driver->getPositionInDegrees();
        REQUIRE((newPosition >= 0 && (newPosition < maxBound || newPosition > minBound)));
    }

    delete driver;
}