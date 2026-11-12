#pragma once

// Small general-purpose helpers shared across the codebase. Definitions in util.cpp.
namespace util{
    void fun();

    int randRange(int min, int max);

    int sgn(double value);

    double clamp(double value, double min, double max);

    int deadband(int joystickValue, int threshold = 5); // joystick values within +/-threshold of 0 are treated as 0

    int expo(int joystickValue, double gain = 0.4); // cubic blend for fine center control: 0 = linear
    int slew(int current, int target, int maxDelta); // rate-limit output change per loop (tip/descore guard)
    int accelLimit(int current, int target, int maxDelta); // slew only when speeding up; slowing down is instant

    // Stops a mechanism driving past the ends of its travel.
    //
    // Returns speed unchanged unless it would push further past a limit it
    // has already reached - then 0. Moving back toward the middle is always
    // allowed, so a mechanism can never trap itself at a limit.
    //
    // FAILS OPEN. A position that isn't a finite number means the sensor
    // didn't answer - a disconnected motor reports infinity, not a number -
    // and in that case the speed passes through untouched. Blocking instead
    // would mean one flaky cable silently disables the mechanism for the
    // rest of the match, with the driver holding the button and nothing
    // happening. A human watching the robot is a better backstop than a
    // reading that is known to be wrong.
    int limitTravel(int speed, double position, double minPosition, double maxPosition);
}