#include <iostream>
#include <cassert>
#include <cmath>
#include <set>
#include <limits>

#include "util.hpp"

static void test_clamp() {
    std::cout << "[test] util::clamp\n";

    assert(util::clamp(5.0, 0.0, 10.0) == 5.0);   // inside range: unchanged
    assert(util::clamp(-5.0, 0.0, 10.0) == 0.0);  // below min: clamped up
    assert(util::clamp(15.0, 0.0, 10.0) == 10.0); // above max: clamped down
    assert(util::clamp(0.0, 0.0, 10.0) == 0.0);   // exactly at min: unchanged
    assert(util::clamp(10.0, 0.0, 10.0) == 10.0); // exactly at max: unchanged
    assert(util::clamp(-127.0, -127.0, 127.0) == -127.0);
    assert(util::clamp(200.0, -127.0, 127.0) == 127.0);   // e.g. arcade-mix overshoot
    assert(util::clamp(-200.0, -127.0, 127.0) == -127.0);
}

static void test_sgn() {
    std::cout << "[test] util::sgn\n";

    assert(util::sgn(5.0) == 1);
    assert(util::sgn(0.001) == 1);
    assert(util::sgn(-5.0) == -1);
    assert(util::sgn(-0.001) == -1);
    assert(util::sgn(0.0) == 0);
}

static void test_deadband() {
    std::cout << "[test] util::deadband\n";

    // Default threshold is 5: values with |value| < 5 are zeroed.
    assert(util::deadband(0) == 0);
    assert(util::deadband(3) == 0);
    assert(util::deadband(-3) == 0);
    assert(util::deadband(5) == 5);   // boundary: strictly less-than, so == threshold passes through
    assert(util::deadband(-5) == -5);
    assert(util::deadband(127) == 127);
    assert(util::deadband(-127) == -127);

    // custom threshold
    assert(util::deadband(15, 20) == 0);
    assert(util::deadband(25, 20) == 25);
}

static void test_rand_range() {
    std::cout << "[test] util::randRange\n";

    util::fun(); // seeds the RNG

    // Invalid range (max <= min) returns min instead of dividing by zero or
    // producing an out-of-order range.
    assert(util::randRange(10, 10) == 10);
    assert(util::randRange(10, 5) == 10);

    // Bounds hold across many draws, and a small range should hit more than
    // one distinct value (not just always returning the same number).
    std::set<int> seen;
    for (int i = 0; i < 500; i++) {
        int value = util::randRange(1, 6);
        assert(value >= 1 && value <= 6);
        seen.insert(value);
    }
    std::cout << "  distinct values seen in [1,6] over 500 draws: " << seen.size() << "\n";
    assert(seen.size() > 1);
}

static void test_expo() {
    std::cout << "[test] util::expo\n";

    assert(util::expo(0) == 0);
    assert(util::expo(127, 0.0) == 127); // linear passes through
    assert(util::expo(-127, 0.0) == -127);
    // expo reduces mid-stick output for fine control but keeps endpoints
    int mid_linear = util::expo(64, 0.0);
    int mid_expo = util::expo(64, 0.4);
    assert(mid_expo < mid_linear);
    assert(mid_expo > 0);
    assert(util::expo(127, 0.4) == 127);
    assert(util::expo(-127, 0.4) == -127);
}

static void test_slew() {
    std::cout << "[test] util::slew\n";

    assert(util::slew(0, 100, 10) == 10);   // rate-limited
    assert(util::slew(0, 5, 10) == 5);      // small step passes through
    assert(util::slew(0, -100, 10) == -10);
    assert(util::slew(50, 50, 10) == 50);
    assert(util::slew(0, 100, 0) == 0);     // zero rate = frozen
}

static void test_accel_limit() {
    std::cout << "[test] util::accelLimit\n";

    assert(util::accelLimit(0, 100, 10) == 10);     // speeding up is rate-limited
    assert(util::accelLimit(0, -100, 10) == -10);
    assert(util::accelLimit(120, 0, 10) == 0);      // releasing the stick stops instantly
    assert(util::accelLimit(120, 40, 10) == 40);    // slowing down is instant
    assert(util::accelLimit(-120, -40, 10) == -40);
    assert(util::accelLimit(120, -100, 10) == -10); // reversing: through 0, then ramp
    assert(util::accelLimit(-5, 3, 10) == 3);
    assert(util::accelLimit(50, 50, 10) == 50);
}


// The cascade limiter compared the raw encoder reading against the limits
// directly. An unplugged or unconfigured motor doesn't report a position -
// get_position() returns PROS_ERR_F, which is infinity - and infinity is
// >= any maximum, so raising was silently blocked for the whole match while
// the driver held the button and nothing happened.
//
// A travel limit exists to protect the mechanism from a position it has
// actually reached. A reading that is known to be broken says nothing about
// where the lift is, so the only safe answer is to hand control back to the
// driver, who can see the robot.
static void test_limit_travel() {
    std::cout << "[test] util::limitTravel\n";

    const double MIN = 0.0, MAX = 1000.0;

    // Mid-travel: nothing is restricted.
    assert(util::limitTravel(127, 500.0, MIN, MAX) == 127);
    assert(util::limitTravel(-127, 500.0, MIN, MAX) == -127);

    // At the top: can't go further up, can still come down.
    assert(util::limitTravel(127, 1000.0, MIN, MAX) == 0);
    assert(util::limitTravel(-127, 1000.0, MIN, MAX) == -127);
    assert(util::limitTravel(127, 1200.0, MIN, MAX) == 0); // overshot
    assert(util::limitTravel(-127, 1200.0, MIN, MAX) == -127);

    // At the bottom: can't go further down, can still go up. A mechanism
    // must never be able to trap itself at a limit.
    assert(util::limitTravel(-127, 0.0, MIN, MAX) == 0);
    assert(util::limitTravel(127, 0.0, MIN, MAX) == 127);
    assert(util::limitTravel(-127, -50.0, MIN, MAX) == 0); // undershot
    assert(util::limitTravel(127, -50.0, MIN, MAX) == 127);

    // Zero stays zero wherever it is.
    assert(util::limitTravel(0, 500.0, MIN, MAX) == 0);

    // THE BUG: a broken reading must not block the mechanism.
    const double unplugged = std::numeric_limits<double>::infinity();
    std::cout << "  unplugged motor (reads infinity): raise="
              << util::limitTravel(127, unplugged, MIN, MAX)
              << " lower=" << util::limitTravel(-127, unplugged, MIN, MAX)
              << " (both must pass through)\n";
    assert(util::limitTravel(127, unplugged, MIN, MAX) == 127 &&
           "an unplugged motor must not disable raising");
    assert(util::limitTravel(-127, unplugged, MIN, MAX) == -127);

    assert(util::limitTravel(127, -unplugged, MIN, MAX) == 127);
    assert(util::limitTravel(127, std::numeric_limits<double>::quiet_NaN(), MIN, MAX) == 127);
}

int main() {
    std::cout << "Host test: util\n";

    test_clamp();
    test_sgn();
    test_deadband();
    test_expo();
    test_slew();
    test_accel_limit();
    test_rand_range();
    test_limit_travel();

    std::cout << "All util tests passed.\n";
    return 0;
}
