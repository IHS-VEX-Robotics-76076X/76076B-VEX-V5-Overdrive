#pragma once

// ---------------------------------------------------------------------------
// CASCADE LIFT SYNC
// ---------------------------------------------------------------------------
//
// Decides, every 20ms loop, what each of the lift's two motors should do so
// that they move at the same speed and stop at the same place.
//
// This file has no PROS in it on purpose - it only does the maths. opcontrol
// reads the encoders, calls Sync::update(), and sends the result to the
// motors. Keeping the decision separate from the hardware is what lets
// tests/test_lift.cpp check it on a laptop, the same way menu.hpp works.
//
// WHY THIS EXISTS
//
// The lift used to be driven with cascade_motors.move(speed), and released
// with move(0). Two problems with that:
//
//   1. move() is a raw VOLTAGE. Two motors given the same voltage only spin
//      at the same speed if they carry exactly the same load and friction,
//      which the two sides of a real lift never do.
//   2. move(0) does NOT use the brake mode. The PROS docs say so outright:
//      "This function will not respect brake modes, and simply sets the
//      voltage to the desired value." So the HOLD set in initialize() never
//      kicked in during driver control. Letting go of R1/R2 put 0 volts on
//      both motors and each one coasted until its own friction stopped it -
//      which is why port 5 kept running after port 6 had stopped.
//
// What happens now:
//
//   DRIVING (R1 or R2 held)
//     Each motor gets a SPEED target in RPM (move_velocity), not a voltage.
//     The motor's own speed controller then pushes as hard as it needs to
//     hit that speed, whatever load its side is carrying.
//
//     On top of that, the two encoders are compared every loop. If one side
//     has got ahead, it is slowed down and the other is sped up until they
//     agree again. That catches anything the speed controllers miss - a side
//     that maxed out its power, or one that started a few ms late.
//
//   STOPPING (just let go of R1/R2)
//     Both motors are told to brake to zero speed straight away - actively,
//     not 0 volts - with the encoder comparison still running. If one motor
//     takes longer to stop than its partner, the gap that opens up makes the
//     quicker one follow it instead of stopping early and leaving the other
//     running on. That is what makes them stop at the same time, not just
//     at the same place.
//
//   HOLDING (stopped)
//     Once both motors have actually stopped (or after a short timeout),
//     both are sent to ONE shared position: the average of the two encoders
//     at that moment. Each motor's position controller then holds that point
//     against gravity. Same target for both = they rest at the same height.
//
//   At a travel limit the braking phase is skipped and the lift holds
//   straight away (clamped to the limit), so it never runs on past the end.
// ---------------------------------------------------------------------------

namespace lift {

struct Settings {
    // Top speed of the cartridge, in RPM. 200 for green.
    double maxRpm;

    // How hard the two sides are pulled back into line while driving: RPM of
    // correction per degree they disagree by. The leading motor is slowed by
    // this much and the trailing one sped up by the same amount.
    double syncKp;

    // Most correction ever applied to one motor, in RPM. Stops a broken
    // encoder reading (a huge "disagreement") from flinging one side.
    double syncMaxRpm;

    // Disagreements this small (degrees) are ignored. Gears and chains have
    // a few degrees of slop, and chasing that would make the motors fight
    // each other for nothing.
    double syncDeadbandDeg;

    // While braking, a motor counts as stopped once it is turning slower
    // than this many RPM. When both have stopped, the hold point is latched.
    double stopSettledRpm;

    // Latch the hold point after this long braking even if the motors still
    // read as moving (e.g. the lift is sliding under its own weight), so the
    // position hold always takes over. Seconds.
    double stopTimeoutSeconds;

    // Length of one opcontrol loop, in seconds. Used to turn how far each
    // motor turned since the last loop into a speed.
    double loopSeconds;

    // Travel limits, so a hold can never park the lift outside them. Only
    // used when limitEnabled is true.
    bool limitEnabled;
    double minDeg;
    double maxDeg;
};

struct Command {
    enum class Mode {
        DRIVE, // spin each motor at rpm[i] (move_velocity)
        HOLD,  // send both motors to holdDeg (move_absolute) and hold it
        BRAKE, // no usable encoder at all: stop with the motor brake mode
    };

    Mode mode = Mode::BRAKE;
    double rpm[2] = {0.0, 0.0}; // DRIVE only. Index 0/1 = CASCADE_MOTOR_PORTS[0]/[1]. + = up.
    double holdDeg = 0.0;       // HOLD only. Same target for both motors.
};

// The lift's height, in encoder degrees: the average of both motors.
//
// If one motor isn't answering (unplugged motors report infinity, not a
// number) its reading is ignored and the other is used on its own. If
// neither is answering, the result is not finite - check with std::isfinite.
double position(double posA, double posB);

// Per-motor speeds for one driving loop. targetRpm is what the driver asked
// for; the two outputs add the sync correction. Neither output ever exceeds
// maxRpm: if the correction would push one past it, both are shifted down
// together so the gap between them - the part that fixes the sync - is kept.
//
// If either position isn't a finite number, no correction is applied (both
// get targetRpm). Same fail-open rule as util::limitTravel: a reading known
// to be broken says nothing about which side is ahead.
void driveRpm(double targetRpm, double posA, double posB,
              const Settings &settings, double out[2]);

class Sync {
public:
    explicit Sync(const Settings &settings) : settings_(settings) {}

    // input: the driver's lift command, -127..127, + = up, AFTER the travel
    // limits have been applied (so 0 at a limit means "stop here").
    // posA/posB: the two encoders in degrees, + = up.
    // atLimit: true when input is 0 only because a travel limit blocked the
    // driver. Skips the braking phase and holds at once, clamped to the limit.
    Command update(int input, double posA, double posB, bool atLimit = false);

    // Forget the hold point, so the next stop latches a fresh one from
    // wherever the lift is. Call this after anything else has moved the lift
    // (e.g. a test routine), or the hold would drag it back to where it was.
    void reset();

private:
    enum class Phase { MOVING, STOPPING, HOLDING };

    // Latch the hold point (once) and return the HOLD command, or BRAKE if
    // there is no working encoder to latch from yet.
    Command hold(double here);

    Settings settings_;
    Phase phase_ = Phase::HOLDING; // first loop latches a hold where the lift is
    bool latched_ = false;         // HOLDING: has holdDeg_ been set yet?
    double holdDeg_ = 0.0;
    double stoppingSeconds_ = 0.0; // STOPPING: how long we've been braking
    double prevA_ = 0.0, prevB_ = 0.0; // last loop's encoders, for measuring speed
    bool havePrev_ = false;
};

} // namespace lift
