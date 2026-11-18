/**
 * Driver control: drivetrain (arcade or tank, see DEFAULT_DRIVE_MODE in
 * config.hpp) plus the cascade/claw/flip/intake mechanisms.
 */

#include "main.h"
#include "chassis.hpp"
#include "config.hpp"
#include "util.hpp"
#include "menu.hpp"
#include "lift.hpp"

#include <cstdio>
#include <atomic>
#include <cmath>

// These all live in main.cpp - this is how we reach them from here.
extern Chassis myRobot;
extern pros::MotorGroup cascade_motors;
extern pros::Motor cascade_motor_a; // CASCADE_MOTOR_PORTS[0]
extern pros::Motor cascade_motor_b; // CASCADE_MOTOR_PORTS[1]
extern pros::MotorGroup intake_motors;
extern pros::adi::DigitalOut claw_piston;
extern pros::adi::DigitalOut flip_piston;

// Menu plumbing, defined in main.cpp.
void menu_service();                 // reads the buttons, then redraws
void set_lift_readout(double degrees, double gap); // live lift position + sync gap for the home screen
void render_menu();                  // redraw only (used mid-routine)
MenuState menu_snapshot();           // consistent read of the menu state
void menu_clear_test_request();      // call once a requested route has run
void run_auton_routine(AutonRoutine routine);

// The old centre-button "show_status" toggle is gone. The menu replaced it:
// HOME shows the battery, and the port check (RIGHT) lists what is actually
// plugged in - which is what that toggle was really being used to find out.

// Runs a routine the driver asked for from the test menu.
//
// This happens HERE, in opcontrol's task, rather than in the button callback
// that requested it. opcontrol owns the motors - if the callback task drove
// them itself, two tasks would be commanding the same motors every 20ms and
// the robot would judder between them.
//
// The routine blocks, so the sticks are dead until it finishes. That is why
// the screen says "RUNNING - sticks off" while it does.
//
// Returns true if a routine ran, so the caller knows the lift may have moved.
static bool service_test_request(const MenuState &state) {
    if (!state.runTestNow) return false;

    myRobot.stop();                  // don't inherit this loop's stick input
    render_menu();                   // show "RUNNING" before we block
    run_auton_routine(state.selected);
    myRobot.stop();                  // and don't leave it coasting
    menu_clear_test_request();
    return true;
}

// Sends one loop's lift decision (lift.hpp) to the two motors.
static void apply_lift(const lift::Command &cmd) {
    switch (cmd.mode) {
        case lift::Command::Mode::DRIVE:
            // A speed each, not a shared voltage. Each motor's own speed
            // controller then works as hard as its side needs to hold that
            // speed, and the two targets carry the sync correction.
            cascade_motor_a.move_velocity(static_cast<std::int32_t>(std::lround(cmd.rpm[0])));
            cascade_motor_b.move_velocity(static_cast<std::int32_t>(std::lround(cmd.rpm[1])));
            break;
        case lift::Command::Mode::HOLD:
            // One target for both, so they stop at the same height. Sent
            // every loop, like the old top-of-travel hold: re-sending the same
            // target doesn't restart the move, and a motor whose cable
            // dropped out for a moment gets its target back.
            cascade_motors.move_absolute(cmd.holdDeg, CASCADE_HOLD_RPM);
            break;
        case lift::Command::Mode::BRAKE:
            // No encoder to hold a position with. brake() - unlike move(0) -
            // does use the HOLD brake mode set in initialize().
            cascade_motors.brake();
            break;
    }
}

void opcontrol() {
    pros::Controller master(pros::E_CONTROLLER_MASTER);
    int prevLeft = 0;
    int prevRight = 0;

    // Keeps the two lift motors moving and stopping together - see lift.hpp.
    // Created here, not as a global, so each run of driver control starts
    // fresh and latches a hold point from wherever the lift is right now.
    lift::Sync liftSync(lift::Settings{
        .maxRpm = CASCADE_MAX_RPM,
        .syncKp = CASCADE_SYNC_KP,
        .syncMaxRpm = CASCADE_SYNC_MAX_RPM,
        .syncDeadbandDeg = CASCADE_SYNC_DEADBAND_DEG,
        .stopSettledRpm = CASCADE_STOP_SETTLED_RPM,
        .stopTimeoutSeconds = CASCADE_STOP_TIMEOUT_MS / 1000.0,
        .loopSeconds = OPCONTROL_LOOP_MS / 1000.0,
        .limitEnabled = CASCADE_LIMIT_ENABLED,
        .minDeg = CASCADE_MIN_DEG,
        .maxDeg = CASCADE_MAX_DEG,
    });

    while (true) {
        // Reads one joystick axis and cleans it up:
        //   deadband - a released stick rarely sits at exactly 0, so ignore
        //              tiny readings or the robot creeps on its own
        //   expo     - softens the middle of the stick travel for fine
        //              control, while full push still gives full power
        //
        // Generic parameter because the real PROS API takes a strongly typed
        // enum here while the host mock takes an int - `auto` accepts both.
        auto axis = [&master](auto channel) {
            return util::expo(util::deadband(static_cast<int>(master.get_analog(channel))),
                              OPCONTROL_EXPO_GAIN);
        };

        int targetLeft = 0;
        int targetRight = 0;

        // Every DriveMode gets its own branch. There used to be only an
        // ARCADE test and an else, so SPLIT_ARCADE - the configured default -
        // fell through to the else and quietly drove as TANK. It compiled
        // clean and no test covered it; the only symptom was the sticks
        // behaving wrong. The static_assert at the bottom means adding a
        // fourth mode without handling it is now a build error rather than
        // another silent fallthrough.
        if constexpr (DEFAULT_DRIVE_MODE == DriveMode::SPLIT_ARCADE) {
            // Drive with the LEFT stick, steer with the RIGHT stick.
            //
            // Adding the turn to one side and subtracting it from the other
            // is what rotates the robot: pushing the right stick left makes
            // the left wheels slower (or reverse) and the right wheels
            // faster, so the robot swings left.
            int forward = axis(ANALOG_LEFT_Y);
            int turn = axis(ANALOG_RIGHT_X);
            targetLeft = forward + turn;
            targetRight = forward - turn;
        } else if constexpr (DEFAULT_DRIVE_MODE == DriveMode::ARCADE) {
            // Same mixing, but both axes come off the left stick.
            int forward = axis(ANALOG_LEFT_Y);
            int turn = axis(ANALOG_LEFT_X);
            targetLeft = forward + turn;
            targetRight = forward - turn;
        } else {
            // Tank: one stick per side, no mixing.
            targetLeft = axis(ANALOG_LEFT_Y);
            targetRight = axis(ANALOG_RIGHT_Y);
        }
        // Ramp the power up gradually so a tall stack doesn't tip when the
        // base jerks to full voltage - but let it come DOWN instantly, so
        // releasing the stick or slamming into reverse stops the robot now
        // rather than coasting through a ramp. util::slew limits both
        // directions; util::accelLimit limits only speeding up, which is
        // what OPCONTROL_SLEW_PER_LOOP's comment already describes.
        prevLeft = util::accelLimit(prevLeft, targetLeft, OPCONTROL_SLEW_PER_LOOP);
        prevRight = util::accelLimit(prevRight, targetRight, OPCONTROL_SLEW_PER_LOOP);
        myRobot.drive(prevLeft, prevRight);

        // cascade lift: R1 up / R2 down. Holding both cancels out to 0, which
        // is what we want.
        int cascadeSpeed = 0;
        if (master.get_digital(E_CONTROLLER_DIGITAL_R1)) cascadeSpeed += 127;
        if (master.get_digital(E_CONTROLLER_DIGITAL_R2)) cascadeSpeed -= 127;

        // Both encoders, + = up for each (PROS flips the reversed one's
        // reading). The lift's height is their average.
        const double liftA = cascade_motors.get_position(0);
        const double liftB = cascade_motors.get_position(1);
        const double liftDeg = lift::position(liftA, liftB);

        // Live lift position and sync gap, for measuring the limits in
        // config.hpp and checking the two sides agree. Handed to the menu to
        // draw rather than printed here: the home screen also owns line 5,
        // so writing it directly meant the two fought over the line.
        set_lift_readout(liftDeg, liftA - liftB);

        int allowed = cascadeSpeed;
        if constexpr (CASCADE_LIMIT_ENABLED) {
            // limitTravel fails open: if liftDeg isn't a finite number the
            // speed passes straight through. Comparing directly used to let
            // an unplugged motor's PROS_ERR_F (infinity) count as "past the
            // top" and silently block raising for the rest of the match.
            //
            // A blocked direction comes back as 0. liftSync is told it was a
            // limit (atLimit below), so it holds at once, clamped to the
            // limits: pressing R1 at the top holds the lift at
            // CASCADE_MAX_DEG instead of cutting power and letting it drop.
            allowed = util::limitTravel(cascadeSpeed, liftDeg,
                                        CASCADE_MIN_DEG, CASCADE_MAX_DEG);
        }
        const bool atLimit = cascadeSpeed != 0 && allowed == 0;

        // Moving: each motor gets its own speed, nudged so the two sides
        // stay level. Released: both brake together, then hold one shared
        // point, so neither coasts on after the other. See lift.hpp.
        apply_lift(liftSync.update(allowed, liftA, liftB, atLimit));

        // intake: X in / B out (reverse clears jams and fixes cup orientation
        // for SC3 - opaque vs transparent matters for yellow scoring).
        int intakeSpeed = 0;
        if (master.get_digital(E_CONTROLLER_DIGITAL_X)) intakeSpeed = 127;
        else if (master.get_digital(E_CONTROLLER_DIGITAL_B)) intakeSpeed = -127;
        intake_motors.move(intakeSpeed); // both motors, same direction

        // Draw the menu and run anything it asked for. After the drive and
        // mechanism commands, so a test routine starts from a stopped robot
        // rather than inheriting this loop's stick values.
        if (service_test_request(menu_snapshot())) {
            // The routine may have moved the lift. Re-latch the hold point
            // from where it is now instead of dragging it back.
            liftSync.reset();
        }
        menu_service();

        // claw: A toggles open/closed. new_press (not get_digital) so holding
        // the button doesn't flip it back and forth every 20ms loop.
        static bool clawOpen = false;
        if (master.get_digital_new_press(E_CONTROLLER_DIGITAL_A)) {
            clawOpen = !clawOpen;
            claw_piston.set_value(clawOpen);
        }

        // Y toggles the flip piston, same edge-triggered reasoning as above.
        static bool flipOut = false;
        if (master.get_digital_new_press(E_CONTROLLER_DIGITAL_Y)) {
            flipOut = !flipOut;
            flip_piston.set_value(flipOut);
        }

        pros::delay(OPCONTROL_LOOP_MS);
    }
}
