#include "lift.hpp"

#include <algorithm>
#include <cmath>

namespace lift {

double position(double posA, double posB) {
    const bool okA = std::isfinite(posA);
    const bool okB = std::isfinite(posB);
    if (okA && okB) return (posA + posB) / 2.0;
    if (okA) return posA;
    if (okB) return posB;
    return posA; // neither answered: hand back the non-finite reading
}

void driveRpm(double targetRpm, double posA, double posB,
              const Settings &s, double out[2]) {
    targetRpm = std::clamp(targetRpm, -s.maxRpm, s.maxRpm);

    double correction = 0.0;
    if (std::isfinite(posA) && std::isfinite(posB)) {
        // How far A is ahead of B (+ = A is higher). Shrink it by the
        // deadband so the correction starts from zero at the edge of it
        // rather than jumping in.
        double ahead = posA - posB;
        if (std::fabs(ahead) <= s.syncDeadbandDeg) {
            ahead = 0.0;
        } else {
            ahead -= std::copysign(s.syncDeadbandDeg, ahead);
        }
        correction = std::clamp(s.syncKp * ahead, -s.syncMaxRpm, s.syncMaxRpm);
    }

    // A ahead -> A slows, B speeds up. This works in both directions:
    // going up, "A ahead" means A is higher and needs to slow its climb;
    // going down, it means A is higher because it is falling slower, and
    // the same sign makes A drop faster and B drop slower.
    double a = targetRpm - correction;
    double b = targetRpm + correction;

    // Near full speed one of the two would need more than the cartridge can
    // give. Shift both toward zero by the overshoot so the gap between them
    // survives - capping only the fast one would throw the correction away
    // exactly when the lift is working hardest.
    const double over = std::max(std::fabs(a), std::fabs(b)) - s.maxRpm;
    if (over > 0.0) {
        const double shift = std::copysign(over, targetRpm);
        a -= shift;
        b -= shift;
    }

    out[0] = a;
    out[1] = b;
}

void Sync::reset() {
    phase_ = Phase::HOLDING;
    latched_ = false;
    havePrev_ = false;
}

Command Sync::hold(double here) {
    Command cmd;
    if (!latched_) {
        // Latch the stop point ONCE, and keep it until the driver moves
        // again. Taking a new reading every loop would chase the lift down
        // as it sagged instead of holding it up.
        if (!std::isfinite(here)) {
            // Neither encoder is answering, so there is no position to hold.
            // Fall back to the motor brake mode, and try to latch again next
            // loop in case a cable was only loose for a moment.
            cmd.mode = Command::Mode::BRAKE;
            return cmd;
        }
        holdDeg_ = here;
        if (settings_.limitEnabled) {
            // Stopping past a limit (momentum can carry it a little over)
            // holds AT the limit, so the lift settles back inside its travel.
            holdDeg_ = std::clamp(holdDeg_, settings_.minDeg, settings_.maxDeg);
        }
        latched_ = true;
    }
    cmd.mode = Command::Mode::HOLD;
    cmd.holdDeg = holdDeg_;
    return cmd;
}

Command Sync::update(int input, double posA, double posB, bool atLimit) {
    const double here = position(posA, posB);

    // How fast each motor is really turning, from how far it went since the
    // last loop. 1 RPM = 6 degrees per second. Not finite if a reading is.
    double rpmA = std::nan(""), rpmB = std::nan("");
    if (havePrev_ && settings_.loopSeconds > 0.0) {
        rpmA = (posA - prevA_) / settings_.loopSeconds / 6.0;
        rpmB = (posB - prevB_) / settings_.loopSeconds / 6.0;
    }
    prevA_ = posA;
    prevB_ = posB;
    havePrev_ = true;

    if (input != 0) {
        // Driver is moving the lift. Any old hold point is forgotten - the
        // next stop latches a fresh one wherever the lift ends up.
        phase_ = Phase::MOVING;
        latched_ = false;
        Command cmd;
        cmd.mode = Command::Mode::DRIVE;
        driveRpm(std::clamp(input, -127, 127) / 127.0 * settings_.maxRpm,
                 posA, posB, settings_, cmd.rpm);
        return cmd;
    }

    if (phase_ == Phase::MOVING) {
        // Just let go. At a travel limit, hold at once rather than braking
        // through it; otherwise brake both motors together first.
        phase_ = atLimit ? Phase::HOLDING : Phase::STOPPING;
        stoppingSeconds_ = 0.0;
    }

    if (phase_ == Phase::STOPPING) {
        stoppingSeconds_ += settings_.loopSeconds;

        // A motor whose speed can't be measured (encoder not answering)
        // doesn't hold up the hold - it can't be waited on anyway.
        auto stopped = [&](double rpm) {
            return !std::isfinite(rpm) || std::fabs(rpm) < settings_.stopSettledRpm;
        };
        const bool bothStopped = stopped(rpmA) && stopped(rpmB);
        const bool timedOut = stoppingSeconds_ >= settings_.stopTimeoutSeconds;
        const bool pastLimit = settings_.limitEnabled && std::isfinite(here) &&
                               (here >= settings_.maxDeg || here <= settings_.minDeg);

        if (!bothStopped && !timedOut && !pastLimit) {
            // Brake: aim both motors at zero speed, sync still on. The motor
            // that has run further gets pulled back and the other pushed on,
            // so the quicker one follows its partner instead of stopping
            // early - they come to rest together.
            Command cmd;
            cmd.mode = Command::Mode::DRIVE;
            driveRpm(0.0, posA, posB, settings_, cmd.rpm);
            return cmd;
        }
        phase_ = Phase::HOLDING;
    }

    return hold(here);
}

} // namespace lift
