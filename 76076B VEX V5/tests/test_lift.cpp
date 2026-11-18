// Host tests for the cascade lift sync (include/lift.hpp).
//
// Run with `make HOST_BUILD=1` from the project folder.
//
// The first half checks the decisions directly. The second half runs those
// decisions against a crude two-motor simulation in which one motor is much
// slower to respond than the other - the situation on the robot, where port 5
// kept moving after port 6 had stopped - and checks that the two sides stay
// level while moving and end up at the same height when stopped.
//
// The simulation is a sanity check of the control logic, not a model of a
// real V5 motor. Tuning still has to happen on the robot.

#include <iostream>
#include <cassert>
#include <cmath>
#include <limits>
#include <deque>
#include <algorithm>

#include "lift.hpp"

using lift::Command;

// Matches config.hpp's defaults, so the simulation tests the real tuning.
static const lift::Settings SETTINGS{
    .maxRpm = 200.0,
    .syncKp = 0.75,
    .syncMaxRpm = 40.0,
    .syncDeadbandDeg = 3.0,
    .stopSettledRpm = 10.0,
    .stopTimeoutSeconds = 0.5,
    .loopSeconds = 0.020,
    .limitEnabled = false,
    .minDeg = 0.0,
    .maxDeg = 1000.0,
};

static const double INF = std::numeric_limits<double>::infinity();

static bool near(double a, double b, double tol = 1e-9) { return std::fabs(a - b) <= tol; }

static void test_position() {
    std::cout << "[test] lift::position averages, and ignores a dead encoder\n";
    assert(near(lift::position(100.0, 110.0), 105.0));
    assert(near(lift::position(INF, 110.0), 110.0));  // A unplugged
    assert(near(lift::position(100.0, INF), 100.0));  // B unplugged
    assert(!std::isfinite(lift::position(INF, INF))); // neither: caller must check
}

static void test_drive_level() {
    std::cout << "[test] level lift: both motors get exactly what the driver asked\n";
    double out[2];
    lift::driveRpm(200.0, 500.0, 500.0, SETTINGS, out);
    assert(near(out[0], 200.0) && near(out[1], 200.0));
    lift::driveRpm(-200.0, 500.0, 500.0, SETTINGS, out);
    assert(near(out[0], -200.0) && near(out[1], -200.0));
    lift::driveRpm(100.0, 500.0, 500.0, SETTINGS, out);
    assert(near(out[0], 100.0) && near(out[1], 100.0));
}

static void test_drive_deadband() {
    std::cout << "[test] gaps inside the deadband are left alone (gear slop)\n";
    double out[2];
    lift::driveRpm(100.0, 503.0, 500.0, SETTINGS, out); // exactly at the edge
    assert(near(out[0], 100.0) && near(out[1], 100.0));
    lift::driveRpm(100.0, 497.5, 500.0, SETTINGS, out);
    assert(near(out[0], 100.0) && near(out[1], 100.0));
    // Just past the edge the correction starts from zero, not with a jump.
    lift::driveRpm(100.0, 504.0, 500.0, SETTINGS, out);
    assert(near(out[0], 99.25) && near(out[1], 100.75));
}

static void test_drive_correction_direction() {
    std::cout << "[test] the side that is ahead slows, the side behind speeds up\n";
    double out[2];

    // Going UP, A 11 deg higher (8 past the deadband -> 6 RPM): A must slow
    // its climb, B must hurry.
    lift::driveRpm(100.0, 511.0, 500.0, SETTINGS, out);
    std::cout << "  up, A ahead by 11:   A=" << out[0] << " B=" << out[1] << "\n";
    assert(near(out[0], 94.0) && near(out[1], 106.0));

    // Going UP, B higher: mirror image.
    lift::driveRpm(100.0, 500.0, 511.0, SETTINGS, out);
    assert(near(out[0], 106.0) && near(out[1], 94.0));

    // Going DOWN, A 11 deg higher means A is falling too slowly. A must
    // fall faster (more negative), B slower.
    lift::driveRpm(-100.0, 511.0, 500.0, SETTINGS, out);
    std::cout << "  down, A higher by 11: A=" << out[0] << " B=" << out[1] << "\n";
    assert(near(out[0], -106.0) && near(out[1], -94.0));

    // Either way the gap closes: d(A-B)/dt has the opposite sign to A-B.
    for (double target : {150.0, 50.0, -50.0, -150.0}) {
        for (double gap : {-60.0, -10.0, 10.0, 60.0}) {
            lift::driveRpm(target, 500.0 + gap, 500.0, SETTINGS, out);
            assert((out[0] - out[1]) * gap < 0.0);
        }
    }
}

static void test_drive_correction_capped() {
    std::cout << "[test] correction is capped, so a wild reading can't fling one side\n";
    double out[2];
    lift::driveRpm(100.0, 5000.0, 500.0, SETTINGS, out);
    assert(near(out[0], 60.0) && near(out[1], 140.0));
}

static void test_drive_saturation_keeps_gap() {
    std::cout << "[test] at full speed the gap survives instead of being clipped off\n";
    double out[2];

    // Full speed up, A ahead by 23 deg (20 past the deadband -> 15 RPM)
    // -> wants A=185, B=215. B can't do 215, so both shift down by 15:
    // A=170, B=200. The 30 RPM gap - the part that actually fixes the
    // sync - is kept.
    lift::driveRpm(200.0, 523.0, 500.0, SETTINGS, out);
    std::cout << "  full up, A ahead by 23: A=" << out[0] << " B=" << out[1] << "\n";
    assert(near(out[0], 170.0) && near(out[1], 200.0));

    // Same going down.
    lift::driveRpm(-200.0, 500.0, 523.0, SETTINGS, out);
    assert(near(out[0], -170.0) && near(out[1], -200.0));

    // Nothing ever asks for more than the cartridge can give.
    for (double target : {-250.0, -200.0, -120.0, 0.0, 120.0, 200.0, 250.0}) {
        for (double gap : {-500.0, -30.0, 0.0, 30.0, 500.0}) {
            lift::driveRpm(target, 500.0 + gap, 500.0, SETTINGS, out);
            assert(std::fabs(out[0]) <= 200.0 + 1e-9 && std::fabs(out[1]) <= 200.0 + 1e-9);
        }
    }
}

static void test_drive_fails_open() {
    std::cout << "[test] a dead encoder means no correction, not a runaway one\n";
    double out[2];
    lift::driveRpm(150.0, INF, 500.0, SETTINGS, out);
    assert(near(out[0], 150.0) && near(out[1], 150.0));
    lift::driveRpm(150.0, 500.0, -INF, SETTINGS, out);
    assert(near(out[0], 150.0) && near(out[1], 150.0));
    lift::driveRpm(150.0, std::nan(""), 500.0, SETTINGS, out);
    assert(near(out[0], 150.0) && near(out[1], 150.0));
}

// One loop at full speed is 200 RPM = 1200 deg/s = 24 degrees per 20ms loop.
static const double DEG_PER_LOOP_FULL = 24.0;

static void test_sync_release_brakes_then_holds() {
    std::cout << "[test] letting go: brake both together, THEN hold one shared point\n";
    lift::Sync sync(SETTINGS);

    Command c = sync.update(127, 0.0, 0.0);
    assert(c.mode == Command::Mode::DRIVE && near(c.rpm[0], 200.0) && near(c.rpm[1], 200.0));
    c = sync.update(127, 24.0, 24.0);
    assert(c.mode == Command::Mode::DRIVE);

    // Released while still doing full speed, A 4 deg ahead. Not a hold yet:
    // both are told to brake to ZERO SPEED (actively - not 0 volts), with
    // the sync on, so A (ahead) is pulled back and B pushed on.
    c = sync.update(0, 48.0, 43.0);
    std::cout << "  braking, A ahead by 5: A=" << c.rpm[0] << " B=" << c.rpm[1] << "\n";
    assert(c.mode == Command::Mode::DRIVE);
    assert(c.rpm[0] < 0.0 && c.rpm[1] > 0.0);
    assert(std::fabs(c.rpm[0]) <= 40.0 && std::fabs(c.rpm[1]) <= 40.0);

    // Still moving (A did 4 deg this loop = 33 RPM): keep braking.
    c = sync.update(0, 52.0, 50.0);
    assert(c.mode == Command::Mode::DRIVE);

    // Both under 10 RPM now: lock onto the average of where they stopped.
    c = sync.update(0, 52.5, 51.0);
    assert(c.mode == Command::Mode::HOLD && near(c.holdDeg, 51.75));

    // The point is latched ONCE. If the lift sags, or a motor drifts, the
    // target stays put - that's what pulls it back.
    c = sync.update(0, 50.0, 49.0);
    assert(c.mode == Command::Mode::HOLD && near(c.holdDeg, 51.75));
    c = sync.update(0, 60.0, 58.0);
    assert(c.mode == Command::Mode::HOLD && near(c.holdDeg, 51.75));

    // Moving again drops the hold; the next stop latches a new point.
    c = sync.update(-127, 51.75, 51.75);
    assert(c.mode == Command::Mode::DRIVE && c.rpm[0] < 0.0 && c.rpm[1] < 0.0);
    c = sync.update(0, 30.0, 30.0);  // released, but moved 21.75 deg: braking
    assert(c.mode == Command::Mode::DRIVE);
    c = sync.update(0, 29.5, 29.8); // stopped
    assert(c.mode == Command::Mode::HOLD && near(c.holdDeg, 29.65));
}

static void test_sync_stop_timeout() {
    std::cout << "[test] the hold takes over after the brake timeout, even if still moving\n";
    lift::Sync sync(SETTINGS);
    double p = 0.0;
    sync.update(127, p, p);
    p += DEG_PER_LOOP_FULL;
    sync.update(127, p, p);

    // Let go, but the lift keeps sliding at full speed (say, falling).
    int brakingLoops = 0;
    Command c;
    for (int i = 0; i < 100; i++) {
        p += DEG_PER_LOOP_FULL;
        c = sync.update(0, p, p);
        if (c.mode != Command::Mode::DRIVE) break;
        brakingLoops++;
    }
    std::cout << "  braked for " << brakingLoops << " loops before holding\n";
    assert(c.mode == Command::Mode::HOLD);
    assert(brakingLoops == 24); // 0.5 s / 20 ms = 25th loop holds
}

static void test_sync_first_loop_holds() {
    std::cout << "[test] the first loop of driver control holds wherever the lift is\n";
    lift::Sync sync(SETTINGS);
    Command c = sync.update(0, 12.0, 10.0);
    assert(c.mode == Command::Mode::HOLD && near(c.holdDeg, 11.0));
}

static void test_sync_reset() {
    std::cout << "[test] reset() re-latches from the current position\n";
    lift::Sync sync(SETTINGS);
    assert(near(sync.update(0, 100.0, 100.0).holdDeg, 100.0));
    // Something else (a test routine) moved the lift to 700.
    assert(near(sync.update(0, 700.0, 700.0).holdDeg, 100.0)); // without reset: drags back
    sync.reset();
    assert(near(sync.update(0, 700.0, 700.0).holdDeg, 700.0));
}

static void test_sync_dead_encoders() {
    std::cout << "[test] encoder trouble while stopping\n";
    lift::Sync sync(SETTINGS);

    // One encoder dead: hold on the one that works.
    Command c = sync.update(0, INF, 250.0);
    assert(c.mode == Command::Mode::HOLD && near(c.holdDeg, 250.0));

    // Both dead: nothing to hold a position with, so brake - and keep
    // trying to latch, so a cable that was only loose recovers.
    lift::Sync blind(SETTINGS);
    c = blind.update(0, INF, INF);
    assert(c.mode == Command::Mode::BRAKE);
    c = blind.update(0, INF, INF);
    assert(c.mode == Command::Mode::BRAKE);
    c = blind.update(0, 80.0, 82.0); // cable back
    assert(c.mode == Command::Mode::HOLD && near(c.holdDeg, 81.0));

    // An encoder that dies mid-brake can't be waited on: the working one
    // decides when the lift has stopped.
    lift::Sync half(SETTINGS);
    half.update(127, 0.0, 0.0);
    half.update(127, 24.0, 24.0);
    c = half.update(0, INF, 30.0);  // A gone; B still moving 6 deg/loop = 50 RPM
    assert(c.mode == Command::Mode::DRIVE && near(c.rpm[0], 0.0) && near(c.rpm[1], 0.0));
    c = half.update(0, INF, 30.5);  // B stopped
    assert(c.mode == Command::Mode::HOLD && near(c.holdDeg, 30.5));
}

static void test_sync_limits() {
    std::cout << "[test] with limits on, a hold never parks the lift outside them\n";
    lift::Settings s = SETTINGS;
    s.limitEnabled = true;
    s.minDeg = 0.0;
    s.maxDeg = 1000.0;

    // Driver pushing up into the top limit: no braking phase - hold at once,
    // AT the limit. (This replaces the old special case in opcontrol that
    // held at CASCADE_MAX_DEG.)
    lift::Sync top(s);
    top.update(127, 976.0, 976.0);
    Command c = top.update(0, 1012.0, 1008.0, /*atLimit=*/true);
    assert(c.mode == Command::Mode::HOLD && near(c.holdDeg, 1000.0));

    // Released just below the top but braking carries it past: stop
    // braking and hold at the limit.
    lift::Sync over(s);
    over.update(127, 960.0, 960.0);
    over.update(127, 984.0, 984.0);
    c = over.update(0, 998.0, 998.0); // still moving, below the top: brake
    assert(c.mode == Command::Mode::DRIVE);
    c = over.update(0, 1004.0, 1004.0); // crossed it
    assert(c.mode == Command::Mode::HOLD && near(c.holdDeg, 1000.0));

    lift::Sync bottom(s);
    c = bottom.update(0, -6.0, -2.0);
    assert(c.mode == Command::Mode::HOLD && near(c.holdDeg, 0.0));

    lift::Sync middle(s);
    c = middle.update(0, 400.0, 404.0);
    assert(c.mode == Command::Mode::HOLD && near(c.holdDeg, 402.0));
}

// ---------------------------------------------------------------------------
// Simulation
// ---------------------------------------------------------------------------
//
// Each motor chases whatever speed it is told with a first-order lag. Motor A
// ("port 5") is given a lag three times longer than motor B ("port 6"), so on
// its own it speeds up late and slows down late - the reported symptom.
//
// `oldCode` replays what the lift did before this fix: same voltage to both
// while moving, then move(0) on release - 0 volts, so each motor just
// coasts down on its own friction (A coasting four times longer than B).
//
// HOLD is modelled as the motor's position controller: aim for the target at
// a speed proportional to the distance, capped at the hold RPM, through the
// same lag. (The gain is kept soft so the slow motor doesn't ring - the real
// motor's controller is better tuned than this, so this is the harder case.)
// Commands reach the motors `delayLoops` loops late, to make sure the sync
// doesn't start oscillating once real-world lag is in the way.

struct SimMotor {
    double pos = 0.0; // degrees
    double vel = 0.0; // RPM
    double tau;       // seconds to reach ~63% of a speed change
};

struct SimResult {
    double peakGapWhileMoving = 0.0;  // anywhere in the move, start included
    double worstGapWhileMoving = 0.0; // once up to speed (after the first 0.5 s)
    double worstGapWhileStopping = 0.0;
    double gapAtRelease = 0.0;
    double finalGap = 0.0;
    double finalOffTarget = 0.0;      // worst motor's distance from the hold point
    double stopMsA = -1.0, stopMsB = -1.0; // ms after release until under 5 RPM for good
    double ranOnDeg = 0.0;            // how far the lift travelled after release
    int gapSignFlips = 0;             // once up to speed, beyond the deadband
};

static SimResult simulate(const lift::Settings &s, int delayLoops, int input,
                          double moveSeconds, bool oldCode = false, double holdRpm = 50.0) {
    const double LOOP = 0.020, SUB = 0.001;
    const double HOLD_GAIN = 1.0; // RPM per degree of hold error
    SimMotor m[2];
    m[0].tau = 0.150; // "port 5": slow to respond
    m[1].tau = 0.050; // "port 6"
    const double COAST_TAU[2] = {0.40, 0.10}; // oldCode only: 0 V, friction alone

    lift::Sync sync(s);
    std::deque<Command> pipe(delayLoops, Command{Command::Mode::HOLD, {0, 0}, 0.0});
    SimResult r;
    int lastSign = 0;
    double holdTarget = 0.0;
    const int moveLoops = static_cast<int>(moveSeconds / LOOP);
    const int totalLoops = moveLoops + static_cast<int>(2.0 / LOOP);
    double stoppedSince[2] = {-1.0, -1.0};
    double releasedAt = 0.0;

    for (int k = 0; k < totalLoops; k++) {
        const bool moving = k < moveLoops;
        if (k == moveLoops) {
            r.gapAtRelease = m[0].pos - m[1].pos;
            releasedAt = (m[0].pos + m[1].pos) / 2.0;
        }

        pipe.push_back(sync.update(moving ? input : 0, m[0].pos, m[1].pos));
        const Command cmd = pipe.front();
        pipe.pop_front();
        if (cmd.mode == Command::Mode::HOLD) holdTarget = cmd.holdDeg;

        for (int sub = 0; sub < static_cast<int>(LOOP / SUB); sub++) {
            for (int i = 0; i < 2; i++) {
                if (oldCode && !moving) {
                    m[i].vel -= m[i].vel * (SUB / COAST_TAU[i]); // move(0): coasting
                    m[i].pos += m[i].vel * 6.0 * SUB;
                    continue;
                }
                double want = 0.0;
                if (cmd.mode == Command::Mode::DRIVE) {
                    want = cmd.rpm[i];
                } else if (cmd.mode == Command::Mode::HOLD) {
                    want = std::clamp(HOLD_GAIN * (cmd.holdDeg - m[i].pos), -holdRpm, holdRpm);
                }
                m[i].vel += (want - m[i].vel) * (SUB / m[i].tau);
                m[i].pos += m[i].vel * 6.0 * SUB; // 1 RPM = 6 deg/s
            }
        }

        const double gap = m[0].pos - m[1].pos;
        const double t = k * LOOP;
        if (moving) r.peakGapWhileMoving = std::max(r.peakGapWhileMoving, std::fabs(gap));
        if (moving && t >= 0.5) {
            r.worstGapWhileMoving = std::max(r.worstGapWhileMoving, std::fabs(gap));
            const int sign = gap > s.syncDeadbandDeg ? 1 : gap < -s.syncDeadbandDeg ? -1 : 0;
            if (sign != 0 && lastSign != 0 && sign != lastSign) r.gapSignFlips++;
            if (sign != 0) lastSign = sign;
        }
        if (!moving) {
            r.worstGapWhileStopping = std::max(r.worstGapWhileStopping, std::fabs(gap));
            const double msAfter = (k - moveLoops + 1) * LOOP * 1000.0;
            for (int i = 0; i < 2; i++) {
                const bool stopped = std::fabs(m[i].vel) < 5.0;
                if (stopped && stoppedSince[i] < 0) stoppedSince[i] = msAfter;
                if (!stopped) stoppedSince[i] = -1.0;
            }
        }
    }
    r.stopMsA = stoppedSince[0];
    r.stopMsB = stoppedSince[1];
    r.ranOnDeg = (m[0].pos + m[1].pos) / 2.0 - releasedAt;
    r.finalGap = m[0].pos - m[1].pos;
    r.finalOffTarget = std::max(std::fabs(m[0].pos - holdTarget), std::fabs(m[1].pos - holdTarget));
    return r;
}

static void print(const char *label, const SimResult &r) {
    std::cout << "  " << label << ": gap peak " << r.peakGapWhileMoving
              << " deg, once at speed <= " << r.worstGapWhileMoving
              << ", at release " << r.gapAtRelease
              << ", while stopping <= " << r.worstGapWhileStopping
              << ", final " << r.finalGap
              << " | stopped A/B at " << r.stopMsA << "/" << r.stopMsB << " ms"
              << ", ran on " << r.ranOnDeg << " deg\n";
}

static void test_sim_keeps_sides_level() {
    std::cout << "[test] sim: a slow 'port 5' stays level with 'port 6' while moving\n";

    lift::Settings noSync = SETTINGS;
    noSync.syncKp = 0.0;
    const SimResult off = simulate(noSync, 1, 127, 1.5);
    const SimResult on = simulate(SETTINGS, 1, 127, 1.5);
    print("sync off", off);
    print("sync on ", on);

    // Without sync, the slow motor falls behind on the way up and never
    // catches up, because both are told the same speed.
    assert(off.worstGapWhileMoving > 100.0);
    // With sync, the slow motor still slips behind while the two are
    // speeding up - no correction can make it accelerate faster than it
    // physically can - but far less, and it is pulled back level once up
    // to speed and kept there, near the deadband.
    assert(on.peakGapWhileMoving < off.peakGapWhileMoving / 2.0);
    assert(on.worstGapWhileMoving < 6.0);
    assert(std::fabs(on.gapAtRelease) < 6.0);
    // ...without hunting back and forth.
    assert(on.gapSignFlips == 0);

    // Same coming down.
    const SimResult down = simulate(SETTINGS, 1, -127, 1.5);
    print("sync on, down", down);
    assert(down.worstGapWhileMoving < 6.0 && down.gapSignFlips == 0);
}

static void test_sim_stops_together() {
    std::cout << "[test] sim: after release both motors stop at the same TIME and HEIGHT\n";

    lift::Settings noSync = SETTINGS;
    noSync.syncKp = 0.0;
    const SimResult old = simulate(noSync, 1, 127, 1.5, /*oldCode=*/true);
    const SimResult alone = simulate(noSync, 1, 127, 1.5); // new stop, sync off
    const SimResult r = simulate(SETTINGS, 1, 127, 1.5);
    print("old move(0)       ", old);
    print("new, sync off     ", alone);
    print("new, sync on      ", r);

    // The reported symptom, reproduced: on 0 volts the slow motor coasts on
    // long after the other has stopped, and they end up far apart.
    assert(old.stopMsA - old.stopMsB > 500.0);
    assert(std::fabs(old.finalGap) > 100.0);

    // Fixed: both stopped, within a few loops of each other...
    assert(r.stopMsA >= 0.0 && r.stopMsB >= 0.0);
    assert(std::fabs(r.stopMsA - r.stopMsB) <= 100.0);
    // ...far closer together than with the motors braking independently,
    // which is what the sync during braking buys...
    assert(std::fabs(r.stopMsA - r.stopMsB) < std::fabs(alone.stopMsA - alone.stopMsB) / 2.0);
    // ...and resting at the same height, on the shared hold point.
    assert(std::fabs(r.finalGap) < 1.0);
    assert(r.finalOffTarget < 0.5);
    // Actively braked, so it runs on far less than the old coast did.
    assert(r.ranOnDeg < old.ranOnDeg / 2.0);

    const SimResult down = simulate(SETTINGS, 1, -127, 1.5);
    print("new, sync on, down", down);
    assert(std::fabs(down.stopMsA - down.stopMsB) <= 100.0);
    assert(std::fabs(down.finalGap) < 1.0 && down.finalOffTarget < 0.5);
}

static void test_sim_tolerates_lag() {
    std::cout << "[test] sim: still stable with commands arriving late\n";

    // 40 ms late: no visible difference.
    const SimResult r2 = simulate(SETTINGS, 2, 127, 1.5);
    print("40 ms lag", r2);
    assert(r2.worstGapWhileMoving < 6.0 && r2.gapSignFlips == 0);
    assert(std::fabs(r2.finalGap) < 1.0);

    // 60 ms late, on top of the very slow 'port 5': overshoots once, then
    // settles - it does not grow into a back-and-forth.
    const SimResult r3 = simulate(SETTINGS, 3, 127, 1.5);
    print("60 ms lag", r3);
    assert(r3.worstGapWhileMoving < 15.0);
    assert(r3.gapSignFlips <= 1);
    assert(std::fabs(r3.gapAtRelease) < 6.0);
    assert(std::fabs(r3.finalGap) < 1.0);
}

static void test_sim_short_tap() {
    std::cout << "[test] sim: a quick tap of R1 still stops both sides together\n";
    const SimResult r = simulate(SETTINGS, 1, 127, 0.1);
    print("100 ms tap", r);
    assert(std::fabs(r.stopMsA - r.stopMsB) <= 100.0);
    assert(std::fabs(r.finalGap) < 1.0 && r.finalOffTarget < 0.5);
}

int main() {
    std::cout << "Host test: Cascade lift sync\n";
    test_position();
    test_drive_level();
    test_drive_deadband();
    test_drive_correction_direction();
    test_drive_correction_capped();
    test_drive_saturation_keeps_gap();
    test_drive_fails_open();
    test_sync_release_brakes_then_holds();
    test_sync_stop_timeout();
    test_sync_first_loop_holds();
    test_sync_reset();
    test_sync_dead_encoders();
    test_sync_limits();
    test_sim_keeps_sides_level();
    test_sim_stops_together();
    test_sim_tolerates_lag();
    test_sim_short_tap();
    std::cout << "All lift tests passed.\n";
    return 0;
}
