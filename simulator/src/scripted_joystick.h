#pragma once
//
// SCRIPTED joystick: plays a test protocol (evaluation/protocols/*.yaml) against SIMULATION time.
//
// Why here and not over DDS: the controller reads the joystick from `lowstate->joystick`, i.e. from
// the wireless_remote field of the LowState message, which the simulator publishes. An external
// publisher cannot overwrite a field of a message published by someone else, so the only injection
// point is inside the MuJoCo bridge.
//
// Why simulation time and not the wall clock: two runs of the same protocol, on different machines
// or under different load, must command the same things at the same physical instants. Simulation
// time is also the time axis of ground_truth.csv. run_protocol.py checks afterwards that the
// real-time factor was ~1, since the controller runs on its own 50 Hz wall clock.
//
// Protocol semantics: every timeline entry has a time `t` and a state; the state is STICKY (it holds
// until a later entry changes it), so the file stays short and an entry can be inserted without
// rewriting the rest.
//
#include <unitree/dds_wrapper/common/unitree_joystick.hpp>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

namespace scripted
{
// Published by the physics loop of main.cc at every mj_step.
inline std::atomic<double> g_sim_time{0.0};
// < 0: no automatic release of the elastic band (manual, key 9).
inline std::atomic<double> g_band_release_at{-1.0};
// Set once the protocol has passed `stop_at`: run_protocol.py uses it as the end signal.
inline std::atomic<bool> g_finished{false};
}  // namespace scripted


class ScriptedJoystick : public unitree::common::UnitreeJoystick
{
public:
    struct Entry
    {
        double t = 0.0;
        std::string action;  // "", "fixstand", "rl"
        double vx = 0.0, vy = 0.0, wz = 0.0;
        double height = 0.98;
    };

    explicit ScriptedJoystick(const std::string & path) { load(path); }

    void update() override
    {
        // Protocol clock: simulation time by default. With `clock: wall_after_rl` (replay of a real
        // experiment) time follows the WALL clock from the X edge of the `rl` action onwards: the
        // controller times its policy steps on the wall clock, and with a real-time factor slightly
        // below 1 the commands would otherwise drift by several policy steps over a long replay.
        double t = scripted::g_sim_time.load();
        if (wall_anchored_)
            t = anchor_t_ + std::chrono::duration<double>(std::chrono::steady_clock::now() - anchor_wall_).count();
        const double dt = (last_t_ < 0.0) ? 0.0 : std::max(0.0, t - last_t_);
        last_t_ = t;

        // The button state is computed FIRST, then every setter is called ONCE at the end. LT and RT
        // are `Axis` with an EMA filter (x <- 0.97 x + 0.03 new): calling RT(0) and then RT(1) in
        // the same cycle means two updates, with fixed point 0.03/(1-0.97^2) = 0.508, right on the
        // 0.5 threshold of R2.pressed in the controller. The bit would flicker and the height ramp
        // would run at a fraction of its nominal speed.
        bool bLT = false, bRT = false, bB = false, bUp = false, bRB = false, bX = false, bA = false;

        const Entry & e = active(t);
        const double since = t - e.t;   // < 0 only before the first timeline entry

        // --- state transitions: a pulse, the RISING EDGE is needed (DSL `.on_pressed`) ------------
        // The controller's DSL wants `LT + up.on_pressed`: the MODIFIER already pressed (LT is an
        // Axis with EMA, it passes 0.5 after ~23 ms) and the EDGE of the button. So the modifier is
        // held for the whole pulse and the button comes kModifierLead later, as a hand does.
        if (!e.action.empty() && since >= 0.0 && since < kPulse)
        {
            const bool btn = since >= kModifierLead;
            if (e.action == "fixstand") { bLT = true; bUp = btn; }   // Passive -> FixStand: LT + up
            else if (e.action == "rl")  { bRB = true; bX = btn;  }   // FixStand -> Squat:   RB + X
            // wall-clock anchor: the X edge, i.e. the controller entering the RL state
            if (wall_after_rl_ && !wall_anchored_ && e.action == "rl" && btn) {
                wall_anchored_ = true;
                anchor_t_ = e.t + kModifierLead;
                anchor_wall_ = std::chrono::steady_clock::now();
                t = anchor_t_;
            }
        }

        // --- height: RT to go down, B to go up, until the target is reached ------------------------
        // Integrator IDENTICAL to `base_height_command` in the controller (same ramps, same clamp).
        // The loop is closed on the MODEL of the target, not on the measured height: the command is
        // reproducible by construction and does not depend on how the policy responds, which is
        // exactly what is being measured.
        //
        // Controller STOP (`stop_height` in the protocol, = cutoff_height + 1 cm): R2 alone stops there,
        // going further requires A + R2. With the key the model acts like a hand: RT down to the stop,
        // then A + RT. Without it (protocols written before the stop, replays of older experiments) A
        // is held with RT in every descent, which reproduces the controller without the stop.
        const bool was_moving = moving_, was_down = moving_down_;
        moving_ = false;
        if (target_ > e.height + kHeightEps)
        {
            bRT = true; moving_ = true; moving_down_ = true;
            const bool past_stop = !has_stop_ || target_ <= stop_ + kHeightEps;
            bA = past_stop;
            target_ = std::max(past_stop ? e.height : std::max(e.height, stop_), target_ - kDescendRate * dt);
        }
        else if (target_ < e.height - kHeightEps)
        {
            bB = true; moving_ = true; moving_down_ = false;
            target_ = std::min(e.height, target_ + kRiseRate * dt);
        }
        else
        {
            // OVERHOLD on ARRIVAL at an end of travel: the button stays pressed `overhold` seconds
            // longer. There the controller CLAMPS `squat_target`, so it ends exactly on the limit
            // whatever the accumulated offset (+-1 policy step at each ramp start/end, since the
            // joystick runs on simulation time and the controller on its own clock), and model and
            // controller are aligned again. It triggers on arrival, not on a timeline entry, so it
            // works with sparse and dense timelines, and never presses anything if the height did
            // not move. The controller stop is a third end of travel, but only when reached FROM
            // ABOVE: there R2 alone clamps; from below, R2 alone does not move anything.
            const bool low = e.height <= deepest_ + 1e-6, high = e.height >= nominal_ - 1e-6;
            const bool at_stop = has_stop_ && std::abs(e.height - stop_) < 1e-6;
            if (was_moving)
            {
                over_left_ = overhold_;
                over_key_ = low ? 'L' : high ? 'H' : (at_stop && was_down) ? 'S' : 0;
            }
            if (over_left_ > 0.0 && over_key_)
            {
                over_left_ -= dt;
                if (over_key_ == 'H') bB = true;
                else { bRT = true; bA = (over_key_ == 'L'); }   // below the stop R2 alone does not go down
            }
            else over_left_ = 0.0;
        }
        target_ = std::clamp(target_, deepest_, nominal_);

        // --- write the state: ONE call per button/axis per cycle -------------------------------
        back(0); start(0); LB(0); Y(0); down(0); left(0); right(0);
        A(bA ? 1 : 0);
        LT(bLT ? 1.0f : 0.0f); RT(bRT ? 1.0f : 0.0f);
        up(bUp ? 1 : 0); RB(bRB ? 1 : 0); X(bX ? 1 : 0); B(bB ? 1 : 0);

        // --- sticks ---------------------------------------------------------------------------------
        // Mapping of the controller (observations.h):  obs = [ly, -lx, -rx]
        ly(static_cast<float>(e.vx));
        lx(static_cast<float>(-e.vy));
        rx(static_cast<float>(-e.wz));
        ry(0.0f);

        if (stop_at_ > 0.0 && t >= stop_at_) scripted::g_finished.store(true);
    }

    double stop_at() const { return stop_at_; }

private:
    static constexpr double kPulse = 0.40;        // duration of a transition pulse [s]
    static constexpr double kModifierLead = 0.15; // the button comes after the modifier [s]
    // Height target ramps: they MUST equal descend_rate / rise_rate in the controller
    // (deploy/include/isaaclab/envs/mdp/observations/observations.h). They are protocol keys
    // (`descend_rate`, `rise_rate`) rather than constants, so that a protocol states the ramps it
    // was written for; when missing, 0.05 m/s is used with a warning.
    double kDescendRate = 0.05;
    double kRiseRate = 0.05;
    static constexpr double kHeightEps = 1e-4;

    void load(const std::string & path)
    {
        YAML::Node cfg;
        try { cfg = YAML::LoadFile(path); }
        catch (const std::exception & ex) {
            std::cerr << "ScriptedJoystick: cannot read " << path << ": " << ex.what() << std::endl;
            std::exit(EXIT_FAILURE);
        }

        name_    = cfg["name"] ? cfg["name"].as<std::string>() : "unnamed";
        nominal_ = cfg["nominal_height"] ? cfg["nominal_height"].as<double>() : 0.98;
        deepest_ = cfg["deepest_height"] ? cfg["deepest_height"].as<double>() : 0.60;
        stop_at_ = cfg["stop_at"] ? cfg["stop_at"].as<double>() : -1.0;
        overhold_ = cfg["overhold"] ? cfg["overhold"].as<double>() : 0.0;
        // `stick_smoothing: false`: sticks AND triggers (LT/RT) skip the simulator-side EMA filter.
        // Used to replay a real experiment: on the robot the remote packet reaches the controller
        // unfiltered, and the only EMA filter is the controller's (present in simulation too).
        // Keeping the simulator's filter as well would double the delay.
        if (cfg["stick_smoothing"] && !cfg["stick_smoothing"].as<bool>()) {
            lx.smooth = ly.smooth = rx.smooth = ry.smooth = 1.0f;
            LT.smooth = RT.smooth = 1.0f;
            stick_raw_ = true;
        }
        wall_after_rl_ = cfg["clock"] && cfg["clock"].as<std::string>() == "wall_after_rl";
        if (cfg["descend_rate"]) kDescendRate = cfg["descend_rate"].as<double>();
        if (cfg["rise_rate"])    kRiseRate    = cfg["rise_rate"].as<double>();
        if (!cfg["descend_rate"] || !cfg["rise_rate"])
            std::cout << "ScriptedJoystick: WARNING, descend_rate/rise_rate missing from the protocol: using "
                      << kDescendRate << "/" << kRiseRate << " m/s. They must match observations.h of the controller."
                      << std::endl;
        // controller stop: it must equal cutoff_height + 1 cm of the policy's deploy.yaml
        // (run_protocol.py checks it before starting). Absent: protocol written for a controller without stop.
        has_stop_ = static_cast<bool>(cfg["stop_height"]);
        if (has_stop_) {
            stop_ = cfg["stop_height"].as<double>();
            if (stop_ <= deepest_ || stop_ >= nominal_) {
                std::cerr << "ScriptedJoystick: stop_height " << stop_ << " outside (" << deepest_ << ", "
                          << nominal_ << ")" << std::endl;
                std::exit(EXIT_FAILURE);
            }
        }
        target_  = nominal_;
        neutral_.height = nominal_;
        neutral_.t = -1e9;

        if (cfg["band_release_at"])
            scripted::g_band_release_at.store(cfg["band_release_at"].as<double>());

        // sticky state: every entry inherits what it does not redeclare from the previous one
        Entry cur;
        cur.height = nominal_;
        for (const auto & n : cfg["timeline"])
        {
            Entry e = cur;
            e.action.clear();  // the action is NOT sticky: it is a pulse
            e.t = n["t"].as<double>();
            if (n["do"])     e.action = n["do"].as<std::string>();
            if (n["vx"])     e.vx = n["vx"].as<double>();
            if (n["vy"])     e.vy = n["vy"].as<double>();
            if (n["wz"])     e.wz = n["wz"].as<double>();
            if (n["height"]) e.height = n["height"].as<double>();
            if (!entries_.empty() && e.t < entries_.back().t) {
                std::cerr << "ScriptedJoystick: timeline not sorted at t=" << e.t << std::endl;
                std::exit(EXIT_FAILURE);
            }
            entries_.push_back(e);
            cur = e;
        }

        if (entries_.empty()) {
            std::cerr << "ScriptedJoystick: empty timeline in " << path << std::endl;
            std::exit(EXIT_FAILURE);
        }

        // The PHYSICAL joystick zeroes everything below 0.15 (deadzone in physics_joystick.h). The
        // scripted joystick does not apply it, since exact values remove a source of variance, but a
        // value in that band could not be reproduced by hand: warn about it.
        if (!stick_raw_)
        for (const auto & e : entries_)
            for (double v : {e.vx, e.vy, e.wz})
                if (std::abs(v) > 1e-9 && std::abs(v) < 0.15) {
                    std::cout << "ScriptedJoystick: WARNING, |command|=" << std::abs(v)
                              << " at t=" << e.t << " is below the 0.15 deadzone of the physical pad: "
                              << "this instant cannot be reproduced by hand." << std::endl;
                    break;
                }

        std::cout << "ScriptedJoystick: protocol '" << name_ << "', " << entries_.size()
                  << " entries, stop_at=" << stop_at_ << " s, band released at "
                  << scripted::g_band_release_at.load() << " s, overhold=" << overhold_ << " s, ramps "
                  << kDescendRate << "/" << kRiseRate << " m/s"
                  << (has_stop_ ? ", stop at " + std::to_string(stop_) + " m (RT down to the stop, then A + RT)"
                                : std::string(", A held with RT (protocol without stop)"))
                  << (wall_after_rl_ ? ", wall clock after RL" : "")
                  << (stick_raw_ ? ", unfiltered sticks" : "") << std::endl;
    }

    const Entry & active(double t) const
    {
        // Before the first entry there is no protocol yet: neutral state. Without this branch the
        // cursor would stay at 0 and the transition pulse would fire immediately (|t - e.t| < pulse
        // also holds for t SMALLER than e.t), sending the robot to FixStand at time 0.
        if (t < entries_.front().t) return neutral_;
        while (cursor_ + 1 < entries_.size() && entries_[cursor_ + 1].t <= t) ++cursor_;
        while (cursor_ > 0 && entries_[cursor_].t > t) --cursor_;  // after a simulator reset
        return entries_[cursor_];
    }

    std::vector<Entry> entries_;
    Entry neutral_;
    mutable size_t cursor_ = 0;
    bool wall_after_rl_ = false, wall_anchored_ = false;
    double anchor_t_ = 0.0;
    std::chrono::steady_clock::time_point anchor_wall_;
    bool moving_ = false, moving_down_ = false;
    double over_left_ = 0.0;
    char over_key_ = 0;   // overhold in progress: 'L' bottom (A + RT), 'S' stop (RT), 'H' standing (B)
    bool has_stop_ = false;
    double stop_ = 0.0;
    bool stick_raw_ = false;
    std::string name_;
    double nominal_ = 0.98, deepest_ = 0.60, stop_at_ = -1.0, overhold_ = 0.0;
    double target_ = 0.98;
    double last_t_ = -1.0;
};
