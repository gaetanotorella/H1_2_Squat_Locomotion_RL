#pragma once
//
// MANUAL pushes for robustness tests: forces and torques applied to a body of the robot by pressing a
// KEY in the MuJoCo window, while driving the robot with the physical joystick. Every key is a preset
// (body, force/torque, duration, point, frame) defined in the YAML file given by
// `disturbance_script:` in config.yaml. Independent of the automated protocols.
//
//   # disturbances.yaml
//   presets:
//     - {key: "1", body: torso_link, force: [100, 0, 0],  dur: 0.2}   # 20 N.s forward
//     - {key: "2", body: torso_link, force: [-100, 0, 0], dur: 0.2}   # 20 N.s backward
//     - {key: "3", body: torso_link, force: [0, 100, 0],  dur: 0.2}   # 20 N.s to the left
//     - {key: "4", body: torso_link, force: [0, -100, 0], dur: 0.2}   # 20 N.s to the right
//     - {key: "5", body: pelvis,     torque: [0, 0, 30],  dur: 0.2}   # yaw torque
//     - {key: "6", body: torso_link, force: [150, 0, 0],  dur: 0.2, point: [0, 0, 0.3]}  # 30 cm above the origin
//
//   force in N, torque in N.m, dur in seconds (0 = a single physics step, 5 ms), point in metres in
//   the body frame (default: centre of mass; another point adds the equivalent torque (P - CoM) x F).
//   frame: "body" (DEFAULT: "forward" is the robot's forward, even after it turned) or "world".
//   Keys are single characters; 7, 8, 9 already belong to the elastic band.
//
// Mechanism: mjData::xfrc_applied[6*body] = force (3, world) + torque (3, world) at the CoM.
// CAUTION: Simulate::Sync() (simulate.cc, rendering thread, ~60 Hz) zeroes ALL of xfrc_applied at
// every frame, to clear the mouse perturbations: anything to be applied must be REWRITTEN in absolute
// value at every physics step, as the elastic band does. If the body is the band's, the value just
// written by the band (which acts first, in the same step) is kept and the push is added to it.
//
// Every applied push is logged to disturbance_events.csv in the run folder (start/end, simulation
// time, body, force, torque, point), when the ground-truth log is active: evaluation/report_pushes.py
// reads it with ground_truth.csv and obs.csv to measure the response.
//
#include <mujoco/mujoco.h>
#include <yaml-cpp/yaml.h>

#include <array>
#include <map>
#include <mutex>
#include <cctype>
#include <cmath>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

namespace pushes
{

class PushDisturbances
{
public:
    struct Event
    {
        double t0 = 0.0, t1 = 0.0;
        int body = -1;
        std::string body_name;
        std::array<double, 3> force{0, 0, 0}, torque{0, 0, 0}, point{0, 0, 0};
        bool body_frame = false, has_point = false;
        bool started = false, finished = false;
    };

    // Loads the presets from `path` (key `presets`). Returns false if it is missing or empty.
    bool load(const std::string & path, const mjModel * m)
    {
        YAML::Node cfg;
        try { cfg = YAML::LoadFile(path); }
        catch (const std::exception & ex) {
            std::cerr << "Pushes: cannot read " << path << ": " << ex.what() << std::endl;
            return false;
        }
        if (!cfg["presets"]) { std::cerr << "Pushes: no `presets` key in " << path << std::endl; return false; }

        std::cout << "Manual pushes from " << path << " -- keys in the MuJoCo window:" << std::endl;
        for (const auto & n : cfg["presets"])
        {
            Event e;
            e.t1 = (n["dur"] ? n["dur"].as<double>() : 0.0);   // here t1 = duration; t0 is set at trigger time
            e.body_name = n["body"] ? n["body"].as<std::string>() : "torso_link";
            e.body = mj_name2id(m, mjOBJ_BODY, e.body_name.c_str());
            if (e.body < 0) {
                std::cerr << "Pushes: body '" << e.body_name << "' does not exist in the model" << std::endl;
                std::exit(EXIT_FAILURE);
            }
            if (n["force"])  for (int i = 0; i < 3; i++) e.force[i]  = n["force"][i].as<double>();
            if (n["torque"]) for (int i = 0; i < 3; i++) e.torque[i] = n["torque"][i].as<double>();
            if (n["point"])  { e.has_point = true; for (int i = 0; i < 3; i++) e.point[i] = n["point"][i].as<double>(); }
            e.body_frame = !(n["frame"] && n["frame"].as<std::string>() == "world");   // default: body
            const std::string key = n["key"].as<std::string>();
            if (key.size() != 1) { std::cerr << "Pushes: `key` must be a single character (" << key << ")" << std::endl; std::exit(EXIT_FAILURE); }
            presets_[std::toupper(key[0])] = e;
            std::cout << "   [" << key << "]  " << e.body_name
                      << "  F=(" << e.force[0] << "," << e.force[1] << "," << e.force[2] << ") N"
                      << "  T=(" << e.torque[0] << "," << e.torque[1] << "," << e.torque[2] << ") N.m"
                      << "  " << e.t1 << " s  " << (e.body_frame ? "body" : "world")
                      << "  -> impulse " << std::sqrt(e.force[0]*e.force[0]+e.force[1]*e.force[1]+e.force[2]*e.force[2]) * std::max(e.t1, 0.005) << " N.s"
                      << std::endl;
        }
        return !presets_.empty();
    }

    bool empty() const { return presets_.empty(); }

    // Called from the keyboard callback: if `key` (GLFW, = upper-case ASCII code) is a preset, the push
    // is scheduled FROM NOW (current simulation time).
    void trigger(int key, double now)
    {
        auto it = presets_.find(key);
        if (it == presets_.end()) return;
        Event e = it->second;
        e.t0 = now; e.t1 = now + it->second.t1;
        {   // trigger() runs in the window thread, apply() in the physics thread
            std::lock_guard<std::mutex> lk(mtx_);
            pending_.push_back(e);
        }
        std::cout << "[push] key " << char(key) << " at t=" << now << " s: " << e.body_name << std::endl;
    }

    // Called at EVERY physics step, before mj_step, AFTER the elastic band.
    // band_body: body the band acts on (-1 if none); band_on: whether in this step the band has just
    // written its force to xfrc_applied[band_body].
    void apply(const mjModel * m, mjData * d, const std::string & log_dir, int band_body = -1, bool band_on = false)
    {
        std::map<int, std::array<double, 6>> acc_by_body;
        {   // take over the pushes requested from the keyboard
            std::lock_guard<std::mutex> lk(mtx_);
            for (auto & e : pending_) events_.push_back(e);
            pending_.clear();
        }

        const double t = d->time;
        for (auto & e : events_)
        {
            const bool active = (t >= e.t0) && (t < e.t1 || (!e.started && e.t1 <= e.t0));  // dur 0 -> one step
            if (!active) {
                if (e.started && !e.finished && t >= e.t1) { e.finished = true; log(e, "end", t, log_dir); }
                continue;
            }
            if (!e.started) { e.started = true; log(e, "start", t, log_dir); }
            if (e.t1 <= e.t0) e.finished = true;  // impulsive: switches off by itself at the next step

            // force and torque in the WORLD frame
            std::array<double, 3> F = e.force, T = e.torque;
            if (e.body_frame) { F = rot(d->xmat + 9 * e.body, e.force); T = rot(d->xmat + 9 * e.body, e.torque); }

            // application point other than the CoM -> equivalent torque (P - CoM) x F
            if (e.has_point)
            {
                const std::array<double, 3> pl = rot(d->xmat + 9 * e.body, e.point);
                std::array<double, 3> r;
                for (int i = 0; i < 3; i++) r[i] = (d->xpos[3 * e.body + i] + pl[i]) - d->xipos[3 * e.body + i];
                T[0] += r[1] * F[2] - r[2] * F[1];
                T[1] += r[2] * F[0] - r[0] * F[2];
                T[2] += r[0] * F[1] - r[1] * F[0];
            }

            auto & acc = acc_by_body[e.body];
            for (int i = 0; i < 3; i++) { acc[i] += F[i]; acc[3 + i] += T[i]; }
        }

        // ABSOLUTE write for every body touched in this step
        for (auto & kv : acc_by_body)
        {
            mjtNum * x = d->xfrc_applied + 6 * kv.first;
            const bool keep_band = (kv.first == band_body) && band_on;
            for (int i = 0; i < 6; i++)
                if (!(keep_band && i < 3)) x[i] = 0.0;      // the band's force stays, the rest restarts from 0
            for (int i = 0; i < 6; i++) x[i] += kv.second[i];
        }
        // bodies pushed at the previous step but not now: Sync() zeroes them at every frame, but
        // there are several physics steps between two frames, so they are zeroed here right away
        // to avoid leaving the force on for an extra step.
        for (int b : touched_)
            if (!acc_by_body.count(b) && b != band_body)
                for (int i = 0; i < 6; i++) d->xfrc_applied[6 * b + i] = 0.0;
        touched_.clear();
        for (auto & kv : acc_by_body) touched_.push_back(kv.first);
    }

private:
    static std::array<double, 3> rot(const mjtNum * R, const std::array<double, 3> & v)
    {
        return {R[0] * v[0] + R[1] * v[1] + R[2] * v[2],
                R[3] * v[0] + R[4] * v[1] + R[5] * v[2],
                R[6] * v[0] + R[7] * v[1] + R[8] * v[2]};
    }

    void log(const Event & e, const char * what, double t, const std::string & dir)
    {
        if (dir.empty()) return;
        if (!log_.is_open()) {
            log_.open(dir + "/disturbance_events.csv");
            log_ << "sim_t,event,body,fx,fy,fz,tx,ty,tz,frame,px,py,pz,t_planned,dur_planned\n";
        }
        log_ << t << "," << what << "," << e.body_name
             << "," << e.force[0] << "," << e.force[1] << "," << e.force[2]
             << "," << e.torque[0] << "," << e.torque[1] << "," << e.torque[2]
             << "," << (e.body_frame ? "body" : "world")
             << "," << e.point[0] << "," << e.point[1] << "," << e.point[2]
             << "," << e.t0 << "," << (e.t1 - e.t0) << "\n";
        log_.flush();
    }

    std::map<int, Event> presets_;
    std::vector<Event> events_, pending_;
    std::mutex mtx_;
    std::vector<int> touched_;
    std::ofstream log_;
};

inline PushDisturbances g_pushes;

}  // namespace pushes
