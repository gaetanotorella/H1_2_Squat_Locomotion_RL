#pragma once

// CSV log of the controller, written only when it is started with --log (see param.h).
//
// Every policy step appends one row to three files:
//   obs.csv       the policy observation vector, one named column per element
//   commands.csv  joint position targets, kp and kd (policy joint order)
//   errors.csv    target minus measured joint position
// Each row carries wall_t (system clock), the key used by evaluation/ to align these files with the
// simulator's ground_truth.csv.
//
// Destination, under param::log_dir (default <repo>/sim_logs):
//   - real robot (network interface other than "lo"): a new exp_<timestamp>/ folder;
//   - sim2sim ("lo"): the sim_<timestamp>/ folder the simulator is writing right now, recognised by
//     a ground_truth.csv modified less than 120 s ago;
//   - sim2sim without a logging simulator: a new ctrl_only_<timestamp>/ folder.
// run_info.md records which policy produced the log.

#include <yaml-cpp/yaml.h>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>
#include "param.h"
#include "isaaclab/assets/articulation/articulation.h"

namespace isaaclab
{

class RunLogger
{
public:
    void log(const YAML::Node & cfg, const ArticulationData & data, const std::vector<float> & q_target,
             const std::vector<float> * obs, long episode_length, float step_dt)
    {
        const size_t nj = q_target.size();
        if (!init_) {
            init_ = true;
            open(cfg, data, nj, obs);
        }

        const double wall_t = std::chrono::duration<double>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        const double ep_t = episode_length * step_dt;
        char wbuf[32];
        std::snprintf(wbuf, sizeof(wbuf), "%.6f", wall_t);  // seconds with microseconds

        obs_out_ << wbuf << "," << std::setprecision(7) << ep_t;
        if (obs) for (float v : *obs) obs_out_ << "," << v;
        obs_out_ << "\n"; obs_out_.flush();

        // kp and kd are stored in SDK order: remap them to policy order, kp_policy[i] = kp_sdk[jmap[i]]
        cmd_out_ << wbuf << "," << std::setprecision(7) << ep_t;
        for (size_t i = 0; i < nj; i++) cmd_out_ << "," << q_target[i];
        for (size_t i = 0; i < nj; i++) cmd_out_ << "," << data.joint_stiffness[jmap_[i]];
        for (size_t i = 0; i < nj; i++) cmd_out_ << "," << data.joint_damping[jmap_[i]];
        cmd_out_ << "\n"; cmd_out_.flush();

        err_out_ << wbuf << "," << std::setprecision(7) << ep_t;
        for (size_t i = 0; i < nj; i++) {
            float q_meas = (i < (size_t)data.joint_pos.size()) ? data.joint_pos[i] : 0.0f;
            err_out_ << "," << (q_target[i] - q_meas);
        }
        err_out_ << "\n"; err_out_.flush();
    }

private:
    static std::string stamp(const char * fmt)
    {
        std::time_t t = std::time(nullptr);
        char buf[64];
        std::strftime(buf, sizeof(buf), fmt, std::localtime(&t));
        return std::string(buf);
    }

    void open(const YAML::Node & cfg, const ArticulationData & data, size_t nj, const std::vector<float> * obs)
    {
        // joint names in policy order, without the _joint suffix; indices as a fallback
        std::vector<std::string> jn;
        if (cfg["joint_names"]) {
            for (auto n : cfg["joint_names"]) {
                std::string s = n.as<std::string>();
                if (s.size() > 6 && s.compare(s.size() - 6, 6, "_joint") == 0) s.resize(s.size() - 6);
                jn.push_back(s);
            }
        }
        if (jn.size() != nj) { jn.clear(); for (size_t i = 0; i < nj; i++) jn.push_back(std::to_string(i)); }

        // joint_ids_map (policy -> SDK), to write kp and kd in policy order
        jmap_.clear();
        for (auto v : data.joint_ids_map) jmap_.push_back((int)std::lround(v));
        if (jmap_.size() != nj) { jmap_.clear(); for (size_t i = 0; i < nj; i++) jmap_.push_back((int)i); }

        // observation column names, per term, in the order of cfg["observations"]
        auto sfx = [](const std::string & t) -> std::string {
            if (t.find("pos") != std::string::npos) return "pos";
            if (t.find("vel") != std::string::npos) return "vel";
            if (t.find("action") != std::string::npos) return "act";
            return t;
        };
        std::vector<std::string> obs_cols;
        if (cfg["observations"]) {
            for (auto it = cfg["observations"].begin(); it != cfg["observations"].end(); ++it) {
                std::string term = it->first.as<std::string>();
                if (term == "scale_first" || term == "use_gym_history") continue;
                size_t dim = it->second["scale"] ? it->second["scale"].size() : 0;
                // With history_length > 1 the observation manager concatenates PER TERM: H consecutive
                // frames of the same term, from the oldest to the most recent.
                int hist = it->second["history_length"] ? it->second["history_length"].as<int>(1) : 1;
                if (hist < 1) hist = 1;
                for (int h = 0; h < hist; ++h) {
                    // the current frame keeps the bare name, older ones get _hm<steps back>
                    const std::string age = (h == hist - 1) ? "" : "_hm" + std::to_string(hist - 1 - h);
                    if (dim == nj) { std::string s = sfx(term); for (auto & j : jn) obs_cols.push_back(j + "_" + s + age); }
                    else for (size_t k = 0; k < dim; k++) obs_cols.push_back(term + "_" + std::to_string(k) + age);
                }
            }
        }
        // if the count does not match, fall back to generic names
        if (obs && obs_cols.size() != obs->size()) {
            obs_cols.clear();
            for (size_t i = 0; i < obs->size(); i++) obs_cols.push_back("obs_" + std::to_string(i));
        }

        const std::filesystem::path base = param::log_dir;
        std::filesystem::path run_dir;
        std::error_code ec;
        std::filesystem::create_directories(base, ec);

        const std::string & iface = param::network_iface;
        const bool real_robot = !(iface.empty() || iface == "lo");

        if (real_robot) {
            run_dir = base / ("exp_" + stamp("%Y-%m-%d_%H-%M-%S"));
            std::filesystem::create_directories(run_dir, ec);
        } else {
            // Attach to the newest folder with a ground_truth.csv, only if that file is fresh: the
            // simulator is running now. Otherwise the CSVs of an old run would be overwritten. The
            // mtime of the FILE is checked, since appending to it does not touch the folder's mtime.
            std::string best;
            std::filesystem::file_time_type best_t{};
            for (auto & e : std::filesystem::directory_iterator(base, ec)) {
                if (!e.is_directory()) continue;
                std::filesystem::path g = e.path() / "ground_truth.csv";
                if (!std::filesystem::exists(g, ec)) continue;
                std::string name = e.path().filename().string();
                if (name > best) { best = name; best_t = std::filesystem::last_write_time(g, ec); }
            }
            bool fresh = false;
            if (!best.empty()) {
                auto age = std::filesystem::file_time_type::clock::now() - best_t;
                fresh = std::chrono::duration_cast<std::chrono::seconds>(age).count() < 120;
            }
            if (fresh) {
                run_dir = base / best;
            } else {
                run_dir = base / ("ctrl_only_" + stamp("%Y-%m-%d_%H-%M-%S"));
                std::filesystem::create_directories(run_dir, ec);
            }
        }

        write_run_info(cfg, run_dir, real_robot, obs ? obs->size() : 0, nj);

        obs_out_.open((run_dir / "obs.csv").string());
        cmd_out_.open((run_dir / "commands.csv").string());
        err_out_.open((run_dir / "errors.csv").string());

        obs_out_ << "wall_t,ep_t";
        for (auto & c : obs_cols) obs_out_ << "," << c;
        obs_out_ << "\n";

        cmd_out_ << "wall_t,ep_t";
        for (auto & j : jn) cmd_out_ << "," << j << "_q_target";
        for (auto & j : jn) cmd_out_ << "," << j << "_kp";
        for (auto & j : jn) cmd_out_ << "," << j << "_kd";
        cmd_out_ << "\n";

        err_out_ << "wall_t,ep_t";
        for (auto & j : jn) err_out_ << "," << j << "_q_err";
        err_out_ << "\n";

        std::cout << "[run_logger] obs/commands/errors -> " << run_dir << std::endl;
    }

    void write_run_info(const YAML::Node & cfg, const std::filesystem::path & run_dir, bool real_robot,
                        size_t n_obs, size_t nj)
    {
        std::ofstream info((run_dir / "run_info.md").string());
        auto yaml_range = [&](const char * cmd, const char * key) {
            try {
                auto n = cfg["commands"][cmd]["ranges"][key];
                char b[64];
                std::snprintf(b, sizeof(b), "[%.3g, %.3g]", n[0].as<float>(), n[1].as<float>());
                return std::string(b);
            } catch (...) { return std::string("n/a"); }
        };

        info << "# Run info\n\n";
        info << "- **policy_dir**: " << param::policy_dir.string() << "\n";
        info << "- **policy_name**: " << param::policy_dir.filename().string() << "\n";
        info << "- **controller**: " << param::bin_path.filename().string() << "\n";
        info << "- **network**: " << (param::network_iface.empty() ? "(default)" : param::network_iface)
             << (real_robot ? "  -> real robot" : "  -> sim2sim") << "\n";
        info << "- **started**: " << stamp("%Y-%m-%d %H:%M:%S") << "\n";
        info << "- **n_obs**: " << n_obs << "\n";
        info << "- **n_joints**: " << nj << "\n";
        info << "- **base_height_range**: " << yaml_range("base_height", "height") << "\n";
        info << "- **lin_vel_x_range**: " << yaml_range("base_velocity", "lin_vel_x") << "\n";
        info << "- **lin_vel_y_range**: " << yaml_range("base_velocity", "lin_vel_y") << "\n";
        info << "- **ang_vel_z_range**: " << yaml_range("base_velocity", "ang_vel_z") << "\n";
        std::error_code ec;
        auto onnx = param::policy_dir / "exported" / "policy.onnx";
        if (std::filesystem::exists(onnx, ec))
            info << "- **policy_onnx_bytes**: " << std::filesystem::file_size(onnx, ec) << "\n";
        std::cout << "[run_logger] policy = " << param::policy_dir.string() << std::endl;
    }

    bool init_ = false;
    std::vector<int> jmap_;  // policy joint i -> SDK joint index
    std::ofstream obs_out_, cmd_out_, err_out_;
};

}  // namespace isaaclab
