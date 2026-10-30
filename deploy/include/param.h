// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.

#pragma once

#include <stdint.h>
#include <chrono>
#include <iostream>
#include <boost/program_options.hpp>
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/rotating_file_sink.h>
#include <memory>
#include <iomanip>

/* ---------- logger ---------- */
namespace spdlog
{
inline void create_logger(std::string log_path)
{
    auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    auto rotating_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(log_path, 5 * 1024 * 1024, 5);

    std::vector<spdlog::sink_ptr> sinks {console_sink, rotating_sink};
    auto logger = std::make_shared<spdlog::logger>("unitree", sinks.begin(), sinks.end());

    logger->set_pattern("[%Y-%m-%d %H:%M:%S] [%^%l%$] %v");
    logger->flush_on(spdlog::level::info);

    spdlog::set_default_logger(logger);
}

} // namespace spdlog


namespace param
{
inline std::string VERSION = "1.0.0.1";
inline std::filesystem::path bin_path;
inline std::filesystem::path proj_dir;
inline std::filesystem::path config_dir;
inline YAML::Node config;

// DDS interface chosen with --network: "lo" (or empty) is sim2sim on this PC, any other interface
// (eth0, enp3s0, ...) is the real robot. The run logger uses it to name the log folder.
inline std::string network_iface;

// Policy folder actually loaded (resolved by parser_policy_dir), written by the run logger to
// run_info.md so that every log can be traced back to its policy.
inline std::filesystem::path policy_dir;

// CSV run log (isaaclab/envs/run_logger.h), enabled with --log. Default folder: <repo>/sim_logs.
inline bool log_enabled = false;
inline std::filesystem::path log_dir;

inline std::filesystem::path get_bin_path() {
    std::vector<char> path(1024);
    ssize_t len = readlink("/proc/self/exe", &path[0], path.size());
    if (len != -1) {
        path[len] = '\0';  // Null-terminate the result
        return std::filesystem::path(&path[0]);
    } else {
        spdlog::error("Failed to get executable path.");
        exit(1);
    }
}

/* ---------- config.yaml ---------- */
// proj_dir and config_dir depend only on where the binary is, never on the chosen config file:
// parser_policy_dir resolves relative paths against proj_dir.
inline void resolve_paths()
{
    assert(std::filesystem::exists(bin_path)); // run param::helper before this function
    if(bin_path.parent_path().filename() == "bin" || bin_path.parent_path().filename() == "build")
    {
        proj_dir = bin_path.parent_path().parent_path();
        config_dir = proj_dir / "config";
    }
    else
    {
        proj_dir = bin_path.parent_path();
        config_dir = proj_dir;
    }
}

// Empty config_override: the config.yaml next to the binary. Otherwise (--config) that file only.
// The automated evaluation uses it to choose the policy under test without editing config.yaml,
// which is the same file used on the real robot.
inline void load_config_file(const std::string & config_override = "")
{
    resolve_paths();

    std::string config_file = config_override.empty()
                                  ? (config_dir / "config.yaml").string()
                                  : config_override;

    if(!config_override.empty())
    {
        if(!std::filesystem::exists(config_file))
        {
            spdlog::critical("Config file not found: {}", config_file);
            exit(1);
        }
        spdlog::info("Config file (override): {}", config_file);
    }

    try {
        if(std::filesystem::exists(config_file))
        {
            config = YAML::LoadFile(config_file);
        }
    } catch (const YAML::BadFile& e) {
        spdlog::error("Failed to load config.yaml: {}", e.what());
        exit(1);
    }
}

inline std::filesystem::path parser_policy_dir(std::filesystem::path policy_dir)
{
    // Load Policy
    if (policy_dir.is_relative()) {
        policy_dir = param::proj_dir / policy_dir;
    }

    // If there is no `exported` folder in this folder,
    // then sort all the folders under this folder and take the last folder
    if (!std::filesystem::exists(policy_dir / "exported")) {
        auto dirs = std::filesystem::directory_iterator(policy_dir);
        std::vector<std::filesystem::path> dir_list;
        for (const auto& entry : dirs) {
            if (entry.is_directory()) {
                dir_list.push_back(entry.path());
            }
        }
        if (!dir_list.empty()) {
            std::sort(dir_list.begin(), dir_list.end());
            // Check if there is an `exported` folder starting from the last folder
            for (auto it = dir_list.rbegin(); it != dir_list.rend(); ++it) {
                if (std::filesystem::exists(*it / "exported")) {
                    policy_dir = *it;
                    break;
                }
            }
        }
    }
    spdlog::info("Policy directory: {}", policy_dir.string());
    param::policy_dir = policy_dir;  // recorded in run_info.md by the run logger
    return policy_dir;
}

/* ---------- Command Line Parameters ---------- */
namespace po = boost::program_options;

//※ This function must be called at the beginning of main() function
inline po::variables_map helper(int argc, char** argv) 
{
    bin_path = get_bin_path();
    resolve_paths();

    po::options_description desc("Unitree Controller");
    desc.add_options()
        ("help,h", "produce help message")
        ("version,v", "show version")
        ("log", "record log files: terminal output in log/log.txt and the CSV run log in --log-dir")
        ("log-dir", po::value<std::string>()->default_value(""), "folder of the CSV run logs (default: <repo>/sim_logs)")
        ("network,n", po::value<std::string>()->default_value(""), "dds network interface")
        ("config,c", po::value<std::string>()->default_value(""),
         "alternative config.yaml (default: the one next to the binary)")
        ;

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    // the config is loaded after parsing, to honour --config
    load_config_file(vm["config"].as<std::string>());

    network_iface = vm["network"].as<std::string>();

    if (vm.count("help"))
    {
        std::cout << desc << std::endl;
        exit(0);
    }
    if (vm.count("version"))
    {
        std::cout << "Version: " << VERSION << std::endl;
        exit(0);
    }

#ifndef NDEBUG
    spdlog::set_level(spdlog::level::debug);
#else
    spdlog::set_level(spdlog::level::info);
#endif
    if(vm.count("log"))
    {
        std::filesystem::create_directories(proj_dir / "log");
        spdlog::create_logger(proj_dir.string() + "/log/log.txt");

        // CSV run log: proj_dir is deploy/robots/<robot>, so the repository root is three levels up
        log_enabled = true;
        const std::string dir = vm["log-dir"].as<std::string>();
        log_dir = dir.empty() ? proj_dir.parent_path().parent_path().parent_path() / "sim_logs"
                              : std::filesystem::absolute(dir);
        spdlog::info("CSV run log in {}", log_dir.string());
    }

    return vm;
}

}