// Ground-truth logger of the MuJoCo simulator, active with log_ground_truth: 1 or --log.
// Writes the TRUE base state (pose + velocity) read from mjData to
// <log_dir>/sim_<YYYY-MM-DD_HH-MM-SS>/ground_truth.csv. The controller started with --log writes its
// obs.csv / commands.csv / errors.csv to the same folder (it attaches to the newest fresh sim_*).
// The two processes are aligned through wall_t (system clock, same PC).
#pragma once

#include <mujoco/mujoco.h>
#include <fstream>
#include <filesystem>
#include <string>
#include <sstream>
#include <iomanip>
#include <iostream>
#include <chrono>
#include <ctime>
#include <cstdlib>
#include <cstdio>

namespace sim_logger
{

inline std::string g_base_dir;  // folder of the logs, set by main.cc
inline std::string g_run_dir;   // folder of the current run (created at the first log)

inline std::string make_timestamp()
{
    std::time_t t = std::time(nullptr);
    std::tm tm = *std::localtime(&t);
    std::ostringstream ss;
    ss << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S");
    return ss.str();
}

inline double wall_seconds()
{
    using namespace std::chrono;
    return duration<double>(system_clock::now().time_since_epoch()).count();
}

// Called at every physics step (internally downsampled to ~100 Hz).
// qpos: [0..2]=x,y,z base, [3..6]=quat(w,x,y,z).
// qvel: [0..2]=linear velocity (world), [3..5]=angular velocity (body frame).
inline void log_ground_truth(const mjData *d)
{
    static std::ofstream out;
    static double last_t = -1e9;
    static bool init = false;

    if (!init)
    {
        init = true;
        g_run_dir = g_base_dir + "/sim_" + make_timestamp();
        std::error_code ec;
        std::filesystem::create_directories(g_run_dir, ec);
        out.open(g_run_dir + "/ground_truth.csv");
        out << "wall_t,sim_t,base_x,base_y,base_z,qw,qx,qy,qz,vx,vy,vz,wx,wy,wz\n";
        std::cout << "[sim_logger] ground_truth -> " << g_run_dir << std::endl;
    }

    // downsampling to ~100 Hz (physics at 200 Hz)
    if (d->time - last_t < 0.01 - 1e-9)
        return;
    last_t = d->time;

    char wbuf[32];
    std::snprintf(wbuf, sizeof(wbuf), "%.6f", wall_seconds());
    out << wbuf << std::setprecision(9)
        << "," << d->time
        << "," << d->qpos[0] << "," << d->qpos[1] << "," << d->qpos[2]
        << "," << d->qpos[3] << "," << d->qpos[4] << "," << d->qpos[5] << "," << d->qpos[6]
        << "," << d->qvel[0] << "," << d->qvel[1] << "," << d->qvel[2]
        << "," << d->qvel[3] << "," << d->qvel[4] << "," << d->qvel[5] << "\n";
    out.flush();
}

} // namespace sim_logger
