// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.

#pragma once

#include "isaaclab/envs/manager_based_rl_env.h"

#include <algorithm>
#include <cmath>
#include <spdlog/spdlog.h>

namespace isaaclab
{
namespace mdp
{

REGISTER_OBSERVATION(base_ang_vel)
{
    auto & asset = env->robot;
    auto & data = asset->data.root_ang_vel_b;
    return std::vector<float>(data.data(), data.data() + data.size());
}

REGISTER_OBSERVATION(projected_gravity)
{
    auto & asset = env->robot;
    auto & data = asset->data.projected_gravity_b;
    return std::vector<float>(data.data(), data.data() + data.size());
}

REGISTER_OBSERVATION(joint_pos)
{
    auto & asset = env->robot;
    std::vector<float> data;

    std::vector<int> joint_ids;
    try {
        joint_ids = params["asset_cfg"]["joint_ids"].as<std::vector<int>>();
    } catch(const std::exception& e) {
    }

    if(joint_ids.empty())
    {
        data.resize(asset->data.joint_pos.size());
        for(size_t i = 0; i < asset->data.joint_pos.size(); ++i)
        {
            data[i] = asset->data.joint_pos[i];
        }
    }
    else
    {
        data.resize(joint_ids.size());
        for(size_t i = 0; i < joint_ids.size(); ++i)
        {
            data[i] = asset->data.joint_pos[joint_ids[i]];
        }
    }

    return data;
}

REGISTER_OBSERVATION(joint_pos_rel)
{
    auto & asset = env->robot;
    std::vector<float> data;

    data.resize(asset->data.joint_pos.size());
    for(size_t i = 0; i < asset->data.joint_pos.size(); ++i) {
        data[i] = asset->data.joint_pos[i] - asset->data.default_joint_pos[i];
    }

    try {
        std::vector<int> joint_ids;
        joint_ids = params["asset_cfg"]["joint_ids"].as<std::vector<int>>();
        if(!joint_ids.empty()) {
            std::vector<float> tmp_data;
            tmp_data.resize(joint_ids.size());
            for(size_t i = 0; i < joint_ids.size(); ++i){
                tmp_data[i] = data[joint_ids[i]];
            }
            data = tmp_data;
        }
    } catch(const std::exception& e) {
    
    }

    return data;
}

REGISTER_OBSERVATION(joint_vel_rel)
{
    auto & asset = env->robot;
    auto data = asset->data.joint_vel;

    try {
        const std::vector<int> joint_ids = params["asset_cfg"]["joint_ids"].as<std::vector<int>>();

        if(!joint_ids.empty()) {
            data.resize(joint_ids.size());
            for(size_t i = 0; i < joint_ids.size(); ++i) {
                data[i] = asset->data.joint_vel[joint_ids[i]];
            }
        }
    } catch(const std::exception& e) {
    }
    return std::vector<float>(data.data(), data.data() + data.size());
}

REGISTER_OBSERVATION(last_action)
{
    auto data = env->action_manager->action();
    return std::vector<float>(data.data(), data.data() + data.size());
};

// Scale factor of the velocity command as a function of the squat height. It must match the
// training (UniformLevelVelocityCommand._update_command): all its parameters come from deploy.yaml,
// which is generated from the training configuration.
//   - cutoff_height (final policy): below this height the command is zeroed. In training, walking
//     environments never go below it, so the policy has never seen a velocity command there.
//     Above it the command is passed unchanged, as in training. It applies to both callers
//     (velocity_commands and gait_phase): with a zero command the gait clock stays frozen.
//   - speed_scale (policies trained with depth scaling): scale = clamp((h - h0) / (h1 - h0))^exponent.
// Without either key the factor is 1.0.
inline float squat_speed_scale(isaaclab::ManagerBasedRLEnv * env)
{
    const auto cmds = env->cfg["commands"]["base_velocity"];
    if (cmds["cutoff_height"] && env->squat_target < cmds["cutoff_height"].as<float>())
        return 0.0f;
    if (!cmds["speed_scale"])
        return 1.0f;

    const float h0 = cmds["speed_scale"]["zero_height"].as<float>();
    const float h1 = cmds["speed_scale"]["full_height"].as<float>();
    if (h1 <= h0)
        return 1.0f;

    const float s = std::clamp((env->squat_target - h0) / (h1 - h0), 0.0f, 1.0f);
    const float e = cmds["speed_scale"]["exponent"] ? cmds["speed_scale"]["exponent"].as<float>() : 1.0f;
    return (e == 1.0f) ? s : std::pow(s, e);
}

// "Command ~ zero" threshold of the gait clock, scaled with the speed scale; must match
// command_gate_threshold in training. The floor keeps the threshold above zero at the deepest squat,
// where the scale is 0: there the command is exactly 0 and the clock must stay frozen.
inline float squat_gait_threshold(isaaclab::ManagerBasedRLEnv * env, float base_threshold)
{
    const auto cmds = env->cfg["commands"]["base_velocity"];
    if (!cmds["speed_scale"])
        return base_threshold;

    const float floor_ = cmds["speed_scale"]["threshold_min_scale"]
                             ? cmds["speed_scale"]["threshold_min_scale"].as<float>()
                             : 0.2f;
    return base_threshold * std::max(squat_speed_scale(env), floor_);
}

REGISTER_OBSERVATION(velocity_commands)
{
    std::vector<float> obs(3);
    auto & joystick = env->robot->data.joystick;

    const auto cfg = env->cfg["commands"]["base_velocity"]["ranges"];

    obs[0] = std::clamp(joystick->ly(), cfg["lin_vel_x"][0].as<float>(), cfg["lin_vel_x"][1].as<float>());
    obs[1] = std::clamp(-joystick->lx(), cfg["lin_vel_y"][0].as<float>(), cfg["lin_vel_y"][1].as<float>());
    obs[2] = std::clamp(-joystick->rx(), cfg["ang_vel_z"][0].as<float>(), cfg["ang_vel_z"][1].as<float>());

    const float scale = squat_speed_scale(env);
    obs[0] *= scale;
    obs[1] *= scale;
    obs[2] *= scale;

    return obs;
}

REGISTER_OBSERVATION(gait_phase)
{
    float period = params["period"].as<float>();
    float delta_phase = env->step_dt * (1.0f / period);

    env->global_phase += delta_phase;
    env->global_phase = std::fmod(env->global_phase, 1.0f);

    std::vector<float> obs(2);
    obs[0] = std::sin(env->global_phase * 2 * M_PI);
    obs[1] = std::cos(env->global_phase * 2 * M_PI);

    // Freeze the clock output at (0, 0) when the joystick command is ~0, as in training
    // (mdp.gait_phase with command_name="base_velocity"): the internal clock keeps advancing, but
    // the output is zero when standing, which removes the 1.25 Hz sway.
    auto & joystick = env->robot->data.joystick;
    const auto cmd_cfg = env->cfg["commands"]["base_velocity"]["ranges"];
    float vx = std::clamp(joystick->ly(), cmd_cfg["lin_vel_x"][0].as<float>(), cmd_cfg["lin_vel_x"][1].as<float>());
    float vy = std::clamp(-joystick->lx(), cmd_cfg["lin_vel_y"][0].as<float>(), cmd_cfg["lin_vel_y"][1].as<float>());
    float wz = std::clamp(-joystick->rx(), cmd_cfg["ang_vel_z"][0].as<float>(), cmd_cfg["ang_vel_z"][1].as<float>());
    // The threshold applies to the SCALED command, as in training, where mdp.gait_phase reads the
    // command from the command manager, already scaled.
    const float gait_scale = squat_speed_scale(env);
    vx *= gait_scale;
    vy *= gait_scale;
    wz *= gait_scale;
    if (std::sqrt(vx * vx + vy * vy + wz * wz) <= squat_gait_threshold(env, 0.1f))
    {
        obs[0] = 0.0f;
        obs[1] = 0.0f;
    }
    return obs;
}

// Squat task: target base height relative to nominal (target - nominal), matching the training
// observation mdp.base_height_command_rel. The target is driven by the gamepad (see below).
REGISTER_OBSERVATION(base_height_command)
{
    const float nominal = params["nominal_height"].as<float>();
    const auto range = env->cfg["commands"]["base_height"]["ranges"]["height"];
    const float h_min = range[0].as<float>();  // deepest squat (0.60)
    const float h_max = range[1].as<float>();  // nominal / standing (e.g. 0.98)

    // Stop at the walking floor: R2 alone stops 1 cm above cutoff_height (0.70 -> 0.71), A + R2 goes
    // further down to h_min. The 1 cm margin is there because the ramp moves 1 mm per step in float:
    // a stop exactly on the cutoff could leave the target at 0.6999, and the cutoff would silently
    // zero the velocity commands. Without cutoff_height in deploy.yaml there is no stop (h_stop = h_min).
    const auto vel_cmds = env->cfg["commands"]["base_velocity"];
    const bool has_cutoff = static_cast<bool>(vel_cmds["cutoff_height"]);
    const float cutoff = has_cutoff ? vel_cmds["cutoff_height"].as<float>() : h_min;
    const float h_stop = has_cutoff ? std::clamp(cutoff + 0.01f, h_min, h_max) : h_min;

    auto & joystick = env->robot->data.joystick;
    const float descend_rate = 0.05f;  // [m/s] descent speed while RT is held
    const float rise_rate = 0.05f;     // [m/s] stand-up speed while B is held (slower -> no overshoot)

    // RT (R2) is digital on this pad -> "hold to squat": the longer RT is held, the deeper.
    // B -> stand up, RAMPED (not a snap) so the base does not overshoot the nominal height and
    // ring afterwards. Held state lives in env->squat_target.
    const float prev = env->squat_target;
    if (joystick->B.pressed)
        env->squat_target += rise_rate * env->step_dt;
    else if (joystick->RT.pressed)
    {
        // Below the stop, R2 alone does nothing (the limit must never move the target up):
        // going further down requires A.
        const float lower = joystick->A.pressed ? h_min : std::min(h_stop, prev);
        env->squat_target = std::max(prev - descend_rate * env->step_dt, lower);
    }
    env->squat_target = std::clamp(env->squat_target, h_min, h_max);

    // Print the height to the terminal, since it cannot be seen otherwise during a test: events
    // immediately, during a ramp at most every 0.25 s, and always the final value when it stops.
    const float h = env->squat_target;
    if (env->squat_print_cooldown > 0)
        env->squat_print_cooldown--;
    const bool stop_reached = has_cutoff && prev > h_stop && h == h_stop;
    const bool went_below = has_cutoff && prev >= cutoff && h < cutoff;
    const bool came_back = has_cutoff && prev < cutoff && h >= cutoff;
    const bool ramp_tick = h != prev && env->squat_print_cooldown == 0;
    const bool ramp_end = h == prev && h != env->squat_shown;
    if (stop_reached)
        spdlog::info("Squat: height {:.3f} m, stopped at the walking floor (hold A + R2 to go down to {:.2f} m)", h, h_min);
    else if (went_below)
        spdlog::warn("Squat: height {:.3f} m, below {:.2f} m: velocity commands DISABLED", h, cutoff);
    else if (came_back)
        spdlog::info("Squat: height {:.3f} m, back above {:.2f} m: velocity commands enabled", h, cutoff);
    else if (ramp_tick || ramp_end)
        spdlog::info("Squat: height {:.3f} m", h);
    if (stop_reached || went_below || came_back || ramp_tick || ramp_end)
    {
        env->squat_shown = h;
        env->squat_print_cooldown = static_cast<int>(std::lround(0.25f / env->step_dt));
    }

    std::vector<float> obs(1);
    obs[0] = env->squat_target - nominal;  // matches training obs base_height_command_rel
    return obs;
}

}
}