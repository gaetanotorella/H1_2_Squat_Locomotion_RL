// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.

#pragma once

#include <eigen3/Eigen/Dense>
#include <yaml-cpp/yaml.h>
#include "isaaclab/manager/observation_manager.h"
#include "isaaclab/manager/action_manager.h"
#include "isaaclab/assets/articulation/articulation.h"
#include "isaaclab/algorithms/algorithms.h"
#include <iostream>
#include "param.h"
#include "isaaclab/envs/run_logger.h"
#include "isaaclab/utils/utils.h"

namespace isaaclab
{

class ObservationManager;
class ActionManager;

class ManagerBasedRLEnv
{
public:
    // Constructor
    ManagerBasedRLEnv(YAML::Node cfg, std::shared_ptr<Articulation> robot_)
    :cfg(cfg), robot(std::move(robot_))
    {
        // Parse configuration
        this->step_dt = cfg["step_dt"].as<float>();
        robot->data.joint_ids_map = cfg["joint_ids_map"].as<std::vector<float>>();
        robot->data.joint_pos.resize(robot->data.joint_ids_map.size());
        robot->data.joint_vel.resize(robot->data.joint_ids_map.size());

        { // default joint positions
            auto default_joint_pos = cfg["default_joint_pos"].as<std::vector<float>>();
            robot->data.default_joint_pos = Eigen::VectorXf::Map(default_joint_pos.data(), default_joint_pos.size());
        }
        { // joint stiffness and damping
            robot->data.joint_stiffness = cfg["stiffness"].as<std::vector<float>>();
            robot->data.joint_damping = cfg["damping"].as<std::vector<float>>();
        }

        robot->update();

        // load managers
        action_manager = std::make_unique<ActionManager>(cfg["actions"], this);
        observation_manager = std::make_unique<ObservationManager>(cfg["observations"], this);
    }

    void reset()
    {
        global_phase = 0;
        episode_length = 0;
        // Every entry into the RL state starts standing (top of the height range): otherwise the height
        // would be the one at the previous exit, and the robot would squat again on its own. Done
        // before observation_manager->reset(), which already evaluates base_height_command.
        if (cfg["commands"]["base_height"])
            squat_target = cfg["commands"]["base_height"]["ranges"]["height"][1].as<float>();
        else
            squat_target = 0.98f;
        squat_shown = squat_target;
        squat_print_cooldown = 0;
        robot->update();
        action_manager->reset();
        observation_manager->reset();
    }

    void step()
    {
        episode_length += 1;
        robot->update();
        auto obs = observation_manager->compute();
        auto action = alg->act(obs);
        action_manager->process_action(action);

        if (param::log_enabled) {
            // the "obs" group, or the only group present
            auto it = obs.find("obs");
            const std::vector<float> * obs_vec = (it != obs.end()) ? &it->second
                                                 : (obs.empty() ? nullptr : &obs.begin()->second);
            logger.log(cfg, robot->data, action_manager->processed_actions(), obs_vec, episode_length, step_dt);
        }
    }

    float step_dt;
    
    YAML::Node cfg;

    std::unique_ptr<ObservationManager> observation_manager;
    std::unique_ptr<ActionManager> action_manager;
    std::shared_ptr<Articulation> robot;
    std::unique_ptr<Algorithms> alg;
    long episode_length = 0;
    float global_phase = 0.0f;
    float squat_target = 0.98f;  // squat task: current commanded base height [m] (held between steps)
    float squat_shown = 0.98f;   // squat task: last height printed to the terminal [m]
    int squat_print_cooldown = 0;  // squat task: steps before the next periodic height print

    RunLogger logger;  // CSV log, active only with --log
};

};