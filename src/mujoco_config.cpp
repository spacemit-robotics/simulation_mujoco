/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file mujoco_config.cpp
 * @brief 机器人配置加载实现
 *
 * 从 YAML 读取 simulation.mujoco 节点下的仿真参数。
 * robot_name / num_dof / xml_path 由调用方传入。
 */

#include "mujoco_sim.h"

#include <yaml-cpp/yaml.h>

#include <filesystem>  // NOLINT(build/c++17)
#include <stdexcept>
#include <string>

namespace fs = std::filesystem;

namespace mujoco_sim {

MujocoConfig MujocoConfig::FromYaml(const std::string &yaml_path,
                                    const std::string &robot_name,
                                    int num_dof,
                                    const std::string &xml_path) {
    const fs::path yaml_abs = fs::absolute(yaml_path);
    if (!fs::exists(yaml_abs)) {
        throw std::runtime_error("[MujocoConfig] 配置文件不存在: " + yaml_abs.string());
    }

    YAML::Node cfg = YAML::LoadFile(yaml_abs.string());

    MujocoConfig config;

    // ==================== 调用方传入的参数 ====================

    config.name = robot_name;
    config.num_dof = num_dof;
    config.xml_path = xml_path;

    if (!fs::exists(config.xml_path)) {
        throw std::runtime_error("[MujocoConfig] MuJoCo XML 文件不存在: " + config.xml_path);
    }

    // ==================== simulation.mujoco 配置 ====================

    const auto sim = cfg["simulation"]["mujoco"];
    if (!sim) {
        throw std::runtime_error("[MujocoConfig] 配置文件缺少 simulation.mujoco 节点");
    }

    // ==================== 仿真参数 ====================

    config.init_height = sim["init_height"] ? sim["init_height"].as<double>() : 0.85;
    config.assist_height = sim["assist_height"] ? sim["assist_height"].as<double>() : 0.75;

    if (!sim["sim_dt"]) {
        throw std::runtime_error("[MujocoConfig] 配置文件缺少 simulation.mujoco.sim_dt");
    }
    config.sim_dt = sim["sim_dt"].as<double>();

    config.assist_kp = sim["assist_kp"] ? sim["assist_kp"].as<double>() : 500.0;
    config.assist_kd = sim["assist_kd"] ? sim["assist_kd"].as<double>() : 100.0;
    config.assist_gravity_compensation =
        sim["assist_gravity_compensation"] ? sim["assist_gravity_compensation"].as<double>() : 0.0;

    // ==================== 控制器参数 ====================

    const auto ctrl = sim["controller"];
    if (!ctrl || !ctrl["kp"]) {
        throw std::runtime_error("[MujocoConfig] 配置文件缺少 simulation.mujoco.controller.kp");
    }
    if (!ctrl["kd"]) {
        throw std::runtime_error("[MujocoConfig] 配置文件缺少 simulation.mujoco.controller.kd");
    }
    if (!ctrl["default_joint_pos"]) {
        throw std::runtime_error(
            "[MujocoConfig] 配置文件缺少 simulation.mujoco.controller.default_joint_pos");
    }

    config.kp = ctrl["kp"].as<std::vector<double>>();
    config.kd = ctrl["kd"].as<std::vector<double>>();
    config.default_joint_pos = ctrl["default_joint_pos"].as<std::vector<double>>();

    return config;
}

}  // namespace mujoco_sim
