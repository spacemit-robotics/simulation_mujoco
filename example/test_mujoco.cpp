/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file test_mujoco.cpp
 * @brief MuJoCo 仿真测试程序
 *
 * 用法:
 *   ./test_mujoco <yaml配置文件路径> <robot_name> <num_dof> <robot_dir> [-d 时长] [--no-assist]
 *
 *   robot_name / num_dof / robot_dir 需单独传入，模拟该调用方行为。
 *   生产场景中这些参数由上层调用方解析后传入。
 *
 * 示例:
 *   ./test_mujoco ../../../application/config/g1.yaml g1 29 ../../../application/robot/g1
 *   ./test_mujoco ../../../application/config/g1.yaml g1 29 ../../../application/robot/g1 -d 30
 *   ./test_mujoco ../../../application/config/g1.yaml g1 29 ../../../application/robot/g1
 * --no-assist
 */

#include <iostream>
#include <string>

#include "mujoco_sim.h"

using mujoco_sim::Simulator;

void PrintHelp(const char *argv0) {
    std::cout << "MuJoCo 仿真可视化\n\n";
    std::cout << "用法: " << argv0
            << " <yaml配置文件> <robot_name> <num_dof> <robot_dir> [选项]\n\n";
    std::cout << "必选参数:\n";
    std::cout << "  <yaml配置文件>      机器人 YAML 配置文件路径\n";
    std::cout << "  <robot_name>        机器人名称（如 g1）\n";
    std::cout << "  <num_dof>           关节自由度数量（如 29）\n";
    std::cout << "  <robot_dir>         机器人资源根目录（绝对路径）\n\n";
    std::cout << "可选参数:\n";
    std::cout << "  -d, --duration SEC  仿真时长（秒），不指定则持续运行\n";
    std::cout << "  --no-assist         禁用悬挂保护\n";
    std::cout << "  -h, --help          显示帮助\n\n";
    std::cout << "示例（从 output/ 目录运行）:\n";
    std::cout << "  " << argv0
            << " ../../../application/config/g1.yaml g1 29 ../../../application/robot/g1\n";
}

int main(int argc, char *argv[]) {
    if (argc < 2) {
        std::cerr << "[错误] 请指定配置文件路径\n\n";
        PrintHelp(argv[0]);
        return 1;
    }

    std::string arg1 = argv[1];
    if (arg1 == "-h" || arg1 == "--help") {
        PrintHelp(argv[0]);
        return 0;
    }

    if (argc < 5) {
        std::cerr << "[错误] 缺少必选参数\n\n";
        PrintHelp(argv[0]);
        return 1;
    }

    const std::string yaml_path = argv[1];
    const std::string robot_name = argv[2];
    const int num_dof = std::stoi(argv[3]);
    const std::string robot_dir = argv[4];
    const std::string xml_path = robot_dir + "/resources/xml/scene.xml";

    double duration = -1;
    bool assist = true;

    for (int i = 5; i < argc; i++) {
        std::string arg = argv[i];
        if ((arg == "-d" || arg == "--duration") && i + 1 < argc) {
            duration = std::stod(argv[++i]);
        } else if (arg == "--no-assist") {
            assist = false;
        }
    }

    try {
        Simulator sim(yaml_path, robot_name, num_dof, xml_path, assist);
        sim.Run(nullptr, nullptr, duration);
    } catch (const std::exception &e) {
        std::cerr << "[错误] " << e.what() << "\n";
        return 1;
    }

    return 0;
}
