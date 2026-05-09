/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file mujoco_sim.h
 * @brief MuJoCo 仿真器公开接口
 *
 * SDK 用户只需包含此文件即可使用全部功能
 */
#ifndef MUJOCO_SIM_H
#define MUJOCO_SIM_H

#include <array>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace mujoco_sim {

/**
 * @brief 仿真器输出的机器人状态
 */
struct SimState {
    int num_dof = 0;
    std::vector<double> joint_pos;
    std::vector<double> joint_vel;
    std::array<double, 3> base_pos = {};
    std::array<double, 4> base_quat = {};
    std::array<double, 6> base_vel = {};  // 线速度(3) + 角速度(3)
    std::array<double, 3> gyro = {};
    std::array<double, 3> rpy = {};
    double time = 0.0;
};

/**
 * @brief 发给仿真器的控制指令
 */
struct SimControl {
    bool enable = false;
    std::vector<double> target_pos;
    std::vector<double> target_vel;
    std::vector<double> kp;
    std::vector<double> kd;
};

/**
 * @brief MuJoCo 配置（从 YAML 加载）
 */
struct MujocoConfig {
    std::string name;
    std::string xml_path;
    int num_dof = 0;
    double init_height = 0.85;
    double assist_height = 0.75;

    // 悬挂控制参数
    static constexpr double kAssistHeightRate = 5.0;  // 悬挂高度变化速率 (m/s)
    double assist_kp = 500.0;  // 悬挂 PD 刚度（默认适用于轻量机器人）
    double assist_kd = 100.0;  // 悬挂 PD 阻尼
    double assist_gravity_compensation = 0.0;  // 额外恒定向上力（N），用于重型机器人抵消重力

    // 仿真参数
    double sim_dt = 0.002;

    // 控制器参数
    std::vector<double> kp;
    std::vector<double> kd;
    std::vector<double> default_joint_pos;

    /**
     * @brief 从 YAML 文件加载 mujoco 自己的仿真参数
     * @param yaml_path YAML 配置文件路径（绝对路径或相对于 cwd 的路径）
     * @param robot_name 机器人名称（用于日志标识）
     * @param num_dof 控制自由度数量
     * @param xml_path MuJoCo XML 模型文件的绝对路径
     *
     * 仅读取 YAML 的 simulation.mujoco 节点（init_height / sim_dt / assist_* 等）。
     * default_joint_pos / kp / kd 等机器人固有属性不在此读取，由调用方通过
     * Simulator 构造函数显式传入。
     */
    static MujocoConfig FromYaml(const std::string &yaml_path,
                                const std::string &robot_name,
                                int num_dof,
                                const std::string &xml_path);
};

// ==================== 仿真器类 ====================

/**
 * @brief 每步回调：传入当前状态，返回控制指令（可选）
 *
 * 返回 std::nullopt 表示本步无新指令，仿真器保持上一帧控制状态不变。
 * 用于 driver_demo 等多进程场景，在每个仿真步之间注入 transport 收发逻辑。
 * 传入 nullptr 时 Run() 使用内部默认控制（保持 default_joint_pos）。
 */
using StepFn = std::function<std::optional<SimControl>(const SimState &)>;

/**
 * @brief MuJoCo 仿真器
 */
class Simulator {
public:
    /**
     * @param yaml_path YAML 配置文件路径（绝对路径或相对于 cwd 的路径）
     *                  仅用于读取 simulation.mujoco 节点
     * @param robot_name 机器人名称
     * @param num_dof 控制自由度数量
     * @param xml_path MuJoCo XML 模型文件的绝对路径
     * @param default_joint_pos 默认关节角度（自然站立姿态），大小 = num_dof
     * @param kp MuJoCo 启动初始 PD 刚度（可选，空则关节被动；进入 RL 后由控制命令覆盖）
     * @param kd MuJoCo 启动初始 PD 阻尼（可选，与 kp 配对）
     * @param assist 是否启用悬挂保护
     */
    Simulator(const std::string &yaml_path,
            const std::string &robot_name,
            int num_dof,
            const std::string &xml_path,
            const std::vector<double> &default_joint_pos,
            const std::vector<double> &kp = {},
            const std::vector<double> &kd = {},
            bool assist = true);
    ~Simulator();

    // 禁止拷贝
    Simulator(const Simulator &) = delete;
    Simulator &operator=(const Simulator &) = delete;

    // ---- 生命周期 ----

    /** @brief 重置仿真到初始状态 */
    void Reset();

    /**
     * @brief 运行仿真主循环
     *
     * 内部管理计时、实时同步、渲染跳帧。
     *
     * @param step_fn  每步回调（可选）：传入当前状态，返回控制指令；
     *                 传入 nullptr 时保持 default_joint_pos 不变。
     * @param continue_fn 继续条件（可选）：返回 false 时停止循环；
     *                    传入 nullptr 时仅依赖窗口关闭退出。
     * @param duration 运行时长（秒），-1 表示持续运行直到退出条件触发
     */
    void Run(StepFn step_fn = nullptr,
            std::function<bool()> continue_fn = nullptr,
            double duration = -1);

    // ---- 状态访问 ----

    /** @return 当前机器人状态 */
    SimState GetState() const;

    /**
     * @brief 设置完整的控制指令
     * @param ctrl 控制指令（包含 enable, target_pos, target_vel, kp, kd）
     */
    void SetControl(const SimControl &ctrl);

    /** @return 当前仿真配置（只读） */
    const MujocoConfig &GetConfig() const;

    // ---- 悬挂控制 ----

    /**
     * @brief 设置悬挂保护开关
     * @param enabled true 启用，false 禁用
     */
    void SetAssistEnabled(bool enabled);

    /** @brief 切换悬挂保护开关 */
    void ToggleAssist();

    /** @return 悬挂保护是否启用 */
    bool IsAssistEnabled() const;

    /**
     * @brief 设置目标悬挂高度
     * @param height 目标高度（m）
     */
    void SetAssistHeight(double height);

    /**
     * @brief 调整悬挂高度
     * @param delta_height 高度增量（m），正值上升
     */
    void AdjustAssistHeight(double delta_height);

    /** @return 目标悬挂高度（m） */
    double GetAssistHeight() const;

    /** @return 当前实际悬挂高度（m），平滑过渡中可能与目标不同 */
    double GetCurrentAssistHeight() const;

    // ---- 渲染 ----

    /** @return 窗口是否仍在运行 */
    bool IsAlive() const;

    /** @return 已执行的仿真步数 */
    int GetStepCount() const;

    /** @return 仿真时间（秒） */
    double GetSimTime() const;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace mujoco_sim

#endif  // MUJOCO_SIM_H
