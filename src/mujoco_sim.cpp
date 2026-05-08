/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file mujoco_sim.cpp
 * @brief MuJoCo 仿真器实现
 */

#include "mujoco_sim.h"

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>  // NOLINT(build/c++17)
#include <iostream>
#include <string>
#include <thread>
#include <vector>

namespace fs = std::filesystem;

namespace mujoco_sim {

// ==================== 常量定义 ====================

// 窗口配置
constexpr int kWindowWidth = 900;
constexpr int kWindowHeight = 600;

// 渲染配置
constexpr int kMaxSceneObjects = 2000;

// 相机默认参数
constexpr double kCameraLookatZ = 0.9;
constexpr double kCameraAzimuth = 90.0;
constexpr double kCameraElevation = -5.0;
constexpr double kCameraDistance = 3.0;

// 悬挂力限制缩放系数
constexpr double kForceScaleXY = 0.2;
constexpr double kForceScaleZ = 2.0;
constexpr double kForceScaleRot = 0.1;

// ==================== 辅助函数 ====================

static void ClipValue(double &v, double max_val) {
    v = std::clamp(v, -max_val, max_val);
}

// 从四元数 [w,x,y,z] 计算 roll/pitch/yaw
static std::array<double, 3> QuatToRpy(const std::array<double, 4> &q) {
    double w = q[0], x = q[1], y = q[2], z = q[3];
    double sinp = 2.0 * (w * y - z * x);
    return {
        std::atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y)),
        std::abs(sinp) >= 1.0 ? std::copysign(M_PI / 2.0, sinp) : std::asin(sinp),
        std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)),
    };
}

// 从 base_vel[3:6]（MuJoCo 世界系角速度）和 base_quat 填充 gyro 和 rpy
static void ComputeImuData(SimState &state) {
    state.rpy = QuatToRpy(state.base_quat);
    state.gyro = {state.base_vel[3], state.base_vel[4], state.base_vel[5]};
}

// ==================== Impl 类定义 ====================

class Simulator::Impl {
public:
    // MuJoCo 对象
    mjModel *model = nullptr;
    mjData *data = nullptr;
    mjvCamera cam;
    mjvOption opt;
    mjvScene scn;
    mjrContext con;
    GLFWwindow *window = nullptr;

    // 配置
    MujocoConfig config;
    bool has_floating_base_ = false;
    int base_body_id_ = 1;

    // 控制器状态
    std::vector<double> target_pos_;
    std::vector<double> target_vel_;
    std::vector<double> current_kp_;
    std::vector<double> current_kd_;
    bool control_enabled_ = true;
    bool is_position_actuator_ = false;

    // 悬挂控制
    bool assist_enabled_ = true;
    double assist_kp_ = 500.0;
    double assist_kd_ = 100.0;
    double assist_gravity_compensation_ = 0.0;
    double target_assist_height_ = 0.75;   // 目标悬挂高度
    double current_assist_height_ = 0.75;  // 当前悬挂高度（用于平滑过渡）

    // 仿真状态
    int step_count_ = 0;
    int render_skip_ = 16;
    bool window_alive_ = true;

    // 鼠标状态
    bool button_left_ = false;
    bool button_middle_ = false;
    bool button_right_ = false;
    double last_x_ = 0;
    double last_y_ = 0;

    // 执行器总开关：兼容 position/motor 等不同 actuator 类型
    void SetActuationEnabled(bool enabled) {
        if (!model) {
            return;
        }
        if (enabled) {
            model->opt.disableflags &= ~mjDSBL_ACTUATION;
        } else {
            model->opt.disableflags |= mjDSBL_ACTUATION;
        }
    }

    // ==================== 初始化 ====================

    void Init(const std::string &yaml_path,
            const std::string &robot_name,
            int num_dof,
            const std::string &xml_path,
            const std::vector<double> &default_joint_pos,
            const std::vector<double> &kp,
            const std::vector<double> &kd,
            bool assist) {
        // 加载 mujoco 自己的仿真参数
        config = MujocoConfig::FromYaml(yaml_path, robot_name, num_dof, xml_path);
        // 机器人固有属性由调用方传入
        config.default_joint_pos = default_joint_pos;
        config.kp = kp;
        config.kd = kd;
        std::cout << "[MuJoCo] 机器人: " << config.name << std::endl;

        // 加载 MuJoCo 模型
        fs::path xml_file_path(config.xml_path);
        if (!fs::exists(xml_file_path)) {
            throw std::runtime_error("XML 文件不存在: " + xml_file_path.string());
        }

        std::string old_cwd = fs::current_path().string();
        fs::path xml_dir = xml_file_path.parent_path();
        if (fs::exists(xml_dir)) {
            fs::current_path(xml_dir);
        }

        char error[1000] = "";
        model = mj_loadXML(xml_file_path.filename().string().c_str(), nullptr, error, 1000);
        if (!model) {
            fs::current_path(old_cwd);
            throw std::runtime_error(std::string("MuJoCo 加载失败: ") + error);
        }

        data = mj_makeData(model);
        model->opt.timestep = config.sim_dt;
        fs::current_path(old_cwd);

        // 检测浮动基座
        has_floating_base_ = model->nq > config.num_dof;
        base_body_id_ = FindFloatingBaseBody();

        // 检测执行器类型
        is_position_actuator_ = (config.num_dof > 0 && model->actuator_biastype[0] == 1);

        // 悬挂PD增益（从YAML配置加载，不同机器人可独立配置）
        assist_kp_ = config.assist_kp;
        assist_kd_ = config.assist_kd;
        assist_gravity_compensation_ = config.assist_gravity_compensation;

        // 初始化控制器
        target_pos_ = config.default_joint_pos;
        target_vel_.assign(config.num_dof, 0.0);
        current_kp_ = config.kp;
        current_kd_ = config.kd;
        control_enabled_ = true;
        SetActuationEnabled(true);
        assist_enabled_ = assist;
        target_assist_height_ = config.assist_height;
        current_assist_height_ = config.assist_height;

        // 初始化 GLFW
        if (!glfwInit()) {
            throw std::runtime_error("GLFW 初始化失败");
        }

        window =
            glfwCreateWindow(kWindowWidth, kWindowHeight, "MuJoCo Simulator", nullptr, nullptr);
        glfwMakeContextCurrent(window);
        glfwSwapInterval(1);

        // 窗口置顶（GLFW 3.2+ 才支持）
#if GLFW_VERSION_MAJOR > 3 || (GLFW_VERSION_MAJOR == 3 && GLFW_VERSION_MINOR >= 2)
        glfwSetWindowAttrib(window, GLFW_FLOATING, GLFW_TRUE);
        std::cout << "[MuJoCo] 窗口置顶: 成功" << std::endl;
#else
        std::cout << "[MuJoCo] 窗口置顶: 不支持 (需要 GLFW 3.2+)" << std::endl;
#endif

        // 初始化渲染
        mjv_defaultCamera(&cam);
        cam.lookat[2] = kCameraLookatZ;
        cam.azimuth = kCameraAzimuth;
        cam.elevation = kCameraElevation;
        cam.distance = kCameraDistance;

        mjv_defaultOption(&opt);
        mjv_defaultScene(&scn);
        mjr_defaultContext(&con);
        mjv_makeScene(model, &scn, kMaxSceneObjects);
        mjr_makeContext(model, &con, mjFONTSCALE_100);

        // 设置回调
        glfwSetWindowUserPointer(window, this);
        glfwSetKeyCallback(window, KeyCallback);
        glfwSetCursorPosCallback(window, MouseMove);
        glfwSetMouseButtonCallback(window, MouseButton);
        glfwSetScrollCallback(window, Scroll);

        PrintInfo();
    }

    void PrintInfo() {
        std::cout << "[MuJoCo] 仿真频率: " << static_cast<int>(1.0 / model->opt.timestep) << " Hz"
                << std::endl;
        std::cout << "[MuJoCo] 自由度: " << config.num_dof << std::endl;
        std::cout << "[MuJoCo] 悬挂保护: " << (assist_enabled_ ? "启用" : "禁用")
                << " (高度: " << current_assist_height_ << "m)" << std::endl;
        std::cout << "[MuJoCo] 按 Ctrl+C 退出" << std::endl;
        std::cout << "[MuJoCo] 按 F 键切换悬挂保护" << std::endl;
        std::cout << "[MuJoCo] 按 ↑/↓ 键调节悬挂高度 (±5cm)" << std::endl;
        std::cout << "[MuJoCo] 按 R 键重置悬挂高度到默认值" << std::endl;
    }

    int FindFloatingBaseBody() {
        for (int i = 0; i < model->njnt; i++) {
            if (model->jnt_type[i] == mjJNT_FREE) {
                return model->jnt_bodyid[i];
            }
        }
        return 1;
    }

    // ==================== 回调函数 ====================

    static void KeyCallback(GLFWwindow *win, int key, int scancode, int act, int mods) {
        auto *impl = static_cast<Impl *>(glfwGetWindowUserPointer(win));
        if (act == GLFW_PRESS) {
            if (key == GLFW_KEY_F) {
                impl->assist_enabled_ = !impl->assist_enabled_;
                std::cout << "\n[MuJoCo] 悬挂保护: " << (impl->assist_enabled_ ? "启用" : "禁用")
                        << std::endl;
            } else if (key == GLFW_KEY_UP) {
                impl->AdjustAssistHeight(0.05);  // 增加5cm
                std::cout << "\n[MuJoCo] 目标悬挂高度: " << impl->GetAssistHeight() << "m"
                        << " (当前: " << impl->GetCurrentAssistHeight() << "m)" << std::endl;
            } else if (key == GLFW_KEY_DOWN) {
                impl->AdjustAssistHeight(-0.05);  // 减少5cm
                std::cout << "\n[MuJoCo] 目标悬挂高度: " << impl->GetAssistHeight() << "m"
                        << " (当前: " << impl->GetCurrentAssistHeight() << "m)" << std::endl;
            } else if (key == GLFW_KEY_R) {
                impl->SetAssistHeight(impl->config.assist_height);  // 重置到默认高度
                std::cout << "\n[MuJoCo] 重置悬挂高度到默认值: " << impl->GetAssistHeight() << "m"
                        << std::endl;
            }
        }
    }

    static void MouseButton(GLFWwindow *win, int button, int act, int mods) {
        auto *impl = static_cast<Impl *>(glfwGetWindowUserPointer(win));
        impl->button_left_ = (glfwGetMouseButton(win, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
        impl->button_middle_ = (glfwGetMouseButton(win, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
        impl->button_right_ = (glfwGetMouseButton(win, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);
        glfwGetCursorPos(win, &impl->last_x_, &impl->last_y_);
    }

    static void MouseMove(GLFWwindow *win, double xpos, double ypos) {
        auto *impl = static_cast<Impl *>(glfwGetWindowUserPointer(win));
        if (!impl->button_left_ && !impl->button_middle_ && !impl->button_right_)
            return;

        double dx = xpos - impl->last_x_;
        double dy = ypos - impl->last_y_;
        impl->last_x_ = xpos;
        impl->last_y_ = ypos;

        int width, height;
        glfwGetWindowSize(win, &width, &height);

        bool mod_shift = (glfwGetKey(win, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
                        glfwGetKey(win, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

        mjtMouse action;
        if (impl->button_right_) {
            action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
        } else if (impl->button_left_) {
            action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
        } else {
            action = mjMOUSE_ZOOM;
        }

        mjv_moveCamera(impl->model, action, dx / height, dy / height, &impl->scn, &impl->cam);
    }

    static void Scroll(GLFWwindow *win, double xoffset, double yoffset) {
        auto *impl = static_cast<Impl *>(glfwGetWindowUserPointer(win));
        mjv_moveCamera(impl->model, mjMOUSE_ZOOM, 0, 0.05 * yoffset, &impl->scn, &impl->cam);
    }

    // ==================== 仿真控制 ====================

    void Reset() {
        // 首先重置所有 qpos 到模型默认值
        mju_copy(data->qpos, model->qpos0, model->nq);

        // 设置浮动基座位置
        if (has_floating_base_) {
            data->qpos[0] = 0;
            data->qpos[1] = 0;
            data->qpos[2] = config.init_height;
            data->qpos[3] = 1;  // w
            data->qpos[4] = 0;  // x
            data->qpos[5] = 0;  // y
            data->qpos[6] = 0;  // z
        }

        // 根据 actuator->joint 映射设置关节位置
        for (int i = 0; i < model->nu && i < config.num_dof; i++) {
            int jnt_id = model->actuator_trnid[2 * i];
            int qpos_adr = model->jnt_qposadr[jnt_id];
            data->qpos[qpos_adr] = config.default_joint_pos[i];
        }

        mju_zero(data->qvel, model->nv);
        mj_forward(model, data);
        step_count_ = 0;
        target_pos_ = config.default_joint_pos;
        target_vel_.assign(config.num_dof, 0.0);
        current_kp_ = config.kp;
        current_kd_ = config.kd;
        control_enabled_ = true;
        SetActuationEnabled(true);
        target_assist_height_ = config.assist_height;
        current_assist_height_ = config.assist_height;
    }

    void UpdateAssistHeight() {
        if (!assist_enabled_)
            return;

        // 平滑过渡到目标悬挂高度
        double height_diff = target_assist_height_ - current_assist_height_;
        double max_change = MujocoConfig::kAssistHeightRate * model->opt.timestep;

        if (std::abs(height_diff) > max_change) {
            current_assist_height_ += std::copysign(max_change, height_diff);
        } else {
            current_assist_height_ = target_assist_height_;
        }

        // 无高度限制，可以无限调节
    }

    void ApplyAssistForce() {
        if (!has_floating_base_ || !assist_enabled_) {
            if (has_floating_base_) {
                for (int i = 0; i < 6; i++) {
                    data->xfrc_applied[base_body_id_ * 6 + i] = 0;
                }
            }
            return;
        }

        // 悬挂保护力限制（根据 assist_kp_ 自适应缩放）
        const double max_force_xy = std::max(100.0, assist_kp_ * kForceScaleXY);
        const double max_force_z = std::max(1000.0, assist_kp_ * kForceScaleZ);
        const double max_force_rot = std::max(50.0, assist_kp_ * kForceScaleRot);

        // 位置恢复力 (x, y)
        for (int i = 0; i < 2; i++) {
            double force = -assist_kp_ * 0.5 * data->qpos[i] - assist_kd_ * 0.5 * data->qvel[i];
            ClipValue(force, max_force_xy);
            data->xfrc_applied[base_body_id_ * 6 + i] = force;
        }

        // z 方向悬挂（使用当前悬挂高度）+ 重力补偿
        double force_z =
            assist_kp_ * (current_assist_height_ - data->qpos[2]) - assist_kd_ * data->qvel[2];
        ClipValue(force_z, max_force_z);
        data->xfrc_applied[base_body_id_ * 6 + 2] = force_z + assist_gravity_compensation_;

        // 姿态恢复力
        const mjtNum *q = data->qpos + 3;
        double roll =
            std::atan2(2 * (q[0] * q[1] + q[2] * q[3]), 1 - 2 * (q[1] * q[1] + q[2] * q[2]));
        double pitch = std::asin(std::clamp(2 * (q[0] * q[2] - q[3] * q[1]), -1.0, 1.0));
        double yaw =
            std::atan2(2 * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (q[2] * q[2] + q[3] * q[3]));

        double angles[3] = {roll, pitch, yaw};
        for (int i = 0; i < 3; i++) {
            double force = -assist_kp_ * angles[i] - assist_kd_ * data->qvel[3 + i];
            ClipValue(force, max_force_rot);
            data->xfrc_applied[base_body_id_ * 6 + 3 + i] = force;
        }
    }

    void ApplyControl() {
        if (!control_enabled_) {
            // 掉电语义：完全关闭 actuator 力（兼容 position actuator）
            SetActuationEnabled(false);
            // 实现掉电：直接清零执行器控制值
            for (int i = 0; i < model->nu; i++) {
                data->ctrl[i] = 0;
            }
            return;
        }

        // 恢复 actuator 力
        SetActuationEnabled(true);

        for (int i = 0; i < model->nu && i < config.num_dof; i++) {
            // 获取执行器对应的 joint id
            int jnt_id = model->actuator_trnid[2 * i];  // transmission target joint
            // 获取该 joint 在 qpos 中的起始地址
            int qpos_adr = model->jnt_qposadr[jnt_id];
            int qvel_adr = model->jnt_dofadr[jnt_id];

            if (is_position_actuator_) {
                data->ctrl[i] = target_pos_[i];
            } else if (static_cast<int>(current_kp_.size()) > i &&
                static_cast<int>(current_kd_.size()) > i) {
                double pos_error = target_pos_[i] - data->qpos[qpos_adr];
                double vel_error = target_vel_[i] - data->qvel[qvel_adr];
                data->ctrl[i] = current_kp_[i] * pos_error + current_kd_[i] * vel_error;
            } else {
                data->ctrl[i] = 0.0;  // kp/kd 未配置时关节被动（assist 兜底）
            }
        }
    }

    void Step() {
        // 悬挂高度平滑过渡
        UpdateAssistHeight();

        ApplyAssistForce();
        ApplyControl();
        mj_step(model, data);
        step_count_++;
    }

    void Render() {
        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);
        mjv_updateScene(model, data, &opt, nullptr, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);
        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    bool IsAlive() const {
        return window && !glfwWindowShouldClose(window);
    }

    // 悬挂高度控制
    void SetAssistHeight(double height) {
        target_assist_height_ = height;
    }

    void AdjustAssistHeight(double delta) {
        SetAssistHeight(target_assist_height_ + delta);
    }

    double GetAssistHeight() const {
        return target_assist_height_;
    }

    double GetCurrentAssistHeight() const {
        return current_assist_height_;
    }

    SimState GetState() const {
        SimState state;
        state.num_dof = config.num_dof;
        state.joint_pos.resize(config.num_dof, 0.0);
        state.joint_vel.resize(config.num_dof, 0.0);

        if (has_floating_base_) {
            // 基座位置
            for (int i = 0; i < 3; i++)
                state.base_pos[i] = data->qpos[i];
            // 基座姿态（四元数）
            for (int i = 0; i < 4; i++)
                state.base_quat[i] = data->qpos[3 + i];
            // 基座速度（线速度 + 角速度）
            for (int i = 0; i < 6; i++)
                state.base_vel[i] = data->qvel[i];

            // 关节状态
            for (int i = 0; i < config.num_dof; i++) {
                state.joint_pos[i] = data->qpos[7 + i];
                state.joint_vel[i] = data->qvel[6 + i];
            }

            // 更新 IMU 数据（从 base_quat 和 base_vel 计算 rpy 和 gyro）
            ComputeImuData(state);
        } else {
            // 无浮动基座：基座状态保持默认值（零）
            // 关节状态
            for (int i = 0; i < config.num_dof; i++) {
                state.joint_pos[i] = data->qpos[i];
                state.joint_vel[i] = data->qvel[i];
            }
            // rpy 和 gyro 保持默认值（零）
        }

        state.time = data->time;
        return state;
    }

    void Cleanup() {
        if (window) {
            mjv_freeScene(&scn);
            mjr_freeContext(&con);
        }
        if (data) {
            mj_deleteData(data);
            data = nullptr;
        }
        if (model) {
            mj_deleteModel(model);
            model = nullptr;
        }
        if (window) {
            glfwDestroyWindow(window);
            window = nullptr;
        }
        glfwTerminate();
    }
};

// ==================== Simulator 公开接口实现 ====================

Simulator::Simulator(const std::string &yaml_path,
                    const std::string &robot_name,
                    int num_dof,
                    const std::string &xml_path,
                    const std::vector<double> &default_joint_pos,
                    const std::vector<double> &kp,
                    const std::vector<double> &kd,
                    bool assist)
    : impl_(std::make_unique<Impl>()) {
    impl_->Init(yaml_path, robot_name, num_dof, xml_path, default_joint_pos, kp, kd, assist);
}

Simulator::~Simulator() {
    impl_->Cleanup();
}

void Simulator::Reset() {
    impl_->Reset();
}

void Simulator::Run(StepFn step_fn, std::function<bool()> continue_fn, double duration) {
    impl_->Reset();

    auto start_time = std::chrono::steady_clock::now();

    while (impl_->IsAlive()) {
        // 外部停止条件
        if (continue_fn && !continue_fn())
            break;

        // 时长限制
        if (duration > 0) {
            double elapsed =
                std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time)
                    .count();
            if (elapsed >= duration)
                break;
        }

        auto step_start = std::chrono::steady_clock::now();

        // 执行回调：有新指令才更新控制
        if (step_fn) {
            auto cmd = step_fn(impl_->GetState());
            if (cmd.has_value()) {
                SetControl(cmd.value());
            }
        }

        impl_->Step();

        if (impl_->step_count_ % impl_->render_skip_ == 0) {
            impl_->Render();
        }

        // 实时同步
        double step_duration =
            std::chrono::duration<double>(std::chrono::steady_clock::now() - step_start).count();
        double sleep_time = impl_->model->opt.timestep - step_duration;
        if (sleep_time > 0) {
            std::this_thread::sleep_for(std::chrono::duration<double>(sleep_time));
        }
    }

    double elapsed =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count();
    std::cout << "\n[MuJoCo] 仿真结束" << std::endl;
    std::cout << "  仿真时间: " << impl_->data->time << "s" << std::endl;
    std::cout << "  实际时间: " << elapsed << "s" << std::endl;
    std::cout << "  实时率: " << impl_->data->time / elapsed << "x" << std::endl;
    std::cout << "  仿真步数: " << impl_->step_count_ << std::endl;
}

SimState Simulator::GetState() const {
    return impl_->GetState();
}

void Simulator::SetControl(const SimControl &ctrl) {
    impl_->control_enabled_ = ctrl.enable;
    if (!impl_->control_enabled_)
        return;

    impl_->target_pos_ = ctrl.target_pos;
    impl_->target_vel_ = ctrl.target_vel;
    impl_->current_kp_ = ctrl.kp;
    impl_->current_kd_ = ctrl.kd;
}

const MujocoConfig &Simulator::GetConfig() const {
    return impl_->config;
}

void Simulator::SetAssistEnabled(bool enabled) {
    impl_->assist_enabled_ = enabled;
}

void Simulator::ToggleAssist() {
    impl_->assist_enabled_ = !impl_->assist_enabled_;
}

bool Simulator::IsAssistEnabled() const {
    return impl_->assist_enabled_;
}

bool Simulator::IsAlive() const {
    return impl_->step_count_;
}

double Simulator::GetSimTime() const {
    return impl_->data ? impl_->data->time : 0.0;
}

// 悬挂高度控制
void Simulator::SetAssistHeight(double height) {
    impl_->SetAssistHeight(height);
}

void Simulator::AdjustAssistHeight(double delta_height) {
    impl_->AdjustAssistHeight(delta_height);
}

double Simulator::GetAssistHeight() const {
    return impl_->GetAssistHeight();
}

double Simulator::GetCurrentAssistHeight() const {
    return impl_->GetCurrentAssistHeight();
}

}  // namespace mujoco_sim
