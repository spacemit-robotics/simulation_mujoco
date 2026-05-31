/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file mujoco_sim.cpp
 * @brief MuJoCo 仿真器实现
 */

#include "mujoco_sim.h"

#if defined(__riscv)
// riscv64（K3）：裸 X11 + 老式 GLX 窗口，配合 gl4es 把桌面 GL 翻译到 PowerVR 硬件 GLES；不走 GLFW
#include <GL/glx.h>
#include <X11/Xlib.h>
#include <X11/keysym.h>
#else
#include <GLFW/glfw3.h>
#endif

#include <mujoco/mujoco.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>  // NOLINT(build/c++17)
#include <iostream>
#include <mutex>
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
#if defined(__riscv)
    // X11/GLX 窗口后端（配合 gl4es），渲染在独立线程
    Display *x_display_ = nullptr;
    Window x_window_ = 0;
    GLXContext glx_ctx_ = nullptr;
    Atom wm_delete_ = 0;
    std::atomic<bool> should_close_{false};
    std::atomic<bool> render_running_{false};
    std::thread render_thread_;
    std::mutex data_mutex_;  // 保护 mjData：物理 Step 与渲染 mjv_updateScene 互斥
#else
    GLFWwindow *window = nullptr;
#endif
    int win_width_ = kWindowWidth;
    int win_height_ = kWindowHeight;
    bool gl_ready_ = false;  // GL 上下文 + 场景已创建（Cleanup 守卫）

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
    double gravity_compensation_ = 0.0;  // 整机重力前馈，Init 时按模型总质量自动计算
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
        // 整机重力前馈：按模型总质量自动抵消重力，使悬挂稳态精确停在 assist_height
        gravity_compensation_ = mj_getTotalmass(model) * std::abs(model->opt.gravity[2]);

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

        // x86：在此建窗口+GL 上下文；riscv64：推迟到渲染线程（见 RenderLoop）。
#if !defined(__riscv)
        WindowInit();
#endif

        // 初始化渲染（mjv_* 纯 CPU，主线程；mjr_makeContext 需 GL 上下文）
        mjv_defaultCamera(&cam);
        cam.lookat[2] = kCameraLookatZ;
        cam.azimuth = kCameraAzimuth;
        cam.elevation = kCameraElevation;
        cam.distance = kCameraDistance;

        mjv_defaultOption(&opt);
        mjv_defaultScene(&scn);
        mjr_defaultContext(&con);

#if defined(__riscv)
        // riscv64（K3 PowerVR/gl4es）：关 MSAA、阴影。
        model->vis.quality.offsamples = 0;
        model->vis.quality.shadowsize = 0;
#endif

        mjv_makeScene(model, &scn, kMaxSceneObjects);
#if !defined(__riscv)
        mjr_makeContext(model, &con, mjFONTSCALE_100);
        gl_ready_ = true;
#else
        // riscv：mjr_makeContext 在渲染线程内调用（GL 上下文归该线程）；这里只设场景渲染标志
        scn.flags[mjRND_SHADOW] = 0;
        scn.flags[mjRND_REFLECTION] = 0;
#endif

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

    // ==================== 窗口 / GL 后端 ====================

    // 创建窗口与桌面 GL 上下文。
    // riscv64：X11 窗口 + 老式 glXCreateContext，经 gl4es 把桌面 GL 翻译到 PowerVR GLES。
    // x86_64：GLFW。
    void WindowInit() {
#if defined(__riscv)
        // gl4es 运行时配置，须在首个 GLX 调用（下方 glXChooseVisual）前设置，gl4es 惰性初始化时读取。
        // 第三参 0：不覆盖调用方已显式设置的值。
        setenv("LIBGL_GL", "21", 0);  // gl4es 声称支持的桌面 GL 版本 = 2.1
        setenv("LIBGL_ES", "2", 0);   // gl4es 翻译目标后端 = GLES 2.0
        XInitThreads();  // 渲染线程内做 X11 调用，须先开启 Xlib 多线程支持
        x_display_ = XOpenDisplay(nullptr);
        if (!x_display_) {
            throw std::runtime_error("无法打开 X11 Display（须在带显示的桌面会话内运行，检查 DISPLAY）");
        }
        // 含 stencil（MuJoCo 渲染器需要模板缓冲，缺失会导致地面等区域渲染异常）
        int attribs[] = {GLX_RGBA, GLX_DEPTH_SIZE, 24, GLX_STENCIL_SIZE, 8, GLX_DOUBLEBUFFER, None};
        XVisualInfo *vi = glXChooseVisual(x_display_, DefaultScreen(x_display_), attribs);
        if (!vi) {
            throw std::runtime_error("glXChooseVisual 失败");
        }
        Window root = DefaultRootWindow(x_display_);
        XSetWindowAttributes swa;
        swa.colormap = XCreateColormap(x_display_, root, vi->visual, AllocNone);
        swa.event_mask = ExposureMask | KeyPressMask | ButtonPressMask | ButtonReleaseMask |
                        PointerMotionMask | StructureNotifyMask;
        x_window_ = XCreateWindow(x_display_, root, 0, 0, kWindowWidth, kWindowHeight, 0, vi->depth,
                                InputOutput, vi->visual, CWColormap | CWEventMask, &swa);
        XStoreName(x_display_, x_window_, "MuJoCo Simulator");
        wm_delete_ = XInternAtom(x_display_, "WM_DELETE_WINDOW", False);
        XSetWMProtocols(x_display_, x_window_, &wm_delete_, 1);
        XMapWindow(x_display_, x_window_);
        glx_ctx_ = glXCreateContext(x_display_, vi, nullptr, GL_TRUE);
        XFree(vi);
        if (!glx_ctx_) {
            throw std::runtime_error("glXCreateContext 失败（gl4es libGL 未生效？确认已随构建装入 staging/lib）");
        }
        glXMakeCurrent(x_display_, x_window_, glx_ctx_);
        std::cout << "[MuJoCo] 渲染后端: X11/GLX + gl4es (PowerVR 硬件 GLES)" << std::endl;
#else
        if (!glfwInit()) {
            throw std::runtime_error("GLFW 初始化失败");
        }
        window =
            glfwCreateWindow(kWindowWidth, kWindowHeight, "MuJoCo Simulator", nullptr, nullptr);
        if (!window) {
            throw std::runtime_error("GLFW 创建窗口失败");
        }
        glfwMakeContextCurrent(window);
        glfwSwapInterval(1);
#if GLFW_VERSION_MAJOR > 3 || (GLFW_VERSION_MAJOR == 3 && GLFW_VERSION_MINOR >= 2)
        glfwSetWindowAttrib(window, GLFW_FLOATING, GLFW_TRUE);
#endif
        glfwSetWindowUserPointer(window, this);
        glfwSetKeyCallback(window, KeyCallback);
        glfwSetCursorPosCallback(window, MouseMove);
        glfwSetMouseButtonCallback(window, MouseButton);
        glfwSetScrollCallback(window, Scroll);
        std::cout << "[MuJoCo] 渲染后端: GLFW" << std::endl;
#endif
    }

    // ---- 共享输入语义（GLFW 回调与 X11 事件循环共用） ----
    void InputToggleAssist() {
        assist_enabled_ = !assist_enabled_;
        std::cout << "\n[MuJoCo] 悬挂保护: " << (assist_enabled_ ? "启用" : "禁用") << std::endl;
    }
    void InputAdjustAssist(double delta) {
        AdjustAssistHeight(delta);
        std::cout << "\n[MuJoCo] 目标悬挂高度: " << GetAssistHeight() << "m"
                << " (当前: " << GetCurrentAssistHeight() << "m)" << std::endl;
    }
    void InputResetAssist() {
        SetAssistHeight(config.assist_height);
        std::cout << "\n[MuJoCo] 重置悬挂高度到默认值: " << GetAssistHeight() << "m" << std::endl;
    }
    void InputDrag(double dx, double dy, int height, bool shift) {
        if (!button_left_ && !button_middle_ && !button_right_) {
            return;
        }
        if (height <= 0) {
            height = 1;
        }
        mjtMouse action;
        if (button_right_) {
            action = shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
        } else if (button_left_) {
            action = shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
        } else {
            action = mjMOUSE_ZOOM;
        }
        mjv_moveCamera(model, action, dx / height, dy / height, &scn, &cam);
    }
    void InputScroll(double yoffset) {
        mjv_moveCamera(model, mjMOUSE_ZOOM, 0, 0.05 * yoffset, &scn, &cam);
    }

#if defined(__riscv)
    // 处理 X11 事件队列（替代 glfwPollEvents），分发键鼠到共享输入语义
    void ProcessX11Events() {
        while (XPending(x_display_)) {
            XEvent ev;
            XNextEvent(x_display_, &ev);
            switch (ev.type) {
                case ConfigureNotify:
                    win_width_ = ev.xconfigure.width;
                    win_height_ = ev.xconfigure.height;
                    break;
                case KeyPress: {
                    KeySym ks = XLookupKeysym(&ev.xkey, 0);
                    if (ks == XK_f || ks == XK_F) {
                        InputToggleAssist();
                    } else if (ks == XK_Up) {
                        InputAdjustAssist(0.05);
                    } else if (ks == XK_Down) {
                        InputAdjustAssist(-0.05);
                    } else if (ks == XK_r || ks == XK_R) {
                        InputResetAssist();
                    } else if (ks == XK_Escape) {
                        should_close_ = true;
                    }
                    break;
                }
                case ButtonPress:
                    if (ev.xbutton.button == Button1) {
                        button_left_ = true;
                    } else if (ev.xbutton.button == Button2) {
                        button_middle_ = true;
                    } else if (ev.xbutton.button == Button3) {
                        button_right_ = true;
                    } else if (ev.xbutton.button == Button4) {
                        InputScroll(1.0);
                    } else if (ev.xbutton.button == Button5) {
                        InputScroll(-1.0);
                    }
                    last_x_ = ev.xbutton.x;
                    last_y_ = ev.xbutton.y;
                    break;
                case ButtonRelease:
                    if (ev.xbutton.button == Button1) {
                        button_left_ = false;
                    } else if (ev.xbutton.button == Button2) {
                        button_middle_ = false;
                    } else if (ev.xbutton.button == Button3) {
                        button_right_ = false;
                    }
                    break;
                case MotionNotify: {
                    double x = ev.xmotion.x;
                    double y = ev.xmotion.y;
                    double dx = x - last_x_;
                    double dy = y - last_y_;
                    last_x_ = x;
                    last_y_ = y;
                    bool shift = (ev.xmotion.state & ShiftMask) != 0;
                    InputDrag(dx, dy, win_height_, shift);
                    break;
                }
                case ClientMessage:
                    if (static_cast<Atom>(ev.xclient.data.l[0]) == wm_delete_) {
                        should_close_ = true;
                    }
                    break;
                default:
                    break;
            }
        }
    }

    // 渲染线程：独占 GL 上下文（窗口/上下文/mjr_makeContext 都在本线程建立）。
    void RenderLoop() {
        try {
            WindowInit();  // 本线程创建 X11 窗口 + GLX 上下文 + glXMakeCurrent
            mjr_makeContext(model, &con, mjFONTSCALE_100);
            gl_ready_ = true;
            while (render_running_ && !should_close_) {
                {
                    std::lock_guard<std::mutex> lk(data_mutex_);
                    mjv_updateScene(model, data, &opt, nullptr, &cam, mjCAT_ALL, &scn);
                }
                mjrRect viewport = {0, 0, win_width_, win_height_};
                mjr_render(viewport, &scn, &con);
                glXSwapBuffers(x_display_, x_window_);
                ProcessX11Events();
            }
            glXMakeCurrent(x_display_, None, nullptr);  // 退出前释放上下文，便于主线程清理
        } catch (const std::exception &e) {
            std::cerr << "[MuJoCo] 渲染线程异常: " << e.what() << std::endl;
            should_close_ = true;
        }
    }

    void StartRenderThread() {
        render_running_ = true;
        render_thread_ = std::thread(&Impl::RenderLoop, this);
    }

    void StopRenderThread() {
        render_running_ = false;
        if (render_thread_.joinable()) {
            render_thread_.join();
        }
    }
#endif

    // ==================== GLFW 回调（仅 x86；riscv 走 ProcessX11Events） ====================

#if !defined(__riscv)
    static void KeyCallback(GLFWwindow *win, int key, int scancode, int act, int mods) {
        auto *impl = static_cast<Impl *>(glfwGetWindowUserPointer(win));
        if (act != GLFW_PRESS) {
            return;
        }
        if (key == GLFW_KEY_F) {
            impl->InputToggleAssist();
        } else if (key == GLFW_KEY_UP) {
            impl->InputAdjustAssist(0.05);
        } else if (key == GLFW_KEY_DOWN) {
            impl->InputAdjustAssist(-0.05);
        } else if (key == GLFW_KEY_R) {
            impl->InputResetAssist();
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
        double dx = xpos - impl->last_x_;
        double dy = ypos - impl->last_y_;
        impl->last_x_ = xpos;
        impl->last_y_ = ypos;
        int width, height;
        glfwGetWindowSize(win, &width, &height);
        bool shift = (glfwGetKey(win, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
                    glfwGetKey(win, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);
        impl->InputDrag(dx, dy, height, shift);
    }

    static void Scroll(GLFWwindow *win, double xoffset, double yoffset) {
        auto *impl = static_cast<Impl *>(glfwGetWindowUserPointer(win));
        impl->InputScroll(yoffset);
    }
#endif

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
        data->xfrc_applied[base_body_id_ * 6 + 2] = force_z + gravity_compensation_;

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

#if !defined(__riscv)
    // riscv 下渲染由 RenderLoop 在独立线程负责，不走此路径。
    void Render() {
        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);
        mjv_updateScene(model, data, &opt, nullptr, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);
        glfwSwapBuffers(window);
        glfwPollEvents();
    }
#endif

    bool IsAlive() const {
#if defined(__riscv)
        // 窗口在渲染线程创建，物理循环以 render_running_ 为存活依据（窗口关闭→should_close_）
        return render_running_ && !should_close_;
#else
        return window && !glfwWindowShouldClose(window);
#endif
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
#if defined(__riscv)
        // 确保渲染线程已停（正常 Run 结束时已 join；此处兜底）
        render_running_ = false;
        if (render_thread_.joinable()) {
            render_thread_.join();
        }
        // 渲染线程退出时已释放上下文；主线程取回以便释放 GL 资源
        if (gl_ready_ && x_display_ && glx_ctx_) {
            glXMakeCurrent(x_display_, x_window_, glx_ctx_);
        }
#endif
        if (gl_ready_) {
            mjv_freeScene(&scn);
            mjr_freeContext(&con);
            gl_ready_ = false;
        }
        if (data) {
            mj_deleteData(data);
            data = nullptr;
        }
        if (model) {
            mj_deleteModel(model);
            model = nullptr;
        }
#if defined(__riscv)
        if (glx_ctx_) {
            glXMakeCurrent(x_display_, None, nullptr);
            glXDestroyContext(x_display_, glx_ctx_);
            glx_ctx_ = nullptr;
        }
        if (x_window_) {
            XDestroyWindow(x_display_, x_window_);
            x_window_ = 0;
        }
        if (x_display_) {
            XCloseDisplay(x_display_);
            x_display_ = nullptr;
        }
#else
        if (window) {
            glfwDestroyWindow(window);
            window = nullptr;
        }
        glfwTerminate();
#endif
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

#if defined(__riscv)
    // riscv：启动独立渲染线程。
    impl_->StartRenderThread();
#endif

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

#if defined(__riscv)
        // riscv：加锁保护 mjData（与渲染线程的 mjv_updateScene 互斥）；渲染由渲染线程负责
        {
            std::lock_guard<std::mutex> lk(impl_->data_mutex_);
            if (step_fn) {
                auto cmd = step_fn(impl_->GetState());
                if (cmd.has_value()) {
                    SetControl(cmd.value());
                }
            }
            impl_->Step();
        }
#else
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
#endif

        // 实时同步
        double step_duration =
            std::chrono::duration<double>(std::chrono::steady_clock::now() - step_start).count();
        double sleep_time = impl_->model->opt.timestep - step_duration;
        if (sleep_time > 0) {
            std::this_thread::sleep_for(std::chrono::duration<double>(sleep_time));
        }
    }

#if defined(__riscv)
    impl_->StopRenderThread();
#endif

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
