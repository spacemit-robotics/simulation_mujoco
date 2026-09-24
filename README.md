# simulation/mujoco — MuJoCo 仿真模块

## 项目简介

基于 MuJoCo 物理引擎的通用人形机器人仿真模块，提供 C++ 接口。

机器人基础信息（name/num_dof/xml_path）和机器人固有属性（`default_joint_pos` / `kp` / `kd`）由调用方传入；仿真参数从调用方提供的 YAML `simulation.mujoco` 节点读取。

## 功能特性

**支持：**
- 实时物理仿真，可选渲染窗口（x86_64 用 MuJoCo 3.4.0 官方包；riscv64/K3 用 SpaceMIT bianbu26 预编译 3.4.0）
- 悬挂保护（assist）：可动态调节高度，防止机器人摔倒损坏
- 每步回调（StepFn）：调用方在回调中读取状态、下发控制指令
- 运行时长控制：可指定时长，也可由窗口关闭或调用方的停止条件结束
- 键盘交互：悬挂高度调节、重置等
- 平台：x86_64（PC 仿真）+ riscv64（K3 板卡，板上原生编译）

**平台/渲染说明：**
- MuJoCo 渲染器用桌面 OpenGL。x86_64 直接对接系统 OpenGL；riscv64 经 [gl4es](https://github.com/ptitSeb/gl4es) 翻译到 PowerVR GLES 后执行，窗口用裸 X11 + GLX（GLFW 与 gl4es 不兼容），渲染在独立线程。gl4es 由 CMake 自动获取、打补丁（`cmake/gl4es-imgtec-stencil.patch`）并编译。
- 开启窗口时，PC 在主线程处理窗口和绘图、在工作线程运行物理仿真；物理线程不等待绘图，画面更新跟不上时可以跳帧。K3 在调用线程运行物理仿真、在独立线程绘图。
- 已知问题：启动一行 `OpenGL error 0x500` 告警可忽略。
- `simulation.mujoco.viewer: false` 为无窗口模式，不创建绘图线程或 GL 窗口。

## 快速开始

### 环境准备

系统依赖（分平台）：

```bash
# 两平台通用
sudo apt install -y libyaml-cpp-dev cmake g++
# x86_64（PC）：窗口用 GLFW
sudo apt install -y libglfw3-dev
# riscv64（K3）：窗口用裸 X11 + gl4es，不用 GLFW
sudo apt install -y libx11-dev
```

> riscv64 的 gl4es 由 `cmake/FindGL4ES.cmake` 在首次配置时自动 git clone 固定 commit、应用补丁并编译；已有现成的可用 `-DGL4ES_ROOT=/path` 指向。

MuJoCo 由 CMake 处理，**按架构自动选择预编译包来源**（包内布局一致，逻辑相同）：

| 架构 | 版本 | 来源 |
| :--- | :--- | :--- |
| x86_64 | 3.4.0 | MuJoCo 官方 GitHub release |
| riscv64（K3） | 3.4.0 | SpaceMIT bianbu26：`archive.spacemit.com/ros2/prebuilt_libs/bianbu26/opt/ext/mujoco/` |

CMake 按以下顺序查找，命中即用：

1. `-DMUJOCO_DIR=...` 编译参数
2. 环境变量 `MUJOCO_DIR`
3. `/usr/local`、`/opt/mujoco`
4. 缓存路径 `~/.cache/thirdparty/mujoco/mujoco-<版本>/`
5. 上述均未命中：从对应架构的预编译包来源拉取并解压到第 4 项缓存路径

> 离线/受限网络环境可设 `SROBOTIS_THIRDPARTY_FETCH_OFF=ON` 禁用第 5 步拉取，并通过
> `export MUJOCO_DIR=/path/to/mujoco-<版本>` 指向预先下载好的目录。

> **注意**：riscv64 官方无预编译 release，故走 SpaceMIT archive 包；K3 的 GL 渲染经 gl4es 翻译到 PowerVR 硬件 GLES，详见上方"平台/渲染说明"。

### 构建编译

**SDK 内编译（mm）**：

```bash
source ~/spacemit_robot/build/envsetup.sh
cd components/simulation/mujoco
mm
```

编译产物安装至 `output/staging/`：
- 动态库：`output/staging/lib/libmujoco_sim.so`
- 测试程序：`output/staging/bin/test_mujoco`

**独立 cmake 编译**：

```bash
cd components/simulation/mujoco
mkdir build && cd build
cmake ..
make
```

### 运行示例

```bash
cd ~/spacemit_robot
./output/staging/bin/test_mujoco application/native/humanoid_unitree_g1/config/g1.yaml g1 29 application/native/humanoid_unitree_g1 --robot-node robot_base
./output/staging/bin/test_mujoco application/native/humanoid_unitree_g1/config/g1.yaml g1 29 application/native/humanoid_unitree_g1 -d 30 --robot-node robot_base
./output/staging/bin/test_mujoco -h
```

> riscv64（K3）须在板子的桌面会话里运行（窗口显示在板载屏幕上）。

**键盘操作：**

| 按键 | 功能 |
| :--- | :--- |
| `F` | 切换悬挂保护开/关 |
| `↑` | 悬挂高度 +5cm |
| `↓` | 悬挂高度 -5cm |
| `R` | 重置悬挂高度到 YAML 默认值 |
| 鼠标左键拖拽 | 旋转视角 |
| 鼠标右键拖拽 | 平移视角 |
| 滚轮 | 缩放 |
| 关闭窗口 | 退出仿真 |

### CI 测试

模块自带 `test.yaml`（CI 用例清单）+ `tests/`，经 SDK 根目录的 `robot-test` 运行：

```bash
scripts/test/robot-test list components/simulation/mujoco
scripts/test/robot-test run  components/simulation/mujoco --scope pr     # 参数/配置错误路径（无需显示器）
scripts/test/robot-test run  components/simulation/mujoco --scope manual # 渲染+物理冒烟（需桌面会话/显示器）
```

PR 档覆盖错误处理和无窗口物理仿真；真实窗口测试需要桌面会话，归 manual。
PC 构建另提供无需显示器的 CTest `mujoco_render_threading`，检查绘图耗时不阻塞物理回调，并覆盖退出和异常处理。

## 详细使用

### 接口说明

本模块对外接口分为数据类型、仿真配置和仿真器三大类。
各接口的详细使用范例，请参考：**[example/test_mujoco.cpp](example/test_mujoco.cpp)**

#### 数据类型

**`SimState`** — 仿真器输出的机器人状态：

| 字段 | 类型 | 说明 |
| :--- | :--- | :--- |
| `num_dof` | `int` | 控制自由度数量 |
| `joint_pos` | `std::vector<double>` | 关节位置 |
| `joint_vel` | `std::vector<double>` | 关节速度 |
| `base_pos` | `std::array<double, 3>` | 基座位置 [x, y, z] |
| `base_quat` | `std::array<double, 4>` | 基座四元数 [w, x, y, z] |
| `base_vel` | `std::array<double, 6>` | 线速度(3) + 角速度(3) |
| `gyro` | `std::array<double, 3>` | 角速度（从 base_vel 提取） |
| `rpy` | `std::array<double, 3>` | 欧拉角（从 base_quat 计算） |
| `time` | `double` | 仿真时间（s） |

**`SimControl`** — 发给仿真器的控制指令：

| 字段 | 类型 | 说明 |
| :--- | :--- | :--- |
| `enable` | `bool` | 是否启用控制（false 时关节松力） |
| `target_pos` | `std::vector<double>` | 目标关节位置 |
| `target_vel` | `std::vector<double>` | 目标关节速度 |
| `kp` | `std::vector<double>` | 各关节 PD 刚度 |
| `kd` | `std::vector<double>` | 各关节 PD 阻尼 |

#### `MujocoConfig` (仿真配置)

从 YAML 文件加载 MuJoCo 仿真所需的全部参数。

| 接口名称 | 参数类型 | 返回值 | 功能说明 |
| :--- | :--- | :--- | :--- |
| `FromYaml` | `yaml_path, robot_name, num_dof, xml_path` | `MujocoConfig` | 静态工厂方法；从 YAML `simulation.mujoco` 节点读取仿真参数。机器人固有属性（`default_joint_pos` / `kp` / `kd`）不在此读取，由调用方填入 `MujocoConfig` 或通过 `Simulator` 构造函数传入 |

**配置字段：**

| 字段 | 类型 | 说明 |
| :--- | :--- | :--- |
| `name` | `std::string` | 机器人名称（调用方传入） |
| `xml_path` | `std::string` | MuJoCo XML 场景文件绝对路径（调用方传入） |
| `num_dof` | `int` | 控制自由度数量（调用方传入） |
| `init_height` | `double` | 机器人初始释放高度（m），默认 0.85 |
| `assist_height` | `double` | 悬挂保护目标高度（m），默认 0.75 |
| `assist_kp` | `double` | 悬挂 PD 刚度，默认 500.0 |
| `assist_kd` | `double` | 悬挂 PD 阻尼，默认 100.0 |
| `sim_dt` | `double` | 仿真时间步长（s） |
| `viewer` | `bool` | 是否创建窗口并渲染，默认 true；设为 false 时仅运行物理仿真 |
| `kp` / `kd` | `std::vector<double>` | 各关节 PD 增益，大小 = num_dof |
| `default_joint_pos` | `std::vector<double>` | 默认关节角度（初始站立姿态），大小 = num_dof |

#### `Simulator` (仿真器)

MuJoCo 仿真器主类，内部管理物理步进、实时同步、渲染和悬挂保护。

| 接口名称 | 参数类型 | 返回值 | 功能说明 |
| :--- | :--- | :--- | :--- |
| 构造 | `yaml_path, robot_name, num_dof, xml_path, default_joint_pos, kp={}, kd={}, assist=true` | — | 加载 `simulation.mujoco` 仿真参数并初始化 MuJoCo 场景；`default_joint_pos` 必传，`kp/kd` 为 MuJoCo 启动初始增益（可选） |
| `Reset` | — | `void` | 重置仿真到初始状态 |
| `Run` | `step_fn, continue_fn, duration=-1` | `void` | 按实时节拍运行物理仿真，可选窗口渲染；`duration=-1` 表示持续运行 |
| `GetState` | — | `SimState` | 获取当前机器人状态 |
| `SetControl` | `const SimControl &ctrl` | `void` | 设置完整控制指令 |
| `GetConfig` | — | `const MujocoConfig &` | 获取当前仿真配置（只读） |
| `SetAssistEnabled` | `bool enabled` | `void` | 启用或禁用悬挂保护 |
| `ToggleAssist` | — | `void` | 切换悬挂保护开关 |
| `IsAssistEnabled` | — | `bool` | 查询悬挂保护是否启用 |
| `SetAssistHeight` | `double height` | `void` | 设置目标悬挂高度（m） |
| `AdjustAssistHeight` | `double delta` | `void` | 调整悬挂高度增量（m），正值上升 |
| `GetAssistHeight` | — | `double` | 获取目标悬挂高度（m） |
| `GetCurrentAssistHeight` | — | `double` | 获取当前实际悬挂高度（m），平滑过渡中可能与目标不同 |
| `IsAlive` | — | `bool` | 查询仿真是否仍在运行；headless 模式由外部停止条件或 duration 结束 |
| `GetStepCount` | — | `int` | 获取累计仿真步数 |
| `GetSimTime` | — | `double` | 获取仿真时间（s） |

**`StepFn` 回调类型：**

```cpp
using StepFn = std::function<std::optional<SimControl>(const SimState &)>;
```

每步回调传入当前状态，返回控制指令；返回 `std::nullopt` 表示本步保持上一帧指令不变。
`Run()` 传入 `nullptr` 时使用内部默认控制（保持 `default_joint_pos`）。

#### 注意事项

1. **参数传入规则**：`robot_name`、`num_dof`、`xml_path`、`default_joint_pos`、`kp`、`kd` 由调用方传入而非由本模块从 YAML 读取。`xml_path` 必须为绝对路径，`FromYaml()` 会校验文件存在性。
2. **平台支持**：x86_64 + riscv64（K3）。CMakeLists.txt 通过 `CMAKE_SYSTEM_PROCESSOR` 检测架构并选择对应预编译包来源与渲染依赖（x86 → glfw；riscv → gl4es libGL + X11 + pthread，gl4es 由 FindGL4ES 自动获取编译）；其余架构（如 aarch64）自动跳过。K3 经 gl4es 走 PowerVR 硬件 GLES，见"平台/渲染说明"。
3. **线程与回调**：PC 开窗模式须在主线程构造、运行和销毁 `Simulator`；`StepFn`、`ObserveFn`、`continue_fn` 在内部物理线程执行。K3 和无窗口模式的回调在调用 `Run()` 的线程执行。调用方跨线程共享数据时须自行同步；回调异常会传给 `Run()` 调用方。耗时回调仍会影响物理实时性。
4. **类型边界**：`SimState` / `SimControl` 是模块自有类型，不依赖任何外部数据结构。调用方负责在 SimState/SimControl 和其自身使用的数据类型之间做转换。

## 常见问题

**Q：x86_64 编译报 `libglfw3-dev not found`？**
运行 `sudo apt install -y libglfw3-dev` 安装。riscv64 不用 glfw（用 gl4es + X11）。

**Q：riscv64（K3）上能编能跑吗？**
能。CMake 检测到 riscv64 会自动拉 SpaceMIT bianbu26 的 mujoco 预编译包（3.4.0）并自动获取编译 gl4es，可视化经 gl4es 翻译到 PowerVR 硬件 GLES（详见"平台/渲染说明"）。在板子的桌面会话里直接跑 `run_*.sh` 或 test_mujoco 即可。aarch64 暂不支持，自动跳过。

**Q：`xml_path` 报文件不存在？**
`xml_path` 必须为绝对路径，且指向机器人 MuJoCo XML 场景文件。确认路径正确后重试。

## 版本与发布

变更记录见 git log。

## 贡献方式

贡献者与维护者名单见：`CONTRIBUTORS.md`

## License

本组件源码文件头声明为 Apache-2.0，最终以本目录 `LICENSE` 文件为准。
