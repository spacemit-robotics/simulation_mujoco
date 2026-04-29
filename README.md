# simulation/mujoco — MuJoCo 仿真模块

## 项目简介

基于 MuJoCo 物理引擎的通用人形机器人仿真模块，提供 C++ 接口。

模块**与机器人型号完全解耦**，机器人基础信息（name/num_dof/xml_path）由调用方传入，仿真参数从 YAML `simulation.mujoco` 节点读取。

## 功能特性

**支持：**
- 实时物理仿真（MuJoCo 3.4.0），内置渲染窗口
- 悬挂保护（assist）：可动态调节高度，防止机器人摔倒损坏
- 每步回调（StepFn）：调用方在回调中读取状态、下发控制指令
- 运行时长控制：可指定仿真时长或持续运行直到窗口关闭
- 键盘交互：悬挂高度调节、重置等

**不支持：**
- aarch64 / RISC-V 平台（MuJoCo 渲染依赖 OpenGL，仅支持 x86_64）
- 无头（headless）模式

## 快速开始

### 环境准备

**PC 端（x86_64）**：

```bash
# 系统依赖
sudo apt install -y libglfw3-dev libyaml-cpp-dev cmake g++

# MuJoCo 3.4.0
mkdir -p ~/.mujoco
wget https://github.com/google-deepmind/mujoco/releases/download/3.4.0/mujoco-3.4.0-linux-x86_64.tar.gz
tar -xzf mujoco-3.4.0-linux-x86_64.tar.gz -C ~/.mujoco/
```

> CMake 查找顺序：`MUJOCO_DIR` 环境变量或编译参数 → `~/.mujoco/mujoco-*`（自动选最新版）→ `~/.mujoco` → `/usr/local` → `/opt/mujoco`。若安装到非默认路径，可通过 `export MUJOCO_DIR=/path/to/mujoco` 或编译时 `-DMUJOCO_DIR=/path/to/mujoco` 指定。其他版本见 [github.com/google-deepmind/mujoco/releases](https://github.com/google-deepmind/mujoco/releases)。

> **注意**：本模块仅支持 **x86_64** 平台（MuJoCo 渲染依赖 OpenGL，不支持 RISC-V 交叉编译）。CMakeLists.txt 在非 x86_64 平台会自动跳过，无需手动处理。

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
./output/staging/bin/test_mujoco application/native/humanoid_unitree_g1/config/g1.yaml g1 29 application/native/humanoid_unitree_g1
./output/staging/bin/test_mujoco application/native/humanoid_unitree_g1/config/g1.yaml g1 29 application/native/humanoid_unitree_g1 -d 30
./output/staging/bin/test_mujoco -h
```

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
| `FromYaml` | `yaml_path, robot_name, num_dof, xml_path` | `MujocoConfig` | 静态工厂方法；从 YAML 读取 `simulation.mujoco` 节点的仿真参数，机器人基础信息由调用方传入 |

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
| `assist_gravity_compensation` | `double` | 额外恒定向上力（N），重型机器人可用，默认 0.0 |
| `sim_dt` | `double` | 仿真时间步长（s） |
| `kp` / `kd` | `std::vector<double>` | 各关节 PD 增益，大小 = num_dof |
| `default_joint_pos` | `std::vector<double>` | 默认关节角度（初始站立姿态），大小 = num_dof |

#### `Simulator` (仿真器)

MuJoCo 仿真器主类，内部管理物理步进、实时同步、渲染和悬挂保护。

| 接口名称 | 参数类型 | 返回值 | 功能说明 |
| :--- | :--- | :--- | :--- |
| 构造 | `yaml_path, robot_name, num_dof, xml_path, assist=true` | — | 加载 YAML 配置并初始化 MuJoCo 场景 |
| `Reset` | — | `void` | 重置仿真到初始状态 |
| `Run` | `step_fn, continue_fn, duration=-1` | `void` | 运行主循环，内部管理计时、实时同步、渲染跳帧；`duration=-1` 表示持续运行 |
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
| `IsAlive` | — | `bool` | 查询仿真窗口是否存活 |
| `GetStepCount` | — | `int` | 获取累计仿真步数 |
| `GetSimTime` | — | `double` | 获取仿真时间（s） |

**`StepFn` 回调类型：**

```cpp
using StepFn = std::function<std::optional<SimControl>(const SimState &)>;
```

每步回调传入当前状态，返回控制指令；返回 `std::nullopt` 表示本步保持上一帧指令不变。
`Run()` 传入 `nullptr` 时使用内部默认控制（保持 `default_joint_pos`）。

#### 注意事项

1. **参数传入规则**：`robot_name`、`num_dof`、`xml_path` 由调用方传入，模块内部不读取 `robot_base` 相关配置。`xml_path` 必须为绝对路径，`FromYaml()` 会校验文件存在性。
2. **平台限制**：本模块仅在 x86_64 平台编译和运行，CMakeLists.txt 通过 `CMAKE_SYSTEM_PROCESSOR` 检测架构，riscv64 上自动跳过，无需手动处理。
3. **`Run()` 内部管理时序**：调用方无需自行维护 sleep、渲染跳帧等逻辑，`StepFn` 回调只需处理传感器读取和指令下发。
4. **类型边界**：`SimState` / `SimControl` 是模块自有类型，与 `robot_base::RobotData` / `ControlCmd` 无关。调用方（如 `driver_demo`）负责在两者之间做转换。

## 常见问题

**Q：编译报 `libglfw3-dev not found`？**
运行 `sudo apt install -y libglfw3-dev` 安装系统依赖。

**Q：在 aarch64 / RISC-V 上编译失败？**
本模块仅支持 x86_64，CMakeLists.txt 在非 x86_64 平台会自动跳过，无需手动处理。

**Q：`xml_path` 报文件不存在？**
`xml_path` 必须为绝对路径，且指向机器人 MuJoCo XML 场景文件。确认路径正确后重试。

## 版本与发布

变更记录见 git log。

## 贡献方式

贡献者与维护者名单见：`CONTRIBUTORS.md`

## License

本组件源码文件头声明为 Apache-2.0，最终以本目录 `LICENSE` 文件为准。
