#!/usr/bin/env bash
# mujoco 渲染+物理冒烟（manual，需桌面会话/显示器）。
# CWD = SDK 根（test.yaml 里 workdir: sdk）。需要一个带 XML 资源的机型 robot_dir。
# 默认 g1；可用环境变量覆盖。-d 跑固定时长后正常退出 -> 通过。
set -uo pipefail

ROBOT_DIR=${SROBOTIS_MJ_TEST_ROBOT_DIR:-application/native/humanoid_unitree_g1}
CONFIG=${SROBOTIS_MJ_TEST_CONFIG:-$ROBOT_DIR/config/g1.yaml}
ROBOT_NAME=${SROBOTIS_MJ_TEST_ROBOT_NAME:-g1}
NUM_DOF=${SROBOTIS_MJ_TEST_NUM_DOF:-29}
DURATION=${SROBOTIS_MJ_TEST_DURATION:-3}

if [[ -z "${DISPLAY:-}" ]]; then
  echo "[FAIL] 无 DISPLAY：本用例需桌面会话/显示器（x86 GLFW 或 K3 gl4es）"; exit 1
fi
if [[ ! -f "$CONFIG" ]]; then
  echo "[FAIL] 配置不存在: $CONFIG"; exit 1
fi

# --no-assist 关悬挂保护跑自由下落也可；这里保留默认。跑 DURATION 秒后正常退出。
if timeout $((DURATION + 30)) test_mujoco "$CONFIG" "$ROBOT_NAME" "$NUM_DOF" "$ROBOT_DIR" -d "$DURATION"; then
  echo "mujoco sim-smoke: PASS ($ROBOT_NAME, ${DURATION}s)"
else
  rc=$?
  echo "[FAIL] test_mujoco 异常退出 (rc=$rc)"; exit 1
fi
