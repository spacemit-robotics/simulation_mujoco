#!/usr/bin/env bash
# mujoco error-path 用例：非法参数 / 坏配置须快速失败，且不需要显示器
# （失败发生在配置加载阶段，早于 MujocoSim 创建 GL 窗口）。
# CWD = 模块根，test_mujoco 经 staging/bin 在 PATH。
set -uo pipefail

fail=0

expect_fail() {  # $1=场景, 其余=命令
  local desc="$1"; shift
  if timeout 15 "$@" >/dev/null 2>&1; then
    echo "[FAIL] 期望非 0 却成功（$desc）: $*"; fail=1
  else
    local rc=$?
    if [[ "$rc" -eq 124 ]]; then echo "[FAIL] hang（$desc）: $*"; fail=1
    else echo "[OK] 正确拒绝(rc=$rc, $desc)"; fi
  fi
}

expect_fail "缺参"         test_mujoco
expect_fail "坏 yaml 路径" test_mujoco /nonexistent.yaml g1 29 /tmp

if [[ "$fail" -ne 0 ]]; then echo "mujoco error-path: FAILED"; exit 1; fi
echo "mujoco error-path: PASS"
