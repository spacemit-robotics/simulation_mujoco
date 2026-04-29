# FindMuJoCo.cmake
#
# 查找 MuJoCo 头文件和库。支持用户通过变量或环境变量指定路径。
#
# 输入变量（可选）:
#   MUJOCO_DIR  — MuJoCo 安装根目录（CMake 变量或同名环境变量）
#
# 输出变量:
#   MUJOCO_INCLUDE_DIR  — 头文件目录（含 mujoco/mujoco.h）
#   MUJOCO_LIB          — libmujoco 路径

set(_mujoco_hints "")
if(DEFINED MUJOCO_DIR)
    list(APPEND _mujoco_hints "${MUJOCO_DIR}")
endif()
if(DEFINED ENV{MUJOCO_DIR})
    list(APPEND _mujoco_hints "$ENV{MUJOCO_DIR}")
endif()

# ~/.mujoco/mujoco-*（通配不同版本号的安装）
file(GLOB _home_mujoco "$ENV{HOME}/.mujoco/mujoco-*")
list(SORT _home_mujoco ORDER DESCENDING)  # 版本号降序，优先用最新版
list(APPEND _mujoco_hints ${_home_mujoco} "$ENV{HOME}/.mujoco")

# 标准系统路径兜底
list(APPEND _mujoco_hints /usr/local /opt/mujoco)

find_path(MUJOCO_INCLUDE_DIR
    NAMES mujoco/mujoco.h
    HINTS ${_mujoco_hints}
    PATH_SUFFIXES include
    NO_DEFAULT_PATH
)

find_library(MUJOCO_LIB
    NAMES mujoco
    HINTS ${_mujoco_hints}
    PATH_SUFFIXES lib
    NO_DEFAULT_PATH
)

if(NOT MUJOCO_INCLUDE_DIR OR NOT MUJOCO_LIB)
    message(FATAL_ERROR
        "MuJoCo not found.\n"
        "  默认安装路径: ~/.mujoco/mujoco-3.x.x/\n"
        "  或手动指定: cmake .. -DMUJOCO_DIR=/path/to/mujoco\n"
        "  或设置环境变量: export MUJOCO_DIR=/path/to/mujoco")
endif()

message(STATUS "MuJoCo: ${MUJOCO_LIB}")
message(STATUS "  includes: ${MUJOCO_INCLUDE_DIR}")
