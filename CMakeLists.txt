cmake_minimum_required(VERSION 3.16)
project(mujoco_sim LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# x86_64（PC 仿真）+ riscv64（K3 板卡）支持；其余架构静默跳过
# （不报错，由 humanoid_common 条件编译 driver_runtime）
# x86_64 取 MuJoCo 官方 release，riscv64 取 SpaceMIT archive 预编译包（见 cmake/FindMuJoCo.cmake）
if(NOT CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|amd64|AMD64|riscv64|riscv")
    message(STATUS "components/simulation/mujoco: skipped on ${CMAKE_SYSTEM_PROCESSOR}")
    return()
endif()

set(CMAKE_INSTALL_RPATH "\$ORIGIN/../lib:\$ORIGIN")
set(CMAKE_BUILD_RPATH_USE_ORIGIN ON)

# ============================================================================
# 依赖
# ============================================================================

include(cmake/FindMuJoCo.cmake)
find_package(yaml-cpp REQUIRED)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "riscv64|riscv")
    # riscv64（K3）：窗口用 X11，GL 上下文用 gl4es 提供的 libGL（含 GLX），
    # 经 gl4es 把 MuJoCo 的桌面 GL 翻译到 PowerVR GLES。gl4es 由 FindGL4ES 按三方包惯例
    # 自动 git clone 固定 commit + 打补丁 + 编译（见 cmake/FindGL4ES.cmake）。
    find_package(X11 REQUIRED)
    find_package(Threads REQUIRED)  # 渲染线程
    include(cmake/FindGL4ES.cmake)
else()
    find_package(glfw3 REQUIRED)
endif()

option(BUILD_EXAMPLES "Build example programs" ON)

# ============================================================================
# 库：mujoco_sim
# ============================================================================

add_library(mujoco_sim SHARED
    src/mujoco_sim.cpp
    src/mujoco_config.cpp
)

target_include_directories(mujoco_sim
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        ${MUJOCO_INCLUDE_DIR}
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "riscv64|riscv")
    target_include_directories(mujoco_sim PRIVATE ${GL4ES_INCLUDE_DIR} ${X11_INCLUDE_DIR})
    target_link_libraries(mujoco_sim
        PUBLIC  ${MUJOCO_LIB} ${GL4ES_LIB} ${X11_LIBRARIES} Threads::Threads
        PRIVATE yaml-cpp
    )
else()
    target_link_libraries(mujoco_sim
        PUBLIC  ${MUJOCO_LIB} glfw
        PRIVATE yaml-cpp
    )
endif()

# ============================================================================
# 安装
# ============================================================================

install(TARGETS mujoco_sim
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)

# libmujoco.so + 版本化实体文件一并装到 staging/lib，让下游链接时能找到
file(GLOB _mujoco_so_files "${MUJOCO_INCLUDE_DIR}/../lib/libmujoco.so*")
install(FILES ${_mujoco_so_files}
    DESTINATION lib
)

# riscv64（K3）：gl4es 的 libGL 一并装到 staging/lib，经下游 RUNPATH（$ORIGIN/../lib）
# 优先于系统 libGL 加载。
if(CMAKE_SYSTEM_PROCESSOR MATCHES "riscv64|riscv")
    file(GLOB _gl4es_so_files "${GL4ES_ROOT}/lib/libGL.so*")
    install(FILES ${_gl4es_so_files} DESTINATION lib)
endif()

install(FILES include/mujoco_sim.h
    DESTINATION include
)

# ============================================================================
# 示例
# ============================================================================

if(BUILD_EXAMPLES)
    add_executable(test_mujoco example/test_mujoco.cpp)
    target_link_libraries(test_mujoco PRIVATE mujoco_sim yaml-cpp)
    target_include_directories(test_mujoco PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    )
    set_target_properties(test_mujoco PROPERTIES BUILD_RPATH_USE_ORIGIN ON)
    install(TARGETS test_mujoco RUNTIME DESTINATION bin)
endif()
