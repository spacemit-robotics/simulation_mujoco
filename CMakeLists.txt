cmake_minimum_required(VERSION 3.16)
project(mujoco_sim LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# x86_64 only，rv64 静默跳过（不报错，由 humanoid_common 条件编译 driver_runtime）
if(NOT CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|amd64|AMD64")
    message(STATUS "components/simulation/mujoco: skipped on ${CMAKE_SYSTEM_PROCESSOR}")
    return()
endif()

set(CMAKE_INSTALL_RPATH "\$ORIGIN/../lib:\$ORIGIN")
set(CMAKE_BUILD_RPATH_USE_ORIGIN ON)

# ============================================================================
# 依赖
# ============================================================================

include(cmake/FindMuJoCo.cmake)
find_package(glfw3 REQUIRED)
find_package(yaml-cpp REQUIRED)

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

target_link_libraries(mujoco_sim
    PUBLIC  ${MUJOCO_LIB} glfw
    PRIVATE yaml-cpp
)

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
