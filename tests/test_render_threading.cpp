/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file test_render_threading.cpp
 * @brief Check physics progress during graphics stalls without a display
 */

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>

#include "mujoco_sim.h"

struct GLFWwindow {
    void *user_pointer = nullptr;
};

namespace {

enum class StallSite { kNone, kScene, kDraw, kSwap, kEvents };

struct TestContext {
    std::thread::id owner = std::this_thread::get_id();
    std::atomic<int> steps{0};
    std::mutex mutex;
    std::condition_variable progress;
    const void *physics_model = nullptr;
    const void *physics_data = nullptr;
    const mjModel *scene_model = nullptr;
    const mjData *scene_data = nullptr;
    double last_scene_time = -1.0;
    bool saw_new_snapshot = false;
    StallSite stall_site = StallSite::kNone;
    bool stalled = false;
    bool close_on_poll = false;
    bool window_closed = false;
    bool throw_on_draw = false;
    bool send_key = false;
    GLFWkeyfun key_callback = nullptr;
};

TestContext *context = nullptr;
GLFWwindow test_window;

void Check(bool condition, const char *message) {
    if (!condition) throw std::runtime_error(message);
}

void CheckOwner() {
    Check(std::this_thread::get_id() == context->owner,
        "GLFW/graphics operation left the main thread");
}

void StallGraphics(StallSite site) {
    CheckOwner();
    if (context->stall_site != site || context->stalled) return;
    context->stalled = true;
    const auto *snapshot = context->scene_data;
    const double snapshot_time = snapshot->time;
    const double snapshot_pos = snapshot->qpos[0];
    std::unique_lock<std::mutex> lock(context->mutex);
    const int initial_steps = context->steps.load();
    const bool progressed = context->progress.wait_for(lock, std::chrono::seconds(2), [&]() {
        return context->steps.load() >= initial_steps + 80;
    });
    Check(progressed, "physics/transport callback blocked by graphics");
    Check(snapshot->time == snapshot_time && snapshot->qpos[0] == snapshot_pos,
        "renderer snapshot changed while graphics were blocked");
}

void ObserveStep(const mujoco_sim::SimState &, const mujoco_sim::ObserveFrame &frame) {
    Check(std::this_thread::get_id() != context->owner,
        "physics callback is running on the graphics thread");
    {
        std::lock_guard<std::mutex> lock(context->mutex);
        context->physics_model = frame.model;
        context->physics_data = frame.data;
        ++context->steps;
    }
    context->progress.notify_all();
}

mujoco_sim::SimControl Command(const mujoco_sim::SimState &state) {
    Check(std::this_thread::get_id() != context->owner,
        "command callback is running on the graphics thread");
    mujoco_sim::SimControl command;
    command.enable = true;
    command.target_pos = {state.time < 0.1 ? 0.1 : -0.1};
    command.target_vel = {0.0};
    command.kp = {10.0};
    command.kd = {1.0};
    return command;
}

void TestStall(const std::string &directory, StallSite site) {
    TestContext test;
    context = &test;
    test.stall_site = site;
    test.send_key = true;
    mujoco_sim::Simulator sim(directory + "/render_threading.yaml", "render_test", 1,
        directory + "/render_threading.xml", {0.0}, {10.0}, {1.0}, false);
    sim.SetObserveCallback(ObserveStep);
    sim.Run(Command, [&]() { return test.steps.load() < 240; }, 5.0);
    Check(test.stalled && test.steps >= 240, "stall/stop condition was not exercised");
    Check(test.saw_new_snapshot, "display did not receive fresh physics snapshots");
    Check(test.scene_model != test.physics_model && test.scene_data != test.physics_data,
        "graphics are reading mutable physics model/data");
    Check(sim.IsAssistEnabled(), "keyboard assist toggle was lost");
    Check(std::abs(sim.GetAssistHeight() - 0.80) < 1.0e-9,
        "keyboard height adjustment was lost");
    Check(std::abs(sim.GetSimTime() - test.steps * 0.001) < 1.0e-8,
        "physics timestep changed");
}

void TestCloseAndDuration(const std::string &directory) {
    TestContext test;
    context = &test;
    mujoco_sim::Simulator sim(directory + "/render_threading.yaml", "render_test", 1,
        directory + "/render_threading.xml", {0.0}, {}, {}, false);
    sim.SetObserveCallback(ObserveStep);
    sim.Run(Command, nullptr, 0.1);
    Check(test.steps > 0, "duration run did not advance physics");
    test.stall_site = StallSite::kEvents;
    test.close_on_poll = true;
    sim.Run(Command, nullptr, 5.0);
    Check(test.window_closed && test.stalled, "window close did not stop physics");
}

void TestExceptions(const std::string &directory) {
    TestContext test;
    context = &test;
    mujoco_sim::Simulator sim(directory + "/render_threading.yaml", "render_test", 1,
        directory + "/render_threading.xml", {0.0}, {}, {}, false);
    sim.SetObserveCallback(ObserveStep);
    bool caught = false;
    try {
        sim.Run([](const mujoco_sim::SimState &) -> std::optional<mujoco_sim::SimControl> {
            throw std::runtime_error("callback failure");
        }, nullptr, 5.0);
    } catch (const std::runtime_error &error) {
        caught = std::string(error.what()) == "callback failure";
    }
    Check(caught, "physics exception was not propagated to Run caller");
    // A second Run must not reuse a live worker from the failed run.
    sim.Run(Command, [&]() { return test.steps.load() < 30; }, 5.0);
    Check(test.steps >= 30, "could not restart after physics exception");

    test.throw_on_draw = true;
    caught = false;
    try {
        sim.Run(Command, nullptr, 5.0);
    } catch (const std::runtime_error &error) {
        caught = std::string(error.what()) == "render failure";
    }
    Check(caught, "render exception was not propagated after stopping physics");
}

}  // namespace

// Replace only graphics entry points; loading, stepping and copying use real MuJoCo.
extern "C" {

int glfwInit() { CheckOwner(); return GLFW_TRUE; }
void glfwTerminate() { CheckOwner(); }
GLFWwindow *glfwCreateWindow(int, int, const char *, GLFWmonitor *, GLFWwindow *) {
    CheckOwner();
    return &test_window;
}
void glfwDestroyWindow(GLFWwindow *) { CheckOwner(); }
void glfwMakeContextCurrent(GLFWwindow *) { CheckOwner(); }
void glfwSwapInterval(int) { CheckOwner(); }
void glfwSetWindowAttrib(GLFWwindow *, int, int) { CheckOwner(); }
void glfwSetWindowUserPointer(GLFWwindow *window, void *pointer) {
    CheckOwner();
    window->user_pointer = pointer;
}
void *glfwGetWindowUserPointer(GLFWwindow *window) {
    CheckOwner();
    return window->user_pointer;
}
GLFWkeyfun glfwSetKeyCallback(GLFWwindow *, GLFWkeyfun callback) {
    CheckOwner();
    context->key_callback = callback;
    return nullptr;
}
GLFWcursorposfun glfwSetCursorPosCallback(GLFWwindow *, GLFWcursorposfun) {
    CheckOwner();
    return nullptr;
}
GLFWmousebuttonfun glfwSetMouseButtonCallback(GLFWwindow *, GLFWmousebuttonfun) {
    CheckOwner();
    return nullptr;
}
GLFWscrollfun glfwSetScrollCallback(GLFWwindow *, GLFWscrollfun) {
    CheckOwner();
    return nullptr;
}
int glfwWindowShouldClose(GLFWwindow *) {
    CheckOwner();
    return context->window_closed;
}
void glfwGetFramebufferSize(GLFWwindow *, int *width, int *height) {
    CheckOwner();
    *width = 900;
    *height = 600;
}
void glfwSwapBuffers(GLFWwindow *) { StallGraphics(StallSite::kSwap); }
void glfwPollEvents() {
    StallGraphics(StallSite::kEvents);
    if (context->send_key) {
        context->send_key = false;
        context->key_callback(&test_window, GLFW_KEY_F, 0, GLFW_PRESS, 0);
        context->key_callback(&test_window, GLFW_KEY_UP, 0, GLFW_PRESS, 0);
    }
    if (context->close_on_poll) context->window_closed = true;
}

void mjr_makeContext(const mjModel *, mjrContext *, int) { CheckOwner(); }
void mjr_freeContext(mjrContext *) { CheckOwner(); }
void mjv_updateScene(const mjModel *model, mjData *data, const mjvOption *,
        const mjvPerturb *, mjvCamera *, int, mjvScene *) {
    CheckOwner();
    context->scene_model = model;
    context->scene_data = data;
    if (context->last_scene_time >= 0 && data->time > context->last_scene_time) {
        context->saw_new_snapshot = true;
    }
    context->last_scene_time = data->time;
    StallGraphics(StallSite::kScene);
}
void mjr_render(mjrRect, mjvScene *, const mjrContext *) {
    StallGraphics(StallSite::kDraw);
    if (context->throw_on_draw) throw std::runtime_error("render failure");
}

}  // extern "C"

int main(int argc, char **argv) {
    try {
        Check(argc == 2, "expected test fixture directory");
        for (const auto site : {StallSite::kScene, StallSite::kDraw,
                StallSite::kSwap, StallSite::kEvents}) {
            TestStall(argv[1], site);
        }
        TestCloseAndDuration(argv[1]);
        TestExceptions(argv[1]);
        std::cout << "render threading: PASS (graphics stalls, snapshots, input, lifecycle)\n";
    } catch (const std::exception &error) {
        std::cerr << "render threading: FAIL: " << error.what() << '\n';
        return 1;
    }
    return 0;
}
