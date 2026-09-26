// simulate_pv.cpp — RIMaR Cobot MuJoCo Simulator
//
// Full LAIR-style simulation window:
//   Left panel  — File / Simulation / Physics / Rendering / Visualization / Groups
//   Right panel — Joint sliders / Control sliders
//   Tab / Shift-Tab  : toggle panels
//   Space            : play/pause        +/-       : speed
//   Left/Right arrow : step back/forward F3/F4/F5  : profiler/sensors/fullscreen
//   Drag-and-drop a new .xml onto the window to hot-swap models

#include "simulate.h"
#include "glfw_adapter.h"

#include <mujoco/mujoco.h>

#include <csignal>
#include <cstdlib>
#include <cstdio>
#include <iostream>
#include <string>
#include <thread>
#include <algorithm>

#include "array_safety.h"

#include "QuadDDSComm.hpp"
#include "memory_types.hpp"

// ─── DDS PLANT comm state ─────────────────────────────────────────────────────

// DDS comm object (PLANT mode) — subscribes joint_command, publishes sensor_data
static std::shared_ptr<QuadDDSComm> g_comm;

// Persistent data structs — DDS thread reads/writes these directly via raw ptrs
static CobotSensorData  g_sensor_data;
static CobotCommandData g_cmd_data;

// Robot identity (can be overridden via CLI)
static std::string g_robot_name   = "cobot_c1";
static std::string g_comm_postfix = "/sim";

// ─── cobot DDS-driven PD controller ──────────────────────────────────────────

static void CobotController(const mjModel* m, mjData* d) {
    int n = (m->nu < 6) ? m->nu : 6;
    for (int i = 0; i < n; ++i) {
        // Use gains from executor if available; fall back to safe defaults
        double kp  = (g_cmd_data.kp(i) > 0) ? g_cmd_data.kp(i) : 30.0;
        double kd  = (g_cmd_data.kd(i) > 0) ? g_cmd_data.kd(i) : 3.0;
        double ctrl = kp * (g_cmd_data.q(i) - d->qpos[i])
                    - kd * d->qvel[i]
                    + g_cmd_data.tau(i);
        d->ctrl[i] = (ctrl >  200.0) ?  200.0 : (ctrl < -200.0) ? -200.0 : ctrl;
    }

}

// ─── plugin discovery ────────────────────────────────────────────────────────

static std::string executableDir() {
    char resolved[4096];
    ssize_t n = readlink("/proc/self/exe", resolved, sizeof(resolved) - 1);
    if (n < 0) return "";
    resolved[n] = '\0';
    std::string p(resolved);
    auto slash = p.rfind('/');
    return (slash == std::string::npos) ? "" : p.substr(0, slash);
}

static void scanPlugins() {
    if (int n = mjp_pluginCount()) {
        std::printf("Built-in plugins:\n");
        for (int i = 0; i < n; ++i)
            std::printf("    %s\n", mjp_getPluginAtSlot(i)->name);
    }
    std::string dir = executableDir();
    if (dir.empty()) return;
    mj_loadAllPluginLibraries(
        (dir + "/mujoco_plugin").c_str(),
        +[](const char* f, int first, int count) {
            std::printf("Plugins from '%s':\n", f);
            for (int i = first; i < first + count; ++i)
                std::printf("    %s\n", mjp_getPluginAtSlot(i)->name);
        });
}

// ─── physics thread ──────────────────────────────────────────────────────────

namespace {
namespace mj  = ::mujoco;
namespace mju = ::mujoco::sample_util;

using Seconds = std::chrono::duration<double>;

constexpr int    kErrLen           = 1024;
constexpr double kSyncMisalign     = 0.1;
constexpr double kRefreshFraction  = 0.7;

mjModel* m         = nullptr;
mjData*  d         = nullptr;
mjtNum*  ctrlnoise = nullptr;

static mjModel* loadModel(const char* file, mj::Simulate& sim) {
    char filename[mj::Simulate::kMaxFilenameLength];
    mju::strcpy_arr(filename, file);
    if (!filename[0]) return nullptr;

    char err[kErrLen] = "";
    mjModel* mnew =
        (mju::strlen_arr(filename) > 4 &&
         !std::strncmp(filename + mju::strlen_arr(filename) - 4, ".mjb",
                       mju::sizeof_arr(filename) - mju::strlen_arr(filename) + 4))
        ? mj_loadModel(filename, nullptr)
        : mj_loadXML(filename, nullptr, err, kErrLen);

    if (err[0]) {
        int len = mju::strlen_arr(err);
        if (err[len - 1] == '\n') err[len - 1] = '\0';
    }
    mju::strcpy_arr(sim.load_error, err);

    if (!mnew) { std::printf("%s\n", err); return nullptr; }
    if (err[0]) { std::printf("Warning (paused):\n  %s\n", err); sim.run = 0; }
    return mnew;
}

static void replaceModel(mj::Simulate& sim, mjModel* mnew, const char* name) {
    mjData* dnew = mj_makeData(mnew);
    if (!dnew) { sim.LoadMessageClear(); mj_deleteModel(mnew); return; }

    sim.Load(mnew, dnew, name);
    const std::unique_lock<std::recursive_mutex> lock(sim.mtx);
    mj_deleteData(d); mj_deleteModel(m);
    m = mnew; d = dnew;
    mj_forward(m, d);
    free(ctrlnoise);
    ctrlnoise = static_cast<mjtNum*>(malloc(sizeof(mjtNum) * m->nu));
    mju_zero(ctrlnoise, m->nu);
}

static void physicsLoop(mj::Simulate& sim) {
    std::chrono::time_point<mj::Simulate::Clock> syncCPU;
    mjtNum syncSim = 0;

    while (!sim.exitrequest.load()) {
        if (sim.droploadrequest.load()) {
            sim.LoadMessage(sim.dropfilename);
            if (mjModel* mnew = loadModel(sim.dropfilename, sim))
                replaceModel(sim, mnew, sim.dropfilename);
            else
                sim.LoadMessageClear();
            sim.droploadrequest.store(false);
        }

        if (sim.uiloadrequest.load()) {
            sim.uiloadrequest.fetch_sub(1);
            sim.LoadMessage(sim.filename);
            if (mjModel* mnew = loadModel(sim.filename, sim))
                replaceModel(sim, mnew, sim.filename);
            else
                sim.LoadMessageClear();
        }

        sim.run && sim.busywait ? std::this_thread::yield()
                                : std::this_thread::sleep_for(std::chrono::milliseconds(1));

        const std::unique_lock<std::recursive_mutex> lock(sim.mtx);
        if (!m) continue;

        if (sim.run) {
            bool stepped = false;
            const auto startCPU   = mj::Simulate::Clock::now();
            const auto elapsedCPU = startCPU - syncCPU;
            double     elapsedSim = d->time - syncSim;

            if (sim.ctrl_noise_std) {
                mjtNum rate  = mju_exp(-m->opt.timestep / mju_max(sim.ctrl_noise_rate, mjMINVAL));
                mjtNum scale = sim.ctrl_noise_std * mju_sqrt(1 - rate * rate);
                for (int i = 0; i < m->nu; i++) {
                    ctrlnoise[i] = rate * ctrlnoise[i] + scale * mju_standardNormal(nullptr);
                    d->ctrl[i] = ctrlnoise[i];
                }
            }

            double slowdown = 100.0 / sim.percentRealTime[sim.real_time_index];
            bool misaligned =
                mju_abs(Seconds(elapsedCPU).count() / slowdown - elapsedSim) > kSyncMisalign;

            if (elapsedSim < 0 || elapsedCPU.count() < 0 ||
                !syncCPU.time_since_epoch().count() || misaligned || sim.speed_changed) {
                syncCPU = startCPU; syncSim = d->time;
                sim.speed_changed = false;
                mj_step(m, d); stepped = true;
            } else {
                mjtNum prevSim     = d->time;
                double refreshTime = kRefreshFraction / sim.refresh_rate;
                while (Seconds((d->time - syncSim) * slowdown) < mj::Simulate::Clock::now() - syncCPU &&
                       mj::Simulate::Clock::now() - startCPU < Seconds(refreshTime)) {
                    mj_step(m, d); stepped = true;
                    if (d->time < prevSim) break;
                }
            }

            // Update sensor data from MuJoCo state so the DDS PLANT thread can
            // publish it. This runs inside sim.mtx so it is safe to read d->qpos.
            if (stepped && m && d) {
                int nq = std::min(m->nq, 6);
                for (int i = 0; i < nq; ++i) {
                    g_sensor_data.q(i)   = d->qpos[i];
                    g_sensor_data.qd(i)  = d->qvel[i];
                    g_sensor_data.tau(i) = d->actuator_force[i];
                }
            }

            if (stepped) sim.AddToHistory();
        } else {
            mj_forward(m, d);
            sim.speed_changed = true;
        }
    }
}

}  // namespace

void PhysicsThread(mj::Simulate* sim, const char* filename) {
    if (filename) {
        sim->LoadMessage(filename);
        m = loadModel(filename, *sim);
        if (m) {
            const std::unique_lock<std::recursive_mutex> lock(sim->mtx);
            d = mj_makeData(m);
        }
        if (d) {
            sim->Load(m, d, filename);
            const std::unique_lock<std::recursive_mutex> lock(sim->mtx);
            mj_forward(m, d);
            ctrlnoise = static_cast<mjtNum*>(malloc(sizeof(mjtNum) * m->nu));
            mju_zero(ctrlnoise, m->nu);
        } else {
            sim->LoadMessageClear();
        }
    }
    // Start DDS comm in PLANT mode (subscribes joint_command, publishes sensor_data)
    g_comm = std::make_shared<QuadDDSComm>(g_robot_name, g_comm_postfix, DATA_ACCESS_MODE::PLANT);
    g_comm->setSensorDataPtr(&g_sensor_data);
    g_comm->setCommandDataPtr(&g_cmd_data);
    g_comm->setUpdateRate(500);
    g_comm->start_thread();
    std::printf("[simulate_pv] DDS PLANT on topics rt/%s%s/{sensor_data,joint_command}\n",
                g_robot_name.c_str(), g_comm_postfix.c_str());

    physicsLoop(*sim);
    free(ctrlnoise);
    mj_deleteData(d);
    mj_deleteModel(m);
}

// ─── main ────────────────────────────────────────────────────────────────────

int main(int argc, char** argv) {
    std::printf("MuJoCo version %s\n", mj_versionString());
    if (mjVERSION_HEADER != mj_version())
        mju_error("Headers and library have different versions");

    std::string model_file =
        "/home/snehkumar/xTerra/RIMaR/src/robots/cobot_c1_description/mujoco/scene.xml";

    for (int i = 1; i < argc; ++i) {
        std::string a(argv[i]);
        if (a == "--help" || a == "-h") {
            std::printf("Usage: simulate_pv [--model <path>] [--robot-only] [--postfix sim|hw] [--robot-name <name>] [<path>]\n"
                        "  Default model: cobot_c1_description/mujoco/scene.xml\n"
                        "  --postfix  : DDS topic postfix ('sim' or 'hw', default: sim)\n"
                        "  --robot-name: robot base name for DDS topics (default: cobot_c1)\n");
            return 0;
        } else if (a == "--model" && i + 1 < argc) {
            model_file = argv[++i];
        } else if (a == "--robot-only") {
            model_file = "/home/snehkumar/xTerra/RIMaR/src/robots/cobot_c1_description/mujoco/cobot_c1.xml";
        } else if (a == "--postfix" && i + 1 < argc) {
            std::string pf = argv[++i];
            g_comm_postfix = (pf == "hw") ? "/hw" : "/sim";
        } else if (a == "--robot-name" && i + 1 < argc) {
            g_robot_name = argv[++i];
        } else if (a[0] != '-') {
            model_file = a;
        }
    }

    std::printf("[simulate_pv] Model: %s\n", model_file.c_str());

    scanPlugins();
    signal(SIGINT, [](int s){ std::printf("Signal %d\n", s); exit(s); });

    mjvCamera cam;   mjv_defaultCamera(&cam);
    mjvOption opt;   mjv_defaultOption(&opt);
    mjvPerturb pert; mjv_defaultPerturb(&pert);

    mjcb_control = CobotController;

    auto sim = std::make_unique<mujoco::Simulate>(
        std::make_unique<mujoco::GlfwAdapter>(),
        &cam, &opt, &pert, /*is_passive=*/false);

    std::thread physics(&PhysicsThread, sim.get(), model_file.c_str());
    sim->RenderLoop();
    physics.join();

    return 0;
}
