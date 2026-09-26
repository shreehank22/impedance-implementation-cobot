/*
 * MIT License
 *
 * Copyright (c) 2023 xTerra Robotics, info@xterrarobotics.com
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS," WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE, AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES, OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT, OR OTHERWISE, ARISING
 * FROM, OUT OF, OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 */

#include "low_interface_shm.hpp"

#include <sys/ipc.h>
#include <sys/mman.h>
#include <sys/shm.h>
#include <sys/times.h>
#include <unistd.h>

#include <chrono>
#include <csignal>
#include <ctime>
#include <fstream>
#include <future>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "low_protocol.h"
#include "pi3hat_low_interface.h"

using namespace mjbots;

using MoteusInterface = moteus::Pi3HatMoteusInterface;

struct tms processTimes;
clock_t startTime;

// Signal handler for SIGINT
void signalHandler(int signum) {
    // struct tms processTimes;
    // clock_t startTime = times(&processTimes);

    // Get the times again after the work
    clock_t endTime = times(&processTimes);
    double elapsedTicks = endTime - startTime;
    double elapsedSeconds =
        elapsedTicks / static_cast<double>(sysconf(_SC_CLK_TCK));

    std::cout << "HW interface ran for (in seconds): " << elapsedSeconds
              << "\n";

    // Get the current system date and time for logging
    std::time_t currentTime = std::time(nullptr);
    std::tm* localTime = std::localtime(&currentTime);

    char dateTimeBuffer[20];  // YYYY-MM-DD HH:MM:SS is 19 characters + null
                              // terminator
    std::strftime(dateTimeBuffer, sizeof(dateTimeBuffer), "%F %T", localTime);

    std::ofstream logFile;
    logFile.open("/home/xterra/RIMaR/logs/robot_runtime.log",
                 std::ios_base::app);  // Open in append mode
    if (logFile.is_open()) {
        logFile << "Data Logged At: " << dateTimeBuffer << " | ";
        logFile << "Elapsed Time (in secs): " << elapsedSeconds << "\n";
        logFile << "-------\n";
        logFile.close();
    } else {
        std::cerr << "Unable to open log file.\n";
    }

    exit(signum);
}

void setupSignalHandling() { signal(SIGINT, signalHandler); }

LowInterface::LowInterface(const YAML::Node& config,
                           const std::shared_ptr<FileLogger>& logger)
    : m_config(config), m_logger(logger) {
    initClass();
}

LowInterface::~LowInterface() { shmdt(motor_command); }

void LowInterface::initClass() {
    startTime = times(&processTimes);

    key_t key = 7070;
    size_t totalSize = sizeof(MoteusCommand);
    int shmid = shmget(key, totalSize, 0666 | IPC_CREAT);

    // std::cout << "[DEBUG] Shared memory size = " << totalSize << " bytes" << std::endl;

    if (shmid == -1) {
        perror("LOW INTERFACE: Failed to allocate a shared memory segment.");
    }

    void* sharedMemory = shmat(shmid, nullptr, 0);

    if (sharedMemory == reinterpret_cast<void*>(01)) {
        perror("LOW INTERFACE: Failed to attach the shared memory segment.");
    }

    motor_command = static_cast<MoteusCommand*>(sharedMemory);

    int status = initParams();

    for (int i = 0; i < 3; i++) {
        motor_command->ref_position[i] = init_sleep_position_rev(i);
        motor_curr[i] = motor_command->ref_position[i];
        motor_command->ref_velocity[i] = 0;
        motor_command->ref_ff_torque[i] = 0;
        motor_command->act_position[i] = 0;
        motor_command->act_velocity[i] = 0;
        motor_command->act_ff_torque[i] = 0;
        motor_command->kp_scale[i] = 0.0;
        motor_command->kd_scale[i] = 0.0;
        // init cutoff tau to zero
        tau_cutoff[i] = 0;
        stop_motor[i] = false;
    }
    motor_command->max_torque = 0.05;
    motor_command->alpha = alpha;

    // cutoff alpha calculation
    alpha_cutoff = pow(eps, period_s / t_cutoff);

    moteusConfiguration.request_attitude = false;
    moteusConfiguration.mounting_deg.yaw = 0;
    moteusConfiguration.mounting_deg.pitch = 0;
    moteusConfiguration.mounting_deg.roll = 0;

    // Switching from CAN-FD 5Mbps to 1Mbps for increased resilience to
    // electrical noise
    for (int i = 0; i < 5; i++) {
        moteusConfiguration.can[i].bitrate_switch = false;
    }
    m_request_attitude_detail = false;

    setupSignalHandling();
}

int LowInterface::initParams() {
    if (m_config["robot"]) {
        m_name = m_config["robot"].as<std::string>();
    }
    // Take initial positions and setups from config
    init_sleep_position_rev = vec3::Zero();
    if (m_config["sleep_pos_rev"].IsSequence() &&
        m_config["sleep_pos_rev"].size() == 3) {
        for (int i = 0; i < 3; ++i) {
            init_sleep_position_rev(i) =
                m_config["sleep_pos_rev"][i].as<double>();
        }
    } else {
        init_sleep_position_rev << 0.006, -0.042, 0.023;
    }

    if (m_config["joint_alpha"]) {
        alpha = m_config["joint_alpha"].as<double>();
    } else {
        if (m_logger->getLevel() == FileLogger::LEVEL::DEBUG) {
            m_logger->log(
                "Joint alpha not found in config. "
                "Setting to 0.9...");
        }
        alpha = 0.9;
    }

    if (m_logger->getLevel() == FileLogger::LEVEL::DEBUG) {
        m_logger->log("All parameters set");
    }

    return 0;
}

std::vector<std::vector<int>> LowInterface::servo_bus_map() const {
    return {
        {5, 90, 91, 92}
    };
}

void LowInterface::InitializeResolution(
    std::vector<MoteusInterface::ServoCommand>* commands) {
    moteus::PositionResolution res;
    res.position = moteus::Resolution::kInt16;
    res.velocity = moteus::Resolution::kInt16;
    res.feedforward_torque = moteus::Resolution::kInt16;
    res.kp_scale = moteus::Resolution::kInt16;
    res.kd_scale = moteus::Resolution::kInt16;
    res.maximum_torque = moteus::Resolution::kIgnore;
    res.stop_position = moteus::Resolution::kIgnore;
    res.watchdog_timeout = moteus::Resolution::kIgnore;
    for (auto& cmd : *commands) {
        cmd.resolution = res;
    }
}

moteus::QueryResult LowInterface::Get(
    const std::vector<MoteusInterface::ServoReply>& replies, int id, int bus) {
    for (const auto& item : replies) {
        if (item.id == id && item.bus == bus) {
            return item.result;
        }
    }
    return {};
}

moteus::PowerDistQueryResult LowInterface::Get(
    const std::vector<MoteusInterface::PowerDistReply>& replies, int id,
    int bus) {
    for (const auto& item : replies) {
        if (item.id == id && item.bus == bus) {
            return item.result;
        }
    }
    return {};
}

void LowInterface::ReturnAttitude(const bool& retAtt,
                                  const bool& retAttDetail) {
    moteusConfiguration.request_attitude = retAtt;
    m_request_attitude_detail = retAttDetail;
}

void LowInterface::SetMountingAngles(const float& roll = 0,
                                     const float& pitch = 0,
                                     const float& yaw = 0) {
    moteusConfiguration.mounting_deg.roll = roll;
    moteusConfiguration.mounting_deg.pitch = pitch;
    moteusConfiguration.mounting_deg.yaw = yaw;
}

void LowInterface::UseSlowCANFD(const bool& slow = false) {
    for (int i = 0; i < 3; ++i) {
        // bitrate switch is true for 5Mbps CAN-FD and false for 1Mbps CAN-FD
        moteusConfiguration.can[i].bitrate_switch = !slow;
    }
}

void LowInterface::Step(
    const std::vector<MoteusInterface::ServoReply>& servo_reply,
    std::vector<MoteusInterface::ServoCommand>* servo_cmd) {
    cycle_count_++;

    // moteus interface loop (read values from shm mem pool and pass to
    // motor_out)
    if (cycle_count_ < 5) {
        for (auto& cmd : *servo_cmd) {
            // We start everything with a stopped command to clear faults.
            cmd.mode = moteus::Mode::kStopped;
            cmd.query.position = moteus::Resolution::kFloat;
            const auto motor_read = Get(servo_reply, cmd.id, cmd.bus);
            int index = int(cmd.id / 90 - 1) * 3 + int(cmd.id % 10);
            if (!std::isnan(motor_read.position)) {
                motor_curr[index] = motor_read.position;
            }
        }
    } else {
        for (auto& cmd : *servo_cmd) {
            // index for all the actuators (constructing from their ID's)
            int index = (cmd.id / 90 - 1) * 3 + cmd.id % 10;

            //DEBUG
            cmd.mode = moteus::Mode::kPosition;

            const auto motor_read = Get(servo_reply, cmd.id, cmd.bus);

            cmd.mode = (stop_motor[index]) ? moteus::Mode::kStopped
                                           : moteus::Mode::kPosition;
            
            // applying a first order filter to the position command
            alpha = motor_command->alpha;
            motor_curr[index] =
                alpha * motor_curr[index] +
                (1 - alpha) * motor_command->ref_position[index];
            // write the reference commands to moteus
            cmd.resolution.position = moteus::Resolution::kFloat;
            cmd.position.position = motor_curr[index];

            float cmd_velocity = 0;
            if (abs(motor_curr[index] - motor_read.position) > 0.5) {
                cmd_velocity =
                    sign(motor_curr[index] - motor_read.position) * 1;
            } else {
                cmd_velocity = motor_command->ref_velocity[index];
            }
            cmd.position.velocity = cmd_velocity;
            cmd.position.feedforward_torque =
                motor_command->ref_ff_torque[index];
            cmd.position.maximum_torque = motor_command->max_torque;
            cmd.position.kp_scale = motor_command->kp_scale[index];
            cmd.position.kd_scale = motor_command->kd_scale[index];

            cmd.query.position = moteus::Resolution::kFloat;
            cmd.query.velocity = moteus::Resolution::kFloat;
            cmd.query.torque = moteus::Resolution::kFloat;
            cmd.query.q_current = moteus::Resolution::kFloat;
            cmd.query.d_current = moteus::Resolution::kFloat;

            // read actual joint data from moteus
            if (!std::isnan(motor_read.position)) {
                motor_command->act_position[index] = motor_read.position;
            }
            if (!std::isnan(motor_read.velocity)) {
                motor_command->act_velocity[index] = motor_read.velocity;
            }
            if (!std::isnan(motor_read.torque)) {
                motor_command->act_ff_torque[index] = motor_read.torque;
            }

            if (cycle_count_ % 500 == 0) {
                // std::cout << "Cycle count: " << cycle_count_ << "\n";
                // std::cout << "Desired position: " << index << ": "
                //           << motor_command->ref_position[index] << " | Actual position: " << ": "
                //           << motor_command->act_position[index]<< std::endl;
                // std::cout << "Actual position: " << index << ": "
                //           << motor_read.position << std::endl;
                // std::cout << "Actual torque : " << index << " : "
                //           << motor_read.torque << '\n';
                // std::cout << "Actual Q_current: " << index << ": "
                //           << motor_read.q_current << '\n';
                // std::cout << "Actual D_current : " << index << " : "
                //           << motor_read.d_current << '\n';
                // std::cout << "Fault mode: " << index << ": " <<
                // motor_read.fault
                //           << '\n';
            }

            if (!std::isnan(motor_read.torque)) {
                tau_cutoff[index] = alpha_cutoff * tau_cutoff[index] +
                                    (1 - alpha_cutoff) * motor_read.torque;

                if (fabs(tau_cutoff[index]) > motor_command->max_torque) {
                    stop_motor[index] = true;
                    std::cout << "Stopping motor id: " << cmd.id << "\n";
                }
            }
        }
    }
}


void LowInterface::Run() {

    moteus::ConfigureRealtime(main_cpu);
    MoteusInterface::Options moteus_options;
    moteus_options.cpu = can_cpu;
    moteus_options.configuration_ = moteusConfiguration;
    MoteusInterface moteus_interface{moteus_options};

    moteus_interface.pi3hat_can_data_.request_attitude_detail = m_request_attitude_detail;
    moteus_interface.pi3hat_can_data_.attitude = new pi3hat::Attitude();

    std::vector<MoteusInterface::ServoCommand> commands;

    for (const auto& bus_id : servo_bus_map()) {
        for (int j = 0; j < 3; ++j) {
            commands.push_back({});
            commands.back().id = bus_id[j + 1];
            commands.back().bus = bus_id[0];
            // std::cout << "[Run] Added command for servo id: " << static_cast<int>(commands.back().id) 
            //           << " on bus: " << static_cast<int>(commands.back().bus) << "\n";
        }
    }

    std::vector<MoteusInterface::ServoReply> replies{commands.size()};
    std::vector<MoteusInterface::ServoReply> saved_replies;

    InitializeResolution(&commands);

    MoteusInterface::Data moteus_data;
    moteus_data.commands = {commands.data(), commands.size()};
    moteus_data.replies = {replies.data(), replies.size()};

    std::future<MoteusInterface::Output> can_result;

    const auto period = std::chrono::microseconds(static_cast<int64_t>(period_s * 1e6));
    auto next_cycle = std::chrono::steady_clock::now() + period;

    uint64_t cycle_count = 0;
    double total_margin = 0.0;
    uint64_t margin_cycles = 0;

    while (true) {
        auto cycle_start = std::chrono::steady_clock::now();

        cycle_count++;
        margin_cycles++;

        const auto now = std::chrono::steady_clock::now();

        int skip_count = 0;
        while (now > next_cycle) {
            skip_count++;
            next_cycle += period;
        }
        if (skip_count) {
            std::cout << "[Run][Cycle " << cycle_count << "] Skipped " << skip_count << " cycles\n";
            if (m_logger->getLevel() == FileLogger::LEVEL::DEBUG) {
                std::string msg = "Skipped " + std::to_string(skip_count) + " cycles";
                m_logger->log(msg);
            }
        }

        const auto pre_sleep = std::chrono::steady_clock::now();
        std::this_thread::sleep_until(next_cycle);
        const auto post_sleep = std::chrono::steady_clock::now();

        std::chrono::duration<double> elapsed = post_sleep - pre_sleep;
        total_margin += elapsed.count();

        next_cycle += period;

        Step(saved_replies, &commands);

        if (moteusConfiguration.request_attitude) {
            motor_command->imuData.attitudeQuaternion[0] =
                moteus_interface.pi3hat_can_data_.attitude->attitude.w;
            motor_command->imuData.attitudeQuaternion[1] =
                moteus_interface.pi3hat_can_data_.attitude->attitude.x;
            motor_command->imuData.attitudeQuaternion[2] =
                moteus_interface.pi3hat_can_data_.attitude->attitude.y;
            motor_command->imuData.attitudeQuaternion[3] =
                moteus_interface.pi3hat_can_data_.attitude->attitude.z;
            motor_command->imuData.accel[0] =
                moteus_interface.pi3hat_can_data_.attitude->accel_mps2.x;
            motor_command->imuData.accel[1] =
                moteus_interface.pi3hat_can_data_.attitude->accel_mps2.y;
            motor_command->imuData.accel[2] =
                moteus_interface.pi3hat_can_data_.attitude->accel_mps2.z;
            motor_command->imuData.rotationRate[0] =
                moteus_interface.pi3hat_can_data_.attitude->rate_dps.x;
            motor_command->imuData.rotationRate[1] =
                moteus_interface.pi3hat_can_data_.attitude->rate_dps.y;
            motor_command->imuData.rotationRate[2] =
                moteus_interface.pi3hat_can_data_.attitude->rate_dps.z;
        } else {
            motor_command->imuData.attitudeQuaternion[0] = 1;
            motor_command->imuData.attitudeQuaternion[1] = 0;
            motor_command->imuData.attitudeQuaternion[2] = 0;
            motor_command->imuData.attitudeQuaternion[3] = 0;
        }

        if (can_result.valid()) {
            const auto current_values = can_result.get();

            const auto rx_count = current_values.query_result_size;

            size_t servo_rx_count = 3;
            saved_replies.resize(servo_rx_count);
            std::copy(replies.begin(), replies.begin() + servo_rx_count, saved_replies.begin());
        }

        auto promise = std::make_shared<std::promise<MoteusInterface::Output>>();
        moteus_interface.Cycle(moteus_data, [promise](const MoteusInterface::Output& output) {
            promise->set_value(output);
        });
        can_result = promise->get_future();

        auto cycle_end = std::chrono::steady_clock::now();
        std::chrono::duration<double> cycle_dur = cycle_end - cycle_start;
        double freq = 1.0 / cycle_dur.count();

        if (cycle_count % 1000 == 0) {
            // std::cout << "[Run][Cycle " << cycle_count << "] Cycle frequency: " << freq << " Hz\n";
        }
    }
}
