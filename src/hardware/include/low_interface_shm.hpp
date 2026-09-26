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

#ifndef __MOTEUS_INTERFACE_SHM_HH__
#define __MOTEUS_INTERFACE_SHM_HH__

#pragma once

#include <sys/mman.h>

#include <chrono>
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

#include "low_command_type.hpp"
#include "low_protocol.h"
#include "pi3hat_low_interface.h"
#include "utils.hpp"

using namespace mjbots;

using MoteusInterface = moteus::Pi3HatMoteusInterface;

namespace {
int sign(const float& x) {
    if (x == 0) {
        return 0;
    }
    return (x > 0) ? 1 : -1;
}

void LockMemory() {
    // We lock all memory so that we don't end up having to page in
    // something later which can take time.
    {
        const int r = ::mlockall(MCL_CURRENT | MCL_FUTURE);
        if (r < 0) {
            throw std::runtime_error("Error locking memory");
        }
    }
}
}  // namespace

class LowInterface {
   public:
    LowInterface(const YAML::Node&, const std::shared_ptr<FileLogger>&);

    ~LowInterface();

    std::vector<std::vector<int>> servo_bus_map() const;

    void InitializeResolution(
        std::vector<MoteusInterface::ServoCommand>* commands);

    moteus::QueryResult Get(
        const std::vector<MoteusInterface::ServoReply>& replies, int id,
        int bus);

    moteus::PowerDistQueryResult Get(
        const std::vector<MoteusInterface::PowerDistReply>& replies, int id,
        int bus);

    void ReturnAttitude(const bool& retAtt = false,
                        const bool& retAttDetail = false);

    void SetMountingAngles(const float& yaw, const float& pitch,
                           const float& roll);

    void UseSlowCANFD(const bool& slow);

    void Step(const std::vector<MoteusInterface::ServoReply>& servo_reply,
              std::vector<MoteusInterface::ServoCommand>* servo_cmd);

    void Run();

   private:
    void initClass();
    int initParams();

    std::string m_name;
    YAML::Node m_config;
    std::shared_ptr<FileLogger> m_logger = NULL;

    float alpha = 1.0;
    double motor_curr[3];
    double tau_cutoff[3];
    float t_cutoff = 5;  // secs
    float alpha_cutoff = 0.99;
    bool stop_motor[3];
    double eps = 0.1;

    uint64_t cycle_count_ = 0;
    int main_cpu = 1;
    int can_cpu = 2;

    double period_s = 0.002;  // 0.002
    MoteusCommand* motor_command;
    pi3hat::Pi3Hat::Configuration moteusConfiguration;
    bool m_request_attitude_detail;

    vec3 init_sleep_position_rev;

    bool power_comm_flag = true;

    std::chrono::time_point<std::chrono::high_resolution_clock>
        m_startTimePoint;
};

#endif