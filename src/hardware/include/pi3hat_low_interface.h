// Copyright 2020 Josh Pieper, jjp@pobox.com.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <condition_variable>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <vector>

#include "low_protocol.h"
#include "pi3hat.h"
#include "realtime.h"

namespace mjbots {
namespace moteus {

/// This class represents the interface to the moteus controllers.
/// Internally it uses a background thread to operate the pi3hat,
/// enabling the main thread to perform work while servo communication
/// is taking place.
class Pi3HatMoteusInterface {
   public:
    pi3hat::Pi3Hat::Input pi3hat_can_data_;
    struct Options {
        int cpu = -1;
        pi3hat::Pi3Hat::Configuration configuration_;
    };

    Pi3HatMoteusInterface(const Options& options)
        : options_(options),
          thread_(std::bind(&Pi3HatMoteusInterface::CHILD_Run, this)) {}

    ~Pi3HatMoteusInterface() {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            done_ = true;
            condition_.notify_one();
        }
        thread_.join();
    }

    struct ServoCommand {
        int id = 0;
        int bus = 1;

        moteus::Mode mode = moteus::Mode::kStopped;

        // For mode = kPosition or kZeroVelocity
        moteus::PositionCommand position;
        moteus::PositionResolution resolution;

        moteus::QueryCommand query;
    };

    struct ServoReply {
        int id = 0;
        int bus = 0;
        moteus::QueryResult result;
    };

    struct PowerDistCommand {
        int id = 0;
        int bus = 5;
        moteus::PowerDistQueryCommand query;
    };

    struct PowerDistReply {
        int id = 0;
        int bus = 0;
        moteus::PowerDistQueryResult result;
    };

    // This describes what you would like to do in a given control cycle
    // in terms of sending commands or querying data.
    struct Data {
        pi3hat::Span<ServoCommand> commands;

        pi3hat::Span<ServoReply> replies;

        pi3hat::Span<PowerDistCommand> power_dist_commands;

        pi3hat::Span<PowerDistReply> power_dist_replies;

        bool comm_power_dist_flag = false;
    };

    struct Output {
        size_t query_result_size = 0;
    };

    using CallbackFunction = std::function<void(const Output&)>;

    /// When called, this will schedule a cycle of communication with
    /// the servos.  The callback will be invoked from an arbitrary
    /// thread when the communication cycle has completed.
    ///
    /// All memory pointed to by @p data must remain valid until the
    /// callback is invoked.
    void Cycle(const Data& data, CallbackFunction callback) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (active_) {
            throw std::logic_error(
                "Cycle cannot be called until the previous has completed");
        }

        callback_ = std::move(callback);
        active_ = true;
        data_ = data;

        condition_.notify_all();
    }

   private:
    void print_mode(Mode mode) {
        switch (mode) {
            case Mode::kStopped:
                std::cout << "Stopped\n";
                break;
            case Mode::kPosition:
                std::cout << "Position\n";
                break;
            case Mode::kZeroVelocity:
                std::cout << "ZeroVelocity\n";
                break;
            case Mode::kFault:
                std::cout << "Fault\n";
                break;
            case Mode::kPositionTimeout:
                std::cout << "PositionTimeout\n";
                break;
            case Mode::kEnabling:
                std::cout << "Enabling\n";
                break;
            case Mode::kCalibrating:
                std::cout << "Calibrating\n";
                break;
            case Mode::kCalibrationComplete:
                std::cout << "CalibrationComplete\n";
                break;
            case Mode::kPwm:
                std::cout << "Pwm\n";
                break;
            case Mode::kVoltage:
                std::cout << "Voltage\n";
                break;
            case Mode::kVoltageFoc:
                std::cout << "VoltageFoc\n";
                break;
            case Mode::kVoltageDq:
                std::cout << "VoltageDq\n";
                break;
            case Mode::kCurrent:
                std::cout << "Current\n";
                break;
            default:
                std::cout << "Unknown\n";
                break;
        }
    }

    void CHILD_Run() {
        ConfigureRealtime(options_.cpu);

        // // meshin: Why is he resetting the object???
        // pi3hat_.reset(new pi3hat::Pi3Hat({}));
        // meshin: maybe we'll need this!
        pi3hat_.reset(new pi3hat::Pi3Hat(options_.configuration_));

        while (true) {
            {
                std::unique_lock<std::mutex> lock(mutex_);
                if (!active_) {
                    condition_.wait(lock);
                    if (done_) {
                        return;
                    }

                    if (!active_) {
                        continue;
                    }
                }
            }

            auto output = CHILD_Cycle();

            CallbackFunction callback_copy;
            {
                std::unique_lock<std::mutex> lock(mutex_);
                active_ = false;
                std::swap(callback_copy, callback_);
            }
            callback_copy(output);
        }
    }

    Output CHILD_Cycle() {
        tx_can_.resize(data_.commands.size() +
                       data_.power_dist_commands.size());
        int out_idx = 0;

        // setup CAN frames in tx_can_ for moteus communication
        for (const auto& cmd : data_.commands) {
            const auto& query = cmd.query;

            auto& can = tx_can_[out_idx++];

            can.expect_reply = query.any_set();
            // std::cout << "cmd.id: " << std::hex << cmd.id << std::endl;
            can.id = cmd.id | (can.expect_reply ? 0x8000 : 0x0000);
            can.bus = cmd.bus;
            can.size = 0;

            moteus::WriteCanFrame write_frame(can.data, &can.size);
            switch (cmd.mode) {
                case Mode::kStopped: {
                    moteus::EmitStopCommand(&write_frame);
                    break;
                }
                case Mode::kPosition:
                case Mode::kZeroVelocity: {
                    moteus::EmitPositionCommand(&write_frame, cmd.position,
                                                cmd.resolution);
                    break;
                }
                default: {
                    throw std::logic_error("unsupported mode");
                }
            }
            moteus::EmitQueryCommand(&write_frame, cmd.query);
        }

        // setup CAN frames in tx_can_ for power_dist communication
        for (const auto& cmd : data_.power_dist_commands) {
            const auto& query = cmd.query;

            auto& can = tx_can_[out_idx++];

            can.expect_reply = true;  // query.expect_reply;
            can.id = cmd.id | (can.expect_reply ? 0x8000 : 0x0000);
            can.bus = cmd.bus;
            can.size = query.query_frame.size;
            std::memcpy(can.data, query.query_frame.data,
                        query.query_frame.size);
        }

        rx_can_.resize(
            (data_.commands.size() + data_.power_dist_commands.size()) * 2);

        pi3hat_can_data_.tx_can = {tx_can_.data(), tx_can_.size()};
        pi3hat_can_data_.rx_can = {rx_can_.data(), rx_can_.size()};
        pi3hat_can_data_.request_attitude =
            options_.configuration_.request_attitude;

        // for(size_t i = 0; i < tx_can_.size(); i++)
        //   std::cout << "can.id: " << std::hex << tx_can_[i].id << "can.bus: "
        //   << tx_can_[i].bus << "\n";

        Output result;

        const auto output = pi3hat_->Cycle(pi3hat_can_data_);

        size_t pd_idx = 0;
        size_t moteus_idx = 0;

        for (size_t i = 0;
             i < output.rx_can_size &&
             i < (data_.replies.size() + data_.power_dist_replies.size());
             i++) {
            const auto& can = rx_can_[i];
            // std::cout << "i: " << i << " | can.id: " << std::dec << can.id <<
            // "\n";

            if ((can.id & 0xFF00) == 0x5a00 || (can.id & 0xFF00) == 0x5b00 ||
                (can.id & 0xFF00) == 0x5c00 || (can.id & 0xFF00) == 0x5d00) {

                    //CAN frame ID matches (took one week)
        //         data_.power_dist_replies[pd_idx].id = (can.id >> 8);
		// std::cout << "i: " << i << " | data_.power_dist_replies[i].id: "
        //                  << data_.power_dist_replies[pd_idx].id << std::endl;
        //         data_.power_dist_replies[pd_idx].bus = can.bus;
        //         data_.power_dist_replies[pd_idx].result =
        //             moteus::ParsePowerDistQueryResult(can.data, can.size);
        //         result.query_result_size++;
            //     pd_idx++;
            // } else {
                data_.replies[moteus_idx].id = (can.id & 0x7f00) >> 8;
                // std::cout << "i: " << i << " | data_.replies[i].id: "
                //          << data_.replies[moteus_idx].id << std::endl;
                data_.replies[moteus_idx].bus = can.bus;
                data_.replies[moteus_idx].result =
                    moteus::ParseQueryResult(can.data, can.size);
                result.query_result_size++;
                moteus_idx++;
                // for(const auto& cmd : data_.commands)
                // std::cout << "i: " << i << " | can.id: " << std::hex << can.id << "\n"; std::cout << "INTERFACE 4.2 | cmd.id: " <<
                // std::dec << static_cast<unsigned int>(data_.commands[0].id)
                // << "\t" << " cmd.bus: " << std::dec << data_.commands[0].bus
                // << "\n"; print_mode(data_.commands[0].mode);
            }
            // std::cout << "\n";
        }

        if (data_.commands[0].id == 0) {
            std::cout << "Manual reset of ID 10.\n";
            data_.commands[0].id = 90;
            data_.commands[0].mode = Mode::kStopped;
        }

        // std::cout << "INTERFACE | query_result_size: " << static_cast<int>(result.query_result_size) << "\n";
        // std::cout << "INTERFACE | pd_idx: " << static_cast<int>(pd_idx)
        //   << " | moteus_idx: " << static_cast<int>(moteus_idx) << std::endl;

        return result;
    }

    const Options options_;

    /// This block of variables are all controlled by the mutex.
    std::mutex mutex_;
    std::condition_variable condition_;
    bool active_ = false;
    ;
    bool done_ = false;
    CallbackFunction callback_;
    Data data_;

    std::thread thread_;

    /// All further variables are only used from within the child thread.

    std::unique_ptr<pi3hat::Pi3Hat> pi3hat_;

    // These are kept persistently so that no memory allocation is
    // required in steady state.
    std::vector<pi3hat::CanFrame> tx_can_;
    std::vector<pi3hat::CanFrame> rx_can_;
};

}  // namespace moteus
}  // namespace mjbots
