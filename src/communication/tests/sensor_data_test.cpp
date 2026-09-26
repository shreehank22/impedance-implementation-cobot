#include "dds_subscriber.hpp"
#include "dds_publisher.hpp"
#include "SensorData.hpp"
#include "JointData.hpp"
#include <iostream>
#include <thread>
#include <chrono>
#include <functional>

using namespace xterra::msg::dds_;

double t_curr = 0;
double t_last = 0;

double get_wall_time_seconds(const std::chrono::time_point<std::chrono::high_resolution_clock>& start) {
    auto now = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(now - start).count();
    return duration / 1e6;
}

void get_sensor_data_cb(const SensorData_& msg) {
    static auto start = std::chrono::high_resolution_clock::now();

    t_last = t_curr;
    t_curr = get_wall_time_seconds(start);

    std::cout << "\nSensorData.q: ";
    auto q_arr = msg.q();
    for (int i = 0; i < 6; ++i) {
        std::cout << q_arr[i] << " ";
    }
    std::cout << "\n";

    if (t_last > 0) {
        double rate = 1.0 / (t_curr - t_last);
        std::cout << "Rate: " << rate << " Hz\n";
    }
}

int main() {
    // Subscriber for sensor data
    auto sensor_sub = std::make_shared<DDSSubscriber<SensorData_>>(
        "rt/cobot_c1/sensor_data",
        std::bind(get_sensor_data_cb, std::placeholders::_1),
        0
    );

    // Publisher for joint command
    auto joint_pub = std::make_shared<DDSPublisher<JointData_>>("rt/cobot_c1/joint_command");

    JointData_ joint_msg{};

    // Access arrays via accessors
    auto& q_arr = joint_msg.q();
    auto& dq_arr = joint_msg.dq();
    auto& tau_arr = joint_msg.tau();
    auto& kp_arr = joint_msg.kp();
    auto& kd_arr = joint_msg.kd();

    for (int i = 0; i < 6; ++i) {
        q_arr[i] = 0.0f;
        dq_arr[i] = 0.0f;
        tau_arr[i] = 0.0f;
        kp_arr[i] = 200.0f;
        kd_arr[i] = 20.0f;
    }

    std::cout << "Publishing zeros to rt/cobot_c1/joint_command with kp = 200 and kd = 20 ...\n";

    // Main loop: publish joint commands every 100ms, sensor data callback invoked asynchronously
    while (true) {
        joint_pub->publish(joint_msg);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    return 0;
}
