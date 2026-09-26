#include <sys/ipc.h>
#include <sys/shm.h>

#include <csignal>
#include <eigen3/Eigen/Dense>
#include <filesystem>
#include <iostream>
#include <mutex>
#include <thread>

#include "low_command_type.hpp"
#include "utils.hpp"

#if defined(USE_ROS2_COMM)
#include "QuadROSComm.hpp"
#elif defined(USE_ROS_COMM)
#include "QuadROS1Comm.hpp"
#elif defined(USE_DDS_COMM)
#include "QuadDDSComm.hpp"
#else
#include "SHM.hpp"
#endif

CobotSensorData sensor_data;
CobotCommandData joint_command_data;
// CobotMeasurementData measurement_data;
// CobotPowerData power_data;
// CobotIndicatorData indicator_data;

// Signal handler flag
bool running = true;

#if defined(USE_ROS2_COMM)
std::shared_ptr<QuadROSComm> comm_data_ptr;
#elif defined(USE_ROS_COMM)
std::shared_ptr<QuadROS1Comm> comm_data_ptr;
#elif defined(USE_DDS_COMM)
std::shared_ptr<QuadDDSComm> comm_data_ptr;
#else
std::shared_ptr<SHM> comm_data_ptr;
#endif

std::chrono::time_point<std::chrono::high_resolution_clock> m_startTimePoint;

double updateTimer() {
    auto currTimePont = std::chrono::high_resolution_clock::now();
    auto start = std::chrono::time_point_cast<std::chrono::microseconds>(
                     m_startTimePoint)
                     .time_since_epoch()
                     .count();
    auto curr =
        std::chrono::time_point_cast<std::chrono::microseconds>(currTimePont)
            .time_since_epoch()
            .count();
    auto duration = curr - start;

    return duration * double(1e-6);
}

float saturate(float value, const float min, const float max) {
    return std::min(std::max(value, min), max);
}

void signalHandler(int signum) {
    std::cout << "Interrupt signal (" << signum << ") received. Shutting down..." << std::endl;
    running = false;
    // exit(signum);
}

// void setIndicatorStatus(const bool led1, const bool led2, const bool led3,
//                         const bool buzzer) {
//     indicator_data.led1 = led1;
//     indicator_data.led2 = led2;
//     indicator_data.led3 = led3;
//     indicator_data.buzzer = buzzer;
// }

int main(int argc, char *argv[]) {
    // std::cout << "Starting main program..." << std::endl;
    ArgumentParser parser(argc, argv);
#if defined(USE_ROS2_COMM)
    std::cout << "Using ROS2 communication" << std::endl;
    rclcpp::init(argc, argv);
#elif defined(USE_ROS_COMM)
    std::cout << "Using ROS1 communication" << std::endl;
    ros::init(argc, argv, "bridge_ros");
#else
    // std::cout << "Using Shared Memory communication" << std::endl;
#endif

    std::string m_name("cobot_c1");
    std::string comm_name = m_name; // The low level interface must be accessible only on HW

#if defined(USE_ROS2_COMM)
    comm_data_ptr =
        std::make_shared<QuadROSComm>(comm_name, DATA_ACCESS_MODE::PLANT);
#elif defined(USE_ROS_COMM)
    comm_data_ptr =
        std::make_shared<QuadROS1Comm>(comm_name, DATA_ACCESS_MODE::PLANT);
    comm_data_ptr->setUpdateRate(500);
#elif defined(USE_DDS_COMM)
    comm_data_ptr =
        std::make_shared<QuadDDSComm>(comm_name, DATA_ACCESS_MODE::PLANT);
    comm_data_ptr->setUpdateRate(500);
#else
    comm_data_ptr = std::make_shared<SHM>(comm_name, DATA_ACCESS_MODE::PLANT);
#endif

    comm_data_ptr->setCommandDataPtr(&joint_command_data);
    comm_data_ptr->setSensorDataPtr(&sensor_data);
    //comm_data_ptr->setMeasurementDataPtr(&measurement_data);

#if defined(USE_ROS2_COMM) || defined(USE_ROS_COMM) || defined(USE_DDS_COMM)
    // std::cout << "Starting communication thread..." << std::endl;
    comm_data_ptr->start_thread();
#if defined(USE_DDS_COMM)
    std::this_thread::sleep_for(std::chrono::milliseconds(500)); // DDS discovery delay
#endif
#endif

    /*************** Setting up the shared memory block ***************/
    key_t key = 7070;
    size_t totalSize = sizeof(MoteusCommand);
    // std::cout << "Creating shared memory with key " << key
    //           << " and size " << totalSize << " bytes." << std::endl;

    int shmid = shmget(key, totalSize, 0666 | IPC_CREAT);
    if (shmid == -1) {
        perror("shmget");
        return 1;
    }
    void *sharedMemory = shmat(shmid, nullptr, 0);
    if (sharedMemory == reinterpret_cast<void *>(-1)) {
        perror("shmat");
        return 1;
    }
    MoteusCommand *command_ptr = static_cast<MoteusCommand *>(sharedMemory);
    // std::cout << "Shared memory attached successfully at " << command_ptr << std::endl;

    /*************** Load configuration ***************/
    std::string config_filename = "";
    if (!parser.has("config")) {
        std::cout << "Config file not provided. Cannot proceed. Exiting..." << std::endl;
        return 1;
    } else {
        config_filename = parser.get("config");
    }
    std::string abs_config_path = "/home/xterra/RIMaR/src/hardware/config/";
    std::string full_path = abs_config_path + config_filename;
    std::cout << "Loading YAML config file: " << full_path << std::endl;

    YAML::Node config = YAML::LoadFile(full_path);

    if (config["sleep_pos_joint"].IsSequence() &&
        config["sleep_pos_joint"].size() == 3) {
        for (int i = 0; i < 3; ++i) {
            joint_command_data.q(i) = config["sleep_pos_joint"][i].as<double>();
        }
    } else {
        joint_command_data.q << 0.001, -0.1, 0.1;
    }
    // std::cout << "Initial joint positions set to: " << joint_command_data.q.transpose() << std::endl;

    vec3 gear_ratio = vec3::Zero();
    if (config["gear_ratio"].IsSequence() && config["gear_ratio"].size() == 3) {
        for (int i = 0; i < 3; ++i) {
            gear_ratio(i) = config["gear_ratio"][i].as<double>();
        }
    } else {
        gear_ratio << 8, 8, 16;
    }
    // std::cout << "Gear ratios: " << gear_ratio.transpose() << std::endl;

    vec3 pos_min = vec3::Zero();
    if (config["pos_min"].IsSequence() && config["pos_min"].size() == 3) {
        for (int i = 0; i < 3; ++i) {
            pos_min(i) = config["pos_min"][i].as<double>();
        }
    } else {
        pos_min << -2, 0.001, 0.025;
    }
    // std::cout << "Position minimum limits: " << pos_min.transpose() << std::endl;

    vec3 pos_max = vec3::Zero();
    if (config["pos_max"].IsSequence() && config["pos_max"].size() == 3) {
        for (int i = 0; i < 3; ++i) {
            pos_max(i) = config["pos_max"][i].as<double>();
        }
    } else {
        pos_max << 2, 4.51, 10.10;
    }
    // std::cout << "Position maximum limits: " << pos_max.transpose() << std::endl;

    int3 jdir = int3::Ones();
    if (config["joint_dir"].IsSequence() && config["joint_dir"].size() == 3) {
        for (int i = 0; i < 3; ++i) {
            jdir(i) = config["joint_dir"][i].as<int>();
        }
    } else {
        jdir << -1, -1, 1;
    }
    // std::cout << "Joint direction multipliers: " << jdir.transpose() << std::endl;

    int3 remap_index = int3::Zero();
    if (config["remap_index"].IsSequence() && config["remap_index"].size() == 3) {
        for (int i = 0; i < 3; ++i) {
            remap_index(i) = config["remap_index"][i].as<int>();
        }
    } else {
        remap_index << 0, 1, 2;
    }
    // std::cout << "Remap indices: " << remap_index.transpose() << std::endl;

    float max_torque = 0.0;
    if (config["max_torque"]) {
        max_torque = config["max_torque"].as<double>();
    } else {
        max_torque = 0.01;
    }
    // std::cout << "Max torque set to: " << max_torque << std::endl;

    float max_velocity = 1;
    if (config["max_velocity"]) {
        max_velocity = config["max_velocity"].as<double>();
    } else {
        max_velocity = 1.0;
    }
    // std::cout << "Max velocity set to: " << max_velocity << std::endl;

    float hfe_kfe_trans = 0.5;
    if (config["hfe_kfe_trans"]) {
        hfe_kfe_trans = config["hfe_kfe_trans"].as<double>();
    }
    // std::cout << "HFE-KFE transmission factor: " << hfe_kfe_trans << std::endl;

    Eigen::VectorXd kp_scale = 4 * Eigen::VectorXd::Ones(3);
    Eigen::VectorXd kd_scale = 0.5 * Eigen::VectorXd::Ones(3);

    float t_curr = 0, t_end = 0;
    u_int delay_ms = 2;
    float tau_fac = 1.0;

    m_startTimePoint = std::chrono::high_resolution_clock::now();

    signal(SIGINT, signalHandler);
    // std::cout << "Signal handler for SIGINT registered." << std::endl;

    while (running) {
        t_curr = updateTimer();

        comm_data_ptr->getCommandData(joint_command_data);

        // std::cout << "Joint command data received: "
        //           << "q = " << joint_command_data.q.transpose() << ", "
        //           << "qd = " << joint_command_data.qd.transpose() << ", "
        //           << "tau = " << joint_command_data.tau.transpose() << ", "
        //           << "kp = " << joint_command_data.kp.transpose() << ", "
        //           << "kd = " << joint_command_data.kd.transpose() << std::endl;

        for (int i = 0; i < 3; i++) {
            if ((i + 1) % 3 == 0) {
                command_ptr->ref_position[i] = saturate(
                    (gear_ratio(i) / (2 * M_PI)) *
                        (jdir(i) * joint_command_data.q(remap_index(i)) -
                         jdir(i - 1) * hfe_kfe_trans *
                             joint_command_data.q(remap_index(i - 1))),
                    pos_min[i], pos_max[i]);
                command_ptr->ref_velocity[i] = saturate(
                    (gear_ratio(i) / (2 * M_PI)) *
                        (jdir(i) * joint_command_data.qd(remap_index(i)) -
                         jdir(i - 1) * hfe_kfe_trans *
                             joint_command_data.qd(remap_index(i - 1))),
                    -max_velocity, max_velocity);
                command_ptr->ref_ff_torque[i] =
                    saturate(tau_fac * (1 / gear_ratio(i)) * jdir(i) *
                                 joint_command_data.tau(remap_index(i)),
                             -max_torque, max_torque);

            } else {
                command_ptr->ref_position[i] =
                    saturate((gear_ratio(i) / (2 * M_PI)) * jdir(i) *
                                 joint_command_data.q(remap_index(i)),
                             pos_min[i], pos_max[i]);
                command_ptr->ref_velocity[i] =
                    saturate((gear_ratio(i) / (2 * M_PI)) * jdir(i) *
                                 joint_command_data.qd(remap_index(i)),
                             -max_velocity, max_velocity);
                command_ptr->ref_ff_torque[i] =
                    saturate(tau_fac * (1 / gear_ratio(i)) * jdir(i) *
                                 joint_command_data.tau(remap_index(i)),
                             -max_torque, max_torque);
            }

            if (joint_command_data.kp(remap_index(i)) != 0) {
                 command_ptr->kp_scale[i] =
                     0.25 * joint_command_data.kp(remap_index(i)) * (2 * M_PI) / (gear_ratio(i) * gear_ratio(i));
            } else {
                command_ptr->kp_scale[i] = 0.25 * kp_scale(remap_index(i));
            }

            if (joint_command_data.kd(remap_index(i)) != 0) {
                 command_ptr->kd_scale[i] =
                     20 * joint_command_data.kd(remap_index(i)) * (2 * M_PI) / (gear_ratio(i) * gear_ratio(i));
            } else {
                command_ptr->kd_scale[i] = 20 * kd_scale(remap_index(i));
            }
        }

        command_ptr->max_torque = max_torque;

        {
            std::lock_guard<std::mutex> lock(comm_data_ptr->get_sensor_data_mutex());

            for (int i = 0; i < 3; ++i) {
                if ((i + 1) % 3 == 0) {
                    sensor_data.q(remap_index(i)) =
                        ((2 * M_PI) / gear_ratio(i)) * jdir(i) *
                        (command_ptr->act_position[i] +
                         2 * hfe_kfe_trans * command_ptr->act_position[i - 1]);
                    sensor_data.qd(remap_index(i)) =
                        ((2 * M_PI) / gear_ratio(i)) * jdir(i) *
                        (command_ptr->act_velocity[i] +
                         2 * hfe_kfe_trans * command_ptr->act_velocity[i - 1]);
                    sensor_data.tau(remap_index(i)) =
                        gear_ratio(i) * jdir(i) * command_ptr->act_ff_torque[i];
                } else {
                    sensor_data.q(remap_index(i)) =
                        ((2 * M_PI) / gear_ratio(i)) * jdir(i) *
                        command_ptr->act_position[i];
                    sensor_data.qd(remap_index(i)) =
                        ((2 * M_PI) / gear_ratio(i)) * jdir(i) *
                        command_ptr->act_velocity[i];
                    sensor_data.tau(remap_index(i)) =
                        gear_ratio(i) * jdir(i) * command_ptr->act_ff_torque[i];
                }                   

                // std::cout << "Sensor data for joint " << i << ": "
                //           << "q = " << sensor_data.q(remap_index(i)) << ", "
                //           << "qd = " << sensor_data.qd(remap_index(i)) << ", "
                //           << "tau = " << sensor_data.tau(remap_index(i)) << std::endl;
            }
        }

        comm_data_ptr->writeSensorData(sensor_data);

        t_end = updateTimer();

        int delay_time = delay_ms - static_cast<int>((t_end - t_curr) * 1e3);
        if (delay_time > 0) {
            // std::cout << "Sleeping for " << delay_time << " ms to maintain loop timing." << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(delay_time));
            wait_us(delay_time * 1e3);
        }
    }

    // std::cout << "Detaching shared memory and shutting down..." << std::endl;
    shmdt(command_ptr);

#if defined(USE_ROS2_COMM)
    rclcpp::shutdown();
#elif defined(USE_ROS_COMM)
    ros::shutdown();
#endif
}
