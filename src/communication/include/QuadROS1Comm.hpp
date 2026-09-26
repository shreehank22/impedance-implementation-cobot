#pragma once

#include "CommunicationManager.hpp"

#include <functional>
#include <memory>
#include <chrono>
#include <thread>

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/Imu.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/Joy.h"
#include "xterra/JointData.h"
#include "xterra/SensorData.h"
#include "xterra/QuadLog.h"

using namespace std::chrono_literals;
// using std::placeholders::_1;

class QuadROS1Comm : public CommunicationManager {
public:
    QuadROS1Comm(const DATA_ACCESS_MODE& mode);
    QuadROS1Comm(const std::string& name, const DATA_ACCESS_MODE& mode);
    ~QuadROS1Comm();

    void setUpdateRate(const float& rate) {
        m_loop_rate = rate;
        m_dt = 1./rate;
    }

    void run();
    void start_thread();

    void initClass();
private:
    std::string m_name;
    float m_loop_rate = 1000;
    float m_dt = 0.001;

    ros::NodeHandle m_nh;

    // Plant pub-sub
    ros::Subscriber m_joint_cmd_sub;
    ros::Publisher m_sensor_data_pub;
    ros::Publisher m_ground_truth_data_pub;
    // ros::Publisher m_joy_data_pub;

    // Executor pub-sub
    ros::Subscriber m_sensor_data_sub;
    ros::Publisher m_joint_cmd_pub;
    ros::Subscriber m_joy_data_sub;

    // Debug data publishers
    ros::Publisher m_estimated_data_pub;
    ros::Publisher m_reference_data_pub;

    // ros::Publisher m_joint_state_pub;
    ros::Publisher m_imu_pub;
    ros::Publisher m_odom_pub;

    // Timer
    ros::WallTimer m_timer;

    // Subscription callbacks
    void get_joint_cmd_cb(const xterra::JointData::ConstPtr& msg);
    void get_sensor_data_cb(const xterra::SensorData::ConstPtr& msg);
    void get_joystick_data_cb(const sensor_msgs::Joy::ConstPtr& msg);
    
    // Timer callback
    void timer_cb(const ros::WallTimerEvent& timer_event);

    xterra::JointData m_joint_data;
    xterra::SensorData m_sensor_data;
    xterra::QuadLog m_estimated_data;
    xterra::QuadLog m_reference_data;
    xterra::QuadLog m_ground_truth_data;

    void write_joint_command();
    void write_sensor_data();
    void write_measurement_data();
    void write_sim_measurement_data();

    sensor_msgs::Joy joystick_data;

    double m_plant_time = 0;
    
    std_msgs::Float32 plant_time;
    ros::Publisher m_plant_time_pub;
    ros::Subscriber m_plant_time_sub;

    void get_plant_time_cb(const std_msgs::Float32::ConstPtr& msg);

    std::thread m_thread;
};