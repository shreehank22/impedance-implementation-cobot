#pragma once

#include "CommunicationManager.hpp"

#include <thread>
#include <chrono>
#include <string>

#include <dds/dds.hpp>

#include "dds_publisher.hpp"
#include "dds_subscriber.hpp"

#include "FloatScalar.hpp"
#include "JointData.hpp"
#include "SensorData.hpp"
#include "CliData.hpp"
// #include "QuadLog.hpp"
#include "JoyData.hpp"
#include "CliData.hpp"
#include "SolverStats.hpp"
// #include "PointCloud2.hpp"
// #include "Point.hpp"

// #include "pcl/point_cloud.h"
// #include "pcl/point_types.h"

#include "timer.hpp"

#define DOMAIN_ID 0

using namespace org::eclipse::cyclonedds;
using namespace xterra::msg::dds_;
// using namespace sensor_msgs::msg::dds_;

class QuadDDSComm : public CommunicationManager {
public:
    QuadDDSComm(const DATA_ACCESS_MODE&);
    QuadDDSComm(const std::string&, const DATA_ACCESS_MODE&);
    QuadDDSComm(const std::string& name, const std::string& postfix, const DATA_ACCESS_MODE& mode);
    ~QuadDDSComm();

    void setUpdateRate(const float& rate) {
        m_loop_rate = rate;
        m_dt = 1./rate;
    }

    void step();
    void run();
    void start_thread();

    void initClass();
private:
    std::string m_name;
    float m_loop_rate = 0;
    float m_dt = 0.001;

    // ACCESS TYPE PLANT
    
    std::shared_ptr<DDSSubscriber<JointData_>> m_joint_cmd_sub_ptr = NULL;
    std::shared_ptr<DDSPublisher<SensorData_>> m_sensor_data_pub_ptr = NULL;

    std::shared_ptr<DDSPublisher<FloatScalar_>> m_plant_time_pub_ptr = NULL;
    
    // ACCESS TYPE EXECUTOR

    std::shared_ptr<DDSPublisher<JointData_>> m_joint_cmd_pub_ptr = NULL;
    std::shared_ptr<DDSSubscriber<SensorData_>> m_sensor_data_sub_ptr = NULL;
    std::shared_ptr<DDSSubscriber<JoyData_>> m_joy_data_sub_ptr = NULL;
    std::shared_ptr<DDSSubscriber<CliData_>> m_cli_data_sub_ptr = nullptr;
    // std::shared_ptr<DDSSubscriber<Point_>> m_obstacle_coords_sub_ptr = NULL;

    std::shared_ptr<DDSSubscriber<FloatScalar_>> m_plant_time_sub_ptr = NULL;

    // Log data publishers
    // std::shared_ptr<DDSPublisher<QuadLog_>> m_estimated_data_pub_ptr = NULL;
    // std::shared_ptr<DDSPublisher<QuadLog_>> m_reference_data_pub_ptr = NULL;
    // std::shared_ptr<DDSPublisher<QuadLog_>> m_ground_truth_data_pub_ptr = NULL;
    // std::shared_ptr<DDSPublisher<Point_>> m_base_err_pub_ptr = NULL;
    std::shared_ptr<DDSPublisher<SolverStats_>> m_solver_stats_pub_ptr = NULL;

    void write_joint_command();
    void write_sensor_data();

    // void write_measurement_data();
    // void write_sim_measurement_data();

    void get_sensor_data_cb(const SensorData_& msg);
    void get_joint_cmd_cb(const JointData_& msg);
    void get_cli_data_cb(const CliData_& msg); 

    void get_joystick_data_cb(const JoyData_& msg);
    // void get_obstacle_coords_cb(const Point_& msg);
    void get_plant_time_cb(const FloatScalar_& time);

    std::thread m_thread;

    bool use_bridge = false;
    bool publish_measurements = true;
    bool convert_dps_to_rps = false;

    double m_plant_time = 0;
    FloatScalar_ plant_time;


    // Data holders
    SensorData_ m_sensor_data_msg;
    JointData_ m_joint_command_msg;
    CliData_ m_cli_data_msg;
    JoyData_ m_joy_data_msg;
    // QuadLog_ m_estimated_data, m_reference_data, m_ground_truth_data;
    // Point_ m_base_err;
    SolverStats_ m_solver_stats;
    // xRockerBtnDataStruct _keyData;

    // PointCloud2_ lidar_point_cloud;
    // Point_ obstacle_coords;
    // pcl::PointCloud<pcl::PointXYZ> pcl_cloud;

    std::chrono::time_point<std::chrono::high_resolution_clock> m_startTimePoint;
    double prev_time = 0, curr_time = 0;
};