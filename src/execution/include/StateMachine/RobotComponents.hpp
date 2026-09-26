#pragma once

#include "Estimator/Estimator.hpp"
#include "Planner/Planner.hpp"
#include "Controller/Controller.hpp"
#include <iostream>
#include "cpputils.hpp"

#if defined(USE_ROS2_COMM)
#include "QuadROSComm.hpp"
#elif defined(USE_ROS_COMM)
#include "QuadROS1Comm.hpp"
#elif defined(USE_DDS_COMM)
#include "QuadDDSComm.hpp"
#else
#include "SHM.hpp"
#endif

#include <typeinfo>

struct RobotComponents
{
public:
    RobotComponents(
        const std::string &name,
        const std::string &comm_postfix,
        std::shared_ptr<Cobot> robot_ptr,
        std::shared_ptr<Estimator> estimator_ptr,
        std::shared_ptr<Planner> planner_ptr,
        std::shared_ptr<Controller> controller_ptr,
        double pec_rate
    )
    : m_name(name),
      m_comm_postfix(comm_postfix),
      m_robot(robot_ptr),
      m_estimator(estimator_ptr),
      m_planner(planner_ptr),
      m_controller(controller_ptr)
    {
        setUpdateRate(pec_rate);
        m_startTimePoint = std::chrono::high_resolution_clock::now();


#if defined(USE_ROS2_COMM)
        m_plant_data_ptr = std::make_shared<QuadROSComm>(m_name + m_comm_postfix, DATA_ACCESS_MODE::EXECUTOR);
#elif defined(USE_ROS_COMM)
        m_plant_data_ptr = std::make_shared<QuadROS1Comm>(m_name + m_comm_postfix, DATA_ACCESS_MODE::EXECUTOR);
#elif defined(USE_DDS_COMM)
        m_plant_data_ptr = std::make_shared<QuadDDSComm>(m_name, m_comm_postfix, DATA_ACCESS_MODE::EXECUTOR);
#else
        m_plant_data_ptr = std::make_shared<SHM>(m_name + m_comm_postfix, DATA_ACCESS_MODE::EXECUTOR);
#endif

        if (m_plant_data_ptr)
        {
            m_plant_data_ptr->setSensorDataPtr(&m_sensor_data);
            m_plant_data_ptr->setEstimationDataPtr(&m_est_data);
            m_plant_data_ptr->setPlannerDataPtr(&m_planner_data);
            m_plant_data_ptr->setCommandDataPtr(&m_cmd_data);
            m_plant_data_ptr->setCliDataPtr(&m_cli_data);
            m_plant_data_ptr->setJoystickDataPtr(&m_joystick_data);
            m_plant_data_ptr->setUpdateRate(500);
            m_plant_data_ptr->setExecutorReady(false);
            m_plant_data_ptr->start_thread();
        }

#if defined(USE_DDS_COMM)
        std::cout << "Waiting for DDS communication manager to be ready...\n";
        while (!m_plant_data_ptr->isReady() && get_wall_time_seconds(m_startTimePoint) < 5)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        while (get_wall_time_seconds(m_startTimePoint) < 1)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        std::cout << "DDS Communication manager ready...\n";
#endif

        m_estimator->setRobot(m_robot);
        m_estimator->setLoopRate(m_rate);
        m_estimator->setSensorDataPtr(&m_sensor_data);
        m_estimator->setEstimationDataPtr(&m_est_data);
        m_estimator->setPlannerDataPtr(&m_planner_data);

        m_planner->setRobot(m_robot);
        m_planner->startFromSleep(true);
        m_planner->setPlannerDataPtr(&m_planner_data);
        m_planner->setEstimationDataPtr(&m_est_data);
        m_planner->setCliDataPtr(&m_cli_data);

        m_controller->setRobot(m_robot);
        m_controller->startFromSleep(true);
        m_controller->setEstimationDataPtr(&m_est_data);
        m_controller->setPlannerDataPtr(&m_planner_data);
        m_controller->setJointCommandDataPtr(&m_cmd_data);

    }

    ~RobotComponents() {}

    std::string get_name() const { return m_name; }

    std::string m_name;
    std::string m_comm_postfix;
    double m_dt = 0;
    double m_rate = 0;
    bool m_first_run = true;
    bool use_plant_time = false;

    std::chrono::time_point<std::chrono::high_resolution_clock> m_startTimePoint;


    void setUpdateRate(const double &rate)
    {
        m_dt = 1. / rate;
        m_rate = rate;
    }

    void runCalibration()
    {
        uint8_t calib_steps = 100;
        uint8_t counter = 0;

        m_estimator->startCalibration();

        while (counter++ <= calib_steps)
        {
            m_plant_data_ptr->getSensorData(m_sensor_data);
            m_estimator->step(m_dt);
        }

        m_estimator->endCalibration();

        vec6 js_init = m_est_data.js;
        js_init.setZero();
        m_planner_data.x.block<6, 1>(0, 0) = m_robot->forwardKinematics(js_init);

        m_planner->setInitStates(m_planner_data.x, js_init);
    }

    void step(const double &dt, const double &t_curr)
    {
        m_plant_data_ptr->getSensorData(m_sensor_data);

        m_estimator->step(dt);
        m_planner->step(dt, t_curr);
        m_controller->step(dt, t_curr);

        if (!m_plant_data_ptr->getExecutorReadinessStatus())
        {
            m_plant_data_ptr->setExecutorReady(true);
        }

        // Write joint command data
        m_plant_data_ptr->writeCommandData(m_cmd_data);

    }

    // Getters
    CobotSensorData &getSensorData() { return m_sensor_data; }
    CobotEstimationData &getEstimationData() { return m_est_data; }
    CobotPlannerData &getPlannerData() { return m_planner_data; }
    CobotCommandData &getCommandData() { return m_cmd_data; }
    CobotJoystickData &getJoystickData() { return m_joystick_data; }
    CobotCliData &getCliData() { return m_cli_data; }

    std::shared_ptr<CommunicationManager> getCommunicationManager() const
    {
        if (!m_plant_data_ptr)
            throw std::runtime_error("Communication manager pointer is null");
        return m_plant_data_ptr;
    }

    std::shared_ptr<Cobot> getCobot() const
    {
        if (!m_robot)
            throw std::runtime_error("Cobot pointer is null");
        return m_robot;
    }

    std::shared_ptr<Planner> getPlanner() const
    {
        if (!m_planner)
            throw std::runtime_error("Planner pointer is null");
        return m_planner;
    }

    std::shared_ptr<Estimator> getEstimator() const
    {
        if (!m_estimator)
            throw std::runtime_error("Estimator pointer is null");
        return m_estimator;
    }

    std::shared_ptr<Controller> getController() const
    {
        if (!m_controller)
            throw std::runtime_error("Controller pointer is null");
        return m_controller;
    }

private:

#if defined(USE_ROS2_COMM)
    std::shared_ptr<QuadROSComm> m_plant_data_ptr;
#elif defined(USE_ROS_COMM)
    std::shared_ptr<QuadROS1Comm> m_plant_data_ptr;
#elif defined(USE_DDS_COMM)
    std::shared_ptr<QuadDDSComm> m_plant_data_ptr;
#else
    std::shared_ptr<SHM> m_plant_data_ptr;
#endif
    // FileLogger& logger;

    std::shared_ptr<Cobot> m_robot;
    std::shared_ptr<Estimator> m_estimator;
    std::shared_ptr<Planner> m_planner;
    std::shared_ptr<Controller> m_controller;

    CobotSensorData m_sensor_data;
    CobotEstimationData m_est_data;
    CobotPlannerData m_planner_data;
    CobotCommandData m_cmd_data;
    CobotJoystickData m_joystick_data;
    CobotCliData m_cli_data;
};
