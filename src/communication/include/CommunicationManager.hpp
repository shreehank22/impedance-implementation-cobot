#pragma once

#include "memory_types.hpp"
#include "utils.hpp"
#include <string>
#include <iostream>
#include <mutex>

enum DATA_ACCESS_MODE {
    PLANT,
    EXECUTOR
};

// Single communication manager for Cobot
class CommunicationManager {
public:
    CommunicationManager();
    CommunicationManager(const DATA_ACCESS_MODE& mode);
    CommunicationManager(const std::string&, const DATA_ACCESS_MODE& mode);
    ~CommunicationManager() {}

    virtual void setPlannerDataPtr(CobotPlannerData* planner_data_ptr) {
        m_planner_data_ptr = planner_data_ptr;
    }
    virtual void setSensorDataPtr(CobotSensorData* sensor_data_ptr) {
        m_sensor_data_ptr = sensor_data_ptr;
    }
    virtual void setCommandDataPtr(CobotCommandData* command_data_ptr) {
        m_joint_command_data_ptr = command_data_ptr;
    }
    // virtual void setMeasurementDataPtr(CobotMeasurementData* measurement_data_ptr) {
    //     m_measurement_data_ptr = measurement_data_ptr;
    // }
    virtual void setEstimationDataPtr(CobotEstimationData* estimation_data_ptr) {
        m_estimation_data_ptr = estimation_data_ptr;
    }
    virtual void setJoystickDataPtr(CobotJoystickData* joystick_data_ptr) {
        m_joystick_data_ptr = joystick_data_ptr;
    }
    virtual void setCliDataPtr(CobotCliData* cli_data_ptr) {
        m_cli_data_ptr = cli_data_ptr;
    }
    virtual void setPlantTimePtr(double* time_ptr) {
        m_plant_time_ptr = time_ptr;
    }

    void writeSensorData(const CobotSensorData& sensor_data);
    void writeCommandData(const CobotCommandData& cmd_data);
    // void writeMeasurementData(const CobotMeasurementData& measure_data);
    void writeEstimationData(const CobotEstimationData& est_data);
    void writeCliData(const CobotCliData& cli_data);
    void writeJoystickData(const CobotJoystickData& joy_data);
    void writePlantTime(const double& time);
    
    void getSensorData(CobotSensorData& sensor_data);
    void getCommandData(CobotCommandData& cmd_data);
    // void getMeasurememtData(CobotMeasurementData& measure_data);
    void getEstimationData(CobotEstimationData& est_data);
    void getJoystickData(CobotJoystickData& joy_data);
    void getCliData(CobotCliData& cli_data);
    void getPlantTime(double& time);

    void setAccessMode(const DATA_ACCESS_MODE& mode) {
        m_mode = mode;
    }

    void stop();

    void setExecutorReady(const bool& flag = false) {
    	m_is_executor_ready = flag;
    }

    bool getExecutorReadinessStatus () const {
    	return m_is_executor_ready;
    }

    bool isReady() {
        return m_communication_ready;
    }

    std::mutex& get_sensor_data_mutex() { return m_sensor_data_mutex; }

    std::mutex& get_command_mutex() { return m_command_data_mutex; }

    double get_last_joystick_update_time() {
        return m_last_joystick_update_time;
    }

    void disable_joystick() { m_disable_joystick = true; }

    void enable_joystick() { m_disable_joystick = false; }

    bool is_joystick_disabled() { return m_disable_joystick; }

protected:
    CobotPlannerData* m_planner_data_ptr = NULL;
    CobotSensorData* m_sensor_data_ptr = NULL;
    CobotCommandData* m_joint_command_data_ptr = NULL;
    CobotEstimationData* m_estimation_data_ptr = NULL;
    CobotCliData* m_cli_data_ptr = NULL;
    // CobotMeasurementData* m_measurement_data_ptr = NULL;
    CobotJoystickData* m_joystick_data_ptr = NULL;
    double* m_plant_time_ptr = NULL;

    int m_mode = DATA_ACCESS_MODE::PLANT;
    bool terminated();
    bool m_is_executor_ready = false;
    
    std::mutex m_sensor_data_mutex;
    std::mutex m_command_data_mutex;

    bool m_communication_ready = false;
    double m_last_joystick_update_time = 0.0;
    bool m_is_joystick_alive = true;
    bool m_disable_joystick = false;

private:
    std::string m_name;
    bool m_shutdown_node = false;
};
