#pragma once

#include "kinodynamics.hpp"
#include <string>
#include "memory_types.hpp"
#include "RateLimitedPrinter.hpp"
#include "Iir.h"

#include <yaml-cpp/yaml.h>
#include "Filter.hpp"

class Estimator {
public:
    // Estimator();
    Estimator(const YAML::Node&);
    ~Estimator() {}

    void setRobot(const std::shared_ptr<Cobot>& robot) {
        m_robot = robot;
        initClass();
    }

    virtual void computeEstimate();

    void setSensorDataPtr(CobotSensorData* sd) {
        m_sensor_data_ptr = sd;
    }
    
    void setEstimationDataPtr(CobotEstimationData* esd) {
        m_estimation_data_ptr = esd;
    }
    
    void setPlannerDataPtr(CobotPlannerData* pd) {
        m_planner_data_ptr = pd;
    }
    
    // void setMeasurementDataPtr(CobotMeasurementData* md) {
    //     m_measurement_data_ptr = md;
    // }
    
    void setLoopRate(const float& rate) {
        m_loop_rate = rate;
        m_dt = 1./rate;
        initClass();
    }

    void step(const double& dt);

    void reset();

    void startCalibration() {
        // reset();
        m_calibration = true;
    }
    
    void endCalibration() {
        m_calibration = false;
    }
    
    void getInitParams();


    vec6 getEndEffectorPosition();
    vec6 getEndEffectorVelocity();
    vec6 getEndEffectorForces();

    vec6 getJointPositions();
    vec6 getJointVelocities();
    vec6 getJointTorques();

    // int4 getEstimatedPickState();

protected:
    std::shared_ptr<Cobot> m_robot;

    YAML::Node m_config;

    // To write
    CobotEstimationData est_data;
    // To read
    CobotSensorData sensor_data;
    CobotSensorData* m_sensor_data_ptr;
    CobotEstimationData* m_estimation_data_ptr;
    CobotPlannerData* m_planner_data_ptr;
    // CobotMeasurementData* m_measurement_data_ptr;


    double m_dt;
    float m_loop_rate = 0;

    bool m_calibration = false;

    RateLimitedPrinter* printer;
private:
    std::string m_name;

    void updateSensorData();
    void updateEstimationData();
    // void updateMeasurementData();

    void initClass();

    Iir::Butterworth::LowPass<2> m_q_filter[6];
    Iir::Butterworth::LowPass<2> m_qd_filter[6];
    Iir::Butterworth::LowPass<2> m_tau_filter[6];


    double m_cutoff_freq_encoder_pos = 0;
    double m_cutoff_freq_encoder_vel = 0;
    double m_cutoff_freq_encoder_tau = 0;
};