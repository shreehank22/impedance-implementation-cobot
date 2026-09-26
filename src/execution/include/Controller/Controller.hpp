#pragma once

#include "kinodynamics.hpp"
#include <string>

#include "memory_types.hpp"
#include "utils.hpp"
#include "timer.hpp"
#include "LowPassFilter.hpp"

// #include "Controller/Solver.hpp"

// #include <qpOASES.hpp>

// using namespace qpOASES;

#define MU 0.6
#define GRAVITY 9.81

static const double NEGATIVE_NUMBER = -1000000.0;
static const double POSITIVE_NUMBER = 1000000.0;

enum class ControlMode {
    POSITION,
    TORQUE
};

class Controller {
public:
    // Controller();

    Controller(const YAML::Node&);

    virtual ~Controller();

    virtual void step(const double& dt);
    virtual void step(const double& dt, const double& t_curr);

    void setRobot(const std::shared_ptr<Cobot>& robot) {
        m_robot = robot;
        initClass();
    }

    void setEstimationDataPtr(CobotEstimationData* est_data_ptr) {
        m_estimation_data_ptr = est_data_ptr;
    }
    
    void setPlannerDataPtr(CobotPlannerData* planner_data_ptr) {
        m_planner_data_ptr = planner_data_ptr;
    }
    
    void setJointCommandDataPtr(CobotCommandData* cmd_data_ptr) {
        m_joint_command_ptr = cmd_data_ptr;
    }
    
    // void setMeasurementDataPtr(CobotMeasurementData* md) {
    //     m_measurement_data_ptr = md;
    // }

    // void updateContactFlagForController();

    void startFromSleep(const bool& sleep_start) {
        m_starting_from_sleep = sleep_start;
        (void)sleep_start;
        initClass();
    }

    void setInitStates(const vec6& x_init, const vec6& js_init);

    virtual vec6 calculateFeedForwardTorque() {
        return vec6::Zero();
    }

    void setStanceJointGains();

    // void setTorqueControlJointGains();

    void setSleepJointGains();

    void setZeroJointVelocity();

    void setZeroTorqueFF();

    void setPositionControlMode();
    void setTorqueControlMode();
    vec6 computeCartesianImpedanceTorque();


    // void setTorqueControlMode();

    bool checkJointCommandSafety();

    bool getSafetyTriggerFlag();

    vec6 getReferenceTorques();

    std::shared_ptr<Cobot> m_robot;

    vec6 m_joint_state_ref, m_ee_state_init, m_joint_state_init;
    vec6 m_ee_state_ref, m_joint_state_act;
    vec6 m_ee_vel_ref, m_ee_acc_ref, m_joint_vel_act, m_joint_acc_act;
    vec6 m_joint_vel_ref;
    vec6 m_ff_torque, m_kp, m_kd;
    vec6 m_cart_k = vec6::Zero();
    vec6 m_cart_d = vec6::Zero();
    vec6 m_tau_limit = vec6::Zero();
    vec6 m_null_kp = vec6::Zero();
    vec6 m_null_kd = vec6::Zero();
    vec6 m_ee_state_act = vec6::Zero();
    vec6 m_ee_vel_act = vec6::Zero();
    bool m_use_cartesian_impedance = false;
    bool isCartesianImpedanceEnabled() const { return m_use_cartesian_impedance; }


   
    vec6 m_state_error;
protected:
    YAML::Node m_config;
    int m_mode = 0;

    vec6 b_max;

    double m_dt = 0.001;
    double m_t_curr = 0;

    float m_loop_rate = 1000;

    // To write
    CobotCommandData* m_joint_command_ptr;
    
    // To read
    CobotEstimationData* m_estimation_data_ptr;
    CobotPlannerData* m_planner_data_ptr;
    // CobotMeasurementData* m_measurement_data_ptr;

    void resize_qpOASES_vars();

    void resize_eigen_vars();
    
    void update_problem_size();

    Eigen::VectorXd clipVector(const Eigen::VectorXd &b, float F);

    bool has_child_thread = false;

    std::unique_ptr<Filter> m_tauFF_filter[12];

    bool is_first_run = true;

    void updateEstimationData();
    
    void updatePlannerData();
    
    void updateJointCommand();

    bool m_trigger_safety = false;

    bool m_use_position_control = true;

    bool m_use_torque_control = false;

    bool m_use_foot_impedance = false;

    int m_solver_type = 0; // 0: GPGD, 1: qpOASES
     
private:
    std::string m_name;

    void initClass();
    
    // if starting from stance then set to false
    bool m_starting_from_sleep = true;
    
};
