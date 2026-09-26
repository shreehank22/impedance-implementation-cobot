#pragma once

#include "kinodynamics.hpp"
#include <string>

#include "utils.hpp"
#include "memory_types.hpp"

class Planner {
public:
    // Planner();
    
    Planner(const YAML::Node&);
    
    virtual ~Planner();

    vec6 getReferenceEeStates();

    void setDesiredEEVelocity(const vec6& joy_ee_vel_cmd);

    virtual void setEndEffectorTarget();
    
    void setRobot(const std::shared_ptr<Cobot>& robot) {
        m_robot = robot;
        initClass();
    }

    void setPlannerDataPtr(CobotPlannerData* pd) {
        m_planner_data_ptr = pd;
    }

    void setEstimationDataPtr(CobotEstimationData* est_ptr) {
        m_estimation_data_ptr = est_ptr;
    }

    void setCliDataPtr(CobotCliData* cli_ptr) {
        m_cli_data_ptr = cli_ptr;
    }

    void setJointSpacePlanning(const bool& plan_js);

    bool setTargetJointAngles(const vec6&, const double&);

    void setInitStates(const vec6& x_init, const vec6& js_init);

    void setInitJointStates(const vec6& js_init);

    virtual void step(const double& dt, const double& t_curr);

    bool shouldPlanTrajectory(double threshold = 0.05); // Flag indicating whether planning is needed

    double m_t_curr = 0;

    void initClass();

    void reset();

    void startFromSleep(const bool& sleep_start);

    void resetTargetAccepted() { m_target_accepted = false; m_iter = 0; }

protected:
    std::shared_ptr<Cobot> m_robot;

    YAML::Node m_config;

    double m_v_cmd_x = 0.0;
    double m_v_cmd_y = 0.0;
    double m_v_cmd_z = 0.0;
    double m_v_cmd_phi = 0.0;
    double m_psi_cmd = 0.0;
    double m_theta_cmd = 0.0;

    double m_dt = 0.002;

    double m_base_yaw = 0;

    double m_base_yaw_vel = 0;

    double m_base_yaw_acc = 0;

    int m_mode = 0;

    int counter = 0;

    vec6 m_joint_state_act;
    vec6 m_joint_vel_act;
    vec6 m_target_joint_pose;
    vec6 m_joint_state_start;
    vec6 m_joint_vel_ref;
    vec6 m_target_ee_pose;

    vec6 m_ee_state_ref, m_ee_state_init, m_joint_state_ref, m_joint_state_init;
    vec6 m_ee_vel_ref, m_ee_acc_ref;

    double m_alpha = 0;
    double m_duration = 5.0;

private:
    void planJointTrajectoryLinear(const double &dt); 

    CobotPlannerData* m_planner_data_ptr;

    CobotEstimationData* m_estimation_data_ptr;

    CobotCliData* m_cli_data_ptr;

    CobotCliData cli_data;

    void updatePlannerData();

    void updateCliData();

    void updateEstimationData();

    bool m_sleep_start = false;

    bool m_plan_ee_space = false;

    std::string m_name;
    std::chrono::time_point<std::chrono::high_resolution_clock> m_startTimePoint;
    std::chrono::time_point<std::chrono::high_resolution_clock> m_currentTimePoint;
    
    int  m_iter = 0;
    int  m_num_steps = 1250;
    bool m_trajectory_reported = false;
    bool m_target_accepted     = false;
    
    double m_max_ee_vel_x         = 0.50;
    double m_max_ee_vel_z         = 0.50;
    double m_max_ee_vel_pitch     = 2.0;
    double m_max_ee_vel_roll      = 2.0;
    double m_max_ee_vel_base_yaw  = 5.0;
    double m_max_ee_vel_jaw       = 2.5;

    double m_max_ee_accel_x         = 5.0;
    double m_max_ee_accel_z         = 5.0;
    double m_max_ee_accel_pitch     = 10.0;
    double m_max_ee_accel_roll      = 10.0;
    double m_max_ee_accel_base_yaw  = 25.0;

    // Workspace limits — base frame (robot base at table surface z=0).
    //
    // x_min=0.22: l4 (wrist=0.20) + 2 cm. Below this, r_adj<0 → IK infeasible.
    // x_max=0.52: tested safe up to 0.55 but 3 cm margin for joint limits.
    // z_min=0.05: 5 cm above table. z=0.03 too close; unreliable tracking.
    // z_max=0.55: at x_max=0.52, arm reach limit is ~0.61 m; 0.55 gives margin.
    //             (x=0.55 z=0.65 fails IK — out of arm reach envelope.)
    // y: ±0.28 — all y_sweep tests passed at ±0.30; 2 cm margin each side.
    // dist_xy_min=0.22: same as x_min (prevents near-singularity at base).
    // dist_xy_max=0.52: matches x_max for diagonal reach consistency.
    double m_min_ee_x_global = 0.22;
    double m_max_ee_x_global = 0.52;
    double m_min_ee_y_global = -0.28;
    double m_max_ee_y_global =  0.28;
    double m_min_ee_z_global =  0.05;
    double m_max_ee_z_global =  0.55;
    double m_min_dist_ee_xy  =  0.22;
    double m_max_dist_ee_xy  =  0.52;
    double m_min_ee_jaw      = 0.0;
    double m_max_ee_jaw      = 1.5;
};
