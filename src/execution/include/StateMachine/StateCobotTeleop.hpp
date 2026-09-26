#pragma once

#include "StateMachine/FSMState.hpp"

class StateCobotTeleop : public FSMState {
   public:

    StateCobotTeleop(std::shared_ptr<RobotComponents>, const YAML::Node&);

    ~StateCobotTeleop() {};

    void enter();
    uint8_t run();
    void exit();
    FSMStateName checkChange();

   private:
    bool m_changeRequested = false;

    void getUserCmd();
    bool checkSafeExit();
    bool checkSafeEntry();
    uint8_t checkSafeRun();
    bool checkIfNearHome();

    // cobot joint commands
    vec6 m_joint_positions_cmd, m_joint_velocities_cmd, m_joint_positions_act, m_joint_velocities_act, m_joint_home_position;
    vec3 m_joint_kp, m_joint_kd;
    int m_prev_joystick_mode;

    // teleop related variables
    vec6 m_joyVelCmd = vec6::Zero();  // ee vel in the body frame of the robot
};