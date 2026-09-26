#pragma once

#include "StateMachine/FSMState.hpp"

class StateCobotP2P: public FSMState {
public:
    StateCobotP2P(std::shared_ptr<RobotComponents>, const YAML::Node&);

    ~StateCobotP2P() {};

    void enter();
    uint8_t run();
    void exit();
    FSMStateName checkChange();

private:

    bool m_changeRequested = false;

    vec6 m_poseCmd = vec6(0, 0, 0, 0, 0, 0);

    vec6 m_joint_positions_act, m_joint_home_position;

    void getUserCmd();
    bool checkIfNearHome();
    bool checkSafeEntry();
    uint8_t checkSafeRun();
};