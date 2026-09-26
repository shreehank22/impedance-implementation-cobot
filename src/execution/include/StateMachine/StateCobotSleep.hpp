#pragma once

#include "StateMachine/FSMState.hpp"

class StateCobotSleep : public FSMState {
   public:
    StateCobotSleep(std::shared_ptr<RobotComponents>, const YAML::Node&);

    ~StateCobotSleep() {};

    void enter();
    uint8_t run();
    void exit();
    FSMStateName checkChange();

   private:
    vec6 m_cobot_positions_act, m_cobot_home_position, m_targetJointPosForSleep;

    double m_duration = 1000;
    double m_alpha = 0;

    bool checkIfNearHome();
    bool checkSafeEntry();
    bool checkSafeExit();
};