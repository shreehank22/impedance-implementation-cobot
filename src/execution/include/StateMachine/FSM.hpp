#pragma once

#include "StateMachine/FSMState.hpp"
#include "StateMachine/StateCobotP2P.hpp"
#include "StateMachine/StateCobotSleep.hpp"
#include "StateMachine/StateCobotTeleop.hpp"

#include "cpputils.hpp"

struct FSMStateList
{
    FSMState *invalid;
    std::shared_ptr<StateCobotSleep> cobot_sleep;
    std::shared_ptr<StateCobotP2P> cobot_p2p;
    std::shared_ptr<StateCobotTeleop> cobot_teleop;

    FSMStateList() : invalid(nullptr) {}
};

class FSM
{
public:
    FSM(std::shared_ptr<RobotComponents>, const YAML::Node&);
    ~FSM();
    void initiate();
    void run(const bool& terminate);

private:
    void stepFSM();
    void ms_wait();
    void us_wait();
    void updateTime();
    std::shared_ptr<FSMState> getNextState(FSMStateName stateName);

    YAML::Node m_config;
    std::shared_ptr<RobotComponents> m_robotComp;
    std::shared_ptr<FSMState> m_currentState;
    std::shared_ptr<FSMState> m_nextState;
    FSMStateName m_nextStateName;
    FSMStateList m_stateList;
    FSMMode m_mode;

    std::chrono::microseconds m_period;
    std::chrono::high_resolution_clock::time_point m_next_wake_time;
    double delay_ms = 1;
    double t_curr = 0;
    double t_last = 0;
    double m_dt = 0;
    double t_switch = 0;

    bool use_us_wait = true;
};