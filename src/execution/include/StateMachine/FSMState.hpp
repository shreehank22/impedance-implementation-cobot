#pragma once

#include "utils.hpp"
#include "memory_types.hpp"
#include "StateMachine/RobotComponents.hpp"

enum class FSMStateName {
    INVALID,
    COBOTP2P,
    COBOTSLEEP,
    COBOTTELEOP,
};

enum class FSMMode {
    NORMAL,
    TRANSITION
};

class FSMState {
public:
    FSMState(std::shared_ptr<RobotComponents> components, FSMStateName stateName, std::string stateNameString, const YAML::Node&);

    virtual ~FSMState() = default;

    virtual void enter() = 0;
    virtual uint8_t run() = 0;
    virtual void exit() = 0;
    virtual FSMStateName checkChange() {return FSMStateName::INVALID;}

    void setTimeOfEntry(const double& t_entry) {m_timeOfEntry = t_entry;}

    FSMStateName m_stateName;
    std::string m_stateNameString;
protected:
    std::shared_ptr<RobotComponents> m_components;
    double m_timeOfEntry = 0.0;
    YAML::Node m_config;
    bool m_completed = false;
    bool m_is_orientation_safe = true;
    bool m_is_exit_safe = true;

    // m_safety_mode is set to true when the robot has auto-transitioned to 'fixed_stand' or 'sleep' due to safety violation, 
    // if true the robot cannot be transitioned to other states immediately after the safety state ('fixed_stand 'or 'sleep'),
    // the robot has to be set upright (in both cases) and commanded back into 'sleep' mode first (if the safety state is 'fixed_stand')
    bool m_safety_mode = false;
};