#include "StateMachine/FSMState.hpp"

FSMState::FSMState(std::shared_ptr<RobotComponents> components, FSMStateName stateName, std::string stateNameString, const YAML::Node& config)
        : m_components(std::move(components)), m_stateName(stateName), m_stateNameString(stateNameString), m_config(config) 
        {}
    