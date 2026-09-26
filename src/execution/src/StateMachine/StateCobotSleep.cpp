#include "StateMachine/StateCobotSleep.hpp"

StateCobotSleep::StateCobotSleep(std::shared_ptr<RobotComponents> components,
                       const YAML::Node& config)
    : FSMState(std::move(components), FSMStateName::COBOTSLEEP, "cobot_sleep", config) {
    if (m_config["q"].IsSequence() && m_config["q"].size() == 6) {
        for (int i = 0; i < m_config["q"].size(); ++i) {
            m_targetJointPosForSleep(i) = m_config["q"][i].as<double>();
        }
    }
}

void StateCobotSleep::enter() {
    m_components->getController()->setPositionControlMode();
    m_components->getPlanner()->setInitJointStates(
    m_components->getEstimator()->getJointPositions());
    m_components->getPlanner()->setJointSpacePlanning(true);
}

uint8_t StateCobotSleep::run() {
    m_completed = m_components->getPlanner()->setTargetJointAngles(
        m_targetJointPosForSleep, 1.0);

    if (m_completed) {
        m_components->getController()->setSleepJointGains();
    }

    return m_completed;
}

void StateCobotSleep::exit() { m_components->runCalibration(); }

bool StateCobotSleep::checkSafeEntry() { return true; }

bool StateCobotSleep::checkSafeExit() {
    if (!m_completed) {
        m_is_exit_safe = false;
        std::cerr << "[COBOTSLEEP] Robot not in sleep yet. Please wait until sleep "
                     "is completed."
                  << std::endl;
    } else {
        m_is_exit_safe =  m_completed;
    }

    return m_is_exit_safe;
}

bool StateCobotSleep::checkIfNearHome() {
    m_cobot_positions_act = m_components->getEstimator()->getJointPositions();

    if ((m_cobot_positions_act.block<3,1>(0,0) - m_cobot_home_position.block<3,1>(0,0)).norm() > 0.1)
        return false;
    else
        return true;
}

FSMStateName StateCobotSleep::checkChange() {
    int requested_mode = m_components->getCliData().mode;
    if (requested_mode == 0) {
        requested_mode = m_components->getJoystickData().mode;
    }

    if (requested_mode == 2 && checkSafeExit() &&
        !m_safety_mode) {
        return FSMStateName::COBOTTELEOP;
    } else if (requested_mode == 3 && checkSafeExit() &&
        !m_safety_mode) {
        return FSMStateName::COBOTP2P;
    } else {
        return FSMStateName::COBOTSLEEP;
    }
}
