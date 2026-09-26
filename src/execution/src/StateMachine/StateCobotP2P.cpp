#include "StateMachine/StateCobotP2P.hpp"

StateCobotP2P::StateCobotP2P(std::shared_ptr<RobotComponents> components, const YAML::Node& config)
    : FSMState(std::move(components), FSMStateName::COBOTP2P, "cobot_p2p", config) {}

bool StateCobotP2P::checkSafeEntry()
{
    return checkIfNearHome();
}

bool StateCobotP2P::checkIfNearHome() {
    m_joint_positions_act = m_components->getEstimator()->getJointPositions();

    if ((m_joint_positions_act.block<3,1>(0,0) - m_joint_home_position.block<3,1>(0,0)).norm() > 0.1)
        return false;
    else
        return true;
}

void StateCobotP2P::enter()
{
    if (m_components->getController()->isCartesianImpedanceEnabled()) {
        m_components->getController()->setTorqueControlMode();
    } else {
        m_components->getController()->setPositionControlMode();
    }
    m_components->getPlanner()->setJointSpacePlanning(false);

    m_changeRequested = false;

    m_joint_home_position = m_components->getEstimator()->getJointPositions();
    std::cout << "Cobot joint home position set: " << m_joint_home_position.transpose() << std::endl;
}

uint8_t StateCobotP2P::checkSafeRun()
{
    if (m_components->getController()->getSafetyTriggerFlag()) {
        std::cerr << "[COBOTP2P] NaN in the joint command data." << std::endl;
        return 1;
    }

    return 0;
}

uint8_t StateCobotP2P::run()
{
    return checkSafeRun();
}

void StateCobotP2P::exit()
{
    m_components->getController()->setPositionControlMode();
    m_components->getPlanner()->setJointSpacePlanning(true);
    m_components->getPlanner()->resetTargetAccepted();
    m_changeRequested = false;
}

FSMStateName StateCobotP2P::checkChange()
{
    int requested_mode = m_components->getCliData().mode;
    if (requested_mode == 0) {
        requested_mode = m_components->getJoystickData().mode;
    }

    if (requested_mode == 1 && !m_changeRequested) {
        m_changeRequested = true;
    }

    if (m_changeRequested) {
        return FSMStateName::COBOTSLEEP;
    } else if (checkSafeRun() == 1) {
        std::cerr << "[COBOTP2P] Safety check failed due to NaN or Inf. Switching "
                     "to sleep.\n";
        return FSMStateName::COBOTSLEEP;
    } else {
        m_changeRequested = false;
        return FSMStateName::COBOTP2P;
    }
}
