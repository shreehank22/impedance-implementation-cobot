#include "StateMachine/StateCobotTeleop.hpp"

StateCobotTeleop::StateCobotTeleop(std::shared_ptr<RobotComponents> components,
                               const YAML::Node& config)
    : FSMState(std::move(components), FSMStateName::COBOTTELEOP, "cobot_teleop",
               config)
               , m_prev_joystick_mode(0) {}

bool StateCobotTeleop::checkIfNearHome() {
    m_joint_positions_act = m_components->getEstimator()->getJointPositions();

    if ((m_joint_positions_act.block<3,1>(0,0) - m_joint_home_position.block<3,1>(0,0)).norm() > 0.1)
        return false;
    else
        return true;
}

void StateCobotTeleop::enter() {
    if (m_components->getController()->isCartesianImpedanceEnabled()) {
        m_components->getController()->setTorqueControlMode();
    } else {
        m_components->getController()->setPositionControlMode();
    }
    m_components->getPlanner()->setJointSpacePlanning(false);

    m_prev_joystick_mode = 0;
    m_changeRequested = false;

    m_joint_home_position = m_components->getEstimator()->getJointPositions();
    std::cout << "Cobot joint home position set: " << m_joint_home_position.transpose() << std::endl;
}

uint8_t StateCobotTeleop::run() {
    getUserCmd();
    m_components->getPlanner()->setDesiredEEVelocity(m_joyVelCmd);

    return checkSafeRun();
}

void StateCobotTeleop::exit() { 
    m_components->getController()->setPositionControlMode();
    m_components->getPlanner()->setJointSpacePlanning(true);
    m_changeRequested = false;
}

bool StateCobotTeleop::checkSafeEntry() { return true; }

uint8_t StateCobotTeleop::checkSafeRun() {
    if (m_components->getController()->getSafetyTriggerFlag()) {
        std::cerr << "[COBOTTELEOP] NaN in the joint command data." << std::endl;
        return 1;
    }

    return 0;
}

bool StateCobotTeleop::checkSafeExit(){ return true; }

FSMStateName StateCobotTeleop::checkChange() {
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
        std::cerr << "[COBOTTELEOP] Safety check failed due to NaN or Inf. Switching "
                     "to sleep.\n";
        return FSMStateName::COBOTSLEEP;
    } else {
        m_changeRequested = false;
        return FSMStateName::COBOTTELEOP;
    }
}

void StateCobotTeleop::getUserCmd() {
    int current_mode = m_components->getJoystickData().mode;

    m_joyVelCmd(0) = m_components->getJoystickData().left_stick_y;  // EE forward/backward
    m_joyVelCmd(1) = m_components->getJoystickData().left_stick_x;  // EE up/down
    m_joyVelCmd(2) = m_components->getJoystickData().right_stick_y; // wrist pitch
    m_joyVelCmd(3) = m_components->getJoystickData().right_stick_x; // wrist roll
    m_joyVelCmd(4) = m_components->getJoystickData().left_trigger;  // base yaw (Q+/E-)
    m_joyVelCmd(5) = m_components->getJoystickData().dpad_x;        // jaw open/close

    m_prev_joystick_mode = current_mode;
}
