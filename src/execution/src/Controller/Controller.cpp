#include "Controller/Controller.hpp"
namespace {
vec6 loadVec6Config(const YAML::Node& cfg, const char* key, const vec6& defaults) {
    vec6 out = defaults;
    if (cfg[key] && cfg[key].IsSequence() && cfg[key].size() == 6) {
        for (int i = 0; i < 6; ++i) {
            out(i) = cfg[key][i].as<double>();
        }
    }
    return out;
}
double wrapToPi(double a) {
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}
}



Controller::Controller(const YAML::Node& config) : m_config(config)
{
    initClass();
}

Controller::~Controller() {
    // cleanup if needed
}


void Controller::initClass()
{

    m_joint_state_ref = vec6::Zero();
    m_joint_vel_ref = vec6::Zero();
    m_ff_torque = vec6::Zero();
    m_kp = vec6::Zero();
    m_kp << 8, 8, 8, 8, 8, 8;
    m_kd = vec6::Zero();
    m_kd << 0.5, 0.5, 0.5, 0.5, 0.5, 0.5;
    vec6 cart_k_default;   cart_k_default << 120.0, 120.0, 140.0, 25.0, 25.0, 20.0;
    vec6 cart_d_default;   cart_d_default << 20.0, 20.0, 24.0, 6.0, 6.0, 5.0;
    vec6 tau_lim_default;  tau_lim_default << 11.0, 11.0, 22.0, 2.4, 2.4, 2.4;
    vec6 null_kp_default = vec6::Zero();
    vec6 null_kd_default = vec6::Zero();

    m_use_cartesian_impedance = m_config["use_cartesian_impedance"]
        ? m_config["use_cartesian_impedance"].as<bool>()
        : false;

    m_cart_k   = loadVec6Config(m_config, "cart_k", cart_k_default);
    m_cart_d   = loadVec6Config(m_config, "cart_d", cart_d_default);
    m_tau_limit = loadVec6Config(m_config, "tau_limit", tau_lim_default);
    m_null_kp  = loadVec6Config(m_config, "null_kp", null_kp_default);
    m_null_kd  = loadVec6Config(m_config, "null_kd", null_kd_default);

    if (m_use_cartesian_impedance) {
        setTorqueControlMode();
    } else {
        setPositionControlMode();
    }

    

    m_ee_state_init = vec6::Zero();

    // Variables
    m_ee_state_ref = vec6::Zero();
    m_ee_vel_ref = vec6::Zero();
    m_ee_acc_ref = vec6::Zero();

    m_joint_state_act = vec6::Zero();
    m_joint_vel_act = vec6::Zero();
    m_joint_acc_act = vec6::Zero();
      
    // initial footholds in default sleep, in global frame
    if (m_robot != NULL) {
        m_ee_state_init = m_robot->getSleepStates();
        m_joint_state_init = m_robot->inverseKinematics(m_ee_state_init);
    } else {
        std::cout << "Robot ptr not set.\n";
    }

    m_ee_state_ref = m_ee_state_init;
    m_joint_state_ref = m_joint_state_init;

    // m_contact_flag_for_controller = int4::Zero();

    m_joint_state_act = m_joint_state_init;
    m_joint_vel_act = vec6::Zero();    
}

void Controller::setInitStates(const vec6 &x_init, const vec6 &js_init)
{
    m_ee_state_init = x_init;
    m_ee_state_ref = x_init;
    m_ee_vel_ref = vec6::Zero();
    m_ee_acc_ref = vec6::Zero();

    m_joint_state_act = js_init;
    m_joint_state_init = js_init;
    m_joint_state_ref = js_init;
}

void Controller::setSleepJointGains()
{
    std::array<float, 6> kp_defaults = {10, 10, 10, 10 , 10, 10};
    std::array<float, 6> kd_defaults = {3, 3, 3, 3, 3, 3};

    for (int i = 0; i < 6; ++i) {
        m_kp(i) = m_config["joint_kp"] && m_config["joint_kp"].size() == 6
            ? m_config["joint_kp"][i].as<double>()
            : kp_defaults[i];

        m_kd(i) = m_config["joint_kd"] && m_config["joint_kd"].size() == 6
            ? m_config["joint_kd"][i].as<double>()
            : kd_defaults[i];
    }

}

void Controller::setZeroJointVelocity() {
    m_joint_vel_ref.block<6,1>(0, 0) = vec6::Zero();
}

void Controller::setZeroTorqueFF() {
    m_ff_torque = vec6::Zero();
}

void Controller::setPositionControlMode() {
    m_use_torque_control = false;
    m_use_position_control = true;
    setZeroJointVelocity();
    // setZeroTorqueFF();
    // setStanceJointGains();
    setSleepJointGains();

}
void Controller::setTorqueControlMode() {
    // Keep stabilizing joint PD active in torque mode so the low-level
    // simulator motor loop can provide posture support in addition to tau_ff.
    setSleepJointGains();

    m_use_torque_control = true;
    m_use_position_control = false;
}


vec6 Controller::getReferenceTorques() {
    return m_ff_torque;
}

vec6 Controller::computeCartesianImpedanceTorque(){
    const vec6 q = m_joint_state_act;
    const vec6 qd = m_joint_vel_act;
    const vec6 x_act = m_robot -> forwardKinematics(q);
    mat6x6 J = m_robot -> JacobianCompute(q);
    const vec6 xd_act = J*qd;
    vec6 e_x  = m_ee_state_ref - x_act;
    vec6 e_xd = m_ee_vel_ref   - xd_act;

    e_x(3) = wrapToPi(e_x(3));
    e_x(4) = wrapToPi(e_x(4));
    e_x(5) = wrapToPi(e_x(5));
    const vec6 F_task = m_cart_k.cwiseProduct(e_x) + m_cart_d.cwiseProduct(e_xd);
    const vec6 tau_task = J.transpose() * F_task;
    const vec6 tau_null = m_null_kp.cwiseProduct(m_joint_state_init-q)+m_null_kd.cwiseProduct(-qd);

    const Eigen::VectorXd tau_gyd= m_robot->getGravityVector(q);
    vec6 tau_g = vec6::Zero();
    tau_g = tau_gyd.head<6>();

    vec6 tau_cmd = tau_g+tau_null+tau_task;
    for (int i = 0; i < 6; ++i) {
        if (tau_cmd(i) >  m_tau_limit(i)) tau_cmd(i) =  m_tau_limit(i);
        if (tau_cmd(i) < -m_tau_limit(i)) tau_cmd(i) = -m_tau_limit(i);
    }

    return tau_cmd;
}
void Controller::step(const double &dt)
{
    m_dt = dt;
    m_loop_rate = 1. / m_dt;

    updateEstimationData();

    updatePlannerData();

    m_joint_state_ref = m_robot->inverseKinematics(m_ee_state_ref);
    const mat6x6 J_ref = m_robot->JacobianCompute(m_joint_state_ref);
    m_joint_vel_ref = J_ref.partialPivLu().solve(m_ee_vel_ref);

    if (m_use_torque_control) {
    if (m_use_cartesian_impedance) {
        m_ff_torque = computeCartesianImpedanceTorque();
    } else {
        m_ff_torque = calculateFeedForwardTorque();
    }
} else {
    m_ff_torque = vec6::Zero();
}

    updateJointCommand();
}

void Controller::step(const double& dt, const double& t_curr) {
    step(dt);
}

bool Controller::checkJointCommandSafety()
{
    if (m_joint_state_ref.block<6,1>(0, 0).hasNaN())
    {
        std::cerr << "NaN in the commanded joint angles. Setting to default angles.\n";
        return true;
    }

    if (m_joint_vel_ref.block<6,1>(0, 0).hasNaN())
    {
        std::cerr << "NaN in the commanded joint velocities. Setting to zero velocoties.\n";
        return true;
    }

    if (m_ff_torque.hasNaN())
    {
        std::cerr << "NaN in the commanded FF joint torques. Setting to zero FF torques.\n";
        return true;
    }

    return false;
}

void Controller::updateEstimationData()
{
    if (m_estimation_data_ptr->hasNanInf()) {
        std::cerr << "[CTRL] NaN or Inf in estimated data." << std::endl;
    }
    else {
        m_joint_state_act = m_estimation_data_ptr->js;
        m_joint_vel_act = m_estimation_data_ptr->jv;
        // m_joint_acc_act = m_estimation_data_ptr->ja;
        // m_cs = m_estimation_data_ptr->cs;
    }
}

void Controller::updatePlannerData()
{
    if (m_planner_data_ptr->hasNanInf()) {
        std::cerr << "[CTRL] NaN or Inf in planner data." << std::endl;
    }
    else {
    m_ee_state_ref = m_planner_data_ptr->x;
    m_ee_vel_ref = m_planner_data_ptr->xd;
    m_ee_acc_ref = m_planner_data_ptr->xdd;
    // m_expected_stance_flag = m_planner_data_ptr->cs_ref;
    m_mode = m_planner_data_ptr->mode;
    }
}

void Controller::updateJointCommand() {

    if (!m_joint_state_ref.block<6, 1>(0, 0).hasNaN() && m_joint_state_ref.block<6, 1>(0, 0).allFinite()) {
        m_joint_command_ptr->q = m_joint_state_ref.block<6, 1>(0, 0);
        if (m_joint_state_ref.block<6, 1>(0, 0).norm() == 0) {
        std::cout << "Zerossssss in ctrl\n";
        }
    }
    else {
        m_trigger_safety = true;
        std::cerr << "[CTRL] Nan or Inf in joint angles. Skipping joint angle update for this time step." << std::endl;
    }

    if (!m_joint_vel_ref.block<6, 1>(0, 0).hasNaN() && m_joint_vel_ref.block<6, 1>(0, 0).allFinite()) {
        m_joint_command_ptr->qd = m_joint_vel_ref.block<6, 1>(0, 0);
    }
    else {
        m_trigger_safety = true;
        m_joint_command_ptr->qd = vec6::Zero();
        std::cerr << "[CTRL] Nan or Inf in joint velocities. Setting zero joint velocity for this time step." << std::endl;
    }

    if (!m_ff_torque.hasNaN() && m_ff_torque.allFinite()) {
        m_joint_command_ptr->tau = m_ff_torque;
    }
    else {
        m_trigger_safety = true;
        m_joint_command_ptr->tau = vec6::Zero();
        std::cerr << "[CTRL] Nan or Inf in joint torques. Setting zero joint torque for this time step." << std::endl;
    }

    if (!m_kp.hasNaN() && m_kp.allFinite()) {
        m_joint_command_ptr->kp = m_kp;
    } 
    else {
        std::cerr << "[CTRL] Nan or Inf in joint Kp values. Skipping joint Kp update for this time step." << std::endl;
    }

    if (!m_kd.hasNaN() && m_kd.allFinite()) {
        m_joint_command_ptr->kd = m_kd;
    } 
    else {
        std::cerr << "[CTRL] Nan or Inf in joint Kd values. Skipping joint Kd update for this time step." << std::endl;
    }

    if (!m_joint_state_ref.block<6, 1>(0, 0).hasNaN() && m_joint_state_ref.block<6, 1>(0, 0).allFinite() &&
        !m_joint_vel_ref.block<6, 1>(0, 0).hasNaN() && m_joint_vel_ref.block<6, 1>(0, 0).allFinite() &&
        !m_ff_torque.hasNaN() && m_ff_torque.allFinite() &&
        !m_kp.hasNaN() && m_kp.allFinite() &&
        !m_kd.hasNaN() && m_kd.allFinite()) {
            // set the safety trigger back to false only when all quantities are NaN and Inf-free
            if (m_trigger_safety) {
                std::cout << "[CTRL] Turning off the safety flag." << std::endl;
            }
            m_trigger_safety = false;
        }
}

bool Controller::getSafetyTriggerFlag() {
    // std::cout << "js_ref: " << m_joint_state_ref.transpose() << std::endl;
    // std::cout << "js_vel: " << m_joint_vel_ref.transpose() << std::endl;
    // std::cout << "js_tau: " << m_ff_torque.transpose() << std::endl;

    return m_trigger_safety;
}
