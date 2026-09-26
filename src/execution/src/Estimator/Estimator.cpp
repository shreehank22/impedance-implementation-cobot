#include "Estimator/Estimator.hpp"

// Estimator::Estimator() {
//     initClass();
// }

Estimator::Estimator(const YAML::Node& config) : m_config(config) {
    initClass();
}
// Estimator::Estimator(const YAML::Node& config) {
//     if (config["cutoff_freq_imu_accel"]) {
//         m_cutoff_freq_imu_accel = config["cutoff_freq_imu_accel"].as<double>();
//     }
//     if (config["cutoff_freq_imu_gyro"]) {
//         m_cutoff_freq_imu_gyro = config["cutoff_freq_imu_gyro"].as<double>();
//     }
//     if (config["cutoff_freq_encoder_pos"]) {
//         m_cutoff_freq_encoder_pos = config["m_cutoff_freq_encoder_pos"].as<double>();
//     }
//     if (config["cutoff_freq_encoder_vel"]) {
//         m_cutoff_freq_encoder_vel = config["m_cutoff_freq_encoder_vel"].as<double>();
//     }
//     if (config["cutoff_freq_encoder_tau"]) {
//         m_cutoff_freq_encoder_tau = config["m_cutoff_freq_encoder_tau"].as<double>();
//     }
//     init_class();
// }

void Estimator::initClass() {

    m_cutoff_freq_encoder_pos = 32;
    m_cutoff_freq_encoder_vel = 32;
    m_cutoff_freq_encoder_tau = 32;

    if (m_config["cutoff_freq_encoder_pos"]) {
        m_cutoff_freq_encoder_pos = m_config["cutoff_freq_encoder_pos"].as<double>();
    }
    if (m_config["cutoff_freq_encoder_vel"]) {
        m_cutoff_freq_encoder_vel = m_config["cutoff_freq_encoder_vel"].as<double>();
    }
    if (m_config["cutoff_freq_encoder_tau"]) {
        m_cutoff_freq_encoder_tau = m_config["cutoff_freq_encoder_tau"].as<double>();
    }
    if (m_config["cutoff_freq_encoder_tau"]) {
        m_cutoff_freq_encoder_tau = m_config["cutoff_freq_encoder_tau"].as<double>();
    }


    if (m_loop_rate != 0) {
        // std::cout << "Setting up filters with loop rate: " << m_loop_rate << "\n";
        float sampling_freq = m_loop_rate;

        for (int i = 0; i < 6; ++i) {
            m_q_filter[i].setup(sampling_freq, m_cutoff_freq_encoder_pos);
            m_qd_filter[i].setup(sampling_freq, m_cutoff_freq_encoder_vel);
            m_tau_filter[i].setup(sampling_freq, m_cutoff_freq_encoder_tau);
        }
    }

    printer = new RateLimitedPrinter(500);
}

void Estimator::step(const double& dt) {
    m_dt = dt;
    m_loop_rate = 1./m_dt;

    updateSensorData();

    computeEstimate();

    updateEstimationData();
}

void Estimator::computeEstimate() {
    // Copy joint position and velocity from sensor data
    est_data.js = sensor_data.q;
    est_data.jv = sensor_data.qd;
    
    // Estimate joint acceleration (finite difference, optional)
    static vec6 prev_jv = vec6::Zero();
    est_data.ja = (est_data.jv - prev_jv) / m_dt;
    prev_jv = est_data.jv;

    // Optional: Compute end-effector position and velocity using forward kinematics
    // This assumes you have some kinematics module to compute them
    // est_data.pE = forwardKinematics(est_data.js);
    // est_data.vE = computeEndEffectorVelocity(est_data.jv);

    // Optional: Update pick state from some condition or function
    // est_data.ps = computePickState(...);
}


void Estimator::updateSensorData() {
    sensor_data.copy(*m_sensor_data_ptr);

    // est_data.obstacle_coords = sensor_data.obstacle_coords;

    for (int i = 0; i < 6; ++i) {
        sensor_data.q(i) = m_q_filter[i].filter(sensor_data.q(i));
        sensor_data.qd(i) = m_qd_filter[i].filter(sensor_data.qd(i));
        sensor_data.tau(i) = m_tau_filter[i].filter(sensor_data.tau(i));
    }


    // if (!m_calibration) {
    //     sensor_data.quat = pinocchio::RotToQuat(pinocchio::QuatToRot(quat_init).transpose() * pinocchio::QuatToRot(sensor_data.quat));
    // } else {
    //     float alpha = 0.1;
    //     g_offset = alpha * g_offset + (1 - alpha) * sensor_data.a_B.norm();
    // }

    // mat3x3 Rot = pinocchio::QuatToRot(sensor_data.quat);

    // sensor_data.a_W = Rot * sensor_data.a_B + vec3(0, 0, -g_offset);
    // sensor_data.w_W = Rot * sensor_data.w_B;

    // m_pc_ref = m_planner_data_ptr -> pc_ref;
}

void Estimator::updateEstimationData() {
    m_estimation_data_ptr->copy(est_data);
    // m_measurement_data_ptr -> estimated_contact_force = this -> m_fc_est;
}

void Estimator::getInitParams() {
    // std::cout << "quat_init: " << quat_init.transpose() << "\n";
    // std::cout << "g_offset: " << g_offset << "\n";
}


vec6 Estimator::getEndEffectorPosition() {
    return est_data.pE;
}

vec6 Estimator::getEndEffectorVelocity() {
    return est_data.vE;
}

vec6 Estimator::getJointPositions() {
    return sensor_data.q;
}

vec6 Estimator::getJointVelocities() {
    return sensor_data.qd;
}

vec6 Estimator::getJointTorques() {
    return sensor_data.tau;
}