#include "QuadDDSComm.hpp"

QuadDDSComm::QuadDDSComm(const DATA_ACCESS_MODE& mode)
    : m_name("cobot_c1"), CommunicationManager("cobot_c1", mode) {
    initClass();
}

QuadDDSComm::QuadDDSComm(const std::string& name, const DATA_ACCESS_MODE& mode)
    : m_name(name), CommunicationManager(name, mode) {
    initClass();
}

QuadDDSComm::QuadDDSComm(const std::string& name, const std::string& postfix, const DATA_ACCESS_MODE& mode)
    : m_name(name + postfix), CommunicationManager(name + postfix, mode) {
    initClass();
}

QuadDDSComm::~QuadDDSComm() {
    if (m_thread.joinable()) {
        m_thread.join();
    }
}

void QuadDDSComm::initClass() {
    if (m_mode == DATA_ACCESS_MODE::PLANT) {
        m_joint_cmd_sub_ptr.reset(new DDSSubscriber<JointData_>(
            "rt/" + m_name + "/joint_command",
            std::bind(&QuadDDSComm::get_joint_cmd_cb, this, std::placeholders::_1),
            DOMAIN_ID
        ));
        m_joint_command_msg = JointData_();

        m_sensor_data_pub_ptr.reset(new DDSPublisher<SensorData_>(
            "rt/" + m_name + "/sensor_data", DOMAIN_ID));
        m_sensor_data_msg = SensorData_();

    } else if (m_mode == DATA_ACCESS_MODE::EXECUTOR) {
        m_sensor_data_sub_ptr.reset(new DDSSubscriber<SensorData_>(
            "rt/" + m_name + "/sensor_data",
            std::bind(&QuadDDSComm::get_sensor_data_cb, this, std::placeholders::_1),
            DOMAIN_ID
        ));
        m_sensor_data_msg = SensorData_();

        m_joy_data_sub_ptr.reset(new DDSSubscriber<JoyData_>(
            "rt/c1/joystick_data",
            std::bind(&QuadDDSComm::get_joystick_data_cb, this,
                      std::placeholders::_1),
            DOMAIN_ID));
        m_joy_data_msg = JoyData_();

        m_cli_data_sub_ptr.reset(new DDSSubscriber<CliData_>(
            "rt/" + m_name + "/cli_data",
            std::bind(&QuadDDSComm::get_cli_data_cb, this, std::placeholders::_1),
            DOMAIN_ID
        ));
        m_cli_data_msg = CliData_();

        m_joint_cmd_pub_ptr.reset(new DDSPublisher<JointData_>(
            "rt/" + m_name + "/joint_command", DOMAIN_ID));
        m_joint_command_msg = JointData_();

        m_solver_stats_pub_ptr.reset(new DDSPublisher<SolverStats_>(
            "rt/" + m_name + "/solver_stats", DOMAIN_ID));
        m_solver_stats = SolverStats_();
    }

    setPlantTimePtr(&m_plant_time);
    m_startTimePoint = std::chrono::high_resolution_clock::now();
}

void QuadDDSComm::get_joint_cmd_cb(const JointData_& msg) {
    if (m_joint_command_data_ptr == nullptr) return;

    for (int i = 0; i < 6; ++i) {
        m_joint_command_data_ptr->q(i) = msg.q()[i];
        m_joint_command_data_ptr->qd(i) = msg.dq()[i];
        m_joint_command_data_ptr->tau(i) = msg.tau()[i];
        m_joint_command_data_ptr->kp(i) = msg.kp()[i];
        m_joint_command_data_ptr->kd(i) = msg.kd()[i];
    }
}

void QuadDDSComm::get_sensor_data_cb(const SensorData_& msg) {
    if (m_sensor_data_ptr == nullptr) return;

    m_communication_ready = true;

    for (int i = 0; i < 6; ++i) {
        m_sensor_data_ptr->q(i) = msg.q()[i];
        m_sensor_data_ptr->qd(i) = msg.dq()[i];
        m_sensor_data_ptr->tau(i) = msg.tau()[i];
    }
}

void QuadDDSComm::get_joystick_data_cb(const JoyData_& msg) {
    if (m_joystick_data_ptr == NULL || !m_is_executor_ready) {
        return;
    }

    m_last_joystick_update_time =
        get_wall_time_seconds(m_startTimePoint);

    if (is_joystick_disabled()) {
        return;
    }

    m_joystick_data_ptr->left_stick_y  = -msg.axes()[1];
    m_joystick_data_ptr->left_stick_x  = -msg.axes()[0];
    m_joystick_data_ptr->right_stick_x = -msg.axes()[3];
    m_joystick_data_ptr->right_stick_y = -msg.axes()[4];
    m_joystick_data_ptr->left_trigger  = -msg.axes()[2];
    m_joystick_data_ptr->right_trigger =  0.0;
    m_joystick_data_ptr->dpad_x        = -msg.axes()[5];

    if (msg.buttons()[0]) {
        m_joystick_data_ptr->mode = 1;
    } else if (!msg.buttons()[4] && !msg.buttons()[5] && msg.buttons()[1]) {
        m_joystick_data_ptr->mode = 2;
    } else if (!msg.buttons()[4] && !msg.buttons()[5] && msg.buttons()[3]) {
        m_joystick_data_ptr->mode = 3;
    } else if (!msg.buttons()[4] && !msg.buttons()[5] && msg.buttons()[2]) {
        m_joystick_data_ptr->mode = 4;
    } else if (msg.buttons()[9]) {
        m_joystick_data_ptr->dpad_y = 1;
    } else if (msg.buttons()[10]) {
        m_joystick_data_ptr->dpad_y = -1;
    }
}

void QuadDDSComm::get_cli_data_cb(const CliData_& msg) {
    if (m_cli_data_ptr == nullptr) return;

    for (int i = 0; i < 6; ++i) {
        m_cli_data_ptr->x(i) = msg.x()[i];
    }
    m_cli_data_ptr->duration = msg.duration();
    m_cli_data_ptr->mode = msg.mode();
}


void QuadDDSComm::write_sensor_data() {
    std::lock_guard<std::mutex> lock(m_sensor_data_mutex);
    
    if (m_sensor_data_ptr == nullptr) return;

    for (int i = 0; i < 6; ++i) {
        m_sensor_data_msg.q()[i] = m_sensor_data_ptr->q(i);
        m_sensor_data_msg.dq()[i] = m_sensor_data_ptr->qd(i);
        m_sensor_data_msg.tau()[i] = m_sensor_data_ptr->tau(i);
    }

    m_sensor_data_pub_ptr->publish(m_sensor_data_msg);
}

void QuadDDSComm::write_joint_command() {
    if (m_joint_command_data_ptr == nullptr || !m_is_executor_ready) return;

    for (int i = 0; i < 6; ++i) {
        m_joint_command_msg.q()[i] = m_joint_command_data_ptr->q(i);
        m_joint_command_msg.dq()[i] = m_joint_command_data_ptr->qd(i);
        m_joint_command_msg.kp()[i] = m_joint_command_data_ptr->kp(i);
        m_joint_command_msg.kd()[i] = m_joint_command_data_ptr->kd(i);
        m_joint_command_msg.tau()[i] = m_joint_command_data_ptr->tau(i);
    }

    m_joint_cmd_pub_ptr->publish(m_joint_command_msg);
}

void QuadDDSComm::step() {
    if (m_mode == DATA_ACCESS_MODE::PLANT) {
        write_sensor_data();
    } else if (m_mode == DATA_ACCESS_MODE::EXECUTOR) {
        write_joint_command();
    }
}

void QuadDDSComm::run() {
    if (m_loop_rate == 0) {
        std::cout << "Communication rate not set. Defaulting to 500 Hz\n";
        m_loop_rate = 500;
        m_dt = 1. / m_loop_rate;
    }

    int delay_ms = int(m_dt * 1e3);

    while (!terminated()) {
        curr_time = get_wall_time_seconds(m_startTimePoint);
        step();
        delay_ms = int(1e3 * (m_dt - get_wall_time_seconds(m_startTimePoint) + curr_time));
        std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
    }
}

void QuadDDSComm::start_thread() {
    m_thread = std::thread(&QuadDDSComm::run, this);
    m_thread.detach();
}
