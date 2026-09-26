#include "QuadROS1Comm.hpp"

QuadROS1Comm::QuadROS1Comm(const DATA_ACCESS_MODE& mode) : m_name("svan"), CommunicationManager(mode) {
    initClass();
}

QuadROS1Comm::QuadROS1Comm(const std::string& name, const DATA_ACCESS_MODE& mode) : m_name(name), CommunicationManager(name, mode) {
    initClass();
}

QuadROS1Comm::~QuadROS1Comm() {

    if (m_thread.joinable()) {
        m_thread.join();
    }
}

void QuadROS1Comm::initClass() {
    m_dt = 1./m_loop_rate;

    auto pub_timer = std::chrono::duration<double>(m_dt);
    // m_timer = m_nh.createTimer(pub_timer, &QuadROS1Comm::timer_cb, this));
    m_timer = m_nh.createWallTimer(ros::WallDuration(m_dt), &QuadROS1Comm::timer_cb, this);

    if (m_mode == DATA_ACCESS_MODE::EXECUTOR) {
        m_joint_cmd_pub = m_nh.advertise<xterra::JointData>("/" + m_name + "/joint_cmd", 1);
        m_sensor_data_sub = m_nh.subscribe<xterra::SensorData>(
            "/" + m_name + "/sensor_data",
            1,
            &QuadROS1Comm::get_sensor_data_cb, this
        );
        
        m_joy_data_sub = m_nh.subscribe<sensor_msgs::Joy>(
            "/" + m_name + "/joystick_data", 
            1, 
            &QuadROS1Comm::get_joystick_data_cb, this
        );

        m_estimated_data_pub = m_nh.advertise<xterra::QuadLog>("/" + m_name + "/estimated", 1);
        m_estimated_data = xterra::QuadLog();
        m_reference_data_pub = m_nh.advertise<xterra::QuadLog>("/" + m_name + "/reference", 1);
        m_reference_data = xterra::QuadLog();
        

        m_plant_time_sub = m_nh.subscribe<std_msgs::Float32>(
            "/" + m_name + "/time",
            1,
            &QuadROS1Comm::get_plant_time_cb, this
        );

        std::cout << "Waiting for joint state publisher...\n";
        while (m_sensor_data_sub.getNumPublishers() < 1) {
        }
        std::cout << "Started receiving joint state data!\n";

    } else if (m_mode == DATA_ACCESS_MODE::PLANT) {

        m_plant_time_pub = m_nh.advertise<std_msgs::Float32>("/" + m_name + "/time", 1);

        m_joint_cmd_sub = m_nh.subscribe<xterra::JointData>(
            "/" + m_name + "/joint_cmd", 
            1, 
            &QuadROS1Comm::get_joint_cmd_cb, this
        );

        m_sensor_data_pub = m_nh.advertise<xterra::SensorData>("/" + m_name + "/sensor_data", 1);

        m_ground_truth_data_pub = m_nh.advertise<xterra::QuadLog>("/" + m_name + "/ground_truth", 1);
        m_ground_truth_data = xterra::QuadLog();
    }

    m_joint_data = xterra::JointData();
    for (int i = 0; i < 12; ++i) {
        m_joint_data.q[i] = 0;
        m_joint_data.dq[i] = 0;
        m_joint_data.tau[i] = 0;
        m_joint_data.kp[i] = 0;
        m_joint_data.kd[i] = 0;
    }

    m_sensor_data = xterra::SensorData();
    for (int i = 0; i < 12; ++i) {
        m_sensor_data.q[i] = 0;
        m_sensor_data.dq[i] = 0;
        m_sensor_data.ddq[i] = 0;
        m_sensor_data.tau_est[i] = 0;
    }
    for (int i = 0; i < 3; ++i) {
        m_sensor_data.quat[i] = 0;
        m_sensor_data.accel[i] = 0;
        m_sensor_data.gyro[i] = 0;
        m_sensor_data.rpy[i] = 0;
    }
    m_sensor_data.quat[3] = 1;

    joystick_data = sensor_msgs::Joy();
    joystick_data.axes.clear();
    joystick_data.buttons.clear();
    // considering 8 axes and 12 buttons
    for (int i = 0; i < 8; ++i) {
        joystick_data.axes.push_back(0);
    }
    for (int i = 0; i < 12; ++i) {
        joystick_data.buttons.push_back(0);
    }

    setPlantTimePtr(&m_plant_time);
}

void QuadROS1Comm::get_sensor_data_cb(const xterra::SensorData::ConstPtr& msg) {
    if (m_sensor_data_ptr == NULL) {
        // std::cout << "Sensor data pointer not set!\n";
        return;
    }

    std::unique_lock<std::mutex> lock(m_sensor_data_mutex);

    for (int i = 0; i < 12; ++i) {
        m_sensor_data_ptr -> q(i) = msg -> q[i];
        m_sensor_data_ptr -> qd(i) = msg -> dq[i];
        m_sensor_data_ptr -> tau(i) = msg -> tau_est[i];
    }
    for (int i = 0; i < 4; ++i) {
        m_sensor_data_ptr -> quat(i) = msg -> quat[i];
    }
    for (int i = 0; i < 3; ++i) {
        m_sensor_data_ptr -> eul(i) = msg -> rpy[i];
        m_sensor_data_ptr -> w_B(i) = msg -> gyro[i];
        m_sensor_data_ptr -> a_B(i) = msg -> accel[i];
    }
}

void QuadROS1Comm::get_joint_cmd_cb(const xterra::JointData::ConstPtr& msg) {
    if (m_joint_command_data_ptr == NULL) {
        // std::cout << "Joint command data pointer not set!\n";
        return;
    }
    for (int i = 0; i < 12; ++i) {
        m_joint_command_data_ptr -> kp(i) = msg -> kp[i];
        m_joint_command_data_ptr -> kd(i) = msg -> kd[i];
        m_joint_command_data_ptr -> q(i) = msg -> q[i];
        m_joint_command_data_ptr -> qd(i) = msg -> dq[i];
        m_joint_command_data_ptr -> tau(i) = msg -> tau[i];
    }
}

void QuadROS1Comm::get_joystick_data_cb(const sensor_msgs::Joy::ConstPtr& msg) {
    if (m_joystick_data_ptr == NULL || !m_is_executor_ready) {
        std::cout << "Joystick pointer not set or executor is not ready!\n";
        return;
    }
    // m_joystick_data_ptr -> lx = -1 * msg.axes[0];
    // m_joystick_data_ptr -> ly = msg.axes[1];
    // m_joystick_data_ptr -> rx = -1 * msg.axes[2];
    // m_joystick_data_ptr -> ry = msg.axes[3];
    // m_joystick_data_ptr -> rt = msg.axes[4];
    // m_joystick_data_ptr -> lt = msg.axes[5];
    // m_joystick_data_ptr -> dx = msg.axes[6];
    // m_joystick_data_ptr -> dy = msg.axes[7];

    // // the mode is set by finding the index of the button that is pressed
    // m_joystick_data_ptr -> mode = std::distance(msg.buttons.begin(), std::find(msg.buttons.begin(), msg->buttons.end(), 1)) + 1;

    m_joystick_data_ptr->vel_x = msg->axes[1];
    m_joystick_data_ptr->vel_y = -msg->axes[0];
    m_joystick_data_ptr->vel_yaw = -msg->axes[2];
    m_joystick_data_ptr->pitch = -msg->axes[3];

    if (msg->buttons[0]) {
        m_joystick_data_ptr->mode = 1;
    } else if (msg->buttons[1]) {
        m_joystick_data_ptr->mode = 4;
    } else if (msg->buttons[3]) {
        m_joystick_data_ptr->mode = 3;
    } else if (msg->buttons[2]) {
        m_joystick_data_ptr->mode = 2;
    }
    if (msg->axes[5] > 0) {
        m_joystick_data_ptr -> up = msg->axes[5];
    } else {
        m_joystick_data_ptr -> down = -msg->axes[5];
    }
    if (msg->axes[4] > 0) {
        m_joystick_data_ptr -> left = msg->axes[4];
    } else {
        m_joystick_data_ptr -> right = -msg->axes[4];
    }
}

void QuadROS1Comm::get_plant_time_cb(const std_msgs::Float32::ConstPtr& msg) {
    m_plant_time =  msg -> data;
}

void QuadROS1Comm::write_measurement_data() {
    if (m_estimation_data_ptr == NULL || m_measurement_data_ptr == NULL) {
        // std::cout << "Estimation or measurement data pointer not set!\n";
        return;
    }

    // write estimated data and publish
    for (int i = 0; i < 4; ++i) {
        m_estimated_data.contact_state[i] = m_estimation_data_ptr->cs(i);
        m_estimated_data.contact_prob[i] = m_estimation_data_ptr->pc(i);
    }

    for (int i = 0; i < 12; ++i) {
        m_estimated_data.contact_force[i] = m_measurement_data_ptr->estimated_contact_force(i);
    }

    m_estimated_data.base_position.x = m_estimation_data_ptr->rB(0);
    m_estimated_data.base_position.y = m_estimation_data_ptr->rB(1);
    m_estimated_data.base_position.z = m_estimation_data_ptr->rB(2);

    m_estimated_data.base_orientation.x = m_estimation_data_ptr->js(3);
    m_estimated_data.base_orientation.y = m_estimation_data_ptr->js(4);
    m_estimated_data.base_orientation.z = m_estimation_data_ptr->js(5);
    m_estimated_data.base_orientation.w = m_estimation_data_ptr->js(6);

    m_estimated_data.linear_velocity.x = m_estimation_data_ptr->vB(0);
    m_estimated_data.linear_velocity.y = m_estimation_data_ptr->vB(1);
    m_estimated_data.linear_velocity.z = m_estimation_data_ptr->vB(2);

    m_estimated_data.angular_velocity.x = m_estimation_data_ptr->jv(3);
    m_estimated_data.angular_velocity.y = m_estimation_data_ptr->jv(4);
    m_estimated_data.angular_velocity.z = m_estimation_data_ptr->jv(5);

    for (int i = 0; i < 6; ++i) {
        m_estimated_data.base_wrench[i] = m_measurement_data_ptr->estimated_base_wrench(i);
    }

    for (int i = 0; i < 12; ++i) {
        m_estimated_data.joint_position[i] = m_estimation_data_ptr->js(7 + i);
        m_estimated_data.joint_velocity[i] = m_estimation_data_ptr->jv(6 + i);
        m_estimated_data.joint_torque[i] = m_sensor_data_ptr->tau(i);
    }

    m_estimated_data.foot_FL_position.x = m_estimation_data_ptr->rP(0);
    m_estimated_data.foot_FL_position.y = m_estimation_data_ptr->rP(1);
    m_estimated_data.foot_FL_position.z = m_estimation_data_ptr->rP(2);

    m_estimated_data.foot_FR_position.x = m_estimation_data_ptr->rP(3);
    m_estimated_data.foot_FR_position.y = m_estimation_data_ptr->rP(4);
    m_estimated_data.foot_FR_position.z = m_estimation_data_ptr->rP(5);

    m_estimated_data.foot_RL_position.x = m_estimation_data_ptr->rP(6);
    m_estimated_data.foot_RL_position.y = m_estimation_data_ptr->rP(7);
    m_estimated_data.foot_RL_position.z = m_estimation_data_ptr->rP(8);

    m_estimated_data.foot_RR_position.x = m_estimation_data_ptr->rP(9);
    m_estimated_data.foot_RR_position.y = m_estimation_data_ptr->rP(10);
    m_estimated_data.foot_RR_position.z = m_estimation_data_ptr->rP(11);

    m_estimated_data_pub.publish(m_estimated_data);

    if (m_is_executor_ready) {

    if(m_planner_data_ptr == NULL) {
        // std::cout << "Planner data pointer not set!\n";
        return;
    }

    // write reference data and publish
    for (int i = 0; i < 4; ++i) {
        m_reference_data.contact_state[i] = m_planner_data_ptr->cs_ref(i);
        m_reference_data.contact_prob[i] = m_planner_data_ptr->pc_ref(i);
    }

    for (int i = 0; i < 12; ++i) {
        m_reference_data.contact_force[i] = m_measurement_data_ptr->desired_contact_force(i);
    }

    m_reference_data.base_position.x = m_planner_data_ptr->x(0);
    m_reference_data.base_position.y = m_planner_data_ptr->x(1);
    m_reference_data.base_position.z = m_planner_data_ptr->x(2);

    m_reference_data.base_orientation.x = m_planner_data_ptr->x(3);
    m_reference_data.base_orientation.y = m_planner_data_ptr->x(4);
    m_reference_data.base_orientation.z = m_planner_data_ptr->x(5);
    m_reference_data.base_orientation.w = m_planner_data_ptr->x(6);

    m_reference_data.linear_velocity.x = m_planner_data_ptr->xd(0);
    m_reference_data.linear_velocity.y = m_planner_data_ptr->xd(1);
    m_reference_data.linear_velocity.z = m_planner_data_ptr->xd(2);

    m_reference_data.angular_velocity.x = m_planner_data_ptr->xd(3);
    m_reference_data.angular_velocity.y = m_planner_data_ptr->xd(4);
    m_reference_data.angular_velocity.z = m_planner_data_ptr->xd(5);

    for (int i = 0; i < 6; ++i) {
        m_reference_data.base_wrench[i] = m_measurement_data_ptr->desired_base_wrench(i);
    }

    for (int i = 0; i < 12; ++i) {
        m_reference_data.joint_position[i] = m_joint_command_data_ptr->q(i);
        m_reference_data.joint_velocity[i] = m_joint_command_data_ptr->qd(i);
        m_reference_data.joint_torque[i] = m_joint_command_data_ptr->tau(i);
    }

    m_reference_data.foot_FL_position.x = m_planner_data_ptr->x(7);
    m_reference_data.foot_FL_position.y = m_planner_data_ptr->x(8);
    m_reference_data.foot_FL_position.z = m_planner_data_ptr->x(9);

    m_reference_data.foot_FR_position.x = m_planner_data_ptr->x(10);
    m_reference_data.foot_FR_position.y = m_planner_data_ptr->x(11);
    m_reference_data.foot_FR_position.z = m_planner_data_ptr->x(12);

    m_reference_data.foot_RL_position.x = m_planner_data_ptr->x(13);
    m_reference_data.foot_RL_position.y = m_planner_data_ptr->x(14);
    m_reference_data.foot_RL_position.z = m_planner_data_ptr->x(15);

    m_reference_data.foot_RR_position.x = m_planner_data_ptr->x(16);
    m_reference_data.foot_RR_position.y = m_planner_data_ptr->x(17);
    m_reference_data.foot_RR_position.z = m_planner_data_ptr->x(18);

    m_reference_data_pub.publish(m_reference_data);
    }
}

void QuadROS1Comm::write_sim_measurement_data() {
    if (m_measurement_data_ptr == NULL) {
        return;
    }
    
    m_ground_truth_data.base_position.x = m_measurement_data_ptr -> base_position(0);
    m_ground_truth_data.base_position.y = m_measurement_data_ptr -> base_position(1);
    m_ground_truth_data.base_position.z = m_measurement_data_ptr -> base_position(2);

    m_ground_truth_data.linear_velocity.x = m_measurement_data_ptr -> base_velocity.linear(0);
    m_ground_truth_data.linear_velocity.y = m_measurement_data_ptr -> base_velocity.linear(1);
    m_ground_truth_data.linear_velocity.z = m_measurement_data_ptr -> base_velocity.linear(2);

    m_ground_truth_data.angular_velocity.x = m_measurement_data_ptr -> base_velocity.angular(0);
    m_ground_truth_data.angular_velocity.y = m_measurement_data_ptr -> base_velocity.angular(1);
    m_ground_truth_data.angular_velocity.z = m_measurement_data_ptr -> base_velocity.angular(2);

    for (int i = 0; i < 12; ++i) {
        m_ground_truth_data.contact_force[i] = m_measurement_data_ptr -> contact_force(i);
    }

    m_ground_truth_data_pub.publish(m_ground_truth_data);
}

void QuadROS1Comm::write_joint_command() {
    if (m_joint_command_data_ptr == NULL || !m_is_executor_ready) {
        // std::cout << "Joint command data pointer not set or executor not ready!\n";
        return;
    }

    for (int i = 0; i < 12; ++i) {
        m_joint_data.q[i] = m_joint_command_data_ptr->q(i);
        m_joint_data.dq[i] = m_joint_command_data_ptr->qd(i);
        m_joint_data.tau[i] = m_joint_command_data_ptr->tau(i);
        m_joint_data.kp[i] = m_joint_command_data_ptr->kp(i);
        m_joint_data.kd[i] = m_joint_command_data_ptr->kd(i);
    }

    m_joint_cmd_pub.publish(m_joint_data);
}

void QuadROS1Comm::write_sensor_data() {
    if (m_sensor_data_ptr == NULL) {
        return;
    }

    std::unique_lock<std::mutex> lock(m_sensor_data_mutex);

    for (int i = 0; i < 12; ++i) {
        m_sensor_data.q[i] = m_sensor_data_ptr->q(i);
        m_sensor_data.dq[i] = m_sensor_data_ptr->qd(i);
        m_sensor_data.tau_est[i] = m_sensor_data_ptr->tau(i);
    }
    for (int i = 0; i < 3; ++i) {
        m_sensor_data.rpy[i] = m_sensor_data_ptr->eul(i);
        m_sensor_data.accel[i] = m_sensor_data_ptr->a_B(i);
        m_sensor_data.gyro[i] = m_sensor_data_ptr->w_B(i);
    }

    vec4 quat_calc = pinocchio::EulToQuat(m_sensor_data_ptr->eul);
    
    for (int i = 0; i < 4; ++i) {
        #ifdef USE_HARDWARE
            m_sensor_data.quat[i] = quat_calc(i);
        #else
            m_sensor_data.quat[i] = m_sensor_data_ptr->quat(i);
        #endif
    }

    m_sensor_data_pub.publish(m_sensor_data);
}

void QuadROS1Comm::timer_cb(const ros::WallTimerEvent& timer_event) {
    if (m_mode == DATA_ACCESS_MODE::EXECUTOR) {
        // m_joint_data.header.stamp = ros::Time::now();
        write_joint_command();
        write_measurement_data();
    } else if (m_mode == DATA_ACCESS_MODE::PLANT) {
        // double time = 0;
        // getPlantTime(time);
        plant_time.data = m_plant_time;
        m_plant_time_pub.publish(plant_time);
        write_sensor_data();
    }
}

void QuadROS1Comm::run() {
    ros::spin();
}

void QuadROS1Comm::start_thread() {
    m_thread = std::thread(&QuadROS1Comm::run, this);
    m_thread.detach();
}
