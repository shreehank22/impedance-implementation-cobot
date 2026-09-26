#include "SHM.hpp"

SHM::SHM() : m_name("cobot_c1"), CommunicationManager(DATA_ACCESS_MODE::PLANT) {
    initClass();
}

SHM::SHM(const DATA_ACCESS_MODE& mode) : m_name("svan"), CommunicationManager(mode) {
    initClass();
}

SHM::SHM(const std::string& name, const DATA_ACCESS_MODE& mode) : m_name(name), CommunicationManager(name, mode) {
    initClass();
}

SHM::~SHM() {
    shmdt(m_joint_command_data_ptr);
    shmdt(m_sensor_data_ptr);
}

void* SHM::getSHMPointer(const key_t& key, const size_t& totalSize) {
    // get the id of the shared memory block with the given key
    int shmid = shmget(key, totalSize, 0666|IPC_CREAT);

    // std::cout << "shmid: " << shmid << "\n";

    // Raise error if the shared memory could not be created/assigned
    if (shmid == -1) {
        perror("shmget");
        return (void*)(01);
    }
    // get pointer to the shared memory block
    void *sharedMemory = shmat(shmid, nullptr, 0);

    // Raise error if the returned pointer is not valid
    if (sharedMemory == reinterpret_cast<void*>(01)) {
        perror("shmat");
        return (void*)(01);
    }

    return sharedMemory;
}

void SHM::SetupMemory() {
    key_t key;
    size_t totalSize;
    void* sharedMemory;

    key = SHM_KEYS::COMMAND_DATA;
    totalSize = sizeof(CobotCommandData);
    // std::cout << "Command data  size: " << totalSize << "\n";
    sharedMemory = getSHMPointer(key, totalSize);
    m_joint_command_data_ptr = static_cast<CobotCommandData*>(sharedMemory);
    m_joint_command_data_ptr->q(3) = -0.5342;
    m_joint_command_data_ptr->q(4) = 0.9573;
    m_joint_command_data_ptr->q(5) = -2.4053;
    m_joint_command_data_ptr->q(0) = 0.5342;
    m_joint_command_data_ptr->q(1) = 0.9573;
    m_joint_command_data_ptr->q(2) = -2.4053;
    m_joint_command_data_ptr->q(6) = 0.5342;
    m_joint_command_data_ptr->q(7) = 0.9573;
    m_joint_command_data_ptr->q(8) = -2.4053;
    m_joint_command_data_ptr->q(9) = -0.5342;
    m_joint_command_data_ptr->q(10) = 0.9573;
    m_joint_command_data_ptr->q(11) = -2.4053;
    // *m_joint_command_data_ptr = CobotCommandData();

    key = SHM_KEYS::SENSOR_DATA;
    totalSize = sizeof(CobotSensorData);
    // std::cout << "Sensor data  size: " << totalSize << "\n";
    sharedMemory = getSHMPointer(key, totalSize);
    m_sensor_data_ptr = static_cast<CobotSensorData*>(sharedMemory);
    // *m_sensor_data_ptr = CobotSensorData();

    // key = SHM_KEYS::MEASUREMENT_DATA;
    // totalSize = sizeof(CobotMeasurementData);
    // // std::cout << "Sensor data  size: " << totalSize << "\n";
    // sharedMemory = getSHMPointer(key, totalSize);
    // m_measurement_data_ptr = static_cast<CobotMeasurementData*>(sharedMemory);
    // // *m_measurement_data_ptr = CobotMeasurementData();

    key = SHM_KEYS::PLANT_TIME;
    totalSize = sizeof(double);
    sharedMemory = getSHMPointer(key, totalSize);
    m_plant_time_ptr = static_cast<double*>(sharedMemory);

    *m_plant_time_ptr = 0;
    // std::cout << "Memory setup completed...\n";
}

void SHM::initClass() {

    // if (m_mode == DATA_ACCESS_MODE::EXECUTOR) {
    //     m_joy_data_sub_ptr.reset(new DDSSubscriber<JoyData_>(
    //         "rt/joystick_data",
    //         std::bind(&SHM::get_joystick_data_cb, this, std::placeholders::_1),
    //         0
    //     ));
    // }

    SetupMemory();
}

// void SHM::get_joystick_data_cb(const JoyData_& msg) {
//     if (m_joystick_data_ptr == NULL || !m_is_executor_ready) {
//         std::cout << "Joystick pointer not set or executor is not ready!\n";
//         return;
//     }

//     if (m_joystick_is_nintendo) {
//         m_joystick_data_ptr->vel_x = -msg.axes()[1];
//         m_joystick_data_ptr->vel_y = -msg.axes()[0];
//         m_joystick_data_ptr->vel_yaw = -msg.axes()[2];
//         m_joystick_data_ptr->pitch = -msg.axes()[3];
//         if (msg.buttons()[0]) {
//             m_joystick_data_ptr->mode = 1;
//         } else if (msg.buttons()[1]) {
//             m_joystick_data_ptr->mode = 4;
//         } else if (msg.buttons()[3]) {
//             m_joystick_data_ptr->mode = 3;
//         } else if (msg.buttons()[2]) {
//             m_joystick_data_ptr->mode = 2;
//         }
//         if (msg.axes()[5] > 0) {
//             m_joystick_data_ptr -> up = msg.axes()[5];
//         } else {
//             m_joystick_data_ptr -> down = -msg.axes()[5];
//         }
//         if (msg.axes()[4] > 0) {
//             m_joystick_data_ptr -> left = msg.axes()[4];
//         } else {
//             m_joystick_data_ptr -> right = -msg.axes()[4];
//         }
//     } else {
//         m_joystick_data_ptr->vel_x = -msg.axes()[1];
//         m_joystick_data_ptr->vel_y = -msg.axes()[0];
//         m_joystick_data_ptr->vel_yaw = -msg.axes()[3];
//         m_joystick_data_ptr->pitch = -msg.axes()[4];

//         if (msg.buttons()[0]) {
//             m_joystick_data_ptr->mode = 1;
//         } else if (msg.buttons()[1]) {
//             m_joystick_data_ptr->mode = 2;
//         } else if (msg.buttons()[3]) {
//             m_joystick_data_ptr->mode = 3;
//         } else if (msg.buttons()[2]) {
//             m_joystick_data_ptr->mode = 4;
//         }
//         if (msg.axes()[5] > 0) {
//             m_joystick_data_ptr -> up = msg.axes()[5];
//         } else {
//             m_joystick_data_ptr -> down = -msg.axes()[5];
//         }
//         if (msg.axes()[4] > 0) {
//             m_joystick_data_ptr -> left = msg.axes()[4];
//         } else {
//             m_joystick_data_ptr -> right = -msg.axes()[4];
//         }
//     }

// }