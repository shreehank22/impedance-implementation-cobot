#ifndef JOYSTICK_INTERFACE_H
#define JOYSTICK_INTERFACE_H

#include <array>
#include <string>
#include "dds_publisher.hpp"

#include "JoyData.hpp"

using namespace xterra::msg::dds_;

class JoystickInterface {
public:
    JoystickInterface();
    ~JoystickInterface();

    bool initialize(const std::string& device_path);
    void run();

private:
    static constexpr int NUM_AXES = 6;
    static constexpr int NUM_BUTTONS = 12;

    int fd; // File descriptor for the joystick device
    JoyData_ joystick_data;
    std::shared_ptr<DDSPublisher<JoyData_>> m_joystick_pub;
    std::array<float, NUM_AXES> max_value;
    std::array<float, NUM_AXES> min_value;
    bool joystick_calibrated = false;
    bool joystick_connected = false;
    bool joystick_data_updated = false;

    bool connectJoystick();
    void disconnectJoystick();

    float normalize(int value, float min_val, float max_val);
    void processJoystickEvent(const struct js_event& e);
    bool isCalibrationDone() const;

    std::string m_device_path;

    int index = -1;
};

#endif // JOYSTICK_INTERFACE_H