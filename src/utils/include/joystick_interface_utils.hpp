#ifndef JOYSTICK_INTERFACE_H
#define JOYSTICK_INTERFACE_H

#include <array>
#include <dds/dds.hpp>
#include "JoyData.hpp"

class JoystickInterface {
public:
    JoystickInterface();
    ~JoystickInterface();

    bool initialize();
    void run();

private:
    static constexpr int NUM_AXES = 8;
    static constexpr int NUM_BUTTONS = 12;

    int fd; // File descriptor for the joystick device
    xterra::msg::dds_::JoyData_ joystick_data;
    std::array<float, NUM_AXES> max_value;
    std::array<float, NUM_AXES> min_value;
    bool joystick_calibrated;

    dds::domain::DomainParticipant participant;
    dds::topic::Topic<xterra::msg::dds_::JoyData_> topic;
    dds::pub::Publisher publisher;
    dds::pub::DataWriter<xterra::msg::dds_::JoyData_> writer;

    float normalize(int value, float min_val, float max_val);
    void processJoystickEvent(const struct js_event& e);
    bool isCalibrationDone() const;
};

#endif // JOYSTICK_INTERFACE_H