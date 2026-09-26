#include "joystick_interface.hpp"
#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <linux/joystick.h>

JoystickInterface::JoystickInterface()
    : fd(-1), joystick_calibrated(false),
      participant(0), topic(participant, "JoystickTopic"),
      publisher(participant), writer(publisher, topic) {
    max_value.fill(0.5f);
    min_value.fill(0.1f);
}

JoystickInterface::~JoystickInterface() {
    if (fd != -1) {
        close(fd);
    }
}

bool JoystickInterface::initialize() {
    fd = open("/dev/input/even2", O_RDONLY);
    if (fd == -1) {
        std::cerr << "Could not open joystick" << std::endl;
        return false;
    }
    return true;
}

void JoystickInterface::run() {
    std::cout << "Calibration started. Rotate the sticks and triggers to their extremes, "
              << "press all four direction buttons, and then press MODE to start." << std::endl;

    struct js_event e;
    while (true) {
        if (read(fd, &e, sizeof(e)) != sizeof(e)) {
            std::cerr << "Error reading joystick" << std::endl;
            break;
        }

        processJoystickEvent(e);

        if (!joystick_calibrated && isCalibrationDone()) {
            std::cout << "Joystick calibrated successfully! Sending commands over DDS..." << std::endl;
            joystick_calibrated = true;
        }

        if (joystick_calibrated) {
            writer.write(joystick_data);
        }
    }
}

float JoystickInterface::normalize(int value, float min_val, float max_val) {
    return (2.0f * static_cast<float>(value - min_val)) / (max_val - min_val) - 1.0f;
}

void JoystickInterface::processJoystickEvent(const struct js_event& e) {
    if (e.type & JS_EVENT_BUTTON) {
        if (e.number < joystick_data.buttons().size()) {
            joystick_data.buttons()[e.number] = e.value;
        }
    } else if (e.type & JS_EVENT_AXIS) {
        int index = e.number;
        if (index < joystick_data.axes().size()) {
            if (e.value > max_value[index]) max_value[index] = e.value;
            if (e.value < min_value[index]) min_value[index] = e.value;
            joystick_data.axes()[index] = normalize(e.value, min_value[index], max_value[index]);

            // Invert Y axes
            if (index == 1 || index == 3) {
                joystick_data.axes()[index] = -joystick_data.axes()[index];
            }
        }
    }
}

bool JoystickInterface::isCalibrationDone() const {
    for (size_t i = 0; i < max_value.size(); ++i) {
        if (max_value[i] == 0.5f || min_value[i] == 0.1f) {
            return false;
        }
    }
    return true;
}

int main() {
    JoystickInterface joystick;
    if (!joystick.initialize()) {
        return 1;
    }
    joystick.run();
    return 0;
}