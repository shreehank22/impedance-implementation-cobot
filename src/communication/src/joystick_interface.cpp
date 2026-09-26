#include "joystick_interface.hpp"

#include <fcntl.h>
#include <linux/joystick.h>
#include <sys/select.h>
#include <unistd.h>

#include <chrono>
#include <iostream>

JoystickInterface::JoystickInterface() : fd(-1), joystick_calibrated(false) {
    max_value.fill(0.5f);
    min_value.fill(0.1f);
}

JoystickInterface::~JoystickInterface() { disconnectJoystick(); }

bool JoystickInterface::initialize(const std::string& device_path) {
    m_device_path = device_path;
    bool status = connectJoystick();
    if (status) {
        m_joystick_pub.reset(
            new DDSPublisher<JoyData_>("rt/bt_usb/c1/joystick_data", 0));
    }
    // Randomly assigned "high" priority.
    joystick_data.priority() = 100;
    return status;
}

void JoystickInterface::disconnectJoystick() {
    if (fd != -1) {
        close(fd);
    }
    joystick_connected = false;
}

bool JoystickInterface::connectJoystick() {
    // Open in non-blocking mode
    fd = open(m_device_path.c_str(), O_RDONLY | O_NONBLOCK);
    if (fd == -1) {
        std::cerr << "Could not open joystick" << std::endl;
        joystick_connected = false;
        return false;
    }
    std::cout << "Joystick connected successfully.\n";
    joystick_connected = true;
    return true;
}

void JoystickInterface::run() {
    std::cout << "Calibration started. Rotate the sticks and press triggers to "
                 "their extremes. "
              << std::endl;

    using namespace std::chrono;
    auto next_publish = steady_clock::now();

    while (true) {
        if (joystick_connected) {
            auto now = steady_clock::now();
            if (now >= next_publish) {
                // Time to publish
                if (joystick_calibrated && joystick_data_updated) {
                    m_joystick_pub->publish(joystick_data);
                }
                next_publish += milliseconds(20);  // Schedule next publish
            }

            // Calculate remaining time until next publish
            auto timeout = next_publish - now;
            if (timeout < 0ms) timeout = 0ms;

            // Wait for events or timeout
            fd_set readfds;
            FD_ZERO(&readfds);
            FD_SET(fd, &readfds);

            timeval tv;
            auto ms = duration_cast<microseconds>(timeout);
            tv.tv_sec = ms.count() / 1000000;
            tv.tv_usec = ms.count() % 1000000;

            int ret = select(fd + 1, &readfds, nullptr, nullptr, &tv);
            if (ret < 0) {
                std::cerr << "select error" << std::endl;
                break;
            } else if (ret > 0) {
                // Read all available events
                struct js_event e;
                while (true) {
                    ssize_t bytes_read = read(fd, &e, sizeof(e));
                    if (bytes_read == sizeof(e)) {
                        processJoystickEvent(e);
                        joystick_data_updated = true;
                    } else {
                        if (errno == EAGAIN || errno == EWOULDBLOCK) {
                            break;  // No more data
                        } else {
                            joystick_data_updated = false;
                            disconnectJoystick();
                            std::cerr << "Joystick disconnected in operation. "
                                         "Trying to reconnect..."
                                      << std::endl;
                            break;
                        }
                    }
                }
            }

            // Check calibration status
            if (!joystick_calibrated && isCalibrationDone()) {
                std::cout << "Joystick calibrated successfully! Sending "
                             "commands over DDS..."
                          << std::endl;
                joystick_calibrated = true;
            }
        } else {
            std::cout << "Joystick disconnected. Retrying connection...\n";
            connectJoystick();
        }
    }
}

float JoystickInterface::normalize(int value, float min_val, float max_val) {
    return (2.0f * static_cast<float>(value - min_val)) / (max_val - min_val) -
           1.0f;
}

void JoystickInterface::processJoystickEvent(const struct js_event& event) {
    switch (event.type) {
        case JS_EVENT_BUTTON:
            if (event.value > 0) {
                joystick_data.buttons()[event.number] = 1;
                // std::cout << "event no.: " << (int)event.number << "\n";
            } else if (event.value < 0) {
                joystick_data.buttons()[event.number] = -1;
                // std::cout << "event < 0\n";
            } else {
                joystick_data.buttons()[event.number] = 0;
                // std::cout << "event = 0\n";
            }
            break;
        case JS_EVENT_AXIS:
            // Skip D-pad/HAT events (typically axes 6 and 7 for most joysticks)
            // if (event.number == 6 || event.number == 7) {
            //     if (event.value > 0) joystick_data.buttons()[event.number] = 1;
	    //             else if (event.value < 0) joystick_data.buttons()[event.number] = -1;
	    //             else joystick_data.buttons()[event.number] = 0;
            //     break;  // Skip processing for D-pad/HAT axes
            // }
			// Handle DPAD vertical
			if (event.number == 7) {
				if (event.value > 0) {
					joystick_data.buttons()[9] = 1;
					//std::cout << "js dpad >0: " << (int)joystick_data.buttons()[9] << std::endl;
				} else if (event.value < 0) {
					joystick_data.buttons()[10] = 1;
					//std::cout << "js dpad <0: " << (int)joystick_data.buttons()[10] << std::endl;
				} else {
					joystick_data.buttons()[9] = 0;
					joystick_data.buttons()[10] = 0;
				}
				break;
			}
			// Handle DPAD horizontal
			if (event.number == 6) {
				if (event.value > 0) {
					joystick_data.buttons()[7] = 1;
				} else if (event.value < 0) {
					joystick_data.buttons()[8] = 1;
				} else {
					joystick_data.buttons()[7] = 0;
					joystick_data.buttons()[8] = 0;
				}
				break;
			}
            index = event.number;
            if (event.value > max_value[index]) max_value[index] = event.value;
            if (event.value < min_value[index]) min_value[index] = event.value;
            joystick_data.axes()[index] =
                normalize(event.value, min_value[index], max_value[index]);
            joystick_data.axes()[index] =
                normalize(event.value, min_value[index], max_value[index]);
			// Offset the triggers by +1 to get 0 in rest position
			// Indices hardcoded for the Cosmic Byte joystick.
	    	if (index == 2 || index == 5) {
				joystick_data.axes()[index] += 1;
				joystick_data.axes()[index] /= 2.0;
	    	}
            break;
        default:
            /* Ignore init events. */
            break;
    }
}

bool JoystickInterface::isCalibrationDone() const {
    // std::cout << "---\n";
    // for (int i = 0; i < 6; i++)
    // {
    //     std::cout << "max_val: " << max_value[i] << " min_val: " <<
    //     min_value[i] << std::endl;
    // }
    // std::cout << "---\n";

    for (size_t i = 0; i < max_value.size(); ++i) {
        if (max_value[i] == 0.5f || min_value[i] == 0.1f) {
            return false;
        }
    }
    return true;
}

int main(int argc, char** argv) {
    JoystickInterface joystick;
    std::string device_path = "/dev/input/js0";
    if (argc > 1) {
        device_path = std::string(argv[1]);
    }
    if (!joystick.initialize(device_path)) {
        return 1;
    }
    joystick.run();
    return 0;
}
