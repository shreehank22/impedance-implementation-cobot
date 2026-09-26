#include <csignal>

#include "DDSMux.hpp"
#include "JoyData.hpp"
#include "timer.hpp"

bool terminated = false;

void signalHandler(int signum) {
    std::cout << "Interrupt signal (" << signum << ") received.\n";

    terminated = true;
}

using namespace xterra::msg::dds_;

bool isActive(const JoyData_& data) {
    for (int i = 0; i < 6; ++i) {
        if (data.axes()[i] != 0.0f) {
            return true;
        }
    }
    for (int i = 0; i < 12; ++i) {
        if (data.buttons()[i] != 0) {
            return true;
        }
    }
    return false;
}

int main(int argc, char** argv) {
    signal(SIGINT, signalHandler);

    std::vector<std::string> input_topics = {
        "rt/web/c1/joystick_data", "rt/bt_usb/c1/joystick_data",
        "rt/keyboard/c1/joystick_data"};
    std::string output_topic = "rt/c1/joystick_data";

    auto getPriority = [](const JoyData_& data) -> int {
        if (isActive(data)) {
            return data.priority();
        } else {
            return -1;
        }
    };

    DDSMux<JoyData_> joystick_mux(output_topic, input_topics, getPriority, 0);

    while (!terminated) {
        joystick_mux.update();  // Periodically re-evaluate
        wait_ms(10);    // Check every 10ms
    }

    return 0;
}