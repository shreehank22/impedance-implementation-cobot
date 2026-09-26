#include <iostream>
#include <memory>
#include <chrono>
#include <thread>

#include "CliData.hpp"
#include "dds_publisher.hpp"

using namespace xterra::msg::dds_;

class CLIDataPublisher {
public:
    CLIDataPublisher() : m_active(false) {}

    bool initialize() {
        m_cli_pub = std::make_unique<DDSPublisher<CliData_>>("rt/cobot_c1/cli_data", 0);
        if (!m_cli_pub) {
            std::cerr << "Failed to initialize DDS publisher\n";
            return false;
        }
        m_active = true;
        return true;
    }

    void run() {
        while (m_active) {
            CliData_ msg;

            std::cout << "\nEnter 6 x values: ";
            for (int i = 0; i < 6; ++i) {
                std::cin >> msg.x()[i];
            }

            std::cout << "Enter duration: ";
            std::cin >> msg.duration();

            std::cout << "Enter mode (1-4): ";
            std::cin >> msg.mode();

            m_cli_pub->publish(msg);

            std::cout << "\n[✔] Published CLI Data:\n";
            std::cout << "  x = [";
            for (int i = 0; i < 6; ++i) {
                std::cout << msg.x()[i] << (i != 5 ? ", " : "");
            }
            std::cout << "]\n";
            std::cout << "  duration = " << msg.duration() << "\n";
            std::cout << "  mode = " << msg.mode() << "\n";

            std::cout << "Send more? (y/n): ";
            char cont;
            std::cin >> cont;
            if (cont != 'y' && cont != 'Y') {
                m_active = false;
            }
        }
    }

private:
    std::unique_ptr<DDSPublisher<CliData_>> m_cli_pub;
    bool m_active;
};

int main() {
    CLIDataPublisher cli_pub;
    if (!cli_pub.initialize()) {
        return 1;
    }
    cli_pub.run();
    return 0;
}
