
#include <iostream>
#include <string>
#include <vector>
#include <signal.h>
#include <unistd.h>
#include <pigpio.h>
#include <memory>
#include "dds_subscriber.hpp"
#include "IndicatorData.hpp"

using namespace xterra::msg::dds_;

std::vector<int> m_indicator_pins = {4, 25, 6}; // BCM pin numbers, not physical

// Signal handler flag
bool running = true;

void signal_handler(int sig) {
    running = false;
}

class IndicatorInterface {
private:
    std::string m_robot;
    std::unique_ptr<DDSSubscriber<IndicatorData_>> m_indicator_sub_ptr;
    const int DOMAIN_ID = 0;  // Default domain, adjust as needed

public:
    IndicatorInterface(const std::string& robot_name = "cobot_c1") 
        : m_robot(robot_name) {
        
        // Initialize GPIO
        // if (gpioInitialise() < 0) {
        //     throw std::runtime_error("Failed to initialize GPIO");
        // }

        // Setup GPIO pins
        for (const auto& pin : m_indicator_pins) {
            gpioSetMode(pin, PI_OUTPUT);
            gpioWrite(pin, 0);  // All LEDs off initially
        }

        // Create topic name
        std::string topic_name = "rt/" + m_robot + "/hw"  + "/indicator_data";
        
        // Create DDS subscriber with bound member function
        m_indicator_sub_ptr.reset(new DDSSubscriber<IndicatorData_>(
            topic_name,
            std::bind(&IndicatorInterface::indicator_callback, this, std::placeholders::_1),
            DOMAIN_ID));
            
        std::cout << "LED subscriber started on topic " << topic_name << ". Waiting for messages..." << std::endl;
    }

    ~IndicatorInterface() {
        // Clean up GPIO
        gpioTerminate();
        std::cout << "GPIO cleaned up, controller destroyed." << std::endl;
    }

    // Member function callback for indicator data
    void indicator_callback(const IndicatorData_& msg) {
        // Update LEDs with individual boolean values
        gpioWrite(m_indicator_pins[0], msg.led1() ? 1 : 0);
        gpioWrite(m_indicator_pins[1], msg.led2() ? 1 : 0);
        gpioWrite(m_indicator_pins[2], msg.led3() ? 1 : 0);
        
        // Print the current state for debugging
        // std::cout << "LED states updated: [" 
        //           << (msg.led1() ? "true" : "false") << ", " 
        //           << (msg.led2() ? "true" : "false") << ", " 
        //           << (msg.led3() ? "true" : "false") << "] (buzzer: " 
        //           << (msg.buzzer() ? "true" : "false") << ")" << std::endl;
    }

    // Run method keeps the program active
    void run() {
        std::cout << "Press Ctrl+C to exit" << std::endl;
        
        // Keep running until signal received
        while (running) {
            usleep(100000);  // Sleep for 100ms
        }
    }
};

int main() {
    try {
        
        // Initialize GPIO
        if (gpioInitialise() < 0) {
            std::cerr << "Failed to initialize GPIO" << std::endl;
            return 1;
        }

        // Register signal handler
        signal(SIGINT, signal_handler);
        
        // Create controller
        IndicatorInterface interface;
        
        // Run main loop
        interface.run();
        
        // Explicit cleanup section - this will run after we exit the loop
        std::cout << "Shutting down, turning off LEDs..." << std::endl;
        // Turn off all LEDs
        for (const auto& pin : m_indicator_pins) {  // Use your actual BCM pin numbers here
            gpioWrite(pin, 0);
        }
    }
    catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}
