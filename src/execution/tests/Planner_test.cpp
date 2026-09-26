#include "StateMachine/RobotComponents.hpp"
#include "Estimator/Estimator.hpp"
#include "Planner/Planner.hpp"
#include "cpputils.hpp"
#include <csignal>
#include <chrono>
#include <iostream>
#include <thread>

bool terminate = false;

void signalHandler(int signum) {
    std::cout << "Interrupt signal (" << signum << ") received.\n";
    terminate = true;
}

int main(int argc, char** argv) {
    signal(SIGINT, signalHandler);

    YAML::Node config = YAML::LoadFile("/home/AIR/src/execution/config/config_c1.yaml");
    double pec_rate = config["pec_rate"] ? config["pec_rate"].as<double>() : 500.0;

    // Load robot from URDF
    std::shared_ptr<Cobot> robot = std::make_shared<Cobot>(
        "/home/AIR/src/robots/cobot_c1_description/urdf/cobot_c1_description.urdf");
    robot->showParams();

    // Create Estimator and Planner from config
    std::shared_ptr<Estimator> estimator = std::make_shared<Estimator>(config["estimator"]);
    std::shared_ptr<Planner> planner = std::make_shared<Planner>(config["planner"]);

    // Create RobotComponents with Estimator and Planner
    std::shared_ptr<RobotComponents> robot_components = std::make_shared<RobotComponents>(
        "cobot_c1", robot, estimator, planner, pec_rate);

    // Run calibration before main loop
    robot_components->runCalibration();

    double dt = 1.0 / pec_rate;
    double t = 0.0;

    while (!terminate) {
        robot_components->step(dt, t);
        t += dt;

        // Access and print estimation data
        const auto& est_data = robot_components->getEstimationData();
        // std::cout << "[JS est]: " << est_data.jv.transpose() << "\n";

        // Access and print planner state (you can adjust this as needed)
        const auto& planner_data = robot_components->getPlannerData();
        std::cout << "[Planner state x]: " << planner_data.x.transpose() << "\n";

        std::this_thread::sleep_for(std::chrono::milliseconds((int)(dt * 1000)));
    }

    return 0;
}
