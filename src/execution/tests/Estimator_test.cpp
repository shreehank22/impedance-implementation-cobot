#include "StateMachine/RobotComponents.hpp"
#include "Estimator/Estimator.hpp"
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

    std::shared_ptr<Cobot> robot = std::make_shared<Cobot>(
        "/home/AIR/src/robots/cobot_c1_description/urdf/cobot_c1_description.urdf");
    robot->showParams();

    std::shared_ptr<Estimator> estimator = std::make_shared<Estimator>(config["estimator"]);

    std::shared_ptr<RobotComponents> robot_components = std::make_shared<RobotComponents>(
        "cobot_c1", robot, estimator, pec_rate);

    robot_components->runCalibration();

    double dt = 1.0 / pec_rate;
    double t = 0.0;

    while (!terminate) {
        robot_components->step(dt, t);
        t += dt;

        const auto& est_data = robot_components->getEstimationData();
        std::cout << "[JS est]: " << est_data.jv.transpose() << "\n";

        std::this_thread::sleep_for(std::chrono::milliseconds((int)(dt * 1000)));
    }

    return 0;
}
