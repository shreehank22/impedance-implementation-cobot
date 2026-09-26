#include "cpputils.hpp"

int main(int argc, char** argv) {
    std::string filepath = "/home/dev/ql/src/utils/tests/config.yaml";
    YAML::Node config = YAML::LoadFile(filepath);
    if (config["name"]) {
        std::cout << "Name: " << config["name"].as<std::string>() << "\n";
    }
    if (config["controller"]) {
        std::cout << "Controller config found\n";
        if (config["controller"]["joint_kp"].IsSequence()) {
            std::vector<double> joint_kp;
            auto node = config["controller"]["joint_kp"];
            // for (auto& kp : ) {
            for (size_t i = 0; i < node.size(); ++i) {
                auto kp = node[i];
                joint_kp.push_back(kp.as<double>());
                std::cout << "Gain: " << kp.as<double>() << "\n";
            }
            // std::cout << "Joint gains: " << config["controller"]["joint_kp"].as<std::vector>() << "\n";
        }
    }
}