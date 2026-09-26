#include <filesystem>

#include "low_interface_shm.hpp"

int main(int argc, char* argv[]) {
    LockMemory();

    ArgumentParser parser(argc, argv);
#if defined(USE_ROS2_COMM)
    // std::cout << "Using ROS2...\n";
    rclcpp::init(argc, argv);
#elif defined(USE_ROS_COMM)
    // std::cout << "Using ROS...\n";
    ros::init(argc, argv, "bridge_ros");
#endif

    std::string logfile_name = getCurrentDateTime();
    if (parser.has("logfile")) {
        logfile_name += parser.get("logfile");
    }
    logfile_name += ".log";

    std::shared_ptr<FileLogger> logger =
        std::make_shared<FileLogger>(logfile_name);

    // Overriding the DEFAULT logging level
    logger->setLevel(FileLogger::LEVEL::DEBUG);

    std::string config_filename = "";
    if (!parser.has("config")) {
        std::cout << "Config file not provided. Cannot proceed. Exiting...\n";
        return 1;
    } else {
        config_filename = parser.get("config");
    }

    std::string abs_config_path = "/home/xterra/RIMaR/src/hardware/config/";
    YAML::Node config = YAML::LoadFile(abs_config_path + config_filename);

    LowInterface interface(config, logger);
    interface.ReturnAttitude(true, false);
    interface.SetMountingAngles(0, -90, 0);
    interface.UseSlowCANFD(false);

    interface.Run();

    return 0;
}
