#include <csignal>
#include <filesystem>

#include "StateMachine/FSM.hpp"
#include "Controller/Controller.hpp"
#include "Estimator/Estimator.hpp"
#include "Planner/Planner.hpp"
#include "cpputils.hpp"

bool terminate = false;

void signalHandler(int signum) {
    std::cout << "Interrupt signal (" << signum << ") received.\n";

    terminate = true;
}

int main(int argc, char** argv) {
    ArgumentParser parser(argc, argv);

#if defined(USE_ROS2_COMM)
    std::cout << "Using ROS2...\n";
    rclcpp::init(argc, argv);
#elif defined(USE_ROS_COMM)
    std::cout << "Using ROS...\n";
    ros::init(argc, argv, "run", ros::init_options::NoSigintHandler);
#elif defined(USE_DDS_COMM)
    std::cout << "Using DDS...\n";
#else
    std::cout << "Using SHM...\n";
#endif

    // Register signal handler for Ctrl+C
    signal(SIGINT, signalHandler);

    std::string rel_config_path = "src/execution/config/";
    std::filesystem::path config_dir = std::filesystem::current_path().parent_path().parent_path() / rel_config_path;
    std::string config_filepath = config_dir.string();
    if (parser.has("config")) {
        config_filepath += parser.get("config");
    } else {
        config_filepath += "config_c1.yaml";
    }
    YAML::Node config = YAML::LoadFile(config_filepath);
    std::cout << "Config directory: " << config_filepath << std::endl;

    std::string logfile_name = getCurrentDateTime();
    if (parser.has("logfile")) {
        logfile_name += parser.get("logfile");
    }
    logfile_name += ".log";

    FileLogger logger(logfile_name);

    std::string robot_name = "cobot_c1";
    if (parser.has("robot")) {
        robot_name = parser.get("robot");
    }
    std::string rel_model_path = "src/robots/" + robot_name + "_description/urdf/" + robot_name + "_description.urdf";
    std::filesystem::path filepath = std::filesystem::current_path().parent_path().parent_path() / rel_model_path;
    std::string urdf_filepath = filepath.string();

    Cobot robot_obj(urdf_filepath);
    std::cout << "Successfully initialized Cobot object!\n";
    
    std::shared_ptr<Cobot> robot = std::make_shared<Cobot>(robot_obj);
    if (std::strcmp(robot_name.c_str(), "c1") == 0 || std::strcmp(robot_name.c_str(), "cobot_c1") == 0) {
        // robot->updateParams(0.4035, 0.12, 0.078, 0.1915, 0.188);
        robot->showParams();
    }

    std::shared_ptr<Estimator> estimator = std::make_shared<Estimator>(config["estimator"]);

    std::shared_ptr<Controller> controller = std::make_shared<Controller>(config["controller"]);

    std::shared_ptr<Planner> planner = std::make_shared<Planner>(config["planner"]);
    
    double pec_rate = 500.0;
    if (config["pec_rate"]) {
        pec_rate = config["pec_rate"].as<double>();
    }

    std::string comm_postfix = "/sim";  // default: simulation bridge
    if (config["comm_postfix"]) {
        std::string pf = config["comm_postfix"].as<std::string>();
        if (pf == "hw") comm_postfix = "/hw";
        else if (pf == "sim") comm_postfix = "/sim";
        else std::cout << "[run] Unknown comm_postfix '" << pf << "', defaulting to /sim\n";
    }
    std::cout << "[run] comm_postfix: " << comm_postfix << "\n";

    std::shared_ptr<RobotComponents> robot_components =
        std::make_shared<RobotComponents>(robot_name, comm_postfix, robot, estimator, planner, controller, pec_rate);

    std::unique_ptr<FSM> fsm = std::make_unique<FSM>(robot_components, config["fsm"]);
 
    fsm->run(terminate);

    return 0; // weird but ros should not shutdown before

#if defined(USE_ROS2_COMM)
    rclcpp::shutdown();
#elif defined(USE_ROS_COMM)
    ros::shutdown();
#endif

}
