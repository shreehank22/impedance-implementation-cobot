// p2p_interface_dds — Point-to-point waypoint publisher for RIMaR
//
// Publishes CliData_ to rt/<robot>/<postfix>/cli_data.
// Two modes of operation:
//
//   Interactive (default):
//     Prompts for EE target pose [x y z pitch roll yaw] + duration
//     each time you press Enter.  Waypoints execute one at a time.
//
//   File mode (--file <waypoints.yaml>):
//     Reads an ordered list of waypoints and executes them in sequence
//     automatically, waiting <duration> seconds between each.
//
// YAML waypoints file format:
//   waypoints:
//     - pose: [0.30, 0.00, 0.35, 0.0, 0.0, 0.0]   # x y z pitch roll yaw (m / rad)
//       duration: 3.0                                # seconds
//     - pose: [0.25, 0.05, 0.40, 0.0, 0.0, 0.0]
//       duration: 2.0
//
// CLI args:
//   --postfix  sim|hw       (default: sim)
//   --robot    <name>       (default: cobot_c1)
//   --file     <path>       (optional)
//   --no-sleep              skip sending mode=1 at the end

#include <array>
#include <atomic>
#include <chrono>
#include <csignal>
#include <filesystem>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "CliData.hpp"
#include "dds_publisher.hpp"

using namespace xterra::msg::dds_;
using Clock = std::chrono::steady_clock;

static std::atomic<bool> g_running{true};

static void signalHandler(int) { g_running.store(false); }

// ─────────────────────────────────────────────────────────────────────────────
struct Waypoint {
    std::array<float, 6> pose{};   // x y z pitch roll base_yaw (m / rad)
    double duration = 5.0;         // seconds
};

// ─────────────────────────────────────────────────────────────────────────────
class P2PInterface {
public:
    P2PInterface(const std::string& robot,
                 const std::string& postfix,
                 bool               no_sleep)
        : m_robot(robot), m_postfix(postfix), m_no_sleep(no_sleep) {}

    bool initialize() {
        std::string topic = "rt/" + m_robot + "/" + m_postfix + "/cli_data";
        m_pub = std::make_unique<DDSPublisher<CliData_>>(topic, 0);
        if (!m_pub) {
            std::cerr << "[p2p] Failed to initialize DDS publisher on " << topic << "\n";
            return false;
        }
        std::cout << "[p2p] Publishing to " << topic << "\n";
        return true;
    }

    // ── Run interactive mode ──────────────────────────────────────────────────
    void runInteractive() {
        printHelp();
        while (g_running.load()) {
            Waypoint wp;
            if (!promptWaypoint(wp)) break;
            executeWaypoint(wp);
            if (!askContinue()) break;
        }
        if (!m_no_sleep) sendSleep();
        std::cout << "[p2p] Done.\n";
    }

    // ── Run file mode ─────────────────────────────────────────────────────────
    void runFile(const std::vector<Waypoint>& waypoints) {
        std::cout << "[p2p] Executing " << waypoints.size() << " waypoints...\n";
        for (size_t i = 0; i < waypoints.size() && g_running.load(); ++i) {
            std::cout << "[p2p] Waypoint " << (i + 1) << "/" << waypoints.size()
                      << "  pose=[" << waypoints[i].pose[0] << " "
                      << waypoints[i].pose[1] << " " << waypoints[i].pose[2] << " "
                      << waypoints[i].pose[3] << " " << waypoints[i].pose[4] << " "
                      << waypoints[i].pose[5] << "]"
                      << "  duration=" << waypoints[i].duration << "s\n";
            executeWaypoint(waypoints[i]);
        }
        if (!m_no_sleep && g_running.load()) sendSleep();
        std::cout << "[p2p] All waypoints complete.\n";
    }

private:
    std::unique_ptr<DDSPublisher<CliData_>> m_pub;
    std::string m_robot;
    std::string m_postfix;
    bool        m_no_sleep;

    // Publish at ~50 Hz for the full duration, then stop.
    void executeWaypoint(const Waypoint& wp) {
        CliData_ msg;
        for (int i = 0; i < 6; ++i) msg.x()[i] = wp.pose[i];
        msg.duration() = wp.duration;
        msg.mode()     = 3;   // P2P

        auto deadline = Clock::now() + std::chrono::duration<double>(wp.duration);
        while (Clock::now() < deadline && g_running.load()) {
            m_pub->publish(msg);
            std::this_thread::sleep_for(std::chrono::milliseconds(20));  // 50 Hz
        }
    }

    // Send mode=1 once to ask the FSM to return to SLEEP.
    void sendSleep() {
        CliData_ msg;
        msg.x()       = {};
        msg.duration() = 0;
        msg.mode()     = 1;   // SLEEP
        // Publish a few times so the executor doesn't miss it.
        for (int i = 0; i < 10 && g_running.load(); ++i) {
            m_pub->publish(msg);
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        std::cout << "[p2p] Sent SLEEP command.\n";
    }

    bool promptWaypoint(Waypoint& wp) {
        std::cout << "\nEnter EE target pose (x y z pitch roll yaw) in metres/radians:\n> ";
        std::string line;
        if (!std::getline(std::cin, line)) return false;
        if (line == "q" || line == "quit") return false;

        std::istringstream ss(line);
        for (int i = 0; i < 6; ++i) {
            if (!(ss >> wp.pose[i])) {
                std::cerr << "[p2p] Need 6 values. Got fewer.\n";
                return true;  // re-prompt
            }
        }

        std::cout << "Enter duration (seconds, default 5.0):\n> ";
        if (!std::getline(std::cin, line)) return false;
        if (!line.empty()) {
            try { wp.duration = std::stod(line); }
            catch (...) { wp.duration = 5.0; }
        }
        return true;
    }

    bool askContinue() {
        std::cout << "\nSend another waypoint? [Y/n]: ";
        std::string line;
        if (!std::getline(std::cin, line)) return false;
        return line.empty() || line[0] == 'y' || line[0] == 'Y';
    }

    void printHelp() {
        std::cout << "\n[p2p] Interactive P2P interface\n"
                  << "  Enter EE pose: x y z pitch roll yaw (metres / radians)\n"
                  << "  Cobot starts at approx [0.20 0.00 0.182 0 0 0]\n"
                  << "  Safe workspace: x=[0.15-0.60], z=[0.19-0.75], y=[-0.10, 0.10]\n"
                  << "  Type 'q' or Ctrl+C to quit.\n";
    }
};

// ─────────────────────────────────────────────────────────────────────────────
static std::vector<Waypoint> loadWaypointsFromFile(const std::string& path) {
    std::vector<Waypoint> waypoints;

    if (!std::filesystem::exists(path)) {
        std::cerr << "[p2p] Waypoints file not found: " << path << "\n";
        return waypoints;
    }

    YAML::Node root = YAML::LoadFile(path);
    if (!root["waypoints"] || !root["waypoints"].IsSequence()) {
        std::cerr << "[p2p] Expected 'waypoints:' sequence in " << path << "\n";
        return waypoints;
    }

    for (const auto& node : root["waypoints"]) {
        Waypoint wp;
        if (node["pose"] && node["pose"].IsSequence() && node["pose"].size() == 6) {
            for (int i = 0; i < 6; ++i)
                wp.pose[i] = node["pose"][i].as<float>();
        } else {
            std::cerr << "[p2p] Waypoint missing 'pose' with 6 elements — skipping.\n";
            continue;
        }
        if (node["duration"])
            wp.duration = node["duration"].as<double>();
        waypoints.push_back(wp);
    }

    std::cout << "[p2p] Loaded " << waypoints.size()
              << " waypoints from " << path << "\n";
    return waypoints;
}

// ─────────────────────────────────────────────────────────────────────────────
int main(int argc, char** argv) {
    std::string robot    = "cobot_c1";
    std::string postfix  = "sim";
    std::string filepath = "";
    bool        no_sleep = false;

    for (int i = 1; i < argc; ++i) {
        std::string arg(argv[i]);
        if (arg == "--robot"   && i + 1 < argc) robot    = argv[++i];
        if (arg == "--postfix" && i + 1 < argc) postfix  = argv[++i];
        if (arg == "--file"    && i + 1 < argc) filepath = argv[++i];
        if (arg == "--no-sleep")                no_sleep = true;
        if (arg == "--help") {
            std::cout << "Usage: p2p_interface_dds [--robot <name>] [--postfix sim|hw]"
                         " [--file <waypoints.yaml>] [--no-sleep]\n"
                      << "\nWaypoints YAML format:\n"
                      << "  waypoints:\n"
                      << "    - pose: [x, y, z, pitch, roll, yaw]\n"
                      << "      duration: 3.0\n";
            return 0;
        }
    }

    std::signal(SIGINT,  signalHandler);
    std::signal(SIGTERM, signalHandler);

    P2PInterface iface(robot, postfix, no_sleep);
    if (!iface.initialize()) return 1;

    if (filepath.empty()) {
        iface.runInteractive();
    } else {
        auto waypoints = loadWaypointsFromFile(filepath);
        if (waypoints.empty()) {
            std::cerr << "[p2p] No valid waypoints — exiting.\n";
            return 1;
        }
        iface.runFile(waypoints);
    }

    return 0;
}
