// keyboard_interface_dds — unified keyboard + P2P waypoint controller for RIMaR
//
// Publishes:
//   JoyData_  → rt/c1/joystick_data
//   CliData_  → rt/<robot>/<postfix>/cli_data
//
// Controls (hold-to-move at 50 Hz):
//   1      SLEEP
//   2      TELEOP
//   3      P2P — prompts for EE target pose then executes trajectory
//   W/S    EE forward / backward
//   A/D    EE up / down
//   I/K    Wrist pitch +/-
//   J/L    Wrist roll +/-
//   Q/E    Base yaw left / right
//   Z/X    Jaw open / close
//   Space  Stop all axes
//   Ctrl+C Quit
//
// CLI args:
//   --postfix  sim|hw    (default: sim)
//   --robot    <name>    (default: cobot_c1)
//   --test               skip tcgetattr for piped-stdin testing

#include <array>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <limits>
#include <mutex>
#include <sstream>
#include <string>
#include <termios.h>
#include <thread>
#include <unistd.h>

#include "CliData.hpp"
#include "JoyData.hpp"
#include "dds_publisher.hpp"

using namespace xterra::msg::dds_;
using Clock     = std::chrono::steady_clock;
using TimePoint = std::chrono::time_point<Clock>;

static std::atomic<bool> g_running{true};
static void signalHandler(int) { g_running.store(false); }

struct AxisState {
    float     value    = 0.0f;
    TimePoint last_hit = Clock::now();
};

class KeyboardInterface {
public:
    KeyboardInterface(const std::string& robot,
                      const std::string& postfix,
                      bool               test_mode)
        : m_robot(robot), m_postfix(postfix),
          m_term_set(false), m_test_mode(test_mode) {}

    ~KeyboardInterface() { restoreTerminal(); }

    bool initialize() {
        m_joy_pub = std::make_unique<DDSPublisher<JoyData_>>("rt/c1/joystick_data", 0);
        if (!m_joy_pub) {
            std::cerr << "[keyboard] Failed to init JoyData publisher\n";
            return false;
        }

        std::string cli_topic = "rt/" + m_robot + "/" + m_postfix + "/cli_data";
        m_cli_pub = std::make_unique<DDSPublisher<CliData_>>(cli_topic, 0);
        if (!m_cli_pub) {
            std::cerr << "[keyboard] Failed to init CliData publisher on " << cli_topic << "\n";
            return false;
        }

        if (!m_test_mode) {
            if (tcgetattr(STDIN_FILENO, &m_old_term) != 0) {
                std::cerr << "[keyboard] tcgetattr failed\n";
                return false;
            }
            m_term_set = true;
            setRawMode();
            int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
            if (flags >= 0) fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
        }

        std::memset(m_buttons, 0, sizeof(m_buttons));

        std::cout << "[keyboard] Joy→ rt/c1/joystick_data  |  CLI→ " << cli_topic << "\n";
        printHelp();
        return true;
    }

    void run() {
        using namespace std::chrono;
        auto next_publish = Clock::now();

        while (g_running.load()) {
            if (m_p2p_prompt_requested.load()) {
                m_p2p_prompt_requested.store(false);
                runP2PPromptAndExecute();
            }

            auto now = Clock::now();
            if (now >= next_publish) {
                publishJoy();
                next_publish += milliseconds(kPeriodMs);
            }
            std::this_thread::sleep_for(milliseconds(2));
        }
    }

    void readLoop() {
        while (g_running.load()) {
            if (m_p2p_prompt_requested.load() || m_p2p_executing.load()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
                continue;
            }

            char ch = 0;
            ssize_t n = read(STDIN_FILENO, &ch, 1);
            if (n == 1) {
                switch (ch) {
                    case '1': setButton(0); break;
                    case '2': setButton(1); break;
                    case '3':
                        setButton(3);
                        m_p2p_executing.store(true);
                        m_p2p_prompt_requested.store(true);
                        break;
                    case 'w': case 'W': setAxis(1, -kVel); break;
                    case 's': case 'S': setAxis(1, +kVel); break;
                    case 'a': case 'A': setAxis(0, -kVel); break;
                    case 'd': case 'D': setAxis(0, +kVel); break;
                    case 'i': case 'I': setAxis(4, -kVel); break;
                    case 'k': case 'K': setAxis(4, +kVel); break;
                    case 'j': case 'J': setAxis(3, -kVel); break;
                    case 'l': case 'L': setAxis(3, +kVel); break;
                    case 'q': case 'Q': setAxis(2, -kVel); break;
                    case 'e': case 'E': setAxis(2, +kVel); break;
                    case 'z': case 'Z': setAxis(5, -kVel); break;
                    case 'x': case 'X': setAxis(5, +kVel); break;
                    case ' ':           stopAxes();         break;
                    case 3:             g_running.store(false); break;
                    default: break;
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }

private:
    std::unique_ptr<DDSPublisher<JoyData_>> m_joy_pub;
    std::unique_ptr<DDSPublisher<CliData_>> m_cli_pub;

    std::string m_robot;
    std::string m_postfix;

    struct termios m_old_term;
    bool           m_term_set;
    bool           m_test_mode;

    std::mutex m_mtx;
    AxisState  m_axes[6];
    uint8_t    m_buttons[12];

    std::atomic<bool> m_p2p_prompt_requested{false};
    std::atomic<bool> m_p2p_executing{false};

    static constexpr float kVel       = 0.6f;
    static constexpr int   kTimeoutMs = 80;
    static constexpr int   kPeriodMs  = 20;

    void runP2PPromptAndExecute() {
        m_p2p_executing.store(true);

        if (m_term_set) setCookedMode();

        std::cout << "\n╔══════════════════════════════════════════════════╗\n"
                  << "║  P2P — enter EE target pose                      ║\n"
                  << "║  x  y  z  pitch  roll  yaw  (metres / radians)   ║\n"
                  << "║  Workspace: x 0.22-0.52  y ±0.28  z 0.05-0.55   ║\n"
                  << "║  Press Enter with no input to cancel              ║\n"
                  << "╚══════════════════════════════════════════════════╝\n"
                  << "Pose> ";
        std::cout.flush();

        if (m_test_mode) {
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        }

        std::string line;
        std::getline(std::cin, line);

        if (line.empty()) {
            std::cout << "[keyboard] P2P cancelled\n";
            setButton(1);
            if (m_term_set) setRawMode();
            m_p2p_executing.store(false);
            return;
        }

        std::array<float, 6> pose{};
        std::istringstream ss(line);
        int parsed = 0;
        for (; parsed < 6; ++parsed) {
            if (!(ss >> pose[parsed])) break;
        }
        if (parsed < 3) {
            std::cout << "[keyboard] Need at least x y z (" << parsed << " given). Cancelled.\n";
            setButton(1);
            if (m_term_set) setRawMode();
            m_p2p_executing.store(false);
            return;
        }

        std::cout << "Duration (s, default 5.0)> ";
        std::cout.flush();
        std::string dur_line;
        std::getline(std::cin, dur_line);
        double duration = 5.0;
        if (!dur_line.empty()) {
            try { duration = std::stod(dur_line); } catch (...) {}
        }
        if (duration <= 0) duration = 5.0;

        std::cout << "[keyboard] P2P → ["
                  << pose[0] << " " << pose[1] << " " << pose[2] << " "
                  << pose[3] << " " << pose[4] << " " << pose[5]
                  << "]  " << duration << "s\n";
        std::cout.flush();

        if (m_term_set) setRawMode();

        CliData_ cli_msg;
        for (int i = 0; i < 6; ++i) cli_msg.x()[i] = pose[i];
        cli_msg.duration() = duration;
        cli_msg.mode()     = 3;

        auto deadline = Clock::now() + std::chrono::duration<double>(duration);
        while (Clock::now() < deadline && g_running.load()) {
            m_cli_pub->publish(cli_msg);
            publishJoyIdle();
            std::this_thread::sleep_for(std::chrono::milliseconds(kPeriodMs));
        }

        CliData_ release_msg;
        for (int i = 0; i < 6; ++i) release_msg.x()[i] = pose[i];
        release_msg.duration() = duration;
        release_msg.mode()     = 0;
        for (int i = 0; i < 5 && g_running.load(); ++i) {
            m_cli_pub->publish(release_msg);
            std::this_thread::sleep_for(std::chrono::milliseconds(kPeriodMs));
        }

        std::cout << "[keyboard] P2P done. Press 1=SLEEP  2=TELEOP  3=new P2P\n";
        std::cout.flush();

        m_p2p_executing.store(false);
    }

    void publishJoy() {
        using namespace std::chrono;
        auto now = Clock::now();

        JoyData_ msg;
        msg.priority() = 50;
        {
            std::lock_guard<std::mutex> lock(m_mtx);
            for (int i = 0; i < 6; ++i) {
                if (m_axes[i].value != 0.0f) {
                    auto age_ms = duration_cast<milliseconds>(now - m_axes[i].last_hit).count();
                    if (age_ms > kTimeoutMs) m_axes[i].value = 0.0f;
                }
                msg.axes()[i] = m_axes[i].value;
            }
            for (int i = 0; i < 12; ++i) {
                msg.buttons()[i] = m_buttons[i];
                m_buttons[i] = 0;
            }
        }
        m_joy_pub->publish(msg);
    }

    void publishJoyIdle() {
        JoyData_ msg;
        msg.priority() = 50;
        for (int i = 0; i < 6;  ++i) msg.axes()[i]   = 0;
        for (int i = 0; i < 12; ++i) msg.buttons()[i] = 0;
        m_joy_pub->publish(msg);
    }

    void setAxis(int idx, float val) {
        if (val >  1.0f) val =  1.0f;
        if (val < -1.0f) val = -1.0f;
        std::lock_guard<std::mutex> lock(m_mtx);
        m_axes[idx].value    = val;
        m_axes[idx].last_hit = Clock::now();
    }

    void setButton(int idx) {
        std::lock_guard<std::mutex> lock(m_mtx);
        m_buttons[idx] = 1;
    }

    void stopAxes() {
        std::lock_guard<std::mutex> lock(m_mtx);
        for (int i = 0; i < 6; ++i) m_axes[i].value = 0.0f;
    }

    void setRawMode() {
        struct termios raw = m_old_term;
        raw.c_lflag &= ~(ECHO | ICANON);
        raw.c_cc[VMIN]  = 0;
        raw.c_cc[VTIME] = 0;
        tcsetattr(STDIN_FILENO, TCSAFLUSH, &raw);
        int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
        if (flags >= 0) fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
    }

    void setCookedMode() {
        if (!m_term_set) return;
        tcsetattr(STDIN_FILENO, TCSAFLUSH, &m_old_term);
        int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
        if (flags >= 0) fcntl(STDIN_FILENO, F_SETFL, flags & ~O_NONBLOCK);
    }

    void restoreTerminal() {
        if (m_term_set) {
            tcsetattr(STDIN_FILENO, TCSAFLUSH, &m_old_term);
            m_term_set = false;
        }
    }

    void printHelp() {
        std::cout << "\nControls:\n"
                  << "  1      SLEEP\n"
                  << "  2      TELEOP\n"
                  << "  3      P2P (prompts for EE pose)\n"
                  << "  W/S    EE forward / backward\n"
                  << "  A/D    EE up / down\n"
                  << "  I/K    Wrist pitch +/-\n"
                  << "  J/L    Wrist roll +/-\n"
                  << "  Q/E    Base yaw left / right\n"
                  << "  Z/X    Jaw open / close\n"
                  << "  Space  Stop axes\n"
                  << "  Ctrl+C Quit\n\n";
        std::cout.flush();
    }
};

int main(int argc, char** argv) {
    std::string robot     = "cobot_c1";
    std::string postfix   = "sim";
    bool        test_mode = false;

    for (int i = 1; i < argc; ++i) {
        std::string arg(argv[i]);
        if (arg == "--robot"   && i + 1 < argc) robot     = argv[++i];
        if (arg == "--postfix" && i + 1 < argc) postfix   = argv[++i];
        if (arg == "--test")                    test_mode = true;
    }

    std::signal(SIGINT,  signalHandler);
    std::signal(SIGTERM, signalHandler);

    KeyboardInterface ki(robot, postfix, test_mode);
    if (!ki.initialize()) return 1;

    std::thread reader([&ki]() { ki.readLoop(); });
    ki.run();
    reader.join();
    return 0;
}
