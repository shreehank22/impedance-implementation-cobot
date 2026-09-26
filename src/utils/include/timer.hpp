#pragma once

#include <chrono>
#include <iostream>
#include <thread>

class Timer {
    public:
        Timer() : Timer("") {
        }

        Timer(const char* name) : m_Name(name), m_Stopped(false) {
            m_StartTimePoint = std::chrono::high_resolution_clock::now();
        }

        ~Timer() {
            if (!m_Stopped) {
                Stop(); 
            }
        }

        void disable_auto_print() {
            m_AutoPrint = false;
        }

        double get_elapsed_time_ms() {
            auto currTimePont = std::chrono::high_resolution_clock::now();
            auto start = std::chrono::time_point_cast<std::chrono::microseconds>(m_StartTimePoint).time_since_epoch().count();
            auto end = std::chrono::time_point_cast<std::chrono::microseconds>(currTimePont).time_since_epoch().count();
            auto duration = end - start;

            double ms = duration * 0.001;
            return ms;
        }
    private:
        const char* m_Name;
        std::chrono::time_point<std::chrono::high_resolution_clock> m_StartTimePoint;
        bool m_Stopped;
        bool m_AutoPrint = true;

        void Stop() {
            auto endTimePont = std::chrono::high_resolution_clock::now();
            auto start = std::chrono::time_point_cast<std::chrono::microseconds>(m_StartTimePoint).time_since_epoch().count();
            auto end = std::chrono::time_point_cast<std::chrono::microseconds>(endTimePont).time_since_epoch().count();
            auto duration = end - start;

            double ms = duration * 0.001;
            if (m_AutoPrint) {
                std::cout << m_Name << ": " << duration << "us (" << ms << "ms)\n";
            }

            m_Stopped = true;
        }
};

// double get_wall_time_seconds() {
//     return std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now()).time_since_epoch().count() * double(1e-6);
// }

inline double get_wall_time_seconds(const std::chrono::time_point<std::chrono::high_resolution_clock>& startTimePoint = std::chrono::time_point<std::chrono::high_resolution_clock>()) {
    auto currTimePont = std::chrono::high_resolution_clock::now();
    auto start = std::chrono::time_point_cast<std::chrono::microseconds>(startTimePoint).time_since_epoch().count();
    auto curr = std::chrono::time_point_cast<std::chrono::microseconds>(currTimePont).time_since_epoch().count();
    auto duration = curr - start;

    return duration * double(1e-6);
}

inline void wait_ms(const int& delay_ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
}
inline void wait_us(const int& delay_us) {
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us));
}

inline void sleep_for_seconds(const double& sleep_sec) {
    int sleep_usec = 1e6 * sleep_sec;
    std::this_thread::sleep_for(std::chrono::microseconds(sleep_usec));
}