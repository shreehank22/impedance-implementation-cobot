#pragma once

#include <iostream>
#include <string>
#include <ctime>
#include <iomanip>

#include "argparser.hpp"
#include "logger.hpp"
#include "timer.hpp"

#include "yaml-cpp/yaml.h"

inline std::string getCurrentDateTime() {
    // Get the current time
    auto now = std::time(nullptr);
    std::tm* localTime = std::localtime(&now);

    // Create a string stream to format the date and time
    std::ostringstream oss;
    oss << std::put_time(localTime, "%Y-%m-%d %H:%M:%S"); // Format: YYYY-MM-DD HH:MM:SS

    return oss.str();
}