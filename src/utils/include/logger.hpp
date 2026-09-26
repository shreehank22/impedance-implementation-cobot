#pragma once

#include <iostream>
#include <fstream>
#include <string>
#include <mutex>

class FileLogger {
public:
    enum class LEVEL {
        DEBUG,
        INFO,
        WARN,
        ERROR
    };

    FileLogger(const std::string& filename)
        : level_(LEVEL::INFO) // default log level
    {
        logFile.open(filename, std::ios::out | std::ios::app);
        if (!logFile.is_open()) {
            std::cerr << "Could not open log file: " << filename << std::endl;
        }
    }

    ~FileLogger() {
        if (logFile.is_open()) {
            logFile.close();
        }
    }

    void log(const std::string& message, LEVEL msgLevel = LEVEL::INFO) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (logFile.is_open() && msgLevel >= level_) {
            logFile << "[" << levelToString(msgLevel) << "] " << message << std::endl;
        }
    }

    void setLevel(LEVEL level) {
        level_ = level;
    }

    LEVEL getLevel() const {
        return level_;
    }

private:
    std::string levelToString(LEVEL level) const {
        switch (level) {
            case LEVEL::DEBUG: return "DEBUG";
            case LEVEL::INFO:  return "INFO";
            case LEVEL::WARN:  return "WARN";
            case LEVEL::ERROR: return "ERROR";
            default:           return "UNKNOWN";
        }
    }

    std::ofstream logFile;
    std::mutex mutex_;
    LEVEL level_;
};
