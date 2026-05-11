#pragma once
#include <string>
#include <vector>
#include <mutex>

class Logger {
public:
    static void addLog(const std::string& message);
    static std::vector<std::string> getLogs();

private:
    static std::vector<std::string> systemLogs;
    static std::mutex logMutex;
};