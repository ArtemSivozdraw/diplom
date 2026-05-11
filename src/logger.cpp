#include "logger.h"

std::vector<std::string> Logger::systemLogs;
std::mutex Logger::logMutex;

void Logger::addLog(const std::string& message) {
    std::lock_guard<std::mutex> lock(logMutex);
    systemLogs.push_back(message);
    if (systemLogs.size() > 45) { // Зберігаємо останні 45 логів
        systemLogs.erase(systemLogs.begin());
    }
}

std::vector<std::string> Logger::getLogs() {
    std::lock_guard<std::mutex> lock(logMutex);
    return systemLogs; // Повертає копію вектора для безпечного малювання
}