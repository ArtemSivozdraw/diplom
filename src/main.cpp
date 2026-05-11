#define NOMINMAX
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <chrono>
#include <atomic>
#include <sstream>
#include <algorithm>
#include <windows.h>
#include "logger.h"
#include "camera.h"
#include "serial_rx.h"
#include "network.h"
#include "joystick.h"

// Глобальний масив каналів
std::atomic<uint16_t> rc_channels[16];

// Допоміжна функція для переносу тексту в логах
std::vector<std::string> wrapText(const std::string& text, int maxWidth, double fontScale, int thickness) {
    std::vector<std::string> wrappedLines;
    std::string currentLine = "";
    int baseline = 0;
    for (char c : text) {
        std::string testLine = currentLine + c;
        cv::Size textSize = cv::getTextSize(testLine, cv::FONT_HERSHEY_SIMPLEX, fontScale, thickness, &baseline);
        if (textSize.width > maxWidth && !currentLine.empty()) {
            wrappedLines.push_back(currentLine);
            currentLine = std::string(1, c);
        } else {
            currentLine += c;
        }
    }
    if (!currentLine.empty()) wrappedLines.push_back(currentLine);
    return wrappedLines;
}

// Функція запису в буфер обміну Windows
void setClipboardText(const std::string& text) {
    if (OpenClipboard(nullptr)) {
        EmptyClipboard();
        HGLOBAL hMem = GlobalAlloc(GMEM_MOVEABLE, text.size() + 1);
        if (hMem != nullptr) {
            memcpy(GlobalLock(hMem), text.c_str(), text.size() + 1);
            GlobalUnlock(hMem);
            SetClipboardData(CF_TEXT, hMem);
        }
        CloseClipboard();
    }
}

// Функція читання з буфера обміну Windows (Ctrl+V)
std::string getClipboardText() {
    std::string text = "";
    if (OpenClipboard(nullptr)) {
        HANDLE hData = GetClipboardData(CF_TEXT);
        if (hData != nullptr) {
            char* pszText = static_cast<char*>(GlobalLock(hData));
            if (pszText != nullptr) {
                text = pszText;
                GlobalUnlock(hData);
            }
        }
        CloseClipboard();
    }
    text.erase(std::remove(text.begin(), text.end(), '\n'), text.end());
    text.erase(std::remove(text.begin(), text.end(), '\r'), text.end());
    return text;
}

int main() {
    const int WINDOW_WIDTH = 1400;
    const int WINDOW_HEIGHT = 720;
    const int LEFT_PANEL_W = 300;
    const int RIGHT_PANEL_W = 300;
    const int INPUT_BOX_H = 60;
    const int VIDEO_MAX_HEIGHT = 600; 

    const std::string chNames[16] = {
        "Roll", "Pitch", "Throttle", "Yaw", 
        "ARM (AUX1)", "AUX2", "AUX3", "MODE (AUX4)", 
        "AUX5", "AUX6", "AUX7", "AUX8", 
        "AUX9", "AUX10", "AUX11", "AUX12"
    };

    // Ініціалізація значень каналів за замовчуванням
    for(int i = 0; i < 16; i++) rc_channels[i].store(992);
    rc_channels[2].store(172); // Газ в нуль
    rc_channels[4].store(172); // Disarm
    rc_channels[7].store(172); // Angle Mode

    Logger::addLog("[SYSTEM] Application started.");

    std::vector<std::string> commandHistory;
    std::vector<std::string> rawHistory;
    int historyIndex = 0;
    std::string currentCommand = "";
    int cursorPosition = 0;
    bool isAppRunning = true;

    try {
        cv::namedWindow("Drone Control Client", cv::WINDOW_AUTOSIZE);

        while (isAppRunning) {
            if (cv::getWindowProperty("Drone Control Client", cv::WND_PROP_VISIBLE) < 1) {
                Logger::addLog("[SYSTEM] Window closed by user.");
                isAppRunning = false;
                break;
            }

            cv::Mat canvas(WINDOW_HEIGHT, WINDOW_WIDTH, CV_8UC3, cv::Scalar(0, 0, 0));
            cv::Mat frameCopy;
            
            // Отримання кадру залежно від ролі
            if (NetworkManager::getRole() == ClientRole::TEST_PILOT) {
                NetworkManager::getPilotFrame(frameCopy);
            } else {
                CameraManager::getFrame(frameCopy);
            }
            std::vector<std::string> logsCopy = Logger::getLogs();

            // --- 1. ВІДЕО ТА ПАНЕЛІ ---
            int currentVideoHeight = 0;
            if (!frameCopy.empty()) {
                currentVideoHeight = frameCopy.rows;
                cv::Rect roi(LEFT_PANEL_W, 0, frameCopy.cols, frameCopy.rows);
                frameCopy.copyTo(canvas(roi));
            } else {
                currentVideoHeight = VIDEO_MAX_HEIGHT;
            }

            cv::line(canvas, cv::Point(LEFT_PANEL_W, 0), cv::Point(LEFT_PANEL_W, WINDOW_HEIGHT), cv::Scalar(255, 255, 255), 1);
            cv::line(canvas, cv::Point(WINDOW_WIDTH - RIGHT_PANEL_W, 0), cv::Point(WINDOW_WIDTH - RIGHT_PANEL_W, WINDOW_HEIGHT), cv::Scalar(255, 255, 255), 1);
            cv::line(canvas, cv::Point(LEFT_PANEL_W, currentVideoHeight), cv::Point(WINDOW_WIDTH - RIGHT_PANEL_W, currentVideoHeight), cv::Scalar(255, 255, 255), 1);
            

            std::string roleStr = (NetworkManager::getRole() == ClientRole::DRONE) ? "[MODE: DRONE]" : 
                                  (NetworkManager::getRole() == ClientRole::TEST_PILOT) ? "[MODE: TEST_PILOT]" : "[MODE: OFFLINE]";
            
            // Зчитуємо RTT з мережевого модуля, якщо є з'єднання
            std::string rttStr = "";
            if (NetworkManager::isConnected()) {
                rttStr = "   |   RTT: " + std::to_string(NetworkManager::getRTT()) + " ms";
            }
            
            cv::putText(canvas, "RC CHANNELS LIVE " + roleStr + rttStr, cv::Point(LEFT_PANEL_W + 10, currentVideoHeight + 20), cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(255, 255, 255), 1);
            
            // --- 2. ВІДОБРАЖЕННЯ КАНАЛІВ (Смужки та назви) ---
            int chStartX = LEFT_PANEL_W + 10;
            int chStartY = currentVideoHeight + 40;
            int colWidth = 195; 

            for (int i = 0; i < 16; i++) {
                int col = i % 4, row = i / 4;
                int x = chStartX + col * colWidth, y = chStartY + row * 20;
                uint16_t val = rc_channels[i].load();
                float pct = (val - 172.0f) / (1811.0f - 172.0f);
                pct = std::max(0.0f, std::min(1.0f, pct));
                int barWidth = 60, filledWidth = static_cast<int>(pct * barWidth);

                cv::putText(canvas, chNames[i] + ":", cv::Point(x, y), cv::FONT_HERSHEY_SIMPLEX, 0.35, cv::Scalar(200, 200, 200), 1);
                cv::rectangle(canvas, cv::Rect(x + 85, y - 8, barWidth, 10), cv::Scalar(100, 100, 100), 1);
                if (filledWidth > 0) cv::rectangle(canvas, cv::Rect(x + 85, y - 8, filledWidth, 10), cv::Scalar(255, 255, 255), cv::FILLED);
                cv::putText(canvas, std::to_string(static_cast<int>(pct * 100)) + "%", cv::Point(x + 150, y), cv::FONT_HERSHEY_SIMPLEX, 0.35, cv::Scalar(255, 255, 255), 1);
            }

            // --- 3. ЛОГИ (Знизу вгору) ---
            cv::Mat rightPanel = canvas(cv::Rect(WINDOW_WIDTH - RIGHT_PANEL_W, 0, RIGHT_PANEL_W, WINDOW_HEIGHT));
            cv::putText(rightPanel, "SYSTEM LOGS", cv::Point(10, 25), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
            cv::line(rightPanel, cv::Point(0, 35), cv::Point(RIGHT_PANEL_W, 35), cv::Scalar(255, 255, 255), 1);
            
            int logY = WINDOW_HEIGHT - 20; 
            for (auto it = logsCopy.rbegin(); it != logsCopy.rend(); ++it) {
                if (logY < 50) break;
                std::vector<std::string> wrappedLogs = wrapText(*it, RIGHT_PANEL_W - 20, 0.4, 1);
                for (auto rit = wrappedLogs.rbegin(); rit != wrappedLogs.rend(); ++rit) {
                    if (logY < 50) break;
                    cv::putText(rightPanel, *rit, cv::Point(10, logY), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255), 1);
                    logY -= 15;
                }
            }

            // --- 4. ТЕРМІНАЛ ТА ІСТОРІЯ ---
            cv::Mat leftPanel = canvas(cv::Rect(0, 0, LEFT_PANEL_W, WINDOW_HEIGHT));
            cv::rectangle(leftPanel, cv::Rect(0, WINDOW_HEIGHT - INPUT_BOX_H + 2, 58, INPUT_BOX_H - 4), cv::Scalar(0, 0, 0), cv::FILLED);
            cv::rectangle(canvas, cv::Rect(0, WINDOW_HEIGHT - INPUT_BOX_H, LEFT_PANEL_W, INPUT_BOX_H), cv::Scalar(255, 255, 255), 1);
            cv::putText(leftPanel, "Input:", cv::Point(10, WINDOW_HEIGHT - 35), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);

            std::string displayCmd = currentCommand;
            std::string cursorStr = ((std::chrono::system_clock::now().time_since_epoch().count() / 5000000) % 2 == 0) ? "|" : " ";
            if (cursorPosition >= 0 && cursorPosition <= (int)displayCmd.length()) displayCmd.insert(cursorPosition, cursorStr);

            std::string visibleCmd = displayCmd;
            int baseline = 0, maxTextWidth = LEFT_PANEL_W - 70; 
            while (true) {
                cv::Size textSize = cv::getTextSize(visibleCmd, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);
                if (textSize.width > maxTextWidth && !visibleCmd.empty()) visibleCmd.erase(0, 1); else break;
            }
            cv::putText(leftPanel, visibleCmd, cv::Point(60, WINDOW_HEIGHT - 35), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);

            cv::putText(leftPanel, "COMMAND HISTORY", cv::Point(10, 25), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
            cv::line(leftPanel, cv::Point(0, 35), cv::Point(LEFT_PANEL_W, 35), cv::Scalar(255, 255, 255), 1);

            int historyY = WINDOW_HEIGHT - INPUT_BOX_H - 20;
            for (auto it = commandHistory.rbegin(); it != commandHistory.rend(); ++it) {
                if (historyY < 50) break;
                std::vector<std::string> wrappedCmds = wrapText(*it, LEFT_PANEL_W - 20, 0.5, 1);
                for (auto rit = wrappedCmds.rbegin(); rit != wrappedCmds.rend(); ++rit) {
                    if (historyY < 50) break;
                    cv::putText(leftPanel, *rit, cv::Point(10, historyY), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
                    historyY -= 25; 
                }
            }

            cv::imshow("Drone Control Client", canvas);

            // --- 5. ОБРОБКА КЛАВІАТУРИ ТА ПАРСЕР ---
            int key = cv::waitKeyEx(15); 
            if (key != -1) {
                if (key == 2490368 || key == 65362) { // ВГОРУ
                    if (!rawHistory.empty() && historyIndex > 0) {
                        historyIndex--; currentCommand = rawHistory[historyIndex]; cursorPosition = currentCommand.length();
                    }
                } 
                else if (key == 2621440 || key == 65364) { // ВНИЗ
                    if (!rawHistory.empty() && historyIndex < (int)rawHistory.size() - 1) {
                        historyIndex++; currentCommand = rawHistory[historyIndex]; cursorPosition = currentCommand.length();
                    } else if (historyIndex == (int)rawHistory.size() - 1) {
                        historyIndex++; currentCommand = ""; cursorPosition = 0;
                    }
                } 
                else if (key == 2424832 || key == 65361) { // ВЛІВО
                    if (cursorPosition > 0) cursorPosition--;
                }
                else if (key == 2555904 || key == 65363) { // ВПРАВО
                    if (cursorPosition < (int)currentCommand.length()) cursorPosition++;
                }
                else {
                    int asciiCode = key & 0xFF; 
                    if (asciiCode == 27) isAppRunning = false;
                    else if (asciiCode == 13 || asciiCode == 10) { // ENTER
                        // --- НОВИЙ БЛОК ДЛЯ КАЛІБРУВАННЯ ---
                        if (JoystickManager::isCalibrating()) {
                            if (currentCommand.empty()) {
                                JoystickManager::nextCalibrationStep();
                                continue;
                            } else if (currentCommand == "skip") {
                                JoystickManager::skipCalibrationStep();
                                currentCommand = ""; cursorPosition = 0;
                                continue;
                            } else if (currentCommand == "cancel") {
                                JoystickManager::cancelCalibration();
                                currentCommand = ""; cursorPosition = 0;
                                continue;
                            }
                        }
                        if (!currentCommand.empty()) {
                            commandHistory.push_back("> " + currentCommand);
                            rawHistory.push_back(currentCommand);
                            historyIndex = rawHistory.size();
                            Logger::addLog("[CMD] " + currentCommand);
                            
                            std::vector<std::string> args;
                            std::stringstream ss(currentCommand);
                            std::string arg;
                            while (ss >> arg) args.push_back(arg);

                            if (!args.empty()) {
                                std::string cmd = args[0];
                                
                                if (cmd == "help") {
                                    Logger::addLog("=== SYSTEM COMMANDS MANUAL ===");
                                    Logger::addLog("help       - Show this manual.");
                                    Logger::addLog("cam        - Video input (Usage: cam list | cam start <ID> | cam stop)");
                                    Logger::addLog("serial     - Telemetry RX (Usage: serial start <PORT> <BAUD> | serial stop)");
                                    Logger::addLog("joy        - RC Controller (Usage: joy list | joy calib <ID>)");
                                    Logger::addLog("connect    - Establish network link. Usage:");
                                    Logger::addLog("             > connect drone <IP:PORT>");
                                    Logger::addLog("             > connect pilot <IP:PORT> <DRONE_ID>");
                                    Logger::addLog("             > connect test_pilot <IP:PORT> <DRONE_ID>");
                                    Logger::addLog("cpid       - Copy your local network ID to clipboard.");
                                    Logger::addLog("arm        - Arm the drone motors (Pilot only).");
                                    Logger::addLog("disarm     - Disarm the drone motors (Pilot only).");
                                    Logger::addLog("mode       - Change flight mode (Usage: mode angle|horizon|acro)");
                                    Logger::addLog("disconnect - Close current network connection.");
                                    Logger::addLog("stop       - Kill all active modules (Cam, Serial, Net, Joy).");
                                }
                                else if (cmd == "cam") {
                                    if (args.size() > 1 && args[1] == "list") {
                                        Logger::addLog("[SYS] Scanning video devices...");
                                        std::vector<std::string> cams = CameraManager::listCameras();
                                        for(const auto& cam : cams) Logger::addLog("  " + cam);
                                    } else if (args.size() > 2 && args[1] == "start") {
                                        int camId = std::stoi(args[2]); CameraManager::stop(); CameraManager::start(camId);
                                    } else if (args.size() > 1 && args[1] == "stop") {
                                        CameraManager::stop();
                                    } else {
                                        Logger::addLog("[ERR] Usage: cam list | cam start <ID> | cam stop");
                                    }
                                } 
                                else if (cmd == "serial") {
                                    if (args.size() > 3 && args[1] == "start") {
                                        SerialRX::stop(); SerialRX::start(args[2], std::stoi(args[3]));
                                    } else if (args.size() > 1 && args[1] == "stop") {
                                        SerialRX::stop();
                                    } else {
                                        Logger::addLog("[ERR] Usage: serial start <PORT> <BAUD> | serial stop");
                                    }
                                } 
                                else if (cmd == "connect") {
                                    if (args.size() > 2 && args[1] == "drone") {
                                        if (!CameraManager::isCameraActive()) {
                                            Logger::addLog("[ERR] Connection blocked: Camera is not running.");
                                        } else if (!SerialRX::isTelemetryActive()) {
                                            Logger::addLog("[ERR] Connection blocked: No RC telemetry from serial.");
                                        } else {
                                            size_t cp = args[2].find(':');
                                            if (cp != std::string::npos) {
                                                NetworkManager::connectAsDrone(args[2].substr(0, cp), std::stoi(args[2].substr(cp + 1)));
                                            } else {
                                                Logger::addLog("[ERR] Invalid format. Expected: connect drone <IP:PORT>");
                                            }
                                        }
                                    } else if (args.size() > 3 && args[1] == "pilot") {
                                        if (!JoystickManager::isCalibrated()) {
                                            Logger::addLog("[ERR] Connection blocked: Joystick not calibrated.");
                                            Logger::addLog("[SYS] Connect remote and run: 'joy list' -> 'joy calib <ID>'.");
                                        } else {
                                            size_t cp = args[2].find(':');
                                            if (cp != std::string::npos) {
                                                NetworkManager::connectAsPilot(args[2].substr(0, cp), std::stoi(args[2].substr(cp + 1)), args[3]);
                                            } else {
                                                Logger::addLog("[ERR] Invalid format. Expected: connect pilot <IP:PORT> <DRONE_ID>");
                                            }
                                        }
                                    } else if (args.size() > 3 && args[1] == "test_pilot") {
                                        size_t cp = args[2].find(':');
                                        if (cp != std::string::npos) {
                                            NetworkManager::connectAsPilot(args[2].substr(0, cp), std::stoi(args[2].substr(cp + 1)), args[3]);
                                        } else {
                                            Logger::addLog("[ERR] Invalid format. Expected: connect test_pilot <IP:PORT> <DRONE_ID>");
                                        }
                                    } else {
                                        Logger::addLog("[ERR] Usage: connect drone <IP:PORT> | connect [test_]pilot <IP:PORT> <ID>");
                                    } 
                                }
                                else if (cmd == "joy") {
                                    if (args.size() > 1 && args[1] == "list") {
                                        Logger::addLog("[SYS] Scanning joysticks...");
                                        std::vector<std::string> joys = JoystickManager::listJoysticks();
                                        for(const auto& j : joys) Logger::addLog("  " + j);
                                    } else if (args.size() > 2 && args[1] == "calib") {
                                        JoystickManager::startCalibration(std::stoi(args[2]));
                                    } else {
                                        Logger::addLog("[ERR] Usage: joy list | joy calib <ID>");
                                    }
                                }
                                else if (cmd == "cpid") {
                                    std::string id = NetworkManager::getMyID();
                                    if (!id.empty()) {
                                        setClipboardText(id);
                                        Logger::addLog("[SYS] My ID " + id + " copied to clipboard.");
                                    } else {
                                        Logger::addLog("[ERR] Not connected to server. No ID assigned yet.");
                                    }
                                }
                                else if (cmd == "arm") {
                                    if (NetworkManager::getRole() == ClientRole::TEST_PILOT || NetworkManager::getRole() == ClientRole::NONE) {
                                        rc_channels[4].store(1811);
                                        Logger::addLog("[PILOT] Sent: ARM");
                                    } else {
                                        Logger::addLog("[ERR] Command only available for pilot.");
                                    }
                                }
                                else if (cmd == "disarm") {
                                    if (NetworkManager::getRole() == ClientRole::TEST_PILOT || NetworkManager::getRole() == ClientRole::NONE) {
                                        rc_channels[4].store(172);
                                        Logger::addLog("[PILOT] Sent: DISARM");
                                    } else {
                                        Logger::addLog("[ERR] Command only available for pilot.");
                                    }
                                }
                                else if (cmd == "mode") {
                                    if ((NetworkManager::getRole() == ClientRole::TEST_PILOT || NetworkManager::getRole() == ClientRole::NONE) && args.size() > 1) {
                                        if (args[1] == "angle") { rc_channels[7].store(172); Logger::addLog("[PILOT] Set Mode: ANGLE"); }
                                        else if (args[1] == "horizon") { rc_channels[7].store(992); Logger::addLog("[PILOT] Set Mode: HORIZON"); }
                                        else if (args[1] == "acro") { rc_channels[7].store(1811); Logger::addLog("[PILOT] Set Mode: ACRO"); }
                                        else { Logger::addLog("[ERR] Unknown mode. Usage: mode angle|horizon|acro"); }
                                    } else {
                                        Logger::addLog("[ERR] Usage: mode <angle|horizon|acro>");
                                    }
                                }
                                else if (cmd == "disconnect") {
                                    NetworkManager::disconnect();
                                }
                                else if (cmd == "stop") { 
                                    NetworkManager::disconnect(); CameraManager::stop(); SerialRX::stop(); JoystickManager::stop(); 
                                }
                                else {
                                    Logger::addLog("[ERR] Unknown command: " + cmd + ". Type 'help' for manual.");
                                }
                            }
                            currentCommand = ""; cursorPosition = 0;
                        }
                    } else if (asciiCode == 8) { // BACKSPACE
                        if (cursorPosition > 0 && !currentCommand.empty()) {
                            currentCommand.erase(cursorPosition - 1, 1); cursorPosition--;
                        }
                    } else if (asciiCode == 22) { // CTRL+V
                        std::string pt = getClipboardText();
                        currentCommand.insert(cursorPosition, pt); cursorPosition += pt.length();
                    } else if (asciiCode >= 32 && asciiCode <= 126) {
                        currentCommand.insert(cursorPosition, 1, (char)asciiCode); cursorPosition++;
                    }
                }
            }
        }
    } catch (const std::exception& e) { std::cerr << "Error: " << e.what() << std::endl; }
    JoystickManager::stop(); NetworkManager::disconnect(); SerialRX::stop(); CameraManager::stop(); cv::destroyAllWindows();
    return 0;
}