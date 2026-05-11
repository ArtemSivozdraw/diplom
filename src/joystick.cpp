#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <mmsystem.h>
#include "joystick.h"
#include "logger.h"
#include <cmath>

#pragma comment(lib, "winmm.lib")

extern std::atomic<uint16_t> rc_channels[16];

std::atomic<JoyState> JoystickManager::state(JoyState::IDLE);
int JoystickManager::currentJoyId = -1;
int JoystickManager::calibIndex = 0;
std::vector<std::string> JoystickManager::controlNames = {
    "Roll", "Pitch", "Throttle", "Yaw", 
    "SA", "SB", "SC", "SD", "SE", "SF"
};
std::map<std::string, ControlMap> JoystickManager::mapping;

int JoystickManager::base_axes[6] = {0};
uint32_t JoystickManager::base_btns = 0;
int JoystickManager::max_diff[6] = {0};
uint32_t JoystickManager::changed_btns = 0;
const std::string JoystickManager::axis_names[6] = {"X", "Y", "Z", "R", "U", "V"};

std::atomic<bool> JoystickManager::isRunning(false);
std::thread JoystickManager::joyThread;
std::mutex JoystickManager::dataMutex;

uint16_t JoystickManager::convertToCRSF(int raw_axis_val) {
    return 172 + (raw_axis_val * (1811 - 172)) / 65535;
}

std::vector<std::string> JoystickManager::listJoysticks() {
    std::vector<std::string> list;
    int numDevs = joyGetNumDevs();
    JOYCAPSA jc;
    for (int i = 0; i < numDevs; i++) {
        if (joyGetDevCapsA(i, &jc, sizeof(jc)) == JOYERR_NOERROR) {
            list.push_back("[" + std::to_string(i) + "] " + std::string(jc.szPname));
        }
    }
    if (list.empty()) list.push_back("[SYS] No joysticks found.");
    return list;
}

void JoystickManager::startCalibration(int joyId) {
    if (isRunning) stop();
    
    JOYINFOEX ji; ji.dwSize = sizeof(JOYINFOEX); ji.dwFlags = JOY_RETURNALL;
    if (joyGetPosEx(joyId, &ji) != JOYERR_NOERROR) {
        Logger::addLog("[ERR] Controller not found. Connect it via USB.");
        return;
    }

    currentJoyId = joyId;
    calibIndex = 0;
    mapping.clear();
    state = JoyState::CALIBRATING;
    isRunning = true;
    
    dataMutex.lock();
    resetDeltasInternal();
    dataMutex.unlock();
    
    joyThread = std::thread(task);

    Logger::addLog("=== SMART CALIBRATION STARTED ===");
    Logger::addLog(">> Move: [" + controlNames[0] + "] and press ENTER.");
    Logger::addLog("(Type 'skip' or 'cancel' if needed)");
}

void JoystickManager::resetDeltasInternal() {
    JOYINFOEX ji; ji.dwSize = sizeof(JOYINFOEX); ji.dwFlags = JOY_RETURNALL;
    if (joyGetPosEx(currentJoyId, &ji) == JOYERR_NOERROR) {
        base_axes[0] = ji.dwXpos; base_axes[1] = ji.dwYpos; base_axes[2] = ji.dwZpos;
        base_axes[3] = ji.dwRpos; base_axes[4] = ji.dwUpos; base_axes[5] = ji.dwVpos;
        base_btns = ji.dwButtons;
    }
    for(int i=0; i<6; i++) max_diff[i] = 0;
    changed_btns = 0;
}

void JoystickManager::nextCalibrationStep() {
    if (state != JoyState::CALIBRATING) return;

    std::lock_guard<std::mutex> lock(dataMutex);
    int best_axis = -1;
    int max_val = 15000; 

    for (int i = 0; i < 6; i++) {
        if (max_diff[i] > max_val) { max_val = max_diff[i]; best_axis = i; }
    }

    std::string ctrlName = controlNames[calibIndex];

    if (best_axis != -1) {
        mapping[ctrlName] = {true, best_axis, axis_names[best_axis]};
        Logger::addLog("[OK] Mapped to axis: " + axis_names[best_axis]);
    } else if (changed_btns != 0) {
        bool found = false;
        for (int i = 0; i < 32; i++) {
            if (changed_btns & (1 << i)) {
                mapping[ctrlName] = {false, i, "BTN_" + std::to_string(i)};
                Logger::addLog("[OK] Mapped to button: " + std::to_string(i));
                found = true; break;
            }
        }
        if (!found) Logger::addLog("[ERR] No movement detected. Skipped.");
    } else {
        Logger::addLog("[ERR] No movement detected. Skipped.");
    }

    calibIndex++;
    if (calibIndex >= 10) {
        state = JoyState::CALIBRATED;
        Logger::addLog("=== CALIBRATION COMPLETE ===");
        Logger::addLog("[SYS] You can now connect as test_pilot.");
    } else {
        resetDeltasInternal();
        Logger::addLog(">> Move: [" + controlNames[calibIndex] + "] and press ENTER.");
    }
}

void JoystickManager::skipCalibrationStep() {
    if (state != JoyState::CALIBRATING) return;
    std::lock_guard<std::mutex> lock(dataMutex);
    Logger::addLog("[SKIPPED] " + controlNames[calibIndex]);
    calibIndex++;
    if (calibIndex >= 10) {
        state = JoyState::CALIBRATED;
        Logger::addLog("=== CALIBRATION COMPLETE ===");
    } else {
        resetDeltasInternal();
        Logger::addLog(">> Move: [" + controlNames[calibIndex] + "] and press ENTER.");
    }
}

void JoystickManager::cancelCalibration() {
    stop();
    Logger::addLog("[SYS] Calibration aborted.");
}

bool JoystickManager::isCalibrating() { return state == JoyState::CALIBRATING; }
bool JoystickManager::isCalibrated() { return state == JoyState::CALIBRATED; }

void JoystickManager::stop() {
    isRunning = false;
    state = JoyState::IDLE;
    if (joyThread.joinable()) joyThread.join();
}

void JoystickManager::task() {
    JOYINFOEX ji; ji.dwSize = sizeof(JOYINFOEX); ji.dwFlags = JOY_RETURNALL;
    while (isRunning) {
        if (joyGetPosEx(currentJoyId, &ji) != JOYERR_NOERROR) {
            Logger::addLog("[ERR] Controller disconnected!");
            state = JoyState::IDLE; isRunning = false; break;
        }

        int cur_axes[6] = {(int)ji.dwXpos, (int)ji.dwYpos, (int)ji.dwZpos, (int)ji.dwRpos, (int)ji.dwUpos, (int)ji.dwVpos};
        uint32_t cur_btns = ji.dwButtons;

        if (state == JoyState::CALIBRATING) {
            std::lock_guard<std::mutex> lock(dataMutex);
            for (int i = 0; i < 6; i++) {
                int diff = std::abs(cur_axes[i] - base_axes[i]);
                if (diff > max_diff[i]) max_diff[i] = diff;
            }
            changed_btns |= (cur_btns ^ base_btns);
        } 
        else if (state == JoyState::CALIBRATED) {
            uint16_t temp_rc[16];
            for(int i=0; i<16; i++) temp_rc[i] = 992;
            temp_rc[2] = 172; 
            
            int ch_idx = 0;
            for (const auto& ctrlName : controlNames) {
                if (mapping.count(ctrlName)) {
                    ControlMap m = mapping[ctrlName];
                    if (m.is_axis) { temp_rc[ch_idx] = convertToCRSF(cur_axes[m.index]); } 
                    else { temp_rc[ch_idx] = (cur_btns & (1 << m.index)) ? 1811 : 172; }
                }
                ch_idx++;
            }
            // Оновлюємо UI та мережу
            for(int i=0; i<16; i++) rc_channels[i].store(temp_rc[i]);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20)); // 50hz
    }
}