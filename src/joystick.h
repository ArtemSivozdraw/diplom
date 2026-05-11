#pragma once
#include <string>
#include <vector>
#include <map>
#include <atomic>
#include <thread>
#include <mutex>
#include <windows.h>
#include <mmsystem.h>

struct ControlMap {
    bool is_axis;
    int index; 
    std::string name;
};

enum class JoyState { IDLE, CALIBRATING, CALIBRATED };

class JoystickManager {
public:
    static std::vector<std::string> listJoysticks();
    static void startCalibration(int joyId);
    static void nextCalibrationStep();
    static void skipCalibrationStep();
    static void cancelCalibration();
    static void stop();
    
    static bool isCalibrating();
    static bool isCalibrated();

private:
    static std::atomic<JoyState> state;
    static int currentJoyId;
    static int calibIndex;
    static std::vector<std::string> controlNames;
    static std::map<std::string, ControlMap> mapping;
    
    static int base_axes[6];
    static uint32_t base_btns;
    static int max_diff[6];
    static uint32_t changed_btns;
    static const std::string axis_names[6];
    
    static std::atomic<bool> isRunning;
    static std::thread joyThread;
    static std::mutex dataMutex;

    static void resetDeltasInternal();
    static void task();
    static uint16_t convertToCRSF(int raw_axis_val);
};