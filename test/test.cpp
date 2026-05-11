#include <iostream>
#include <vector>
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>
#include <conio.h>
#include <windows.h>
#include <algorithm>

// CRSF Protocol Constants
const uint8_t CRSF_ADDRESS_FLIGHT_CONTROLLER = 0xC8;
const uint8_t CRSF_FRAMETYPE_RC_CHANNELS_PACKED = 0x16;

// Global atomic array for 16 RC channels (Thread-safe)
std::atomic<uint16_t> rc_channels[16];

// CRC8 calculation (Polynomial 0xD5)
uint8_t crsf_crc8(const uint8_t* data, uint8_t len) {
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x80) crc = (crc << 1) ^ 0xD5;
            else crc <<= 1;
        }
    }
    return crc;
}

// Windows Serial Port Interface
class SerialPort {
private:
    HANDLE hSerial;
    std::mutex mtx;

public:
    SerialPort(const char* portName, int baudrate) {
        hSerial = CreateFileA(portName, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
        if (hSerial == INVALID_HANDLE_VALUE) {
            throw std::runtime_error("Critical: Failed to open serial port.");
        }

        DCB dcbSerialParams = { 0 };
        dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
        GetCommState(hSerial, &dcbSerialParams);
        dcbSerialParams.BaudRate = baudrate;
        dcbSerialParams.ByteSize = 8;
        dcbSerialParams.StopBits = ONESTOPBIT;
        dcbSerialParams.Parity = NOPARITY;
        SetCommState(hSerial, &dcbSerialParams);

        COMMTIMEOUTS timeouts = { 0 };
        timeouts.WriteTotalTimeoutConstant = 50;
        SetCommTimeouts(hSerial, &timeouts);
    }

    bool write(const std::vector<uint8_t>& data) {
        std::lock_guard<std::mutex> lock(mtx);
        DWORD bytesWritten;
        return WriteFile(hSerial, data.data(), data.size(), &bytesWritten, NULL);
    }

    ~SerialPort() { CloseHandle(hSerial); }
};

// Pack 16 channels (11 bits each) into 22 bytes payload with hardware safety limits
void packChannels(uint8_t* payload) {
    uint32_t bits = 0;
    uint8_t bitsAvailable = 0;
    uint8_t byteIdx = 0;
    
    for (int i = 0; i < 16; ++i) {
        // 1. Read raw input from hardware/memory
        uint16_t raw_val = rc_channels[i].load();
        
        // 2. Hardware Safety: Clamp values strictly to CRSF valid range (172 to 1811)
        uint16_t safe_val = std::clamp(raw_val, (uint16_t)172, (uint16_t)1811);
        
        // 3. Mask out to 11 bits just in case, then prepare for shift
        uint32_t val = safe_val & 0x07FF; 
        
        bits |= (val << bitsAvailable);
        bitsAvailable += 11;
        
        while (bitsAvailable >= 8) {
            payload[byteIdx++] = bits & 0xFF;
            bits >>= 8;
            bitsAvailable -= 8;
        }
    }
}

// High-precision transmission thread (100Hz)
void heartbeat_thread(SerialPort& port) {
    const auto frame_interval = std::chrono::milliseconds(10);
    auto next_frame_time = std::chrono::steady_clock::now();
    
    while (true) {
        next_frame_time += frame_interval;
        
        std::vector<uint8_t> packet = {
            CRSF_ADDRESS_FLIGHT_CONTROLLER, // Destined for Drone FC
            24,                             // Frame Length
            CRSF_FRAMETYPE_RC_CHANNELS_PACKED
        };
        
        uint8_t payload[22] = {0};
        packChannels(payload);
        
        for(int i = 0; i < 22; i++) {
            packet.push_back(payload[i]);
        }
        
        // Append CRC
        packet.push_back(crsf_crc8(&packet[2], packet.size() - 2));

        // Push to UART
        port.write(packet);
        
        // --- Hybrid High-Precision Timer ---
        auto now = std::chrono::steady_clock::now();
        
        // 1. Sleep to yield CPU, leaving 1ms for precise spin-wait
        if (now < next_frame_time - std::chrono::milliseconds(1)) {
            std::this_thread::sleep_for(next_frame_time - now - std::chrono::milliseconds(1));
        }
        
        // 2. Spin-wait for the remaining microseconds to bypass Windows OS Scheduler jitter
        while (std::chrono::steady_clock::now() < next_frame_time) {
            std::this_thread::yield(); 
        }
    }
}

// User input and control logic thread
void input_thread() {
    bool is_armed = false;
    bool is_angle = false;
    
    std::cout << "==================================" << std::endl;
    std::cout << "  DRONE CONTROL LINK ESTABLISHED  " << std::endl;
    std::cout << "==================================" << std::endl;
    std::cout << "[A] - Toggle ARM   (AUX 1)" << std::endl;
    std::cout << "[G] - Toggle ANGLE (AUX 4)" << std::endl;
    std::cout << "[Q] - Quit System" << std::endl;
    std::cout << "----------------------------------" << std::endl;

    while (true) {
        if (_kbhit()) {
            int ch = _getch();
            
            // Toggle ARM (Index 4)
            if (ch == 'A' || ch == 'a') {
                is_armed = !is_armed;
                rc_channels[4].store(is_armed ? 1811 : 172); 
                std::cout << (is_armed ? "[ACTION] >>> SYSTEM ARMED" : "[ACTION] >>> SYSTEM DISARMED") << std::endl;
            } 
            // Toggle ANGLE (Index 7)
            else if (ch == 'G' || ch == 'g') {
                is_angle = !is_angle;
                rc_channels[7].store(is_angle ? 172 : 992); 
                std::cout << (is_angle ? "[ACTION] >>> ANGLE MODE ON" : "[ACTION] >>> ANGLE MODE OFF") << std::endl;
            }
            // Exit Protocol
            else if (ch == 'Q' || ch == 'q') {
                std::cout << "[SYSTEM] Terminating connection..." << std::endl;
                exit(0);
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

int main() {
    // --- Initialize Default Channel States ---
    rc_channels[0].store(992); // Roll     - Center (~1500)
    rc_channels[1].store(993); // Pitch    - Center (~1500)
    rc_channels[2].store(172); // Throttle - MIN    (~988)
    rc_channels[3].store(992); // Yaw      - Center (~1500)
    rc_channels[4].store(172); // AUX 1    - DISARM (~988)
    
    // Set remaining AUX channels to center
    for(int i = 5; i < 16; i++) {
        rc_channels[i].store(992); 
    }

    try {
        SerialPort port("\\\\.\\COM4", 115200);
        std::cout << "[SYSTEM] Serial Port COM4 successfully opened." << std::endl;

        std::thread t1(heartbeat_thread, std::ref(port));
        std::thread t2(input_thread);

        t1.join();
        t2.join();
    }
    catch (const std::exception& e) {
        std::cerr << "[ERROR] " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}