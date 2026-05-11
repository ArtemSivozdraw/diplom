#pragma once
#include <string>
#include <atomic>
#include <thread>

class SerialRX {
public:
    static void start(const std::string& portName, int baudRate);
    static void stop();
    static bool isTelemetryActive();

private:
    static std::atomic<bool> isRunning;
    static std::atomic<bool> telemetryActive;
    static std::thread serialThread;
    
    // Перейменовано з readThread на ioThread, оскільки тепер тут і читання, і запис
    static void ioThread(std::string portName, int baudRate); 
    
    // Функції протоколу CRSF
    static uint8_t crc8_dvb_s2(uint8_t crc, unsigned char a);
    static void pack_crsf_channels(uint16_t* channels, uint8_t* payload);
};