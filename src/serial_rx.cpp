#include "serial_rx.h"
#include "logger.h"
#include <windows.h>
#include <sstream>
#include <iomanip>
#include <chrono>

// Доступ до глобального масиву каналів з main.cpp
extern std::atomic<uint16_t> rc_channels[16];

std::atomic<bool> SerialRX::isRunning(false);
std::atomic<bool> SerialRX::telemetryActive(false);
std::thread SerialRX::serialThread;

uint8_t SerialRX::crc8_dvb_s2(uint8_t crc, unsigned char a) {
    crc ^= a;
    for (int i = 0; i < 8; ++i) {
        if (crc & 0x80) crc = (crc << 1) ^ 0xD5;
        else crc = crc << 1;
    }
    return crc;
}

void SerialRX::pack_crsf_channels(uint16_t* channels, uint8_t* payload) {
    payload[0] = (channels[0] & 0x07FF);
    payload[1] = ((channels[0] & 0x07FF) >> 8) | ((channels[1] & 0x07FF) << 3);
    payload[2] = ((channels[1] & 0x07FF) >> 5) | ((channels[2] & 0x07FF) << 6);
    payload[3] = ((channels[2] & 0x07FF) >> 2);
    payload[4] = ((channels[2] & 0x07FF) >> 10) | ((channels[3] & 0x07FF) << 1);
    payload[5] = ((channels[3] & 0x07FF) >> 7) | ((channels[4] & 0x07FF) << 4);
    payload[6] = ((channels[4] & 0x07FF) >> 4) | ((channels[5] & 0x07FF) << 7);
    payload[7] = ((channels[5] & 0x07FF) >> 1);
    payload[8] = ((channels[5] & 0x07FF) >> 9) | ((channels[6] & 0x07FF) << 2);
    payload[9] = ((channels[6] & 0x07FF) >> 6) | ((channels[7] & 0x07FF) << 5);
    payload[10] = ((channels[7] & 0x07FF) >> 3);
    payload[11] = ((channels[7] & 0x07FF) >> 11) | ((channels[8] & 0x07FF) << 0);
    payload[12] = ((channels[8] & 0x07FF) >> 8) | ((channels[9] & 0x07FF) << 3);
    payload[13] = ((channels[9] & 0x07FF) >> 5) | ((channels[10] & 0x07FF) << 6);
    payload[14] = ((channels[10] & 0x07FF) >> 2);
    payload[15] = ((channels[10] & 0x07FF) >> 10) | ((channels[11] & 0x07FF) << 1);
    payload[16] = ((channels[11] & 0x07FF) >> 7) | ((channels[12] & 0x07FF) << 4);
    payload[17] = ((channels[12] & 0x07FF) >> 4) | ((channels[13] & 0x07FF) << 7);
    payload[18] = ((channels[13] & 0x07FF) >> 1);
    payload[19] = ((channels[13] & 0x07FF) >> 9) | ((channels[14] & 0x07FF) << 2);
    payload[20] = ((channels[14] & 0x07FF) >> 6) | ((channels[15] & 0x07FF) << 5);
    payload[21] = ((channels[15] & 0x07FF) >> 3);
}

void SerialRX::start(const std::string& portName, int baudRate) {
    if (isRunning) return;
    isRunning = true;
    telemetryActive = false;
    serialThread = std::thread(ioThread, portName, baudRate);
}

void SerialRX::stop() {
    isRunning = false;
    telemetryActive = false;
    if (serialThread.joinable()) {
        serialThread.join();
    }
}

bool SerialRX::isTelemetryActive() {
    return telemetryActive.load();
}

void SerialRX::ioThread(std::string portName, int baudRate) {
    std::string winPortName = "\\\\.\\" + portName;
    
    HANDLE hSerial = CreateFileA(winPortName.c_str(), GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, 0);
    
    if (hSerial == INVALID_HANDLE_VALUE) {
        Logger::addLog("[COM_ERR] Failed to open " + portName);
        isRunning = false;
        return;
    }

    DCB dcbSerialParams = {0};
    dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
    
    if (GetCommState(hSerial, &dcbSerialParams)) {
        dcbSerialParams.BaudRate = baudRate;
        dcbSerialParams.ByteSize = 8;
        dcbSerialParams.StopBits = ONESTOPBIT;
        dcbSerialParams.Parity   = NOPARITY;
        SetCommState(hSerial, &dcbSerialParams);
    }

    COMMTIMEOUTS timeouts = {0};
    timeouts.ReadIntervalTimeout = MAXDWORD;
    timeouts.ReadTotalTimeoutConstant = 0;
    timeouts.ReadTotalTimeoutMultiplier = 0;
    SetCommTimeouts(hSerial, &timeouts);

    Logger::addLog("[COM] Connected to " + portName + " at " + std::to_string(baudRate));

    char buffer[256];
    DWORD bytesRead;
    DWORD bytesWritten;

    auto next_tx_time = std::chrono::steady_clock::now();
    const auto TX_INTERVAL = std::chrono::milliseconds(20);

    // Змінні для парсера протоколу CRSF
    uint8_t rx_buf[256];
    int rx_state = 0;
    int rx_expected_len = 0;
    int rx_ptr = 0;

    while (isRunning) {
        // --- 1. ФАЗА ЧИТАННЯ ТА ВАЛІДАЦІЇ ТЕЛЕМЕТРІЇ ---
        if (ReadFile(hSerial, buffer, sizeof(buffer), &bytesRead, NULL) && bytesRead > 0) {
            
            for (DWORD i = 0; i < bytesRead; ++i) {
                uint8_t b = (uint8_t)buffer[i];
                
                if (rx_state == 0) {
                    // Шукаємо байт синхронізації
                    if (b == 0xC8 || b == 0xEA || b == 0xEE) {
                        rx_buf[0] = b;
                        rx_state = 1;
                    }
                } else if (rx_state == 1) {
                    if (b > 1 && b <= 64) {
                        rx_buf[1] = b;
                        rx_expected_len = b;
                        rx_ptr = 2;
                        rx_state = 2;
                    } else {
                        rx_state = 0;
                    }
                } else if (rx_state == 2) {
                    rx_buf[rx_ptr++] = b;
                    
                    if (rx_ptr == rx_expected_len + 2) {
                        uint8_t crc = 0;
                        for (int j = 2; j < rx_expected_len + 1; ++j) {
                            crc = crc8_dvb_s2(crc, rx_buf[j]);
                        }
                        
                        // Якщо CRC правильний
                        if (crc == rx_buf[rx_expected_len + 1]) {
                            uint8_t packet_type = rx_buf[2];
                            
                            // 0x08 - CRSF_FRAMETYPE_BATTERY_SENSOR
                            if (packet_type == 0x08) {
                                if (!telemetryActive) {
                                    telemetryActive = true;
                                    Logger::addLog("[SYS] Battery telemetry received. Drone connected!");
                                }
                            }
                        }
                        
                        rx_state = 0;
                    }
                }
            }
        }

        // --- 2. ФАЗА ЗАПИСУ (КЕРУВАННЯ CRSF) ---
        auto now = std::chrono::steady_clock::now();
        if (now >= next_tx_time) {
            uint16_t current_channels[16];
            for (int i = 0; i < 16; i++) {
                current_channels[i] = rc_channels[i].load();
            }

            uint8_t crsf_packet[26];
            crsf_packet[0] = 0xC8; 
            crsf_packet[1] = 24;   
            crsf_packet[2] = 0x16; // Тип пакета: RC Channels Packed

            uint8_t payload[22];
            pack_crsf_channels(current_channels, payload);
            memcpy(&crsf_packet[3], payload, 22);

            uint8_t crc = 0;
            crc = crc8_dvb_s2(crc, crsf_packet[2]);
            for (int i = 0; i < 22; i++) {
                crc = crc8_dvb_s2(crc, payload[i]);
            }
            crsf_packet[25] = crc;

            WriteFile(hSerial, crsf_packet, sizeof(crsf_packet), &bytesWritten, NULL);
            
            next_tx_time = now + TX_INTERVAL;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    CloseHandle(hSerial);
    Logger::addLog("[COM] Port closed.");
}