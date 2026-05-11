#pragma once
#include <string>
#include <atomic>
#include <thread>
#include <opencv2/opencv.hpp>
#include <mutex>
#include <vector>
#include <map>

enum class ClientRole { NONE, DRONE, TEST_PILOT };

class NetworkManager {
public:
    static void connectAsDrone(const std::string& ip, int port);
    static void connectAsPilot(const std::string& ip, int port, const std::string& droneId);
    static void disconnect();
    
    static ClientRole getRole();
    static bool isConnected();
    static bool getPilotFrame(cv::Mat& outFrame); 
    static std::string getMyID(); // Новий метод
    static long long getRTT();

private:
    static std::atomic<long long> current_rtt;
    static std::atomic<bool> isRunning;
    static std::atomic<bool> connected;
    static std::thread netThread;
    static ClientRole currentRole;
    static std::string my_id;
    static std::string target_id;

    static cv::Mat pilotFrame;
    static std::mutex frameMutex;
    static std::mutex timeMutex;
    
    static std::map<uint32_t, long long> send_times;
    static std::atomic<int> current_jpeg_quality;

    static void networkTask(std::string ip, int port, ClientRole role);
    static void parseControlPacket(const std::string& data);
    static std::string buildControlPacket();
};