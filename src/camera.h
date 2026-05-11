#pragma once
#include <opencv2/opencv.hpp>
#include <atomic>
#include <mutex>
#include <thread>
#include <vector>
#include <string>

class CameraManager {
public:
    static std::vector<std::string> listCameras();
    static void start(int cameraId = 0); // Додали параметр ID камери (за замовчуванням 0)
    static void stop();
    static bool getFrame(cv::Mat& outFrame);
    static bool isCameraActive();

private:
    static std::atomic<bool> isRunning;
    static cv::Mat sharedFrame;
    static std::mutex frameMutex;
    static std::thread camThread;
    
    static void captureThread(int cameraId);
};