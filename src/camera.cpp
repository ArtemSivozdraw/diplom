#include "camera.h"
#include "logger.h"
#include <windows.h>
#include <dshow.h>

#pragma comment(lib, "strmiids.lib")
#pragma comment(lib, "ole32.lib")
#pragma comment(lib, "oleaut32.lib")

std::atomic<bool> CameraManager::isRunning(false);
cv::Mat CameraManager::sharedFrame;
std::mutex CameraManager::frameMutex;
std::thread CameraManager::camThread;

std::vector<std::string> CameraManager::listCameras() {
    std::vector<std::string> cameras;
    HRESULT hr = CoInitializeEx(NULL, COINIT_MULTITHREADED);
    bool coInitCalled = SUCCEEDED(hr);

    ICreateDevEnum* pDevEnum = NULL;
    hr = CoCreateInstance(CLSID_SystemDeviceEnum, NULL, CLSCTX_INPROC_SERVER, IID_PPV_ARGS(&pDevEnum));
    if (SUCCEEDED(hr)) {
        IEnumMoniker* pEnum = NULL;
        hr = pDevEnum->CreateClassEnumerator(CLSID_VideoInputDeviceCategory, &pEnum, 0);
        if (hr == S_OK) {
            IMoniker* pMoniker = NULL;
            int index = 0;
            while (pEnum->Next(1, &pMoniker, NULL) == S_OK) {
                IPropertyBag* pPropBag;
                hr = pMoniker->BindToStorage(0, 0, IID_PPV_ARGS(&pPropBag));
                if (SUCCEEDED(hr)) {
                    VARIANT varName;
                    VariantInit(&varName);
                    hr = pPropBag->Read(L"FriendlyName", &varName, 0);
                    if (SUCCEEDED(hr)) {
                        int len = WideCharToMultiByte(CP_UTF8, 0, varName.bstrVal, -1, NULL, 0, NULL, NULL);
                        std::string name(len - 1, '\0');
                        WideCharToMultiByte(CP_UTF8, 0, varName.bstrVal, -1, &name[0], len, NULL, NULL);
                        cameras.push_back("[" + std::to_string(index) + "] " + name);
                        VariantClear(&varName);
                    }
                    pPropBag->Release();
                }
                pMoniker->Release();
                index++;
            }
            pEnum->Release();
        }
        pDevEnum->Release();
    }
    if (coInitCalled) CoUninitialize();
    
    if (cameras.empty()) {
        cameras.push_back("[SYS] No video devices found.");
    }
    return cameras;
}

void CameraManager::start(int cameraId) {
    if (isRunning) return;
    isRunning = true;
    camThread = std::thread(captureThread, cameraId);
}

void CameraManager::stop() {
    isRunning = false;
    if (camThread.joinable()) {
        camThread.join();
    }
}

bool CameraManager::getFrame(cv::Mat& outFrame) {
    std::lock_guard<std::mutex> lock(frameMutex);
    if (!sharedFrame.empty()) {
        outFrame = sharedFrame.clone();
        return true;
    }
    return false;
}

void CameraManager::captureThread(int cameraId) {
    cv::VideoCapture cap(cameraId, cv::CAP_DSHOW);
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    cap.set(cv::CAP_PROP_FPS, 30);
    cap.set(cv::CAP_PROP_BUFFERSIZE, 1);


    if (!cap.isOpened()) {
        Logger::addLog("[ERROR] Cannot open camera ID: " + std::to_string(cameraId));
        isRunning = false;
        return;
    }
    
    Logger::addLog("[VIDEO] Camera ID " + std::to_string(cameraId) + " initialized.");
    cv::Mat localFrame, resizedFrame;
    
    while (isRunning) {
        cap.grab();
        cap.retrieve(localFrame);
        
        if (!localFrame.empty()) {
            cv::resize(localFrame, resizedFrame, cv::Size(640, 480));
            cv::GaussianBlur(resizedFrame, resizedFrame, cv::Size(3, 3), 0);
            std::lock_guard<std::mutex> lock(frameMutex);
            sharedFrame = resizedFrame.clone();
        }
    }
    cap.release();
    Logger::addLog("[VIDEO] Camera thread stopped.");
    
    std::lock_guard<std::mutex> lock(frameMutex);
    sharedFrame = cv::Mat(); 
}

bool CameraManager::isCameraActive() {
    return isRunning.load();
}