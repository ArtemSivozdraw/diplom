#define _WINSOCK_DEPRECATED_NO_WARNINGS
#include <winsock2.h>
#include <ws2tcpip.h>
#include "network.h"
#include "logger.h"
#include "camera.h"
#include <sstream>
#include <chrono>
#include <cmath>
#include <algorithm>

#pragma comment(lib, "ws2_32.lib")

extern std::atomic<uint16_t> rc_channels[16];

std::atomic<bool> NetworkManager::isRunning(false);
std::atomic<bool> NetworkManager::connected(false);
std::thread NetworkManager::netThread;
ClientRole NetworkManager::currentRole = ClientRole::NONE;
std::string NetworkManager::my_id = "";
std::string NetworkManager::target_id = "";
std::atomic<long long> NetworkManager::current_rtt(0);

std::atomic<int> NetworkManager::current_jpeg_quality(30);
std::map<uint32_t, long long> NetworkManager::send_times;
cv::Mat NetworkManager::pilotFrame;
std::mutex NetworkManager::frameMutex;
std::mutex NetworkManager::timeMutex;

long long NetworkManager::getRTT() { return current_rtt.load(); }

std::string NetworkManager::getMyID() { return my_id; }

struct FrameBuffer {
    uint8_t total_chunks = 0;
    std::map<uint8_t, std::vector<char>> chunks;
};

long long now_ms() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

void NetworkManager::connectAsDrone(const std::string& ip, int port) {
    disconnect(); 
    isRunning = true; connected = false; currentRole = ClientRole::DRONE;
    netThread = std::thread(networkTask, ip, port, currentRole);
}

void NetworkManager::connectAsPilot(const std::string& ip, int port, const std::string& droneId) {
    disconnect(); 
    isRunning = true; connected = false; currentRole = ClientRole::TEST_PILOT;
    target_id = droneId;
    netThread = std::thread(networkTask, ip, port, currentRole);
}

void NetworkManager::disconnect() {
    isRunning = false; connected = false; currentRole = ClientRole::NONE;
    if (netThread.joinable()) netThread.join();
    Logger::addLog("[NET] Disconnected.");
}

ClientRole NetworkManager::getRole() { return currentRole; }
bool NetworkManager::isConnected() { return connected.load(); }

bool NetworkManager::getPilotFrame(cv::Mat& outFrame) {
    std::lock_guard<std::mutex> lock(frameMutex);
    if (!pilotFrame.empty()) { outFrame = pilotFrame.clone(); return true; }
    return false;
}

void NetworkManager::parseControlPacket(const std::string& data) {
    if (data.find("CTRL:") == std::string::npos) return;
    size_t pos = data.find("CTRL:");
    std::string valuesStr = data.substr(pos + 5);
    std::stringstream ss(valuesStr);
    std::string item;
    int index = 0;
    while (std::getline(ss, item, ',') && index < 16) {
        try { rc_channels[index].store(std::stoi(item)); } catch (...) {}
        index++;
    }
}

std::string NetworkManager::buildControlPacket() {
    std::string packet = "CTRL:";
    for (int i = 0; i < 16; i++) {
        packet += std::to_string(rc_channels[i].load());
        if (i < 15) packet += ",";
    }
    return packet;
}

void NetworkManager::networkTask(std::string ip, int port, ClientRole role) {
    WSADATA wsaData; WSAStartup(MAKEWORD(2, 2), &wsaData);

    // --- TCP AUTH ---
    SOCKET tcpSock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    sockaddr_in serverAddr; serverAddr.sin_family = AF_INET; serverAddr.sin_port = htons(port);
    inet_pton(AF_INET, ip.c_str(), &serverAddr.sin_addr);

    if (connect(tcpSock, (SOCKADDR*)&serverAddr, sizeof(serverAddr)) == SOCKET_ERROR) {
        Logger::addLog("[NET_ERR] Server unreachable."); closesocket(tcpSock); WSACleanup(); isRunning = false; return;
    }

    char tcpBuf[1024];
    int r = recv(tcpSock, tcpBuf, 1023, 0);
    if (r <= 0) { closesocket(tcpSock); WSACleanup(); isRunning = false; return; }
    tcpBuf[r] = '\0';
    
    my_id = std::string(tcpBuf).substr(std::string(tcpBuf).find(':') + 1);
    my_id.erase(std::remove(my_id.begin(), my_id.end(), '\n'), my_id.end());
    my_id.erase(std::remove(my_id.begin(), my_id.end(), '\r'), my_id.end());

    if (role == ClientRole::TEST_PILOT) {
        std::string watchCmd = "WATCH_VIDEO " + target_id + "\n";
        send(tcpSock, watchCmd.c_str(), watchCmd.length(), 0);
    }

    // --- UDP SETUP ---
    SOCKET udpSock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    u_long mode = 1; ioctlsocket(udpSock, FIONBIO, &mode); 
    int rBuf = 1024 * 1024; setsockopt(udpSock, SOL_SOCKET, SO_RCVBUF, (char*)&rBuf, sizeof(rBuf));

    #define SIO_UDP_CONNRESET _WSAIOW(IOC_VENDOR, 12)
    BOOL bNewBehavior = FALSE; DWORD dwBytesReturned = 0;
    WSAIoctl(udpSock, SIO_UDP_CONNRESET, &bNewBehavior, sizeof bNewBehavior, NULL, 0, &dwBytesReturned, NULL, NULL);

    sockaddr_in udpAddr; udpAddr.sin_family = AF_INET; udpAddr.sin_port = htons(port);
    inet_pton(AF_INET, ip.c_str(), &udpAddr.sin_addr);

    sendto(udpSock, (my_id + "PING").c_str(), my_id.length() + 4, 0, (SOCKADDR*)&udpAddr, sizeof(udpAddr));
    
    connected = true;
    Logger::addLog("[SYS] MTU-SAFE Link Active. ID: " + my_id);

    auto next_ctrl_time = std::chrono::steady_clock::now();
    auto next_video_time = std::chrono::steady_clock::now();
    long long last_keepalive = now_ms();
    
    uint32_t f_id = 0;
    uint32_t highest_f = 0;
    std::map<uint32_t, FrameBuffer> assembly;
    char buf[2048];
    const int CHUNK_SIZE = 1300;

    while (isRunning) {
        // --- ПРИЙОМ (UDP) ---
        while (true) {
            int n = recvfrom(udpSock, buf, sizeof(buf), 0, NULL, NULL);
            if (n <= 0) break;

            std::string msg(buf, n);
            
            // 1. ДРОН: Отримує ACK, рахує RTT і ділиться ним з пілотом
            if (msg.find("ACK:") != std::string::npos && role == ClientRole::DRONE) {
                uint32_t ack_f_id = std::stoul(msg.substr(msg.find("ACK:") + 4));
                long long sent_time = 0;
                {
                    std::lock_guard<std::mutex> lock(timeMutex);
                    if (send_times.count(ack_f_id)) { sent_time = send_times[ack_f_id]; send_times.erase(ack_f_id); }
                }
                if (sent_time > 0) {
                    long long rtt = now_ms() - sent_time;
                    current_rtt.store(rtt);

                    // ВІДПРАВКА СИНХРОНІЗАЦІЇ РЕАЛЬНОГО RTT ПІЛОТУ
                    std::string rtt_sync = my_id + "RTT_SYNC:" + std::to_string(rtt);
                    sendto(udpSock, rtt_sync.c_str(), rtt_sync.length(), 0, (SOCKADDR*)&udpAddr, sizeof(udpAddr));

                    // Deadband
                    int qual = current_jpeg_quality.load();
                    if (rtt < 70) { qual += 2; if (qual > 80) qual = 80; }
                    else if (rtt > 120) { qual -= 5; if (qual < 15) qual = 15; }
                    current_jpeg_quality.store(qual);
                }
            }
            // 2. ДРОН: Отримує команди керування
            else if (msg.find("CTRL:") != std::string::npos && role == ClientRole::DRONE) {
                parseControlPacket(msg);
            }
            // 3. ПІЛОТ: Отримує готовий RTT від дрона (дзеркалювання)
            else if (msg.find("RTT_SYNC:") != std::string::npos && role == ClientRole::TEST_PILOT) {
                try {
                    long long synced_rtt = std::stoll(msg.substr(msg.find("RTT_SYNC:") + 9));
                    current_rtt.store(synced_rtt);
                } catch (...) {}
            }
            // 4. ПІЛОТ: Отримує відео чанки
            else if (n > 6 && role == ClientRole::TEST_PILOT) {
                uint32_t nf; memcpy(&nf, buf, 4);
                uint32_t recv_f_id = ntohl(nf);
                uint8_t t_ch = buf[4];
                uint8_t ch_idx = buf[5];

                if (recv_f_id >= highest_f) {
                    highest_f = recv_f_id;
                    assembly[recv_f_id].total_chunks = t_ch;
                    assembly[recv_f_id].chunks[ch_idx] = std::vector<char>(buf + 6, buf + n);

                    if (assembly[recv_f_id].chunks.size() == t_ch) {
                        std::vector<uchar> jpg;
                        for (int i = 0; i < t_ch; i++) jpg.insert(jpg.end(), assembly[recv_f_id].chunks[i].begin(), assembly[recv_f_id].chunks[i].end());
                        cv::Mat decoded = cv::imdecode(jpg, cv::IMREAD_COLOR);
                        if (!decoded.empty()) { std::lock_guard<std::mutex> lock(frameMutex); pilotFrame = decoded; }
                        assembly.erase(recv_f_id);
                        
                        std::string ack = my_id + "ACK:" + std::to_string(recv_f_id);
                        sendto(udpSock, ack.c_str(), ack.length(), 0, (SOCKADDR*)&udpAddr, sizeof(udpAddr));
                    }
                }
            }
        }

        // --- ВІДПРАВКА (UDP) ---
        auto now = std::chrono::steady_clock::now();
        if (role == ClientRole::TEST_PILOT) {
            if (now >= next_ctrl_time) {
                std::string ctrl = my_id + buildControlPacket();
                sendto(udpSock, ctrl.c_str(), ctrl.length(), 0, (SOCKADDR*)&udpAddr, sizeof(udpAddr));
                next_ctrl_time = now + std::chrono::milliseconds(20);
            }
            
            if (now_ms() - last_keepalive > 1000) {
                // Базовий серверний Keep-Alive
                std::string ka = my_id + "PING";
                sendto(udpSock, ka.c_str(), ka.length(), 0, (SOCKADDR*)&udpAddr, sizeof(udpAddr));
                last_keepalive = now_ms();
            }
        } else if (role == ClientRole::DRONE) {
            if (now >= next_video_time) {
                cv::Mat frame;
                if (CameraManager::getFrame(frame)) {
                    f_id++;
                    std::vector<uchar> jpgBuf;
                    std::vector<int> p = {cv::IMWRITE_JPEG_QUALITY, current_jpeg_quality.load()};
                    cv::imencode(".jpg", frame, jpgBuf, p);
                    { std::lock_guard<std::mutex> lock(timeMutex); send_times[f_id] = now_ms(); }

                    uint8_t total_ch = std::ceil((float)jpgBuf.size() / CHUNK_SIZE);
                    for (uint8_t i = 0; i < total_ch; i++) {
                        std::vector<char> pkt;
                        pkt.insert(pkt.end(), my_id.begin(), my_id.end());
                        uint32_t nf = htonl(f_id); pkt.insert(pkt.end(), (char*)&nf, (char*)&nf + 4);
                        pkt.push_back(total_ch); pkt.push_back(i);
                        int off = i * CHUNK_SIZE;
                        int len = std::min((int)CHUNK_SIZE, (int)jpgBuf.size() - off);
                        pkt.insert(pkt.end(), jpgBuf.begin() + off, jpgBuf.begin() + off + len);
                        sendto(udpSock, pkt.data(), pkt.size(), 0, (SOCKADDR*)&udpAddr, sizeof(udpAddr));
                    }
                }
                next_video_time = now + std::chrono::milliseconds(33);
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    closesocket(udpSock); closesocket(tcpSock); WSACleanup();
}