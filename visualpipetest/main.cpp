// ============================================================
//  visualpipetest — 投放区视觉管道 + 侦察区 WS 回传测试 (室内手持验证)
// ============================================================
//  仿照真机投弹区扫描架构的最小化测试程序, 验证:
//   1. C++ 控制端能否通过 /tmp/vision_pipe 收到桶检测
//   2. 投放区阶段结束 (默认 40s) 后自动切侦察区, 检测 WS 回传是否正常
//
//  阶段流程 (仿真机状态机):
//   [0 → --sec]      DROP_SEARCH       投放区扫描 (仿 scanForTargets)
//   [--sec]          TRANSIT_TO_RECON  仿 handleTransitToRecon (视觉切 RECON 模式)
//   [+3s 转场]       RECON_SCAN        仿 handleReconScan (开始检测 WS 回传)
//   [+--recon-sec]   结束 (0 = 直到 Ctrl+C)
//
//  与真机代码的对应关系:
//    multiBucketPipe    ← src/vision/visionInterface.cpp (协议与解析完全一致)
//    hDetectionPipe     ← src/vision/visionInterface.cpp (后台读线程)
//    reconPipe          ← src/vision/visionInterface.cpp (后台读线程)
//    missionCmdPipe     ← src/vision/visionInterface.cpp (发送任务状态)
//    altitudePipeWriter ← src/comm/droneLink.cpp enableAltitudePipe (2Hz 高度写)
//    主扫描循环          ← src/bomb/bombDropSystem.cpp scanForTargets (100ms 轮询)
//    管道就绪门控        ← src/mission/missionStateMachine.cpp handleDropSearch
//    WS 回传验证         ← 本机 recon_viewer.py 同款客户端 (ws://127.0.0.1:8765,
//                          每 2s 自动重连, JPEG 帧统计 + SOI/EOI 校验)
//
//  与真机的差异:
//    - 无 MAVSDK/飞控连接, 高度由 --alt 固定注入 (真机为遥测高度)
//    - 任务状态每秒重发 (真机只在状态切换时发一次, 此处防视觉后启动)
//    - 高度管道连接失败时后台重试 (真机 init 时同步重试 6s, 会阻塞测试流程)
//    - h/recon 读线程遇 EOF 自动重试 (真机读线程遇 EOF 直接退出)
//    - 不包含 SCAN→SELECT→GOTO 等投弹阶段与像素→世界坐标映射
//
//  测试流程:
//    1. Jetson 上启动 detector_unified.py (无 --sim, 无 --display)
//    2. Jetson 上运行本程序, 手持相机对准桶 (投放区阶段)
//    3. 本机 (笔记本) 运行 recon_viewer.py 查看 WS 回传画面
//    4. 40s 后程序自动切侦察区, 打印 WS 帧数/fps/JPEG 校验统计
//    5. 退出时输出各阶段 PASS/FAIL 总结
//
//  构建:  cmake -B build && cmake --build build
//         或 g++ main.cpp -o visualpipetest -pthread
//  运行:  ./visualpipetest [--alt 1.5] [--sec 40] [--recon-sec 0] [--state DROP_SEARCH]
// ============================================================

#include <fcntl.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/select.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <csignal>
#include <cstring>
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <atomic>
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <deque>
#include <iomanip>
#include <iostream>
#include <map>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

using namespace std::this_thread;
using namespace std::chrono;

namespace {

std::atomic<bool> g_running{true};

// 测试阶段: 0=投放区, 1=侦察转场(TRANSIT_TO_RECON), 2=侦察扫描(RECON_SCAN)
std::atomic<int> g_phase{0};

void onSignal(int) { g_running.store(false); }

double elapsedSec() {
    static auto t0 = steady_clock::now();
    return duration<double>(steady_clock::now() - t0).count();
}

void log(const std::string& tag, const std::string& msg) {
    std::cout << "[" << std::fixed << std::setprecision(1)
              << elapsedSec() << "s] [" << tag << "] " << msg << std::endl;
}

const char* bucketLabel(int id) {
    switch (id) {
        case 1: return "15cm(桶1)";
        case 2: return "20cm(桶2)";
        case 3: return "25cm(桶3)";
        default: return "未知";
    }
}

} // namespace

// ─── 与 src/mission/types.h 一致的最小结构 ───────────────────

struct bucketDetection {
    int bucketId;
    double cx;
    double cy;
};

struct multiBucketData {
    int count = 0;
    std::vector<bucketDetection> buckets;
    bool empty() const { return count == 0 || buckets.empty(); }
};

// ─── multiBucketPipe: 变长二进制协议 (与 visionInterface.cpp 一致) ───
// Protocol: [count:1B] [id:1B cx:4B cy:4B] * count
// Total = 1 + count * 9 bytes

class multiBucketPipe {
public:
    explicit multiBucketPipe(const std::string& path) : path_(path), fd_(-1) {}
    ~multiBucketPipe() { close(); }

    bool open() {
        if (fd_ >= 0) return true;
        if (::access(path_.c_str(), F_OK) == -1) {
            ::mkfifo(path_.c_str(), 0666);
        }
        log("PIPE", "Opening multi-bucket pipe (non-blocking)...");
        fd_ = ::open(path_.c_str(), O_RDONLY | O_NONBLOCK);
        if (fd_ < 0) {
            log("PIPE", "WARNING: Cannot open pipe: " + path_ + " (" + strerror(errno) + ")");
            return false;
        }
        log("PIPE", "Multi-bucket pipe opened: " + path_);
        buffer_.clear();
        return true;
    }

    void close() {
        if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
        buffer_.clear();
    }

    bool readLatest(multiBucketData& data) {
        if (fd_ < 0) return false;

        int flags = fcntl(fd_, F_GETFL, 0);
        fcntl(fd_, F_SETFL, flags | O_NONBLOCK);

        uint8_t tmp[512];
        int bytesRead = 0;
        while (true) {
            ssize_t n = read(fd_, tmp, sizeof(tmp));
            if (n <= 0) break;
            buffer_.insert(buffer_.end(), tmp, tmp + n);
            bytesRead += n;
        }

        fcntl(fd_, F_SETFL, flags);

        bool got = false;
        bool gotBuckets = false;  // 是否已在当前批次中找到有效桶数据
        size_t consumedUntil = 0;

        size_t offset = 0;
        while (offset < buffer_.size()) {
            if (offset + 1 > buffer_.size()) break;
            uint8_t count = buffer_[offset];
            size_t msgSize = 1 + static_cast<size_t>(count) * 9;
            if (offset + msgSize > buffer_.size()) break;

            size_t pos = offset + 1;
            multiBucketData parsed;
            parsed.count = count;
            for (int i = 0; i < count; i++) {
                uint8_t id = buffer_[pos]; pos += 1;
                float cx, cy;
                std::memcpy(&cx, &buffer_[pos], 4); pos += 4;
                std::memcpy(&cy, &buffer_[pos], 4); pos += 4;
                parsed.buckets.push_back({(int)id, (double)cx, (double)cy});
            }

            offset += msgSize;
            consumedUntil = offset;

            // 只保留最后一条 count>0 的消息, 丢弃中间的 count=0
            if (count > 0) {
                data = parsed;
                gotBuckets = true;
            } else if (!gotBuckets) {
                data = parsed;  // 全是 count=0 时返回最后一条
            }
            got = true;
        }

        if (consumedUntil > 0) {
            buffer_.erase(buffer_.begin(), buffer_.begin() + consumedUntil);
        }

        (void)bytesRead;
        return got;
    }

    bool isOpen() const { return fd_ >= 0; }

private:
    std::string path_;
    int fd_;
    std::vector<uint8_t> buffer_;
};

// ─── hDetectionPipe: H 降落标识文本协议 ("None\n" / "cx,cy\n") ───

class hDetectionPipe {
public:
    explicit hDetectionPipe(const std::string& path)
        : path_(path), fd_(-1), running_(false), latestCx_(0), latestCy_(0),
          hasNewData_(false), hasDetection_(false) {}

    ~hDetectionPipe() { close(); }

    bool open() {
        if (mkfifo(path_.c_str(), 0666) != 0 && errno != EEXIST) {
            log("H", "WARNING: h_pipe mkfifo: " + std::string(strerror(errno)));
        }
        fd_ = ::open(path_.c_str(), O_RDONLY | O_NONBLOCK);
        if (fd_ < 0) {
            log("H", "ERROR: Cannot open H pipe: " + path_);
            return false;
        }
        running_ = true;
        readerThread_ = std::thread(&hDetectionPipe::readLoop, this);
        log("H", "H pipe opened: " + path_);
        return true;
    }

    void close() {
        running_ = false;
        if (readerThread_.joinable()) readerThread_.join();
        if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
    }

    bool readLatest(double& cx, double& cy) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!hasNewData_) return false;
        hasNewData_ = false;
        if (!hasDetection_) return false;
        cx = latestCx_;
        cy = latestCy_;
        return true;
    }

    bool isOpen() const { return running_.load(); }

private:
    void readLoop() {
        char buf[256];
        std::string line;
        fd_set fds;
        timeval tv{0, 50000};

        while (running_) {
            FD_ZERO(&fds);
            FD_SET(fd_, &fds);
            int ret = select(fd_ + 1, &fds, nullptr, nullptr, &tv);
            if (ret <= 0) continue;

            ssize_t n = read(fd_, buf, sizeof(buf) - 1);
            if (n < 0) {
                if (errno == EAGAIN) continue;
                break;
            }
            if (n == 0) {
                // EOF: 写端(视觉)未连接或已断开, 稍后重试 (真机版直接退出)
                sleep_for(milliseconds(200));
                continue;
            }
            buf[n] = '\0';
            line += buf;

            size_t pos;
            while ((pos = line.find('\n')) != std::string::npos) {
                std::string msg = line.substr(0, pos);
                line.erase(0, pos + 1);

                std::lock_guard<std::mutex> lock(mutex_);
                if (msg == "None" || msg.empty()) {
                    hasDetection_ = false;
                } else {
                    std::istringstream iss(msg);
                    char comma;
                    if (iss >> latestCx_ >> comma >> latestCy_) {
                        hasDetection_ = true;
                    } else {
                        hasDetection_ = false;
                    }
                }
                hasNewData_ = true;
            }
        }
    }

    std::string path_;
    int fd_;
    std::atomic<bool> running_;
    std::thread readerThread_;
    mutable std::mutex mutex_;
    double latestCx_;
    double latestCy_;
    bool hasNewData_;
    bool hasDetection_;
};

// ─── reconPipe: 侦察区文本协议 (真机同款后台读线程) ───

class reconPipe {
public:
    explicit reconPipe(const std::string& path)
        : path_(path), fd_(-1), running_(false), hasNewData_(false) {}

    ~reconPipe() { close(); }

    bool open() {
        if (mkfifo(path_.c_str(), 0666) != 0 && errno != EEXIST) {
            log("RECON", "WARNING: recon_pipe mkfifo: " + std::string(strerror(errno)));
        }
        fd_ = ::open(path_.c_str(), O_RDONLY | O_NONBLOCK);
        if (fd_ < 0) {
            log("RECON", "ERROR: Cannot open recon pipe: " + path_);
            return false;
        }
        running_ = true;
        readerThread_ = std::thread(&reconPipe::readLoop, this);
        log("RECON", "Recon pipe opened: " + path_);
        return true;
    }

    void close() {
        running_ = false;
        if (readerThread_.joinable()) readerThread_.join();
        if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
    }

    bool readLatest(std::string& result) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!hasNewData_) return false;
        hasNewData_ = false;
        result = latestResult_;
        return true;
    }

private:
    void readLoop() {
        char buf[512];
        std::string line;
        fd_set fds;
        timeval tv{0, 50000};

        while (running_) {
            FD_ZERO(&fds);
            FD_SET(fd_, &fds);
            int ret = select(fd_ + 1, &fds, nullptr, nullptr, &tv);
            if (ret <= 0) continue;

            ssize_t n = read(fd_, buf, sizeof(buf) - 1);
            if (n < 0) {
                if (errno == EAGAIN) continue;
                break;
            }
            if (n == 0) {
                // EOF: 写端(视觉)未连接或已断开, 稍后重试 (真机版直接退出)
                sleep_for(milliseconds(200));
                continue;
            }
            buf[n] = '\0';
            line += buf;

            size_t pos;
            while ((pos = line.find('\n')) != std::string::npos) {
                std::string msg = line.substr(0, pos);
                line.erase(0, pos + 1);
                std::lock_guard<std::mutex> lock(mutex_);
                latestResult_ = msg;
                hasNewData_ = true;
            }
        }
    }

    std::string path_;
    int fd_;
    std::atomic<bool> running_;
    std::thread readerThread_;
    mutable std::mutex mutex_;
    std::string latestResult_;
    bool hasNewData_;
};

// ─── missionCmdPipe: 发送任务状态到视觉 (真机同款) ───

class missionCmdPipe {
public:
    explicit missionCmdPipe(const std::string& path) : path_(path), fd_(-1) {}

    ~missionCmdPipe() { close(); }

    bool open() {
        if (mkfifo(path_.c_str(), 0666) != 0 && errno != EEXIST) {
            log("CMD", "WARNING: cmd pipe mkfifo: " + std::string(strerror(errno)));
        }
        log("CMD", "Cmd pipe file at: " + path_ + " (will connect when vision reader is ready)");
        return true;
    }

    void close() {
        if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
    }

    void sendState(const std::string& state) {
        if (fd_ < 0) {
            fd_ = ::open(path_.c_str(), O_WRONLY | O_NONBLOCK);
            if (fd_ < 0) return;
        }
        std::string msg = state + "\n";
        ssize_t ret = write(fd_, msg.c_str(), msg.size());
        if (ret < 0 && (errno == EPIPE || errno == EBADF)) {
            ::close(fd_);
            fd_ = -1;
        }
    }

private:
    std::string path_;
    int fd_;
};

// ─── altitudePipeWriter: 高度管道 (仿 droneLink::enableAltitudePipe) ───

class altitudePipeWriter {
public:
    explicit altitudePipeWriter(const std::string& path) : path_(path), fd_(-1), running_(false), altitude_(1.5) {}

    ~altitudePipeWriter() { stop(); }

    bool open() {
        ::unlink(path_.c_str());
        ::mkfifo(path_.c_str(), 0666);
        // 单次尝试: 成功即连上; 失败由写线程后台每 0.5s 重试
        // (真机 enableAltitudePipe 此处同步重试 6s, 测试程序改为后台重试防阻塞)
        fd_ = ::open(path_.c_str(), O_WRONLY | O_NONBLOCK);
        if (fd_ >= 0) {
            log("ALT", "Altitude pipe opened: " + path_);
            return true;
        }
        log("ALT", "Altitude pipe 读端未就绪, 写线程后台每 0.5s 重试");
        return false;
    }

    void start(double altitudeM) {
        altitude_ = altitudeM;
        running_ = true;
        writerThread_ = std::thread(&altitudePipeWriter::writeLoop, this);
    }

    void stop() {
        running_ = false;
        if (writerThread_.joinable()) writerThread_.join();
        if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
    }

private:
    void writeLoop() {
        // 真机: 位置遥测 10Hz 每 5 次写一次 → 2Hz
        bool wasConnected = false;
        while (running_) {
            if (fd_ < 0) {
                fd_ = ::open(path_.c_str(), O_WRONLY | O_NONBLOCK);
                if (fd_ >= 0) {
                    if (!wasConnected)
                        log("ALT", "Altitude pipe opened: " + path_);
                    wasConnected = true;
                } else {
                    wasConnected = false;
                    sleep_for(milliseconds(500));
                    continue;
                }
            }
            char buf[32];
            std::snprintf(buf, sizeof(buf), "%.2f\n", altitude_);
            ssize_t ret = ::write(fd_, buf, strlen(buf));
            if (ret < 0 && (errno == EPIPE || errno == EBADF || errno == EAGAIN)) {
                ::close(fd_);
                fd_ = -1;
                wasConnected = false;
            }
            sleep_for(milliseconds(500));
        }
    }

    std::string path_;
    int fd_;
    std::atomic<bool> running_;
    std::thread writerThread_;
    double altitude_;
};

// ─── WS 回传验证客户端 (仿 recon_viewer.py: 连接 8765, 断线 2s 重连) ───
// 只依赖 POSIX socket, 自行实现 RFC6455 客户端帧 (握手/PING-PONG/掩码)。

namespace {

std::string b64(const unsigned char* in, size_t n) {
    static const char* T = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    std::string out;
    size_t i = 0;
    while (i + 3 <= n) {
        uint32_t v = (in[i] << 16) | (in[i + 1] << 8) | in[i + 2];
        out += T[(v >> 18) & 63]; out += T[(v >> 12) & 63];
        out += T[(v >> 6) & 63];  out += T[v & 63];
        i += 3;
    }
    if (i + 1 == n) {
        uint32_t v = in[i] << 16;
        out += T[(v >> 18) & 63]; out += T[(v >> 12) & 63]; out += "==";
    } else if (i + 2 == n) {
        uint32_t v = (in[i] << 16) | (in[i + 1] << 8);
        out += T[(v >> 18) & 63]; out += T[(v >> 12) & 63]; out += T[(v >> 6) & 63]; out += "=";
    }
    return out;
}

} // namespace

class wsClient {
public:
    wsClient(const std::string& host, int port)
        : host_(host), port_(port), running_(false), fd_(-1),
          connected_(false), connects_(0), totalFrames_(0), totalBytes_(0), jpegOk_(0) {}

    ~wsClient() { stop(); }

    void start() {
        running_ = true;
        thread_ = std::thread(&wsClient::runLoop, this);
    }

    void stop() {
        running_ = false;
        if (fd_ >= 0) ::shutdown(fd_, SHUT_RDWR);
        if (thread_.joinable()) thread_.join();
        if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
    }

    struct Stats {
        bool connected = false;
        int connects = 0;
        uint64_t totalFrames = 0;
        uint64_t totalBytes = 0;
        uint64_t jpegOk = 0;
        uint64_t phaseFrames[3] = {0, 0, 0};
        uint64_t phaseBytes[3] = {0, 0, 0};
        uint64_t phaseJpegOk[3] = {0, 0, 0};
        double fps5s = 0.0;
    };

    Stats stats() const {
        std::lock_guard<std::mutex> lock(mutex_);
        Stats s;
        s.connected = connected_;
        s.connects = connects_;
        s.totalFrames = totalFrames_;
        s.totalBytes = totalBytes_;
        s.jpegOk = jpegOk_;
        std::copy(std::begin(phaseFrames_), std::end(phaseFrames_), s.phaseFrames);
        std::copy(std::begin(phaseBytes_), std::end(phaseBytes_), s.phaseBytes);
        std::copy(std::begin(phaseJpegOk_), std::end(phaseJpegOk_), s.phaseJpegOk);
        double now = elapsedSec();
        while (!frameTimes_.empty() && frameTimes_.front() < now - 5.0) frameTimes_.pop_front();
        s.fps5s = frameTimes_.size() / 5.0;
        return s;
    }

private:
    int connectServer() {
        int fd = ::socket(AF_INET, SOCK_STREAM, 0);
        if (fd < 0) return -1;
        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(static_cast<uint16_t>(port_));
        if (inet_pton(AF_INET, host_.c_str(), &addr.sin_addr) != 1) {
            ::close(fd);
            return -1;
        }
        timeval tv{2, 0};
        setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
        setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
        if (::connect(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
            ::close(fd);
            return -1;
        }
        return fd;
    }

    bool sendAll(const void* data, size_t n) {
        size_t sent = 0;
        while (sent < n) {
            ssize_t r = send(fd_, static_cast<const char*>(data) + sent, n - sent, MSG_NOSIGNAL);
            if (r <= 0) return false;
            sent += static_cast<size_t>(r);
        }
        return true;
    }

    bool sendAll(const std::string& s) { return sendAll(s.data(), s.size()); }

    bool readExact(void* dst, size_t n) {
        size_t got = 0;
        while (got < n) {
            ssize_t r = recv(fd_, static_cast<char*>(dst) + got, n - got, 0);
            if (r <= 0) return false;
            got += static_cast<size_t>(r);
        }
        return true;
    }

    bool handshake() {
        unsigned char keyBytes[16];
        for (int i = 0; i < 16; ++i)
            keyBytes[i] = static_cast<unsigned char>(rand() & 0xFF);
        std::string req = "GET / HTTP/1.1\r\n"
                          "Host: " + host_ + ":" + std::to_string(port_) + "\r\n"
                          "Upgrade: websocket\r\n"
                          "Connection: Upgrade\r\n"
                          "Sec-WebSocket-Key: " + b64(keyBytes, 16) + "\r\n"
                          "Sec-WebSocket-Version: 13\r\n\r\n";
        if (!sendAll(req)) return false;

        std::string resp;
        char buf[1024];
        while (resp.find("\r\n\r\n") == std::string::npos) {
            ssize_t n = recv(fd_, buf, sizeof(buf), 0);
            if (n <= 0) return false;
            resp.append(buf, n);
            if (resp.size() > 8192) return false;
        }
        return resp.find("101") != std::string::npos;
    }

    bool recvFrame(int& opcode, std::vector<uint8_t>& payload) {
        payload.clear();
        uint8_t hdr[2];
        if (!readExact(hdr, 2)) return false;
        opcode = hdr[0] & 0x0F;
        bool masked = (hdr[1] & 0x80) != 0;
        uint64_t len = hdr[1] & 0x7F;
        if (len == 126) {
            uint8_t ext[2];
            if (!readExact(ext, 2)) return false;
            len = (static_cast<uint64_t>(ext[0]) << 8) | ext[1];
        } else if (len == 127) {
            uint8_t ext[8];
            if (!readExact(ext, 8)) return false;
            len = 0;
            for (int i = 0; i < 8; ++i) len = (len << 8) | ext[i];
        }
        if (len > 64u * 1024u * 1024u) return false;
        uint8_t mask[4] = {0, 0, 0, 0};
        if (masked && !readExact(mask, 4)) return false;
        payload.resize(len);
        if (len > 0 && !readExact(payload.data(), len)) return false;
        if (masked) {
            for (size_t i = 0; i < len; ++i) payload[i] ^= mask[i % 4];
        }
        return true;
    }

    bool sendFrame(int opcode, const std::vector<uint8_t>& payload) {
        std::vector<uint8_t> f;
        f.push_back(0x80 | (opcode & 0x0F));
        uint8_t mask[4];
        for (int i = 0; i < 4; ++i) mask[i] = static_cast<uint8_t>(rand() & 0xFF);
        size_t len = payload.size();
        if (len < 126) {
            f.push_back(0x80 | static_cast<uint8_t>(len));
        } else if (len <= 0xFFFF) {
            f.push_back(0x80 | 126);
            f.push_back(static_cast<uint8_t>(len >> 8));
            f.push_back(static_cast<uint8_t>(len & 0xFF));
        } else {
            f.push_back(0x80 | 127);
            for (int i = 7; i >= 0; --i)
                f.push_back(static_cast<uint8_t>(len >> (8 * i)));
        }
        f.insert(f.end(), mask, mask + 4);
        for (size_t i = 0; i < len; ++i) f.push_back(payload[i] ^ mask[i % 4]);
        return sendAll(f.data(), f.size());
    }

    void runLoop() {
        while (running_) {
            fd_ = connectServer();
            if (fd_ < 0) {
                static int failCnt = 0;
                if (++failCnt % 5 == 1)
                    log("WS", "等待 WS 服务 ws://" + host_ + ":" + std::to_string(port_) +
                        " (每 2s 重试, 仿 recon_viewer)");
                sleep_for(seconds(2));
                continue;
            }
            if (!handshake()) {
                ::close(fd_);
                fd_ = -1;
                sleep_for(seconds(2));
                continue;
            }
            {
                std::lock_guard<std::mutex> lock(mutex_);
                connected_ = true;
                connects_++;
            }
            log("WS", "已连接 ws://" + host_ + ":" + std::to_string(port_) +
                " (第 " + std::to_string(connects_) + " 次连接, 仿 recon_viewer 客户端)");

            while (running_) {
                int opcode = 0;
                std::vector<uint8_t> payload;
                if (!recvFrame(opcode, payload)) break;

                if (opcode == 0x2) {  // binary: JPEG 帧
                    bool ok = payload.size() > 4 &&
                              payload[0] == 0xFF && payload[1] == 0xD8 &&
                              payload[payload.size() - 2] == 0xFF &&
                              payload[payload.size() - 1] == 0xD9;
                    std::lock_guard<std::mutex> lock(mutex_);
                    totalFrames_++;
                    totalBytes_ += payload.size();
                    if (ok) jpegOk_++;
                    int p = g_phase.load();
                    if (p < 0 || p > 2) p = 0;
                    phaseFrames_[p]++;
                    phaseBytes_[p] += payload.size();
                    if (ok) phaseJpegOk_[p]++;
                    frameTimes_.push_back(elapsedSec());
                } else if (opcode == 0x9) {  // PING → PONG (服务端心跳)
                    sendFrame(0xA, payload);
                } else if (opcode == 0x8) {  // CLOSE
                    break;
                }
            }

            {
                std::lock_guard<std::mutex> lock(mutex_);
                connected_ = false;
            }
            ::close(fd_);
            fd_ = -1;
            log("WS", "连接断开, 2s 后重连 (仿 recon_viewer)");
            sleep_for(seconds(2));
        }
    }

    std::string host_;
    int port_;
    std::atomic<bool> running_;
    std::thread thread_;
    int fd_;

    mutable std::mutex mutex_;
    bool connected_;
    int connects_;
    uint64_t totalFrames_;
    uint64_t totalBytes_;
    uint64_t jpegOk_;
    uint64_t phaseFrames_[3];
    uint64_t phaseBytes_[3];
    uint64_t phaseJpegOk_[3];
    mutable std::deque<double> frameTimes_;
};

// ─── 主程序 ───────────────────────────────────────────────────

void printUsage() {
    std::cout <<
        "用法: ./visualpipetest [选项]\n"
        "  --alt <米>       注入视觉的高度 (真机为遥测高度), 默认 1.5 (手持高度)\n"
        "  --sec <秒>       投放区阶段时长, 默认 40; 0 = 立即切侦察区\n"
        "  --recon-sec <秒> 侦察区 WS 检测时长, 默认 0 = 直到 Ctrl+C\n"
        "  --state <名>     投放区任务状态, 默认 DROP_SEARCH\n"
        "  -h, --help       显示本帮助\n";
}

int main(int argc, char** argv) {
    double altitude = 1.5;
    double dropSec = 40.0;
    double reconSec = 0.0;
    std::string state = "DROP_SEARCH";

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        auto next = [&]() -> std::string {
            return (i + 1 < argc) ? std::string(argv[++i]) : std::string();
        };
        if (a == "--alt") {
            altitude = std::stod(next());
        } else if (a == "--sec") {
            dropSec = std::stod(next());
        } else if (a == "--recon-sec") {
            reconSec = std::stod(next());
        } else if (a == "--state") {
            state = next();
        } else if (a == "-h" || a == "--help") {
            printUsage();
            return 0;
        } else {
            std::cout << "未知参数: " << a << "\n";
            printUsage();
            return 1;
        }
    }

    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);
    std::signal(SIGPIPE, SIG_IGN);
    std::srand(static_cast<unsigned>(steady_clock::now().time_since_epoch().count()));

    std::cout << "============================================\n"
              << "  visualpipetest — 投放区管道 + 侦察区 WS 回传测试\n"
              << "============================================\n"
              << "  模拟高度: " << altitude << " m\n"
              << "  投放区状态: " << state << " (" << dropSec << "s)\n"
              << "  " << dropSec << "s 后自动切侦察区 (TRANSIT_TO_RECON → RECON_SCAN)\n"
              << "  侦察区 WS 检测时长: "
              << (reconSec > 0 ? std::to_string(reconSec) + " s" : "直到 Ctrl+C") << "\n"
              << "  配合: Jetson 上运行 detector_unified.py, 本机运行 recon_viewer.py\n"
              << "============================================\n";

    // 1. 打开全部管道 (仿 missionStateMachine::init)
    missionCmdPipe cmdPipe("/tmp/mission_cmd");
    cmdPipe.open();
    multiBucketPipe bucketPipe("/tmp/vision_pipe");
    bucketPipe.open();
    hDetectionPipe hPipe("/tmp/h_pipe");
    hPipe.open();
    reconPipe reconPipe("/tmp/recon_pipe");
    reconPipe.open();
    altitudePipeWriter altPipe("/tmp/altitude_pipe");
    altPipe.open();
    altPipe.start(altitude);

    // 2. WS 回传验证客户端 (仿 recon_viewer: 全流程保持连接)
    wsClient ws("127.0.0.1", 8765);
    ws.start();

    // 3. 通知视觉进入投放模式 (仿 setState(dropSearch) → notifyVision)
    cmdPipe.sendState(state);
    log("TEST", "已发送任务状态: " + state + " (每秒重发, 防视觉进程后启动)");

    // 4. 两阶段主循环
    int phase = 0;
    double reconScanAt = -1.0;
    std::string currentState = state;
    bool visionPipeReady = false;
    const int EMPTY_TIMEOUT_FRAMES = 30;  // 3s 连续无检测才清空统计
    int emptyFrames = 0;
    int pollCount = 0;
    int pipeSuccess = 0;
    int bucketFrames = 0;
    int maxCount = 0;
    double lastDetectTime = -1.0;
    std::map<int, int> idCounts;
    auto lastStateSend = steady_clock::now();

    while (g_running.load()) {
        double now = elapsedSec();

        // ── 阶段切换 (仿真机状态机) ──
        if (phase == 0 && now >= dropSec) {
            currentState = "TRANSIT_TO_RECON";
            cmdPipe.sendState(currentState);
            g_phase.store(1);
            reconScanAt = now + 3.0;
            phase = 1;
            log("STATE", "TRANSIT_TO_RECON → 视觉切侦察区模式 (仿 handleTransitToRecon)");
        }
        if (phase == 1 && now >= reconScanAt) {
            currentState = "RECON_SCAN";
            cmdPipe.sendState(currentState);
            g_phase.store(2);
            phase = 2;
            log("STATE", "RECON_SCAN → 侦察区扫描 (仿 handleReconScan), 开始检测 WS 回传");
        }
        if (phase == 2 && reconSec > 0.0 && now >= reconScanAt + reconSec) {
            log("TEST", "侦察区检测时长到 (" + std::to_string(reconSec) + "s), 结束");
            break;
        }

        // 每秒重发当前状态 (真机只在切换时发一次)
        if (duration<double>(steady_clock::now() - lastStateSend).count() >= 1.0) {
            cmdPipe.sendState(currentState);
            lastStateSend = steady_clock::now();
        }

        if (phase == 0) {
            // ── 投放区扫描 (仿 scanForTargets 的 100ms 轮询 + handleDropSearch 门控) ──
            multiBucketData vis;
            bool got = bucketPipe.readLatest(vis);
            pollCount++;
            if (got) pipeSuccess++;

            if (!visionPipeReady) {
                if (got) {
                    visionPipeReady = true;
                    log("TEST", "Vision pipe ready (first frame received), 投放区扫描开始");
                } else {
                    static int waitCnt = 0;
                    if (++waitCnt % 20 == 1)
                        log("TEST", "Waiting for vision pipe to become ready...");
                    sleep_for(milliseconds(100));
                    continue;
                }
            }

            if (!got || vis.empty()) {
                emptyFrames++;
                if (emptyFrames >= EMPTY_TIMEOUT_FRAMES) {
                    emptyFrames = 0;
                    log("TEST", "连续 3s 无检测 (仿 scanForTargets 清空聚类)");
                }
                static int emptyLog = 0;
                if (++emptyLog % 50 == 1)
                    log("TEST", "scanning... 本帧无桶 (poll=" + std::to_string(pollCount) +
                        " success=" + std::to_string(pipeSuccess) + ")");
                sleep_for(milliseconds(100));
                continue;
            }

            emptyFrames = 0;
            bucketFrames++;
            lastDetectTime = elapsedSec();
            if (vis.count > maxCount) maxCount = vis.count;

            std::ostringstream oss;
            oss << "got multi-bucket: count=" << vis.count;
            for (const auto& b : vis.buckets) {
                oss << " [" << bucketLabel(b.bucketId) << " cx=" << std::fixed
                    << std::setprecision(0) << b.cx << " cy=" << b.cy << "]";
                idCounts[b.bucketId]++;
            }
            log("PIPE", oss.str());

            sleep_for(milliseconds(100));
        } else {
            // ── 侦察区 (仿 handleReconScan 悬停期: 每 100ms 查 H/侦察管道) ──
            double hCx, hCy;
            if (hPipe.readLatest(hCx, hCy)) {
                log("H", "H detected at (" + std::to_string(hCx) + "," +
                    std::to_string(hCy) + ")");
            }
            std::string reconResult;
            if (reconPipe.readLatest(reconResult)) {
                log("RECON", "Recon result: " + reconResult);
            }

            // WS 回传统计 (每 5s 一次)
            static double lastWsPrint = -100.0;
            if (now - lastWsPrint >= 5.0) {
                lastWsPrint = now;
                auto s = ws.stats();
                std::ostringstream oss;
                if (s.connected) {
                    uint64_t reconFrames = s.phaseFrames[1] + s.phaseFrames[2];
                    uint64_t reconOk = s.phaseJpegOk[1] + s.phaseJpegOk[2];
                    oss << "WS 回传正常: 连接=是(第" << s.connects << "次), 侦察区帧数="
                        << reconFrames << ", 5s 窗口 fps=" << std::fixed
                        << std::setprecision(1) << s.fps5s << ", JPEG 校验 OK=" << reconOk;
                    if (reconFrames == 0) oss << " (尚无帧, 请确认视觉已切 RECON 模式)";
                } else {
                    oss << "WS 未连接 (自动重连中) — 请检查 detector_unified.py 的 WS 服务"
                           "与笔记本 recon_viewer 画面";
                }
                log("WS", oss.str());
            }
            sleep_for(milliseconds(100));
        }
    }

    // 5. 总结
    auto s = ws.stats();
    uint64_t reconFrames = s.phaseFrames[1] + s.phaseFrames[2];
    uint64_t reconJpegOk = s.phaseJpegOk[1] + s.phaseJpegOk[2];
    bool bucketOk = bucketFrames > 0;
    bool wsOk = reconFrames > 0 && reconJpegOk > 0;

    log("TEST", "========================================");
    log("TEST", "测试总结");
    log("TEST", "  [投放区] 含桶数据帧数:   " + std::to_string(bucketFrames) +
        " (轮询 " + std::to_string(pollCount) + " 次, 读到数据 " +
        std::to_string(pipeSuccess) + " 次, 单帧最大桶数 " + std::to_string(maxCount) + ", 最后检出 " +
        (lastDetectTime >= 0 ? std::to_string(lastDetectTime).substr(0, 6) + "s" : "无") + ")");
    std::ostringstream ids;
    for (const auto& [id, n] : idCounts) ids << " 桶" << id << "=" << n;
    log("TEST", "  [投放区] 各桶检出次数:  " + (ids.str().empty() ? "无" : ids.str()));
    log("TEST", "  [投放区] 结果: " + std::string(bucketOk ? "PASS" : "FAIL") +
        " (通过 /tmp/vision_pipe 收到桶检测)");
    log("TEST", "  [WS回传] 连接次数:       " + std::to_string(s.connects) +
        ", 总帧数: " + std::to_string(s.totalFrames));
    log("TEST", "  [WS回传] 侦察区帧数:    " + std::to_string(reconFrames) +
        ", JPEG 校验 OK: " + std::to_string(reconJpegOk) +
        ", 5s 窗口 fps: " + [&] {
            std::ostringstream o;
            o << std::fixed << std::setprecision(1) << s.fps5s;
            return o.str();
        }());
    log("TEST", "  [WS回传] 结果: " + std::string(wsOk ? "PASS" : "FAIL") +
        " (切侦察区后 WS 图像回传" + std::string(wsOk ? "正常" : "异常") + ")");
    log("TEST", "  总结果: " + std::string((bucketOk && wsOk) ? "PASS" : "FAIL"));
    if (!bucketOk)
        log("TEST", "  检查: detector_unified.py 是否运行 / 是否切到 DROP 模式 / 相机是否对准桶");
    if (!wsOk)
        log("TEST", "  检查: 视觉 WS 服务 (ws://<Jetson IP>:8765) / 笔记本 recon_viewer 是否显示画面");
    log("TEST", "========================================");

    ws.stop();
    altPipe.stop();
    cmdPipe.close();
    bucketPipe.close();
    hPipe.close();
    reconPipe.close();

    return (bucketOk && wsOk) ? 0 : 1;
}
