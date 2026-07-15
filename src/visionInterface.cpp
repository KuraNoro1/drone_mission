#include "visionInterface.h"
#include <fcntl.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/select.h>
#include <cstring>
#include <sstream>
#include <cerrno>
#include <iostream>
#include <iomanip>
#include <chrono>
#include <algorithm>

using namespace std::chrono;

namespace {
    double elapsedSec() {
        static auto t0 = steady_clock::now();
        return duration<double>(steady_clock::now() - t0).count();
    }
    void log(const std::string& msg) {
        std::cout << "[" << std::fixed << std::setprecision(1)
                  << elapsedSec() << "s] [PIPE] " << msg << std::endl;
    }
}

// ─── multiBucketPipe: variable-length binary protocol ───────────────────
// Protocol: [count:1B] [id:1B cx:4B cy:4B] * count
// Total = 1 + count * 9 bytes

multiBucketPipe::multiBucketPipe(const std::string& path) : path_(path), fd_(-1) {}

multiBucketPipe::~multiBucketPipe() { close(); }

bool multiBucketPipe::open() {
    if (fd_ >= 0) return true;
    if (::access(path_.c_str(), F_OK) == -1) {
        ::mkfifo(path_.c_str(), 0666);
    }
    log("Opening multi-bucket pipe (non-blocking)...");
    fd_ = ::open(path_.c_str(), O_RDONLY | O_NONBLOCK);
    if (fd_ < 0) {
        log("WARNING: Cannot open pipe: " + path_ + " (" + strerror(errno) + ")");
        return false;
    }
    log("Multi-bucket pipe opened: " + path_);
    buffer_.clear();
    return true;
}

void multiBucketPipe::close() {
    if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
    buffer_.clear();
}

bool multiBucketPipe::readLatest(multiBucketData& data) {
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
        data = parsed;
        got = true;
    }

    if (consumedUntil > 0) {
        buffer_.erase(buffer_.begin(), buffer_.begin() + consumedUntil);
    }

    if (got && bytesRead > 0) {
        static int logCounter = 0;
        if (++logCounter % 10 == 1) {
            std::ostringstream oss;
            oss << "got multi-bucket: count=" << data.count;
            for (const auto& b : data.buckets) {
                oss << " [桶" << b.bucketId << " cx=" << std::fixed << std::setprecision(0)
                    << b.cx << " cy=" << b.cy << "]";
            }
            log("  " + oss.str());
        }
    }
    return got;
}

bool multiBucketPipe::isOpen() const { return fd_ >= 0; }

// ─── hDetectionPipe: text protocol for H landing pad ───────────────────
// Receives: "None\n" or "cx,cy\n"

hDetectionPipe::hDetectionPipe(const std::string& path)
    : path_(path), fd_(-1), running_(false), latestCx_(0), latestCy_(0),
      hasNewData_(false), hasDetection_(false) {}

hDetectionPipe::~hDetectionPipe() { close(); }

bool hDetectionPipe::open() {
    unlink(path_.c_str());
    if (mkfifo(path_.c_str(), 0666) != 0) {
        log("WARNING: h_pipe mkfifo: " + std::string(strerror(errno)));
    }
    fd_ = ::open(path_.c_str(), O_RDONLY | O_NONBLOCK);
    if (fd_ < 0) {
        log("ERROR: Cannot open H pipe: " + path_);
        return false;
    }
    running_ = true;
    readerThread_ = std::thread(&hDetectionPipe::readLoop, this);
    log("H pipe opened: " + path_);
    return true;
}

void hDetectionPipe::close() {
    running_ = false;
    if (readerThread_.joinable()) readerThread_.join();
    if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
}

void hDetectionPipe::readLoop() {
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
        if (n <= 0) {
            if (errno == EAGAIN) continue;
            break;
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

bool hDetectionPipe::readLatest(double& cx, double& cy) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!hasNewData_) return false;
    hasNewData_ = false;
    if (!hasDetection_) return false;
    cx = latestCx_;
    cy = latestCy_;
    return true;
}

bool hDetectionPipe::isOpen() const { return running_.load(); }

// ─── missionCmdPipe ────────────────────────────────────────────────────

missionCmdPipe::missionCmdPipe(const std::string& path) : path_(path), fd_(-1) {}

missionCmdPipe::~missionCmdPipe() { close(); }

bool missionCmdPipe::open() {
    unlink(path_.c_str());
    if (mkfifo(path_.c_str(), 0666) != 0) {
        log("WARNING: cmd pipe mkfifo: " + std::string(strerror(errno)));
    }
    fd_ = ::open(path_.c_str(), O_WRONLY | O_NONBLOCK);
    if (fd_ < 0) {
        log("WARNING: Cannot open cmd pipe (vision may connect later): " + std::string(strerror(errno)));
        return true;
    }
    return true;
}

void missionCmdPipe::close() {
    if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
}

void missionCmdPipe::sendState(const std::string& state) {
    if (fd_ < 0) return;
    std::string msg = state + "\n";
    ssize_t ret = write(fd_, msg.c_str(), msg.size());
    (void)ret;
}
