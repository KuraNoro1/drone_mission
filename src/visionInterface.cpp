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

using namespace std::chrono;

namespace {
    double elapsedSec() {
        static auto t0 = steady_clock::now();
        return duration<double>(steady_clock::now() - t0).count();
    }
    void log(const std::string& msg) {
        std::cout << "[" << std::fixed << std::setprecision(1)
                  << elapsedSec() << "s] " << msg << std::endl;
    }
}

// ─── binaryVisionPipe ─────────────────────────────────────

binaryVisionPipe::binaryVisionPipe(const std::string& path) : path_(path), fd_(-1) {}

binaryVisionPipe::~binaryVisionPipe() { close(); }

bool binaryVisionPipe::open() {
    if (fd_ >= 0) return true;
    if (::access(path_.c_str(), F_OK) == -1) {
        ::mkfifo(path_.c_str(), 0666);
    }
    log("Opening binary pipe (blocking, waiting for Python)...");
    fd_ = ::open(path_.c_str(), O_RDONLY);
    if (fd_ < 0) {
        log("WARNING: Cannot open binary pipe: " + path_);
        return false;
    }
    log("Binary vision pipe opened: " + path_);
    return true;
}

void binaryVisionPipe::close() {
    if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
}

bool binaryVisionPipe::readLatest(visionBinaryData& data) {
    if (fd_ < 0) return false;

    // 切换到非阻塞模式进行排空读取
    int flags = fcntl(fd_, F_GETFL, 0);
    fcntl(fd_, F_SETFL, flags | O_NONBLOCK);

    bool got = false;
    int count = 0;
    visionBinaryData temp;
    while (true) {
        ssize_t n = read(fd_, &temp, sizeof(temp));
        if (n == sizeof(temp)) {
            data = temp;
            got = true;
            count++;
        } else {
            break;
        }
    }

    // 恢复原始标志 (阻塞模式)
    fcntl(fd_, F_SETFL, flags);

    if (got) {
        static int logCounter = 0;
        if (++logCounter % 20 == 1) {
            std::cout << "  [pipe] got " << count << " msgs, latest: cx="
                      << data.cx << " cy=" << data.cy
                      << " conf=" << data.confidence << std::endl;
        }
    }
    return got;
}

bool binaryVisionPipe::isOpen() const { return fd_ >= 0; }

// ─── visionPipe ───────────────────────────────────────────

visionPipe::visionPipe(const std::string& path, int imgW, int imgH)
    : path_(path), imgW_(imgW), imgH_(imgH), fd_(-1), running_(false), hasNewData_(false) {}

visionPipe::~visionPipe() { close(); }

bool visionPipe::open(bool createPipe) {
    if (createPipe) {
        unlink(path_.c_str());
        if (mkfifo(path_.c_str(), 0666) != 0) {
            log("WARNING: mkfifo failed: " + std::string(strerror(errno)));
        }
    }

    fd_ = ::open(path_.c_str(), O_RDONLY | O_NONBLOCK);
    if (fd_ < 0) {
        log("ERROR: Cannot open vision pipe: " + path_ + " (" + strerror(errno) + ")");
        return false;
    }

    running_ = true;
    readerThread_ = std::thread(&visionPipe::readLoop, this);
    log("Vision pipe opened: " + path_);
    return true;
}

void visionPipe::close() {
    running_ = false;
    if (readerThread_.joinable()) {
        readerThread_.join();
    }
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

void visionPipe::readLoop() {
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

            detection det{};
            std::istringstream iss(msg);
            if (iss >> det.classId >> det.cx >> det.cy >> det.width >> det.height >> det.confidence) {
                std::lock_guard<std::mutex> lock(mutex_);
                latestDetection_ = det;
                hasNewData_ = true;
            }
        }
    }
}

bool visionPipe::readLatest(detection& det) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!hasNewData_) return false;
    det = latestDetection_;
    hasNewData_ = false;
    return true;
}

std::vector<detection> visionPipe::readAll() {
    std::vector<detection> result;
    detection det;
    while (readLatest(det)) {
        result.push_back(det);
    }
    return result;
}

bool visionPipe::isOpen() const { return running_.load(); }

int visionPipe::imageWidth() const { return imgW_; }
int visionPipe::imageHeight() const { return imgH_; }
double visionPipe::imageCx() const { return imgW_ / 2.0; }
double visionPipe::imageCy() const { return imgH_ / 2.0; }

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
