#pragma once
#include <string>
#include <vector>
#include <atomic>
#include <thread>
#include <mutex>
#include "types.h"

struct visionBinaryData {
    float cx;
    float cy;
    float confidence;
};

class binaryVisionPipe {
public:
    explicit binaryVisionPipe(const std::string& path);
    ~binaryVisionPipe();

    bool open();
    void close();
    bool readLatest(visionBinaryData& data);
    bool isOpen() const;

private:
    std::string path_;
    int fd_;
};

class visionPipe {
public:
    explicit visionPipe(const std::string& path, int imgW = 640, int imgH = 640);
    ~visionPipe();

    bool open(bool createPipe = true);
    void close();

    bool readLatest(detection& det);
    std::vector<detection> readAll();
    bool isOpen() const;

    int imageWidth() const;
    int imageHeight() const;
    double imageCx() const;
    double imageCy() const;

private:
    void readLoop();
    std::string path_;
    int imgW_;
    int imgH_;
    int fd_;
    std::atomic<bool> running_;
    std::thread readerThread_;
    mutable std::mutex mutex_;
    detection latestDetection_;
    bool hasNewData_;
};

class missionCmdPipe {
public:
    explicit missionCmdPipe(const std::string& path);
    ~missionCmdPipe();

    bool open();
    void close();
    void sendState(const std::string& state);

private:
    std::string path_;
    int fd_;
};
