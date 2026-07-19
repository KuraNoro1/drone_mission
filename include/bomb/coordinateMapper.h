#pragma once
#include <cmath>

struct CameraIntrinsics {
    double fx;  // focal length x (pixels)
    double fy;  // focal length y (pixels)
    double cx;  // principal point x (pixels)
    double cy;  // principal point y (pixels)
};

struct CameraExtrinsics {
    double offsetForward;  // camera forward offset in body frame (m)
    double offsetRight;    // camera right offset in body frame (m)
    double offsetDown;     // camera down offset in body frame (m)
};

struct WorldTarget {
    double north;     // NED North (m)
    double east;      // NED East (m)
    double diameter;  // world diameter (m)
    bool valid;       // whether the result is valid
};

// ── 3x3 旋转矩阵 ────────────────────────────────────────
struct Matrix33 {
    double m[9];
    Matrix33();
    Matrix33(double m00, double m01, double m02,
             double m10, double m11, double m12,
             double m20, double m21, double m22);
};

struct Vector3 {
    double x, y, z;
};

// ── 姿态转换工具 ────────────────────────────────────────
Matrix33 eulerToRotation(double roll, double pitch, double yaw);
Matrix33 cameraToBodyRotation();
Matrix33 matMul(const Matrix33& a, const Matrix33& b);
Vector3 matVecMul(const Matrix33& m, const Vector3& v);

// ── 核心函数：像素坐标 → NED世界坐标 ──────────────────────
// u, v:        像素坐标 (图像坐标系, 原点左上角)
// radiusPx:    像素半径 (用于估算真实直径)
// intrinsics:  相机内参
// extrinsics:  相机外参 (安装位置相对于无人机重心)
// altitude:    相对高度 (m, 正值向上)
// roll, pitch, yaw: 无人机欧拉角 (弧度)
// droneNorth, droneEast: 无人机当前NED位置 (m)
WorldTarget pixelToWorld(double u, double v, double radiusPx,
                         const CameraIntrinsics& intrinsics,
                         const CameraExtrinsics& extrinsics,
                         double altitude,
                         double roll, double pitch, double yaw,
                         double droneNorth, double droneEast);
