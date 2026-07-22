#include "coordinateMapper.h"

// ── Matrix33 ────────────────────────────────────────────

Matrix33::Matrix33() {
    for (int i = 0; i < 9; ++i) m[i] = 0;
    m[0] = m[4] = m[8] = 1;
}

Matrix33::Matrix33(double m00, double m01, double m02,
                   double m10, double m11, double m12,
                   double m20, double m21, double m22) {
    m[0] = m00; m[1] = m01; m[2] = m02;
    m[3] = m10; m[4] = m11; m[5] = m12;
    m[6] = m20; m[7] = m21; m[8] = m22;
}

// ── 欧拉角 → 旋转矩阵 (ENU 或 body→world, NED变体) ───────
// roll:  绕 X 轴 (forward)
// pitch: 绕 Y 轴 (right)
// yaw:   绕 Z 轴 (down)
Matrix33 eulerToRotation(double roll, double pitch, double yaw) {
    double cr = std::cos(roll),  sr = std::sin(roll);
    double cp = std::cos(pitch), sp = std::sin(pitch);
    double cy = std::cos(yaw),   sy = std::sin(yaw);

    // Rz(yaw) * Ry(pitch) * Rx(roll)
    return Matrix33(
        cy * cp,  cy * sp * sr - sy * cr,  cy * sp * cr + sy * sr,
        sy * cp,  sy * sp * sr + cy * cr,  sy * sp * cr - cy * sr,
        -sp,      cp * sr,                 cp * cr
    );
}

// ── 相机系 → 机体系 (相机朝下安装) ────────────────────────
// 相机系: Xc=右, Yc=下, Zc=前(指向地面)
// 机体系: Xb=前(North), Yb=右(East), Zb=下(Down)
// 映射:  Xb = Yc(图像行→前进方向),  Yb = Xc(图像列→右),  Zb = Zc(相机前方=朝下)
Matrix33 cameraToBodyRotation() {
    return Matrix33(
         0,  1,  0,
         1,  0,  0,
         0,  0,  1
    );
}

// ── 矩阵乘法 ──────────────────────────────────────────────
Matrix33 matMul(const Matrix33& a, const Matrix33& b) {
    Matrix33 r;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) {
            r.m[i * 3 + j] = a.m[i * 3 + 0] * b.m[0 * 3 + j]
                           + a.m[i * 3 + 1] * b.m[1 * 3 + j]
                           + a.m[i * 3 + 2] * b.m[2 * 3 + j];
        }
    return r;
}

Vector3 matVecMul(const Matrix33& m, const Vector3& v) {
    return {
        m.m[0] * v.x + m.m[1] * v.y + m.m[2] * v.z,
        m.m[3] * v.x + m.m[4] * v.y + m.m[5] * v.z,
        m.m[6] * v.x + m.m[7] * v.y + m.m[8] * v.z
    };
}

// ── 像素→世界坐标 ────────────────────────────────────────
WorldTarget pixelToWorld(double u, double v, double radiusPx,
                         const CameraIntrinsics& intrinsics,
                         const CameraExtrinsics& extrinsics,
                         double altitude,
                         double roll, double pitch, double yaw,
                         double droneNorth, double droneEast) {
    WorldTarget result{0, 0, 0, false};

    if (altitude < 0.1) altitude = 0.1;

    // 1. 像素归一化
    double xn = (u - intrinsics.cx) / intrinsics.fx;
    double yn = (v - intrinsics.cy) / intrinsics.fy;

    // 2. 相机系射线方向
    Vector3 rCam = {xn, yn, 1.0};

    // 3. 相机系 → 机体系
    Matrix33 R_c2b = cameraToBodyRotation();
    Vector3 rBody = matVecMul(R_c2b, rCam);

    // 4. 机体系 → NED世界系
    Matrix33 R_b2n = eulerToRotation(roll, pitch, yaw);
    Vector3 rNed = matVecMul(R_b2n, rBody);

    // 5. 相机安装偏移 (机体系)
    Vector3 camOffsetBody = {extrinsics.offsetForward,
                             extrinsics.offsetRight,
                             extrinsics.offsetDown};
    Vector3 camOffsetNed = matVecMul(R_b2n, camOffsetBody);

    // 6. 相机在NED系的位置 (仅使用N/E分量, Z分量通过groundZ计算)
    double camN = droneNorth + camOffsetNed.x;
    double camE = droneEast  + camOffsetNed.y;

    // 7. 与地平面求交 (地面在 NED D = -altitude, 即z_ground = -altitude)
    double groundZ = 0.0;
    double camZ = -altitude + camOffsetNed.z;  // NED Z (正值=下)

    if (std::abs(rNed.z) < 1e-9) {
        return result;
    }

    double t = (groundZ - camZ) / rNed.z;
    if (t <= 0) {
        return result;
    }

    result.north = camN + t * rNed.x;
    result.east  = camE + t * rNed.y;
    result.valid = true;

    // 8. 真实直径估算 (使用斜距替代高度, 确保不同高度下同一目标直径一致)
    if (radiusPx > 0) {
        double slantDist = t * std::sqrt(rNed.x * rNed.x + rNed.y * rNed.y + rNed.z * rNed.z);
        result.diameter = radiusPx * slantDist * 2.0 / intrinsics.fx;
    } else {
        result.diameter = 0;
    }

    return result;
}

// ── 世界坐标 → 像素坐标 ────────────────────────────────────
bool worldToPixel(double worldN, double worldE,
                  const CameraIntrinsics& intrinsics,
                  const CameraExtrinsics& extrinsics,
                  double altitude, double roll, double pitch, double yaw,
                  double droneNorth, double droneEast,
                  double& u, double& v) {
    if (altitude < 0.1) altitude = 0.1;

    Matrix33 R_b2n = eulerToRotation(roll, pitch, yaw);
    Vector3 camOffsetBody = {extrinsics.offsetForward,
                             extrinsics.offsetRight,
                             extrinsics.offsetDown};
    Vector3 camOffsetNed = matVecMul(R_b2n, camOffsetBody);

    double camZ = -altitude + camOffsetNed.z;

    // NED向量: 从相机指向世界目标
    double dN = worldN - droneNorth - camOffsetNed.x;
    double dE = worldE - droneEast  - camOffsetNed.y;
    double dD = 0.0 - camZ;   // 地面在z=0

    // R_b2n^T * [dN, dE, dD]^T (旋转矩阵的逆=转置)
    double cy = std::cos(yaw), sy = std::sin(yaw);
    double cp = std::cos(pitch), sp = std::sin(pitch);
    double cr = std::cos(roll),  sr = std::sin(roll);

    double bod_x = cy*cp*dN + sy*cp*dE - sp*dD;
    double bod_y = (cy*sp*sr - sy*cr)*dN + (sy*sp*sr + cy*cr)*dE + cp*sr*dD;
    double bod_z = (cy*sp*cr + sy*sr)*dN + (sy*sp*cr - cy*sr)*dE + cp*cr*dD;

    // R_c2b^T: camera = R_c2b^T * body (R_c2b = [[0,1,0],[1,0,0],[0,0,1]])
    double cam_x = bod_y;
    double cam_y = bod_x;
    double cam_z = bod_z;

    if (cam_z <= 0) return false;   // 点在相机后方

    double xn = cam_x / cam_z;
    double yn = cam_y / cam_z;

    u = xn * intrinsics.fx + intrinsics.cx;
    v = yn * intrinsics.fy + intrinsics.cy;
    return true;
}
