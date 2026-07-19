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
    // 实际上从相机位置指向地面: camD + t * rNed.z = -altitude (地平面)
    // 但注意: 我们以无人机NED位置为参考, 地面在NED D=0处
    // 相机NED位置: camN, camE, camD (-altitude + 相机偏移z分量)
    // 地面平面: z = 0 (在相对NED系中, 地面z=0)
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

    // 8. 真实直径估算 (相似三角形, 用fx近似)
    if (radiusPx > 0) {
        result.diameter = radiusPx * altitude * 2.0 / intrinsics.fx;
    } else {
        result.diameter = 0;
    }

    return result;
}
