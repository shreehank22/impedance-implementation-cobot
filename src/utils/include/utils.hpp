#pragma once

#include <memory_types.hpp>
#include <iostream>
#include "cpputils.hpp"

#define __FILENAME__ (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)

// Define the DEBUG_COUT macro
#define DEBUG_COUT(message) \
    std::cout << "[" << __FILENAME__ << ":" << __LINE__ << "] " << message << std::endl;

enum ROT_SEQ {
    XYZ,
    ZYX,
    ZYZ
};

template <typename T> 
int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

template<typename T>
T saturate(const T& val, const T& min, const T& max) {
    T lower = min, upper = max;
    if (min > max) {
        lower = max;
        upper = min;
    }

    if (val >= max)
    {
        // std::cout << "Saturated at max.\n";
    }
    else if (val <= min)
    {
        // std::cout << "Saturated at min.\n";
    }

    return std::min(std::max(val, lower), upper);
}



namespace pinocchio {
    inline mat3x3 SkewSymm(const vec3 &v) {
        mat3x3 vx;
        vx << 0, -v(2), v(1),
            v(2), 0, -v(0),
            -v(1), v(0), 0;

        return vx;
    }

    inline mat3x3 Rx(double x)
    {
        mat3x3 R = mat3x3::Zero();

        R << 1, 0, 0,
            0, cos(x), -sin(x),
            0, sin(x), cos(x);

        return R;
    }
    
    inline mat3x3 Ry(double x)
    {
        mat3x3 R = mat3x3::Zero();

        R << cos(x), 0, sin(x),
            0, 1, 0,
            -sin(x), 0, cos(x);

        return R;
    }
    
    inline mat3x3 Rz(double x)
    {
        mat3x3 R = mat3x3::Zero();

        R << cos(x), -sin(x), 0,
            sin(x), cos(x), 0,
            0, 0, 1;

        return R;
    }

    inline mat3x3 EulXYZToRot(const vec3 &eul)
    {
        // XYZ intrinsic euler angles, converts from base to global
        return Rx(eul(0)) * Ry(eul(1)) * Rz(eul(2));
    }

    inline mat3x3 QuatToRot(const vec4 &q) {
        // pinocchio convention of {x, y, z, w}
        double q0 = q(3);
        vec3 qv = q.block<3, 1>(0, 0);
        mat3x3 Rot = (2 * q0 * q0 - 1) * mat3x3::Identity() + 2 * q0 * SkewSymm(qv) + 2 * qv * qv.transpose();

        return Rot;
    }

    inline vec4 Zeta(const vec3 &v) {
        double v_norm = v.norm();
        vec4 q = vec4(0, 0, 0, 1);
        q(3) = cos(v_norm / 2);
        if (v_norm == 0) {
            q.block<3, 1>(0, 0) = vec3::Zero(3);
        } else {
            q.block<3, 1>(0, 0) = v * sin(v_norm / 2) / v_norm;
        }
        return q;
    }

    inline vec4 RotToAxisAngle(const mat3x3 &R) {
        vec4 axang = vec4::Zero();

        double tmp = (R.trace() - 1) / 2;
        axang.block<3, 1>(1, 0) = vec3(0, 0, 0);
        if (tmp >= 1.) {
            axang(0) = 0;
        } else if (tmp <= -1.) {
            axang(0) = M_PI;
        } else {
            axang(0) = std::acos(tmp);
            axang.block<3, 1>(1, 0) = (0.5 / sin(axang(0))) * vec3(R(2, 1) - R(1, 2), R(0, 2) - R(2, 0), R(1, 0) - R(0, 1));
        }

        return axang;
    }

    inline vec3 RotToPhi(const mat3x3 &R) {
        vec4 axang = RotToAxisAngle(R);

        return axang(0) * axang.block<3, 1>(1, 0);
    }

    inline vec4 EulXYZToQuat(const vec3& eul) {
        return Zeta(RotToPhi(EulXYZToRot(eul)));
    }

    inline vec4 EulToQuat(const vec3& eul, ROT_SEQ seq = ROT_SEQ::XYZ) {

        // the implementation of this function is from MATLAB's eul2quat function

        vec4 quaternion = vec4(0, 0, 0, 1); // order (x, y, z, w)

        vec3 c = vec3(cos(eul(0)/2), cos(eul(1)/2), cos(eul(2)/2));
        vec3 s = vec3(sin(eul(0)/2), sin(eul(1)/2), sin(eul(2)/2));

        switch (seq) {
            case ROT_SEQ::XYZ:
                // Compute quaternion for XYZ rotation sequence
                quaternion(3) = c(0) * c(1) * c(2) - s(0) * s(1) * s(2);
                quaternion(0) = s(0) * c(1) * c(2) + c(0) * s(1) * s(2);
                quaternion(1) = -s(0) * c(1) * s(2) + c(0) * s(1) * c(2);
                quaternion(2) = c(0) * c(1) * s(2) + s(0) * s(1) * c(2);
                break;

            case ROT_SEQ::ZYX:
                // Compute quaternion for ZYX rotation sequence
                quaternion(3) = c(0) * c(1) * c(2) + s(0) * s(1) * s(2);
                quaternion(0) = c(0) * c(1) * s(2) - s(0) * s(1) * c(2);
                quaternion(1) = c(0) * s(1) * c(2) + s(0) * c(1) * s(2);
                quaternion(2) = s(0) * c(1) * c(2) - c(0) * s(1) * s(2);
                break;

            case ROT_SEQ::ZYZ:
                // Compute quaternion for ZYZ rotation sequence
                quaternion(3) = c(0) * c(1) * c(2) - s(0) * c(1) * s(2);
                quaternion(0) = c(0) * s(1) * s(2) - s(0) * s(1) * c(2);
                quaternion(1) = c(0) * s(1) * c(2) + s(0) * s(1) * s(2);
                quaternion(2) = s(0) * c(1) * c(2) + c(0) * c(1) * s(2);
                break;

            default:
                // Handle unexpected rotation sequence
                throw std::invalid_argument("Unknown rotation sequence");
        }

        return quaternion;
    }

    inline vec3 RotToEulXYZ(const mat3x3& R) {
        // reference: https://www.geometrictools.com/Documentation/EulerAngles.pdf : Page 4-5
        vec3 eul(0, 0, 0);
        if (R(0, 2) < 1) {
            if (R(0, 2) > -1) {
                eul(0) = atan2(-R(1, 2), R(2, 2));
                eul(1) = asin(R(0, 2));
                eul(2) = atan2(-R(0, 1), R(0, 0));
            } else {
                eul(0) = -atan2(R(1, 0), R(1, 1));
                eul(1) = -M_PI / 2;
                eul(2) = 0;
            }
        } else {
            eul(0) = atan2(R(1, 0), R(1, 1));
            eul(1) = M_PI / 2;
            eul(2) = 0;
        }

        return eul;
    }
    
    inline vec4 RotToQuat(const mat3x3 &R) {
        return EulXYZToQuat(RotToEulXYZ(R));
    }

    inline vec3 MatrixLogRot(const mat3x3 &R) {
        double theta;

        double tmp = (R.trace() - 1) / 2;
        if (tmp >= 1.) {
            theta = 0;
        } else if (tmp <= -1.) {
            theta = M_PI;
        } else {
            theta = std::acos(tmp);
        }

        vec3 omega = vec3(R(2, 1) - R(1, 2), R(0, 2) - R(2, 0), R(1, 0) - R(0, 1));

        if (theta > 1e-4) {
            omega *= theta / (2 * sin(theta));
        }
        else {
            omega /= 2;
        }

        return omega;
    }

    inline vec3 QuatToEulXYZ(vec4 quaternion) {

        vec3 eulXYZ = vec3::Zero();

        double magnitude = quaternion.norm();

        quaternion = quaternion/magnitude;

        // double qw = quaternion(3);
        // double qx = quaternion(0);
        // double qy = quaternion(1);
        // double qz = quaternion(2);

        // from MATLAB's implementation quat2eul
        // eulXYZ(0) = std::atan2( -2 * (qy*qz - qx*qw), qw*qw - qx*qx - qy*qy + qz*qz);
        // eulXYZ(1) = std::asin( 2 * (qx*qz + qy*qw) );
        // eulXYZ(2) = std::atan2( -2 * (qx*qy - qz*qw), qw*qw + qx*qx - qy*qy - qz*qz);

        // ChatGPT's mixed fruit juice
        // eulXYZ(0) = std::atan2(2 * (qy*qz + qx*qw), qw*qw - qx*qx - qy*qy + qz*qz);
        // eulXYZ(1) = std::asin(std::max(std::min(2 * ( - qx*qz + qy*qw), 1.0), -1.0));
        // eulXYZ(2) = std::atan2(2 * (qx*qy + qz*qw), qw*qw + qx*qx - qy*qy - qz*qz);

        // return eulXYZ;

        return RotToEulXYZ(QuatToRot(quaternion));
    }
}

inline mat3x3 GetPlaneRotationMatrix(const vec3& plane_normal) {
    vec3 np = plane_normal.normalized();
    double nx = np(0);
    double ny = np(1);
    double nz = np(2);

    mat3x3 R;
    R.setZero();

    // vec3 v1 = vec3(plane_normal(0), plane_normal(1), 0);

    // vec3 u2 = v1.cross(plane_normal).normalized();
    // vec3 u1 = u2.cross(plane_normal).normalized();
    vec3 z_global = vec3(0, 0, 1);
    vec3 rotation_axis = z_global.cross(np);
    if (rotation_axis.norm() < 1e-3) {
        if (z_global.dot(np) > 0) {
            R = mat3x3::Identity();
        } else {
            R = pinocchio::Rx(M_PI);
        }
        return R;
    }

    rotation_axis.normalize();

    double cos_phi = z_global.dot(np);
    double phi = std::acos(cos_phi);

    mat3x3 K = pinocchio::SkewSymm(rotation_axis);
    R = mat3x3::Identity() + sin(phi) * K + (1 - cos_phi) * K * K;

    // R.block<3,1>(0, 0) = u1;
    // R.block<3,1>(0, 1) = u2;
    // R.block<3,1>(0, 2) = plane_normal;

    return R;
}

inline double height_map(const vec3& np, const double& x, const double& y) {
    if (np(2) == 0) {
        std::cout << "Plane is vertical! Can't calculate height map value!\n";
        return 0;
    }
    return (-np(0) * x + np(1) * y) / np(2);
}