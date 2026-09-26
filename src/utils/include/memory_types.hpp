#pragma once

#include <eigen3/Eigen/Dense>

enum SOLVER_TYPE {
    ITERATIVE,
    ANALYTICAL
};

typedef Eigen::Matrix<int, 3, 1> int3;
typedef Eigen::Matrix<double, 6, 1> vec6;
typedef Eigen::Matrix<double, 5, 1> vec5;
typedef Eigen::Matrix<double, 3, 1> xyz;
typedef Eigen::Matrix<double, 6, 6> mat6x6;

//jjust to run code
typedef Eigen::Matrix<double, 3, 3> mat3x3;
typedef Eigen::Matrix<double, 3, 1> vec3;
typedef Eigen::Matrix<double, 4, 1> vec4;
typedef Eigen::Matrix<double, 12, 1> vec12;

typedef struct CobotJoystickData {
    double left_stick_y = 0;
    double left_stick_x = 0;
    double right_stick_x = 0;
    double right_stick_y = 0;
    double right_trigger = 0;
    double left_trigger = 0;
    double dpad_x = 0;
    double dpad_y = 0;
    int mode = 0;

    void copy(const CobotJoystickData& jd) {
        this->left_stick_y = jd.left_stick_y;
        this->left_stick_x = jd.left_stick_x;
        this->right_stick_x = jd.right_stick_x;
        this->right_stick_y = jd.right_stick_y;
        this->left_trigger = jd.left_trigger;
        this->right_trigger = jd.right_trigger;
        this->dpad_x = jd.dpad_x;
        this->dpad_y = jd.dpad_y;
        this->mode = jd.mode;
    }

    void setZero() {
        left_stick_y = 0;
        left_stick_x = 0;
        right_stick_x = 0;
        right_stick_y = 0;
        right_trigger = 0;
        left_trigger = 0;
        dpad_x = 0;
        dpad_y = 0;
        mode = 0;
    }

} CobotJoystickData;

typedef struct CobotSensorData {
    vec6 q, qd, tau;
    CobotSensorData() {
        q = vec6::Zero();
        qd = vec6::Zero();
        tau = vec6::Zero();
    }

    void copy(const CobotSensorData& sd) {
        this->q = sd.q;
        this->qd = sd.qd;
        this->tau = sd.tau;
    }
    // bool hasNanInf() const {
    //     // Check all vector fields
    //     if (q.hasNaN() || !q.allFinite() ||
    //         qd.hasNaN() || !qd.allFinite() ||
    //         tau.hasNaN() || !tau.allFinite() ||) {
    //         return true;
    //     }
    //     return false;
    // }
} CobotSensorData;

typedef struct CobotEstimationData {
    vec6 pE; // Position of End-Effector [x, y, z, phi, psi, theta]
    vec6 vE; // rate of change of pE
    int ps;  // pick state

    // Joint state (pos), velocity, acceleration
    vec6 js;
    vec6 jv;
    vec6 ja;

    CobotEstimationData() {
        pE = vec6::Zero();
        vE = vec6::Zero();
        ps = 0;
        js = vec6::Zero();
        jv = vec6::Zero();
        ja = vec6::Zero();
    }

    void copy(const CobotEstimationData& est) {
        this->pE = est.pE;
        this->vE = est.vE;
        this->ps = est.ps;
        this->js = est.js;
        this->jv = est.jv;
        this->ja = est.ja;
    }
    bool hasNanInf() const {
        // Check all vector fields
        if (pE.hasNaN() || !pE.allFinite() ||
            vE.hasNaN() || !vE.allFinite() ||
            // ps.hasNaN() || !ps.Finite() ||
            js.hasNaN() || !js.allFinite() ||
            jv.hasNaN() || !jv.allFinite() ||
            ja.hasNaN() || !ja.allFinite()) {
            return true;
        }
        return false;
    }
} CobotEstimationData;

typedef struct CobotCommandData {
    vec6 q, qd, tau, kp, kd;
    CobotCommandData() {
        q   = vec6::Zero();
        qd  = vec6::Zero();
        tau = vec6::Zero();
        kp  = vec6::Zero();
        kd  = vec6::Zero();
    }
    void copy(const CobotCommandData& cmd) {
        this->q = cmd.q;
        this->qd = cmd.qd;
        this->tau = cmd.tau;
        this->kp = cmd.kp;
        this->kd = cmd.kd;
    }
} CobotCommandData;

typedef struct CobotPlannerData {
    vec6 x;
    vec6 xd, xdd;
    int mode;
    CobotPlannerData() {
        x = vec6::Zero();
        xd = vec6::Zero();
        xdd = vec6::Zero();
        mode = 0;
    }
    bool hasNanInf() const {
        // Check all vector fields
        if (x.hasNaN() || !x.allFinite() ||
            xd.hasNaN() || !xd.allFinite() ||
            xdd.hasNaN() || !xdd.allFinite()) {
            return true;
        }
        // No need to check cs_ref (integer vector) and mode (integer)
        return false;
    }
} CobotPlannerData;

typedef struct CobotCliData {
    vec6 x;
    double duration;
    int mode = 0; // 1: sleep, 2: fixed stand, 3: free stand, 4: move

    // Constructor with default initialization
    CobotCliData() 
        : x(0.2, 0, 0.182, 0, 0, 0), duration(5), mode(0) {}

    void copy(const CobotCliData& cd) {
        this->x = cd.x;
        this->duration = cd.duration;
        this->mode = cd.mode;
    }

    void setZero() {
        x = vec6(0.2,0,0.182,0,0,0);
        duration = 5;
        mode = 0;
    }

} CobotCliData;