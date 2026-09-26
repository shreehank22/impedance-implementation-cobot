#include <kinodynamics.hpp>

#include <Eigen/Dense>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>

#include <Eigen/Dense>
#include <cmath>
#include <algorithm>
#include <cassert>
#include <iostream>

Cobot::Cobot()
{
    std::cout <<"Simulation start>>";
}

Cobot::Cobot(const std::string &urdf_path)
{
    pinocchio::urdf::buildModel(urdf_path, model);
    data = pinocchio::Data(model);
    initClass();
}
void Cobot::showParams() {
    std::cout << "Robot DOF: " << getDOF() << std::endl;
    // Add any other prints you need here
}
void Cobot::initClass()
{
    if (!model.existFrame("finger_link"))
        throw std::runtime_error("Frame 'finger_link' does not exist in the URDF model.");

    offsets << 0, -M_PI, M_PI - 0.2245, 0.2245, 0, 0;

    l1 = 0.11;
    l2 = 0.29;
    l3 = 0.31;
    l4 = 0.20;

    x_sleep << 0.095, 0.0, 0.190, 0.0, 0.0, 0.0;
    q_sleep = inverseKinematics(x_sleep);
}

int Cobot::getDOF() const
{
    return model.nv;
}

vec6 Cobot::forwardKinematics(const vec6 &q)
{
    // Ensure the input has exactly 6 joint angles
    if (q.size() != 6) {
        throw std::invalid_argument("Incorrect Input.");
    }

    // Apply joint offsets
    vec6 joint_angles = q - offsets;

    double t1 = joint_angles(0);
    double t2 = joint_angles(1);
    double t3 = joint_angles(2);
    double t4 = joint_angles(3);
    double t5 = joint_angles(4);
    double t6 = joint_angles(5);

    // Step 1: Intermediate positions
    double z0 = l1;

    double r1 = l2 * cos(t2);
    double z1 = l2 * sin(t2);

    double r2 = l3 * cos(t2 + t3);
    double z2 = l3 * sin(t2 + t3);

    double phi = t2 + t3 + t4;
    double psi = t5;
    double theta = t6;

    double r3 = l4 * cos(phi);
    double z3 = l4 * sin(phi);

    double r = r1 + r2 + r3;

    // Final coordinates in 3D space
    double x = r * cos(t1);
    double y = r * sin(t1);
    double z = z0 + z1 + z2 + z3;

    vec6 ee_pose;
    ee_pose << x, y, z, phi, psi, theta;

    return ee_pose;
}

bool Cobot::isReachable(const vec6 &x) const
{
    if (x.size() != 6) return false;

    double px = x(0), py = x(1), pz = x(2);
    double phi = x(3);

    // Wrist-adjusted radial distance and height
    double r = sqrt(px*px + py*py) - l4 * cos(phi);
    double z = pz - l1 - l4 * sin(phi);
    double D = sqrt(r*r + z*z);

    // Outside arm reach envelope
    if (D > l2 + l3 - 0.01) return false;

    // cos(t3) must be in [-1, 1]; use unclipped value to detect OOB
    double c3 = (D*D - l2*l2 - l3*l3) / (2.0 * l2 * l3);
    if (c3 < -0.995 || c3 > 0.995) return false;

    double s3 = -sqrt(1.0 - c3*c3);
    double t3 = atan2(s3, c3);
    double beta  = atan2(z, r);
    double alpha = atan2(l3*sin(t3), l2 + l3*cos(t3));
    double t2 = beta - alpha;
    double t4 = phi - (t2 + t3);

    // Check all active joints stay within ±π
    double t2_abs = t2 + offsets(1);
    double t3_abs = t3 + offsets(2);
    double t4_abs = t4 + offsets(3);

    if (std::fabs(t2_abs) > M_PI) return false;
    if (std::fabs(t3_abs) > M_PI) return false;
    if (std::fabs(t4_abs) > M_PI) return false;

    return true;
}

vec6 Cobot::inverseKinematics(const vec6 &x)

{
    if (x.size() != 6)
        throw std::invalid_argument("Invalid EE dimension");

    double px = x(0), py = x(1), pz = x(2);
    double phi = x(3), psi = x(4), theta = x(5);

    double t1 = atan2(py, px);

    double r = sqrt(px * px + py * py) - l4 * cos(phi);
    double z = pz - l1 - l4 * sin(phi);

    double D = sqrt(r * r + z * z);

    double c3 = std::clamp((D * D - l2 * l2 - l3 * l3) / (2 * l2 * l3), -0.995, 0.995);
    double s3 = -sqrt(1.0 - c3 * c3);

    double t3 = atan2(s3, c3);

    double beta  = atan2(z, r);
    double alpha = atan2(l3 * sin(t3), l2 + l3 * cos(t3));

    double t2 = beta - alpha;
    double t4 = phi - (t2 + t3);

    vec6 q;
    q << t1, t2, t3, t4, psi, theta;

    return q + offsets;
}



mat6x6 Cobot::JacobianCompute(const vec6 &q) {
    if (q.size() != 6) {
        throw std::invalid_argument("Input must be a 6-element vector [theta1, theta2, theta3, theta4, psi, theta]");
    }

    vec6 joint_angles = q;
    double t1 = joint_angles(0);
    double t2 = joint_angles(1);
    double t3 = joint_angles(2);
    double t4 = joint_angles(3);
    double t5 = joint_angles(4);
    double t6 = joint_angles(5);

    double t23 = t2 + t3;
    double t234 = t23 + t4;

    // Precompute trig values
    double cos_t1 = cos(t1), sin_t1 = sin(t1);
    double sin_t2 = sin(t2), cos_t2 = cos(t2);
    double sin_t23 = sin(t23), cos_t23 = cos(t23);
    double sin_t234 = sin(t234), cos_t234 = cos(t234);

    double r = l2 * cos_t2 + l3 * cos_t23 + l4 * cos_t234;
    double dz2 = l2 * sin_t2 + l3 * sin_t23 + l4 * sin_t234;
    double dz3 = l3 * sin_t23 + l4 * sin_t234;
    double dz4 = l4 * sin_t234;

    double dz2_z = l2 * cos_t2 + l3 * cos_t23 + l4 * cos_t234;
    double dz3_z = l3 * cos_t23 + l4 * cos_t234;
    double dz4_z = l4 * cos_t234;

    mat6x6 J = mat6x6::Zero();

    // Position derivatives
    J(0,0) = -r * sin_t1;
    J(0,1) = -dz2 * cos_t1;
    J(0,2) = -dz3 * cos_t1;
    J(0,3) = -dz4 * cos_t1;

    J(1,0) =  r * cos_t1;
    J(1,1) = -dz2 * sin_t1;
    J(1,2) = -dz3 * sin_t1;
    J(1,3) = -dz4 * sin_t1;

    J(2,1) = dz2_z;
    J(2,2) = dz3_z;
    J(2,3) = dz4_z;

    // Orientation derivatives
    J(3,1) = 1;
    J(3,2) = 1;
    J(3,3) = 1;

    J(4,4) = 1;  // ψ (psi)
    J(5,5) = 1;  // θ (theta)

    return J;
}


vec6 Cobot::getEndEffectorVelocity(const vec6 &q, const vec6 &qd)
{
    return JacobianCompute(q) * qd;
}

vec6 Cobot::getJointVelocity(const vec6 &q, const vec6 &xd)
{
    mat6x6 J = JacobianCompute(q);
    return J.completeOrthogonalDecomposition().solve(xd);
}


vec6 Cobot::getSleepStates()
{
    return x_sleep;
}

void Cobot::updateParams(const float &l1_new,
                         const float &l2_new,
                         const float &l3_new,
                         const float &l4_new)
{
    l1 = l1_new;
    l2 = l2_new;
    l3 = l3_new;
    l4 = l4_new;
}

Eigen::MatrixXd Cobot::getMassMatrix(const Eigen::VectorXd &q)
{
    pinocchio::crba(model, data, q);
    data.M.triangularView<Eigen::StrictlyLower>() =
        data.M.transpose().triangularView<Eigen::StrictlyLower>();
    return data.M;
}

Eigen::MatrixXd Cobot::getCoriolisMatrix(const Eigen::VectorXd &q,
                                         const Eigen::VectorXd &qd)
{
    pinocchio::computeCoriolisMatrix(model, data, q, qd);
    return data.C;
}

Eigen::VectorXd Cobot::ComputeGeneralizedGravity(const Eigen::VectorXd &q)
{
    pinocchio::computeGeneralizedGravity(model, data, q);
    return data.g;
}

Eigen::VectorXd Cobot::getGravityVector(const Eigen::VectorXd &q)
{
    Eigen::VectorXd v = Eigen::VectorXd::Zero(model.nv);
    Eigen::VectorXd a = Eigen::VectorXd::Zero(model.nv);
    pinocchio::rnea(model, data, q, v, a);
    return data.tau;
}
Eigen::MatrixXd Cobot::PinJacobian(const pinocchio::Model& model,
    pinocchio::Data& data,
    const Eigen::VectorXd& q,
    const std::string& ee_frame_name,
    const vec6& FK)
{
    // Compute forward kinematics to update frame poses
    pinocchio::forwardKinematics(model, data, q);

    // Get the frame index from the frame name
    pinocchio::FrameIndex frame_id = model.getFrameId(ee_frame_name);

    if (frame_id >= model.nframes) {
        throw std::runtime_error("Frame '" + ee_frame_name + "' not found in model");
    }

    // Compute the Jacobian in the LOCAL frame of the end-effector
    Eigen::MatrixXd J_local = Eigen::MatrixXd::Zero(6, model.nv);
    pinocchio::computeFrameJacobian(model, data, q, frame_id, pinocchio::LOCAL, J_local);

    return J_local;
}

Eigen::Matrix<double, 6, 1> Cobot::wrench_disturbance(
    const pinocchio::Model& model,
    pinocchio::Data& data,
    const Eigen::VectorXd& q,
    const Eigen::VectorXd& q_dot,
    const Eigen::VectorXd& q_ddot,
    const Eigen::VectorXd& tau,
    const mat6x6 Jb)
{
    assert(q.size() == model.nq);
    assert(q_dot.size() == model.nv);
    assert(q_ddot.size() == model.nv);
    assert(tau.size() == model.nv);
    pinocchio::crba(model,data,q);
    data.M.triangularView<Eigen::StrictlyLower>() =
        data.M.transpose().triangularView<Eigen::StrictlyLower>();
    pinocchio::nonLinearEffects(model, data, q, q_dot);
    Eigen::Matrix<double, 6, Eigen::Dynamic> M_b = data.M.topRows<6>();
    Eigen::Matrix<double, 6, Eigen::Dynamic> C_g = data.nle.head<6>();
    Eigen::VectorXd tau_residual = M_b*q_ddot + C_g - tau;
    Eigen::MatrixXd Jb_T = Jb.transpose();
    Eigen::MatrixXd pinv_Jb = Jb_T.completeOrthogonalDecomposition().pseudoInverse();
    Eigen::Matrix<double,6,1> W_d = pinv_Jb*tau_residual;
    return W_d;
}
