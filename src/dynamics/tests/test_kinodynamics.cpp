#include "kinodynamics.hpp"
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <iostream>
#include <vector>
#include <cmath>

int Cobot::getDOF() const {
    return model.nv;
}

void validation_checks(Cobot& robot)
{
    int nv = robot.getDOF();
    double eps = 1e-4;

    Eigen::VectorXd q = Eigen::VectorXd::Zero(nv);
    q << 71.1, -48.7, 110.0, -25.2, 44.7, -63.0;

    Eigen::VectorXd offsets(6);
    offsets << 90, -90, 90, 0, 90, 0;

    q = (q - offsets) * M_PI / 180.0;


    Eigen::VectorXd q_dot(nv);
    q_dot << 0.05, -0.02, 0.03, 0.01, 0.0, 0.0;
    Eigen::VectorXd q_ddot(nv);
    q_ddot << 0,0,0,0,0,0;


    Eigen::MatrixXd M = robot.getMassMatrix(q);
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig(M);
    double min_eig = eig.eigenvalues().minCoeff();

    std::cout << "[MASS MATRIX EIGENVALUE CHECK]\n";
    std::cout << "Minimum Eigenvalue: " << min_eig << "\n";

    Eigen::LLT<Eigen::MatrixXd> llt(M);

    // Check for Mass Matrix, Coriolis Terms and Gravity Vector
    std::cout << "[MASS MATRIX VALIDATION CHECK]\n";
    std::cout << "Symmetric: " << (M.isApprox(M.transpose(), 1e-9) ? "YES" : "NO") << "\n";
    std::cout << "Positive Definite: " << (llt.info() == Eigen::Success ? "YES" : "NO") << "\n";
    Eigen::MatrixXd M_next = robot.getMassMatrix(q + q_dot * eps);
    Eigen::MatrixXd M_dot = (M_next - M) / eps;
    Eigen::MatrixXd C = robot.getCoriolisMatrix(q, q_dot);
    Eigen::MatrixXd N = M_dot - 2.0 * C;
    
    Eigen::VectorXd G = robot.getGravityVector(q);
    Eigen::VectorXd G_pin = robot.ComputeGeneralizedGravity(q);
    Eigen::VectorXd G_diff = G - G_pin;
    std::cout << "Gravity Vector Difference Norm: " << G_diff.norm() << "\n";
    
    std::cout << "[GRAVITY VECTOR CHECK]\n";
    std::cout << "G: " << G.transpose() << "\n";
    std::cout << "G_pin: " << G_pin.transpose() << "\n";
    std::cout << "Difference: " << (G - G_pin).norm() << "\n";

    std::cout << "[CORIOLIS SKEW SYMMETRY CHECK]\n";
    std::cout << "Skew Symmetric: " << (N.isApprox(-N.transpose(), 1e-4) ? "YES" : "NO") << "\n";

    Eigen::MatrixXd J = robot.JacobianCompute(q);
    Eigen::FullPivLU<Eigen::MatrixXd> lu(J);
    std::cout << "Rank: " << lu.rank() << " / " << std::min(6, nv) << "\n";

    Eigen::VectorXd ee_pose = robot.forwardKinematics(q);
    Eigen::VectorXd ee_pose_next = robot.forwardKinematics(q + q_dot * eps);
    Eigen::VectorXd ee_vel_num = (ee_pose_next - ee_pose) / eps;

    Eigen::VectorXd force(6);
    force << 0, 0, -2, 0, 0, 0;
    Eigen::VectorXd ee_vel_analytical = J * q_dot;
    Eigen::VectorXd vel_error = ee_vel_num - ee_vel_analytical;
    Eigen::VectorXd tau = J.transpose()*force;
    vec6 q_test;
    q << 0.0, -0.25, 0.55, -0.2, 0.0, 0.0;
    vec6 x_res = robot.forwardKinematics(q);
    std::cout << "[FK] q: " << q.transpose() << std::endl;
    std::cout << "[FK] x: " << x_res.transpose() << std::endl;
    std::cout << "[VELOCITY CONSISTENCY CHECK FOR JACOBIAN]\n";
    std::cout << "Max Error: " << vel_error.cwiseAbs().maxCoeff() << "\n";
    std::cout << "Consistent: " << (vel_error.cwiseAbs().maxCoeff() < eps ? "YES" : "NO") << "\n";



    std::cout <<"Jacobian from Pinocchio\n";
    Eigen::MatrixXd J_pin = robot.PinJacobian(robot.getModel(), robot.getData(),q,"finger_link", ee_pose);
    std::cout <<J_pin<<"\n";
    std::cout <<"Jacobian from Function\n";
    std::cout <<J<<"\n";

    Eigen::Matrix<double, 6, 1> disturbance = robot.wrench_disturbance(robot.getModel(), robot.getData(),q,q_dot,q_ddot,tau,J);
    std::cout << "Wrench " << disturbance.transpose() << "\n";
}

void fk_ik_tests(Cobot& robot)
{
     vec6 q_test;
    q_test << 0, 0, 0, 0, 0, 0;

    vec6 ee_pose = robot.forwardKinematics(q_test);
    std::cout << "Forward Kinematics Result: " << ee_pose.transpose() << "\n";

    vec6 ee_home;
    ee_home << 0.0, -0.032, 0.071, -0.039, 0.0, 0.0;
    vec6 q_ik = robot.inverseKinematics(ee_home);
    std::cout << "Inverse Kinematics Result: " << q_ik.transpose() << "\n";

    std::vector<double> errors;
    vec6 ee_intermediate = ee_home;
    vec6 q_intermediate = q_test;
    for (int i = 0; i < 10; i++) {
        vec6 q_result = robot.inverseKinematics(ee_intermediate);
        vec6 ee_intermediate = robot.forwardKinematics(q_result);
        vec6 ee_result = robot.forwardKinematics(q_intermediate);
        std::cout << "Iteration " << i << " - EE Pose: " << ee_result.transpose() << "\n";
        vec6 q_intermediate = robot.inverseKinematics(ee_result);
        double err = (ee_home - ee_intermediate).norm();
        std::cout << "Iteration " << i << " - Joint Angles: " << q_intermediate.transpose() << "\n";
        // double err = (q_test - q_intermediate).norm();
        errors.push_back(err);
    }

    double mean_error = 0.0, min_error = errors[0], max_error = errors[0];
    for (double e : errors) {
        mean_error += e;
        min_error = std::min(min_error, e);
        max_error = std::max(max_error, e);
    }
    mean_error /= errors.size();

    double variance = 0.0;
    for (double e : errors) {
        variance += (e - mean_error) * (e - mean_error);
    }
    variance /= errors.size();
    double std_dev = std::sqrt(variance);

    std::cout << "\n--- IK->FK Round-trip Statistics (10 iterations) ---\n";
    std::cout << "Mean Error: " << mean_error << "\n";
    std::cout << "Std Dev: " << std_dev << "\n";
    std::cout << "Min Error: " << min_error << "\n";
    std::cout << "Max Error: " << max_error << "\n";
}

int main(int argc, char** argv)
{
    std::string robot_name = "cobot_c1";
    if (argc >= 2)
        robot_name = std::string(argv[1]);

    std::string base_path = "/home/shreehan/RIMaR/src/robots";
    std::string urdf_path = base_path + "/" + robot_name +
                            "_description/urdf/" +
                            robot_name + "_description.urdf";

    try {
        Cobot robot(urdf_path);
        validation_checks(robot);
        fk_ik_tests(robot);
    }
    catch (const std::exception& e) {
        std::cerr << "Execution failed: " << e.what() << "\n";
    }

    return 0;
}
