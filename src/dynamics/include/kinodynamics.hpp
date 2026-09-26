#pragma once

#include <Eigen/Dense>
#include <string>

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>

#include <utils.hpp>   // vec6, mat6x6

class Cobot
{
public:
  
    Cobot();
    explicit Cobot(const std::string &urdf_path);
    ~Cobot() = default;


    int getDOF() const;
    const pinocchio::Model& getModel() const { return model; }
    pinocchio::Data& getData() { return data; }

    vec6   forwardKinematics(const vec6 &q);
    vec6   inverseKinematics(const vec6 &x);
    bool   isReachable(const vec6 &x) const;
    mat6x6 JacobianCompute(const vec6 &q);
    vec6 getPinocchioFK(const vec6 &q);
    void showParams();

  
    vec6 getEndEffectorVelocity(const vec6 &q, const vec6 &qd);
    vec6 getJointVelocity(const vec6 &q, const vec6 &xd);


    vec6 getSleepStates();
    void updateParams(const float &l1,
                      const float &l2,
                      const float &l3,
                      const float &l4);

    Eigen::MatrixXd PinJacobian(const pinocchio::Model& model, 
    pinocchio::Data& data, 
    const Eigen::VectorXd& q,
const std::string& ee_frame_name, const vec6& FK);
    Eigen::MatrixXd getMassMatrix(const Eigen::VectorXd &q);
    Eigen::MatrixXd getCoriolisMatrix(const Eigen::VectorXd &q,
                                      const Eigen::VectorXd &qd);
    Eigen::VectorXd ComputeGeneralizedGravity(const Eigen::VectorXd &q);
    Eigen::VectorXd getGravityVector(const Eigen::VectorXd &q);
    Eigen::Matrix<double, 6, 1> wrench_disturbance(
    const pinocchio::Model& model, 
    pinocchio::Data& data, 
    const Eigen::VectorXd& q,
    const Eigen::VectorXd& q_dot,
    const Eigen::VectorXd& q_ddot,
    const Eigen::VectorXd& tau,
    const mat6x6 J);

    const std::string& getURDFFilepath() const { return urdf_filepath; }

private:
    void initClass();
    std::string urdf_filepath;

    pinocchio::Model model;
    pinocchio::Data  data;

    vec6 offsets;
    vec6 q_sleep;
    vec6 x_sleep;

    double l1{0.0}, l2{0.0}, l3{0.0}, l4{0.0};
};