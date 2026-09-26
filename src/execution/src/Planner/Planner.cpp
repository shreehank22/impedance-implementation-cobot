#include "Planner/Planner.hpp"
#include <iomanip>

Planner::Planner(const YAML::Node& config) : m_config(config) {
    initClass();
}

Planner::~Planner() {}

void Planner::initClass()
{
    if (m_robot != nullptr) {
        m_ee_state_init = m_robot->getSleepStates();
        m_target_ee_pose = m_ee_state_init;
    }

    m_ee_state_ref = m_ee_state_init;
    m_ee_vel_ref = vec6::Zero();
    m_ee_acc_ref = vec6::Zero();

    m_joint_state_ref = vec6::Zero();
    m_joint_state_ref(5) = 1;
    m_joint_state_init = m_joint_state_ref;

    m_joint_vel_act = vec6::Zero();
    m_t_curr = 0;

    m_iter = 0;
    m_num_steps = 1250;
    m_duration = 5.0; // sane default duration
}

void Planner::setInitStates(const vec6 &x_init, const vec6 &js_init)
{
    m_ee_state_ref = x_init;
    m_target_joint_pose = vec6::Zero();

    m_ee_vel_ref = vec6::Zero();
    m_ee_acc_ref = vec6::Zero();

    m_joint_state_act = js_init;
    m_joint_state_ref = js_init;
    m_joint_state_init = js_init;
}

void Planner::setInitJointStates(const vec6 &js_init)
{
    m_joint_state_init = js_init;
}

void Planner::startFromSleep(const bool &sleep_start)
{
    m_sleep_start = sleep_start;
    initClass();
}

bool Planner::setTargetJointAngles(const vec6 &joint_target,
                                   const double &duration) {
    m_alpha += m_dt / duration;
    m_alpha = m_alpha >= 1. ? 1. : m_alpha;
    m_joint_state_ref = (1. - m_alpha) * m_joint_state_init +
        m_alpha * joint_target;

    if (m_alpha == 1) {
        return true;
    }
    return false;
}

void Planner::setJointSpacePlanning(const bool &plan_js) {
    m_plan_ee_space = !plan_js;
    m_alpha = 0;
    m_joint_state_ref = m_joint_state_init;
}

vec6 Planner::getReferenceEeStates()
{
    return m_ee_state_ref;
}

void Planner::setDesiredEEVelocity(const vec6 &joy_ee_vel_cmd)
{
    vec6 v_cmd_ee = vec6::Zero();
    vec6 a_cmd_ee = vec6::Zero();

    double dist_ee_xy = sqrt(m_ee_state_ref(0) * m_ee_state_ref(0) +
                              m_ee_state_ref(1) * m_ee_state_ref(1));
    dist_ee_xy = saturate(dist_ee_xy, m_min_dist_ee_xy, m_max_dist_ee_xy);

    mat3x3 rot_yaw = pinocchio::Rz(m_base_yaw);

    // Direct velocity control: map joystick to EE velocity immediately (no PD lag).
    // Velocities in base frame, then rotate X/Y into global frame.
    v_cmd_ee(0) = m_max_ee_vel_x        * joy_ee_vel_cmd(0);
    v_cmd_ee(2) = m_max_ee_vel_z        * joy_ee_vel_cmd(1);
    v_cmd_ee(3) = m_max_ee_vel_pitch    * joy_ee_vel_cmd(2);
    v_cmd_ee(4) = m_max_ee_vel_roll     * joy_ee_vel_cmd(3);
    v_cmd_ee(5) = m_max_ee_vel_base_yaw * joy_ee_vel_cmd(4);
    v_cmd_ee(1) = v_cmd_ee(5) * dist_ee_xy;

    m_ee_acc_ref.block<2,1>(0,0) = rot_yaw.block<2,2>(0,0) * v_cmd_ee.block<2,1>(0,0);
    m_ee_acc_ref(2) = v_cmd_ee(2);
    m_ee_acc_ref(3) = v_cmd_ee(3);
    m_ee_acc_ref(4) = v_cmd_ee(4);
    m_ee_acc_ref(5) = 0.0;
    m_base_yaw_acc  = v_cmd_ee(5);

    // Set velocity reference directly from command (instant response).
    m_ee_vel_ref(0) = m_ee_acc_ref(0);
    m_ee_vel_ref(1) = m_ee_acc_ref(1);
    m_ee_vel_ref(2) = m_ee_acc_ref(2);
    m_ee_vel_ref(3) = m_ee_acc_ref(3);
    m_ee_vel_ref(4) = m_ee_acc_ref(4);
    m_ee_vel_ref(5) = m_max_ee_vel_jaw * joy_ee_vel_cmd(5);

    m_base_yaw_vel = saturate(m_base_yaw_acc,
                              -m_max_ee_vel_base_yaw, m_max_ee_vel_base_yaw);
    
}

void Planner::setEndEffectorTarget()
{
    m_base_yaw = fmod(
        m_joint_state_ref(0) + m_base_yaw_vel * m_dt + 0.5 * m_base_yaw_acc * m_dt * m_dt,
        2 * M_PI);
    
    if (m_base_yaw > M_PI) {
        m_base_yaw -= 2 * M_PI;
    } else if (m_base_yaw < -M_PI) {
        m_base_yaw += 2 * M_PI;
    }

    m_ee_state_ref(0) += m_ee_vel_ref(0) * m_dt + m_ee_acc_ref(0) * m_dt * m_dt;
    m_ee_state_ref(1) += m_ee_vel_ref(1) * m_dt + m_ee_acc_ref(1) * m_dt * m_dt;
    m_ee_state_ref(2) += m_ee_vel_ref(2) * m_dt + m_ee_acc_ref(2) * m_dt * m_dt;
    m_ee_state_ref(3) += m_ee_vel_ref(3) * m_dt + m_ee_acc_ref(3) * m_dt * m_dt;
    m_ee_state_ref(4) += m_ee_vel_ref(4) * m_dt + m_ee_acc_ref(4) * m_dt * m_dt;
    m_ee_state_ref(5) += m_ee_vel_ref(5) * m_dt;
    m_ee_state_ref(5) = saturate(m_ee_state_ref(5), m_min_ee_jaw, m_max_ee_jaw);

    m_ee_state_ref(0) = saturate(m_ee_state_ref(0), m_min_ee_x_global, m_max_ee_x_global);
    m_ee_state_ref(1) = saturate(m_ee_state_ref(1), m_min_ee_y_global, m_max_ee_y_global);
    // z_min enforces 5 cm table clearance for TELEOP as well as P2P.
    m_ee_state_ref(2) = saturate(m_ee_state_ref(2), m_min_ee_z_global, m_max_ee_z_global);

    m_joint_state_ref = m_robot->inverseKinematics(m_ee_state_ref);
}

bool Planner::shouldPlanTrajectory(double threshold)
{
    vec6 ee_pose_current = m_robot->forwardKinematics(m_joint_state_act);
    vec3 pos_diff = m_target_ee_pose.head<3>() - ee_pose_current.head<3>();
    double diff_norm = pos_diff.norm();

    return diff_norm > threshold;
}

void Planner::planJointTrajectoryLinear(const double& dt)
{
    if (m_iter == 0) {
        m_joint_state_start = m_joint_state_act;
        m_trajectory_reported = false;
    }
    else if (m_iter >= m_num_steps) {
        if (!m_trajectory_reported) {
        m_trajectory_reported = true;
        vec6 q_actual  = m_joint_state_act;
        vec6 ee_actual = m_robot->forwardKinematics(q_actual);
        vec6 q_ik      = m_robot->inverseKinematics(m_target_ee_pose);
        vec6 ee_ik_fk  = m_robot->forwardKinematics(q_ik);
        vec6 ee_error  = m_target_ee_pose - ee_actual;

        std::cout << "\n[P2P] ─── Waypoint reached ─────────────────────────────\n";
        std::cout << std::fixed << std::setprecision(5);
        std::cout << "  Target EE pose  (x y z φ ψ θ):  "
                  << m_target_ee_pose.transpose() << "\n";
        std::cout << "  IK solution     (q1..q6) [rad]: "
                  << q_ik.transpose() << "\n";
        std::cout << "  FK(IK target)   (x y z φ ψ θ):  "
                  << ee_ik_fk.transpose() << "\n";
        std::cout << "  Actual q        (q1..q6) [rad]: "
                  << q_actual.transpose() << "\n";
        std::cout << "  Actual EE pose  (x y z φ ψ θ):  "
                  << ee_actual.transpose() << "\n";
        std::cout << "  EE error        (target-actual): "
                  << ee_error.transpose() << "\n";
        std::cout << "  |pos error| [m]: "
                  << ee_error.head<3>().norm() << "\n";
        std::cout << "[P2P] ─────────────────────────────────────────────────\n\n";
        std::cout.flush();
        }
        m_iter = 0;
        return;
    }

    m_target_joint_pose = m_robot->inverseKinematics(m_target_ee_pose);

    // Clamp IK solution to joint limits ±π to prevent runaway if target is
    // geometrically marginal (e.g. x close to wrist-length singularity).
    for (int i = 0; i < 6; ++i) {
        m_target_joint_pose(i) = saturate(m_target_joint_pose(i), -M_PI, M_PI);
    }

    m_num_steps = m_duration / m_dt;
    double alpha = static_cast<double>(m_iter) / m_num_steps;

    m_joint_state_ref = (1.0 - alpha) * m_joint_state_start + alpha * m_target_joint_pose;
    m_joint_vel_ref = (m_target_joint_pose - m_joint_state_start) / m_duration;
    ++m_iter;
}

void Planner::updateCliData() {
    if (!m_cli_data_ptr) {
        std::cout << "[Planner] m_cli_data_ptr is null!" << std::endl;
        return;
    }

    cli_data.copy(*m_cli_data_ptr);

    static vec6   last_accepted_pose = vec6::Zero();
    static int    last_mode          = -1;
    static double last_duration      = -1.0;

    // Only run the safety check + accept a new target when mode is P2P (3)
    // AND either the pose or duration has actually changed.
    // Ignore mode=0 release messages and any change that is not a new P2P command.
    bool is_p2p      = (cli_data.mode == 3);
    bool pose_new    = !cli_data.x.isApprox(last_accepted_pose);
    bool dur_new     = (std::fabs(cli_data.duration - last_duration) > 0.001);

    if (is_p2p && (pose_new || dur_new)) {
        const vec6& candidate = cli_data.x;

        // ── Pre-execution safety checks — ALL must pass before trajectory starts ──
        bool ok = true;

        // 1. Table clearance: EE tip must be ≥ 5 cm above table surface (z_base=0).
        if (candidate(2) < m_min_ee_z_global) {
            std::cerr << std::fixed << std::setprecision(3)
                      << "[P2P] REJECTED z=" << candidate(2)
                      << " m — must be ≥ " << m_min_ee_z_global
                      << " m (5 cm table clearance).\n";
            ok = false;
        }

        // 2. Workspace box limits.
        if (ok) {
            double x = candidate(0), y = candidate(1), z = candidate(2);
            if (x < m_min_ee_x_global || x > m_max_ee_x_global) {
                std::cerr << "[P2P] REJECTED x=" << x
                          << " — outside workspace ["
                          << m_min_ee_x_global << ", " << m_max_ee_x_global << "].\n";
                ok = false;
            } else if (y < m_min_ee_y_global || y > m_max_ee_y_global) {
                std::cerr << "[P2P] REJECTED y=" << y
                          << " — outside workspace ["
                          << m_min_ee_y_global << ", " << m_max_ee_y_global << "].\n";
                ok = false;
            } else if (z > m_max_ee_z_global) {
                std::cerr << "[P2P] REJECTED z=" << z
                          << " — above workspace max " << m_max_ee_z_global << " m.\n";
                ok = false;
            }
        }

        // 3. Radial distance from base (avoids near-base singularity).
        if (ok) {
            double dist = std::sqrt(candidate(0)*candidate(0) +
                                    candidate(1)*candidate(1));
            if (dist < m_min_dist_ee_xy || dist > m_max_dist_ee_xy) {
                std::cerr << "[P2P] REJECTED dist_xy=" << dist
                          << " m — outside range ["
                          << m_min_dist_ee_xy << ", " << m_max_dist_ee_xy << "].\n";
                ok = false;
            }
        }

        // 4. IK reachability: joint angles must stay within ±π.
        if (ok && m_robot && !m_robot->isReachable(candidate)) {
            std::cerr << "[P2P] REJECTED ["
                      << candidate.transpose()
                      << "] — IK infeasible (joints would exceed ±π).\n";
            ok = false;
        }

        if (ok) {
            m_target_ee_pose      = candidate;
            m_duration            = (cli_data.duration > 0) ? cli_data.duration : 5.0;
            m_iter                = 0;
            m_trajectory_reported = false;
            m_target_accepted     = true;
            last_accepted_pose    = candidate;
            last_duration         = cli_data.duration;
            std::cout << std::fixed << std::setprecision(3)
                      << "[P2P] Accepted target ["
                      << candidate.transpose()
                      << "]  duration=" << m_duration << "s\n";
            std::cout.flush();
        } else {
            // Reject: disarm any active trajectory so the robot holds position.
            m_target_accepted = false;
            m_iter            = 0;
        }
    }

    // Always update mode so Planner::step can see it.
    last_mode = cli_data.mode;
}

void Planner::updatePlannerData()
{
    if (m_planner_data_ptr == nullptr)
    {
        std::cout << "Planner data pointer not set. Returning empty\n";
        return;
    }

    m_planner_data_ptr->x = m_ee_state_ref;
    m_planner_data_ptr->xd = m_ee_vel_ref;
    m_planner_data_ptr->xdd = m_ee_acc_ref;
    m_planner_data_ptr->mode = m_mode;
}

void Planner::updateEstimationData()
{
    if (m_estimation_data_ptr == nullptr)
    {
        std::cout << "Estimation data pointer not set. Returning empty\n";
        return;
    }

    m_joint_state_act = m_estimation_data_ptr->js;
    m_joint_vel_act = m_estimation_data_ptr->jv;
}

void Planner::step(const double &dt, const double &t_curr) {
    m_dt = dt;
    m_t_curr = t_curr;

    updateEstimationData();
    updateCliData();
    m_mode = cli_data.mode;

    if (cli_data.mode == 3 && m_target_accepted) {
        planJointTrajectoryLinear(dt);
        m_ee_state_ref = m_robot->forwardKinematics(m_joint_state_ref);
        m_ee_vel_ref = vec6::Zero();
        m_ee_acc_ref = vec6::Zero();
    } else if (cli_data.mode == 3 && !m_target_accepted) {
        // Target rejected — hold current joint position, do not move.
        m_ee_state_ref = m_robot->forwardKinematics(m_joint_state_act);
        m_ee_vel_ref   = vec6::Zero();
        m_ee_acc_ref   = vec6::Zero();
    } else if (m_plan_ee_space) {
        setEndEffectorTarget();
    } else {
        m_ee_state_ref = m_robot->forwardKinematics(m_joint_state_ref);
        m_ee_vel_ref = vec6::Zero();
        m_ee_acc_ref = vec6::Zero();
    }

    updatePlannerData();
}

void Planner::reset()
{
    m_startTimePoint = std::chrono::high_resolution_clock::now();

    if (m_planner_data_ptr == nullptr)
    {
        std::cout << "Planner data pointer not set.\n";
        return;
    }

    m_planner_data_ptr->x = m_ee_state_init;
    m_planner_data_ptr->xd = vec6::Zero();
    m_planner_data_ptr->xdd = vec6::Zero();
    m_planner_data_ptr->mode = 0;

    initClass();
}
