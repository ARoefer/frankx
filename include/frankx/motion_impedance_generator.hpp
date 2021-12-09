#pragma once

#include <franka/duration.h>
#include <franka/robot_state.h>

#include <movex/robot/motion_data.hpp>
#include <movex/robot/robot_state.hpp>
#include <movex/motion/motion_impedance.hpp>


namespace frankx {
    using namespace movex;

template<class RobotType>
struct ImpedanceMotionGenerator: public MotionGenerator {
    double time {0.0}, motion_init_time {0.0};
    RobotType* robot;

    Eigen::Matrix<double, 6, 6> stiffness, damping;
    Affine initial_affine;
    Eigen::Vector3d position_d;
    Eigen::Quaterniond orientation_d;

    franka::RobotState initial_state;
    franka::Model* model;

    Affine frame;
    ImpedanceMotion& motion;
    MotionData& data;

    const double delta_tau_max_{1.0};

    explicit ImpedanceMotionGenerator(RobotType* robot, const Affine& frame, ImpedanceMotion& motion, MotionData& data): robot(robot), frame(frame), motion(motion), data(data) {
        if (motion.type == ImpedanceMotion::Type::Joint) {
            throw std::runtime_error("joint impedance is not implemented yet.");
        }

        stiffness.setZero();
        stiffness.topLeftCorner(3, 3) << motion.translational_stiffness * Eigen::MatrixXd::Identity(3, 3);
        stiffness.bottomRightCorner(3, 3) << motion.rotational_stiffness * Eigen::MatrixXd::Identity(3, 3);
        damping.setZero();
        damping.topLeftCorner(3, 3) << 2.0 * sqrt(motion.translational_stiffness) * Eigen::MatrixXd::Identity(3, 3);
        damping.bottomRightCorner(3, 3) << 2.0 * sqrt(motion.rotational_stiffness) * Eigen::MatrixXd::Identity(3, 3);

        initial_state = robot->readOnce();
        model = new franka::Model(robot->loadModel());

        initial_affine = Affine(initial_state.O_T_EE);
        position_d = Eigen::Vector3d(initial_affine.translation());
        orientation_d = Eigen::Quaterniond(initial_affine.quaternion());
    }

    void init(const franka::RobotState& robot_state, franka::Duration period) {
        Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
        motion.target = Affine(transform);
        motion.is_active = true;
    }

    franka::Torques operator()(const franka::RobotState& robot_state, franka::Duration period) {
        time += period.toSec();

        motion.setRobotState(robot_state);

        if (time == 0.0) {
            init(robot_state, period);
        }
        
        std::array<double, 7> coriolis_array = model->coriolis(robot_state);
        std::array<double, 42> jacobian_array = model->zeroJacobian(franka::Frame::kEndEffector, robot_state);
        std::array<double, 49> mass_array = model->mass(robot_state);
        std::array<double, 7> gravity_array = model->gravity(robot_state);

        Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
        Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
        Eigen::Map<const Eigen::Matrix<double, 7, 7>> M(mass_array.data());
        Eigen::Map<const Eigen::Matrix<double, 7, 1>> g(gravity_array.data());
        Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
        Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
        Eigen::Map<const Eigen::Matrix<double, 7, 1>> tau(robot_state.tau_J.data());
        Eigen::Map<const Eigen::Matrix<double, 7, 1>> tau_J_d(robot_state.tau_J_d.data());
        Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
        Eigen::Vector3d position(transform.translation());
        Eigen::Quaterniond orientation(transform.linear());

        Eigen::Matrix<double, 6, 1> error;
        error.head(3) << position - position_d;

        if (orientation_d.coeffs().dot(orientation.coeffs()) < 0.0) {
            orientation.coeffs() << -orientation.coeffs();
        }

        Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d);
        error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
        error.tail(3) << -transform.linear() * error.tail(3);

        // compute forward dynamics
        Eigen::Matrix<double, 7, 1> ddq;
        ddq = M.inverse() * (tau - coriolis - g);

        // compute cartesian mass matrix
        Eigen::Matrix<double, 6, 6> Lambda;
        Lambda = (jacobian * M.inverse() * jacobian.transpose()).inverse();
        
        Eigen::VectorXd wrench_cartesian(6), tau_task(7), tau_d(7);
        wrench_cartesian = -stiffness * error - damping * (jacobian * dq);

        // Force constraints
        for (const auto force_constraint : motion.force_constraints) {
            switch (force_constraint.first) {
                case ImpedanceMotion::Axis::X: {
                    wrench_cartesian(0) = force_constraint.second;
                } break;
                case ImpedanceMotion::Axis::Y: {
                    wrench_cartesian(1) = force_constraint.second;
                } break;
                case ImpedanceMotion::Axis::Z: {
                    wrench_cartesian(2) = force_constraint.second;
                } break;
            }
        }

        tau_task << jacobian.transpose() * wrench_cartesian;
        tau_d << tau_task + coriolis; 
        
        if (motion.has_nullspace_pose) {
            // compute null space projection matrix
            Eigen::Matrix<double, 7, 7> N;
            Eigen::MatrixXd I = Eigen::MatrixXd::Identity(7, 7);
            N = I - jacobian.transpose() * Lambda * jacobian * M.inverse();

            // nullspace PD control with damping ratio = 1
            Eigen::VectorXd tau_nullspace(7);
            Eigen::Map<const Eigen::Matrix<double, 7, 1>> q_d_nullspace(motion.q_d_nullspace.data());
            tau_nullspace << N * (motion.nullspace_stiffness * (q_d_nullspace - q) - (2.0 * sqrt(motion.nullspace_stiffness)) * dq);
            tau_d << tau_d + tau_nullspace;
        }
        // Saturate torque rate to avoid discontinuities
        tau_d << saturateTorqueRate(tau_d, tau_J_d);

        std::array<double, 7> tau_d_array{};
        Eigen::VectorXd::Map(&tau_d_array[0], 7) = tau_d;

#ifdef WITH_PYTHON
        if (robot->stop_at_python_signal && Py_IsInitialized() && PyErr_CheckSignals() == -1) {
            motion.is_active = false;
            return franka::MotionFinished(franka::Torques(tau_d_array));
        }
#endif

        if (motion.should_finish) {
            motion.is_active = false;
            return franka::MotionFinished(franka::Torques(tau_d_array));
        }

        // Update target with target motion
        switch (motion.target_motion) {
            case ImpedanceMotion::TargetMotion::Exponential: {
                position_d = motion.exponential_decay * motion.target.translation() + (1.0 - motion.exponential_decay) * position_d;
                orientation_d = orientation_d.slerp(motion.exponential_decay, motion.target.quaternion());
            } break;
            case ImpedanceMotion::TargetMotion::Linear: {
                if (!motion.linear_motion.initialized) {
                    initial_affine = Affine(Eigen::Affine3d(Eigen::Matrix4d::Map(initial_state.O_T_EE.data())));
                    motion_init_time = time;
                    motion.linear_motion.initialized = true;
                }

                double transition_parameter = (time - motion_init_time) / (motion.linear_motion.duration);
                if (transition_parameter <= 1.0) {  // [ms] to [s]
                    motion.target = initial_affine.slerp(motion.linear_motion.relative_target * initial_affine, transition_parameter);
                    position_d = motion.target.translation();
                    orientation_d = motion.target.quaternion();
                } else if (motion.linear_motion.finish_after && transition_parameter > motion.linear_motion.finish_wait_factor) { // Wait a bit longer to stop
                    motion.should_finish = true;
                }
            } break;
            case ImpedanceMotion::TargetMotion::Spiral: {
                if (!motion.spiral_motion.initialized) {
                    motion_init_time = time;
                    motion.spiral_motion.initialized = true;
                }

                double time_diff = motion_init_time - time;
                // target = spiral_motion.center * Affine(0, 0, 0, 2 * pi * spiral_motion.revolutions_per_second * time) * Affine(spiral_motion.radius_increment_per_revolution * time);
                // position_d = target.translation();
                // orientation_d = target.quaternion();
            } break;
        }

        return franka::Torques(tau_d_array);
    }
    Eigen::Matrix<double, 7, 1> saturateTorqueRate(
        const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
        const Eigen::Matrix<double, 7, 1>& tau_J_d) {  // NOLINT (readability-identifier-naming)
        Eigen::Matrix<double, 7, 1> tau_d_saturated{};
        for (size_t i = 0; i < 7; i++) {
            double difference = tau_d_calculated[i] - tau_J_d[i];
        tau_d_saturated[i] =
            tau_J_d[i] + std::max(std::min(difference, delta_tau_max_), -delta_tau_max_);
        }
        return tau_d_saturated;
    }
};

} // namespace frankx
