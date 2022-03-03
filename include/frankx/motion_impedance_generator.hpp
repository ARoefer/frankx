#pragma once

#include <franka/duration.h>
#include <franka/robot_state.h>

#include <frankx/jac_dot.hpp>
#include <movex/robot/motion_data.hpp>
#include <movex/robot/robot_state.hpp>
#include <movex/motion/motion_impedance.hpp>
#include <Eigen/Eigenvalues> 
#include <math.h>
#include<cmath>

namespace frankx {
    using namespace movex;

template<class RobotType>
struct ImpedanceMotionGenerator: public MotionGenerator {
    double time {0.0}, motion_init_time {0.0};
    RobotType* robot;

    Eigen::Matrix<double, 6, 6> stiffness;
    Eigen::DiagonalMatrix<double, 6> D_xi;
    Affine initial_affine;
    Eigen::Vector3d position_d;
    Eigen::Quaterniond orientation_d;

    franka::RobotState initial_state;
    franka::Model* model;

    Affine frame;
    ImpedanceMotion& motion;
    MotionData& data;

    JacDot* jac_dot_solver;

    Eigen::GeneralizedEigenSolver<Eigen::MatrixXd> ges;

    const double delta_tau_max_{1.0};

    explicit ImpedanceMotionGenerator(RobotType* robot, const Affine& frame, ImpedanceMotion& motion, MotionData& data): robot(robot), frame(frame), motion(motion), data(data) {
        if (motion.type == ImpedanceMotion::Type::Joint) {
            throw std::runtime_error("joint impedance is not implemented yet.");
        }

        stiffness.setZero();
        stiffness.topLeftCorner(3, 3) << motion.translational_stiffness * Eigen::MatrixXd::Identity(3, 3);
        stiffness.bottomRightCorner(3, 3) << motion.rotational_stiffness * Eigen::MatrixXd::Identity(3, 3);
        Eigen::Map<const Eigen::Matrix<double, 6, 1>> damping_xi(motion.damping_xi.data());
        D_xi.diagonal() = damping_xi;
        
        initial_state = robot->readOnce();
        model = new franka::Model(robot->loadModel());

        initial_affine = Affine(initial_state.O_T_EE);
        position_d = Eigen::Vector3d(initial_affine.translation());
        orientation_d = Eigen::Quaterniond(initial_affine.quaternion());

        jac_dot_solver = new JacDot(robot->urdf_path);
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

        // compute Jdot
        Eigen::Matrix<double, 6, 7> jdot = jac_dot_solver->jacobian_dot(q, dq);
        // compute ddx
        Eigen::Matrix<double, 6, 1> ddx = jacobian * ddq + jdot * dq;        
        
        
        // compute cartesian mass matrix
        Eigen::Matrix<double, 6, 6> lambda;
        lambda = (jacobian * M.inverse() * jacobian.transpose()).inverse();

        // compute damping with Double Diagonalization Design
        // see (Cartesian Impedance Control of Redundant Robots: Recent Results with the DLR-Light-Weight-Arms, Albu-Schaffer et al.)
        ges.compute(stiffness, lambda);

        Eigen::Matrix<double, 6, 6> v;
        v << ges.eigenvectors().real();
        for (int i = 0; i < 6; i++) {
            double x = sqrt((v.col(i).transpose() * lambda * v.col(i)).value());
            v.col(i) /= x;
        }
        Eigen::Matrix<double, 6, 6> K_d0;
        
        K_d0 << ges.eigenvalues().real().asDiagonal().toDenseMatrix();
        Eigen::Matrix<double, 6, 6> Q = v.transpose().inverse();
        Eigen::Matrix<double, 6, 6> D_d = 2 * Q * D_xi * K_d0.cwiseSqrt() * Q.transpose();
        
        // std::cout << "D_d" << std::endl;
        // std::cout << D_d << std::endl << std::endl;

        Eigen::VectorXd wrench_cartesian(6), tau_task(7), tau_d(7);
        wrench_cartesian = - stiffness * error - D_d * (jacobian * dq);
        // std::cout <<  -lambda * ddx << std::endl;
        // std::cout <<  wrench_cartesian << std::endl << std::endl;

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
        

        // std::cout << "q" << std::endl;
        // std::cout << q << std::endl << std::endl;
        // std::cout << "F" << std::endl;
        // std::cout << wrench_cartesian << std::endl << std::endl;
        // std::cout << "J" << std::endl;
        // std::cout << jacobian << std::endl << std::endl;

        // compute joint limit avoidance torque (Constrained resolved acceleration control for humanoids, Dariush el al.)
        
        
        // Eigen::VectorXd tau_jl(7);
        // Eigen::VectorXd tau_new(7);
        // Eigen::Map<const Eigen::Matrix<double, 7, 1>> q_mid(robot->q_mid.data());
        // tau_jl << (10 * (q_mid - q) - (2.0 * sqrt(10)) * dq);

        // tau_new << tau_jl.cwiseProduct(h) + tau_task.cwiseProduct(ones - h);
        // // std::cout << "tau_new" << std::endl;
        // std::cout <<  << std::endl << std::endl;


        if (motion.has_nullspace_pose) {
            // compute null space projection matrix
            Eigen::Matrix<double, 7, 7> N;
            Eigen::MatrixXd I = Eigen::MatrixXd::Identity(7, 7);
            N = I - jacobian.transpose() * lambda * jacobian * M.inverse();

            // nullspace PD control with damping ratio = 1
            Eigen::VectorXd tau_nullspace(7);
            Eigen::Map<const Eigen::Matrix<double, 7, 1>> q_d_nullspace(motion.q_d_nullspace.data());
            tau_nullspace << N * (motion.nullspace_stiffness * (q_d_nullspace - q) - (2.0 * sqrt(motion.nullspace_stiffness)) * dq);
            tau_d << tau_d + tau_nullspace;
        }

        // compute forward dynamics for desired torque
        // Eigen::VectorXd tau_new(7);
        // tau_new = tau_d;
        Eigen::Matrix<double, 7, 1> ddq_d;
        ddq_d = M.inverse() * (tau_d - coriolis - g);
        for (int i = 0; i < 7; i++) {
            double qmin = robot->q_min[i]; 
            double qmax = robot->q_max[i];
            double beta = (qmax - qmin) * 0.1;
            double q_lbuf = qmin + beta;
            double q_ubuf = qmax - beta;
            double h = 1.0;
            if (q_ubuf < q[i] && ddq_d(i) > 0) {
                h = 0.5 - 0.5 * sin((M_PI / beta ) * (q[i] - q_ubuf) - (M_PI / 2));
            } else if (q[i] < q_lbuf && ddq_d(i) < 0) {
                h = 0.5 + 0.5 * sin((M_PI / beta ) * (q[i] - q_lbuf) + (M_PI / 2));
            }
            tau_d(i) *= h;
            // std::cout << h << std::endl;
        }
        // std::cout << "tau" << std::endl;
        // std::cout << tau_d << std::endl << std::endl;

        // std::cout << "tau_clipped" << std::endl;
        // std::cout << tau_new << std::endl << std::endl;

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
