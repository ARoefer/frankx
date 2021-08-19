#pragma once

#include <franka/duration.h>
#include <franka/robot_state.h>

#include <movex/robot/motion_data.hpp>
#include <movex/robot/robot_state.hpp>
#include <movex/motion/motion_joint_waypoint.hpp>
#include <ruckig/ruckig.hpp>


namespace frankx {
    using namespace movex;
    using Affine = affx::Affine;
    using JointPosition = std::array<double, 7>;

template<class RobotType>
struct JointWaypointMotionGenerator: public MotionGenerator {
    ruckig::Ruckig<RobotType::degrees_of_freedoms> trajectory_generator {RobotType::control_rate};
    ruckig::InputParameter<RobotType::degrees_of_freedoms> input_para;
    ruckig::OutputParameter<RobotType::degrees_of_freedoms> output_para;
    ruckig::Result result;

    JointWaypointMotion current_motion;
    std::vector<JointPosition>::iterator target_iterator;

    JointPosition old_target;
    JointPosition joint_positions;
    // Affine old_affine;
    // double old_elbow;
    double time {0.0};
    RobotType* robot;

    // Affine frame;
    JointWaypointMotion& motion;
    MotionData& data;

    bool set_target_at_zero_time {true};

    const size_t cooldown_iterations {5};
    size_t current_cooldown_iteration {0};

    explicit JointWaypointMotionGenerator(RobotType* robot, JointWaypointMotion& motion, MotionData& data): robot(robot), motion(motion), current_motion(motion), data(data) { }

    void reset() {
        time = 0.0;
        current_cooldown_iteration = 0;
        set_target_at_zero_time = false;
    }

    void init(const franka::RobotState& robot_state, franka::Duration period) {
        input_para.current_position = robot_state.q_d;
        input_para.current_velocity = toStd(Vector7d::Zero());
        input_para.current_acceleration = toStd(Vector7d::Zero());

        if (set_target_at_zero_time) {
            target_iterator = current_motion.targets.begin();

            const auto current_target = *target_iterator;

            input_para.target_position = current_target;
            input_para.target_velocity = toStd(Vector7d::Zero());
            input_para.target_acceleration = toStd(Vector7d::Zero());

            for (size_t dof = 0; dof < RobotType::degrees_of_freedoms; dof += 1) {
                input_para.max_velocity[dof] = RobotType::max_joint_velocity[dof] * robot->velocity_rel * data.velocity_rel;
                input_para.max_acceleration[dof] = 0.3 * RobotType::max_joint_acceleration[dof] * robot->acceleration_rel * data.acceleration_rel;
                input_para.max_jerk[dof] = 0.3 * RobotType::max_joint_jerk[dof] * robot->jerk_rel * data.jerk_rel;
            }

            old_target = current_target;
        }
    }

    franka::JointPositions operator()(const franka::RobotState& robot_state, franka::Duration period) {
        time += period.toSec();
        
        motion.setRobotState(robot_state);

        // current_motion.printRobotState();
        
        if (time == 0.0) {
            init(robot_state, period);
        }
        if (motion.stop_motion) {
            robot->stop();
            return franka::MotionFinished(franka::JointPositions(input_para.current_position));
        }

#ifdef WITH_PYTHON
        if (robot->stop_at_python_signal && Py_IsInitialized() && PyErr_CheckSignals() == -1) {
            robot->stop();
        }
#endif

        const int steps = std::max<int>(period.toMSec(), 1);
        for (int i = 0; i < steps; i++) {
            result = trajectory_generator.update(input_para, output_para);
            joint_positions = output_para.new_position;
            if (motion.reload || result == ruckig::Result::Finished) {
                bool has_new_target {false};

                if (target_iterator != current_motion.targets.end()) {
                    ++target_iterator;
                    has_new_target = (target_iterator != current_motion.targets.end());
                }

                if (motion.return_when_finished && target_iterator >= current_motion.targets.end()) {
                    // Allow cooldown of motion, so that the low-pass filter has time to adjust to target values
                    joint_positions = input_para.target_position;
                    if (current_cooldown_iteration < cooldown_iterations) {
                        current_cooldown_iteration += 1;
                        
                        return franka::JointPositions(joint_positions);
                    }
                    return franka::MotionFinished(franka::JointPositions(joint_positions));

                } else if (motion.reload) {
                    current_motion = motion;
                    target_iterator = current_motion.targets.begin();
                    motion.reload = false;
                    current_motion.reload = false;
                    has_new_target = true;
                }

                if (has_new_target) {
                    const auto current_target = *target_iterator;

                    input_para.target_position = current_target;
                    input_para.target_velocity = toStd(Vector7d::Zero());
                    input_para.target_acceleration = toStd(Vector7d::Zero());

                    old_target = current_target;
                }

            } else if (result == ruckig::Result::Error) {
                std::cout << "[frankx robot] Invalid inputs:" << std::endl;
                return franka::MotionFinished(franka::JointPositions(joint_positions));
            }

            input_para.current_position = output_para.new_position;
            input_para.current_velocity = output_para.new_velocity;
            input_para.current_acceleration = output_para.new_acceleration;
        }

        return franka::JointPositions(joint_positions);
    }
};

} // namespace frankx
