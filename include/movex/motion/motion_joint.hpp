#pragma once

#include <Eigen/Core>
#include <franka/robot_state.h>
#include <affx/affine.hpp>

namespace movex {

/**
* A motion in the joint space
*/
struct JointMotion {
    using Affine = affx::Affine;
    using Vector7d = Eigen::Matrix<double, 7, 1>;

    const Vector7d target;
    franka::RobotState current_state;

    explicit JointMotion(const std::array<double, 7> target): target(target.data()) { }

    void setRobotState(const franka::RobotState& robot_state) {
        current_state = robot_state;
    }

    franka::RobotState getRobotState() {
        return current_state;
    }

    Affine currentPose(const Affine& frame) {
        return Affine(current_state.O_T_EE) * frame;
    }
};

} // namespace movex
