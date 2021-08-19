#pragma once

#include <atomic>
#include <optional>
#include <franka/robot_state.h>
#include <affx/affine.hpp>


namespace movex {

/**
* A motion following multiple waypoints (with intermediate zero velocity) in a time-optimal way.
* Works with aribtrary initial conditions.
*/
struct JointWaypointMotion {
    using Affine = affx::Affine;
    using JointPosition = std::array<double, 7>;

    bool reload {false};
    bool return_when_finished {true};
    bool stop_motion {false};

    std::vector<JointPosition> targets;
    
    franka::RobotState current_state;

    explicit JointWaypointMotion() {}
    explicit JointWaypointMotion(const std::vector<JointPosition>& targets): targets(targets) {}
    explicit JointWaypointMotion(const std::vector<JointPosition>& targets, bool return_when_finished): targets(targets), return_when_finished(return_when_finished) {}

    void setNextTarget(const JointPosition& target) {
        targets = { target };
        reload = true;
    }

    void setNextTargets(const std::vector<JointPosition>& targets) {
        this->targets = targets;
        reload = true;
    }

    void finish() {
        return_when_finished = true;
        reload = true;
    }

    void stop() {
        stop_motion = true;
    }

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
