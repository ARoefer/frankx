#pragma once

#include <optional>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <kdl/chainjnttojacdotsolver.hpp>
#include <boost/scoped_ptr.hpp>

class JacDot {
public:
    JacDot(std::string urdf_path);
    Eigen::Matrix<double, 6, 7> jacobian_dot(const Eigen::Matrix<double, 7, 1>& q, const Eigen::Matrix<double, 7, 1>& dq);

    boost::scoped_ptr<KDL::ChainJntToJacDotSolver> jacdot_solver;
    unsigned int numJoints;
    KDL::Jacobian jacdot;
    KDL::JntArrayVel qs;
    KDL::Chain panda_chain;
};
