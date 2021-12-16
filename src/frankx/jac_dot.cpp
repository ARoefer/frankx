#pragma once

#include <optional>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <frankx/jac_dot.hpp>
#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainjnttojacdotsolver.hpp>
#include <kdl/frames_io.hpp>

JacDot::JacDot(std::string urdf_path) {
    urdf::ModelInterfaceSharedPtr robot_model = urdf::parseURDFFile(urdf_path);
    if (!robot_model) {
        std::cerr << "Could not generate robot model" << std::endl;
        exit(1);
    }
    KDL::Tree panda_tree;
    if (!kdl_parser::treeFromUrdfModel(*robot_model, panda_tree)) {
        std::cerr << "Could not extract kdl tree" << std::endl;
        exit(1);
    }    
    
    bool exit_value;
    if (!panda_tree.getChain("panda_link0", "EE", panda_chain)) {
        std::cerr << "Could not extract kdl chain" << std::endl;
        exit(1);
    }
    numJoints = panda_chain.getNrOfJoints();
    std::cout << numJoints << std::endl;
    jacdot_solver.reset(new KDL::ChainJntToJacDotSolver(panda_chain));
    qs.q.resize(numJoints);
    qs.qdot.resize(numJoints);
    jacdot.resize(numJoints);

}
Eigen::Matrix<double, 6, 7> JacDot::jacobian_dot(const Eigen::Matrix<double, 7, 1>& q, const Eigen::Matrix<double, 7, 1>& dq) {
    for(int i = 0; i < numJoints; i++){
        qs.q.data[i] = q[i];
        qs.qdot.data[i] = dq[i];
    }
    int status = jacdot_solver->JntToJacDot(qs, jacdot);
    if (status != 0) {
        std::cerr << "Error computing Jdot. Error code: " << status << std::endl;
    }
    return jacdot.data;
}

