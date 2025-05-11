#ifndef IK_SOLVER_LIB__BASE__IK_SOLVER_BASE_HPP_
#define IK_SOLVER_LIB__BASE__IK_SOLVER_BASE_HPP_

#include <string>

#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <rclcpp/rclcpp.hpp>

namespace ik_solver_lib
{

class IKSolverBase
{
public:
  virtual ~IKSolverBase() {}
  virtual void initialize(
    rclcpp::Node::SharedPtr node,
    const std::string & chain_start, const std::string & chain_end,
    const std::string & urdf_param, double timeout, double eps) = 0;
  virtual bool solveIK(const KDL::Frame & desired_pose, KDL::JntArray & result_joint_positions) = 0;
};

}

#endif // IK_SOLVER_LIB__BASE__IK_SOLVER_BASE_HPP_
