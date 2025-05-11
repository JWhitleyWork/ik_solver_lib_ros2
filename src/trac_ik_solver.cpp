#include "ik_solver_lib/trac_ik/trac_ik_solver.hpp"

namespace ik_solver_lib
{

void TracIKSolver::initialize(
  const std::string & chain_start, const std::string & chain_end,
  const std::string & urdf_param, double timeout, double eps)
{
  ik_solver_ptr_ = std::make_shared<TRAC_IK::TRAC_IK>(chain_start, chain_end, urdf_param, timeout,
      eps, TRAC_IK::Speed);
  bool valid = ik_solver_ptr_->getKDLChain(chain_);

  if (!valid) {
    RCLCPP_ERROR(logger_, "Failed to load chain from parameter server");
  }

  valid = ik_solver_ptr_->getKDLLimits(ll_, ul_);
  if (!valid) {
    RCLCPP_ERROR(logger_, "Failed to get KDL joint limits");
  }
}

bool TracIKSolver::solveIK(const KDL::Frame & desired_pose, KDL::JntArray & result_joint_positions)
{
  KDL::JntArray initial_joint_positions(chain_.getNrOfJoints());
  int rc = ik_solver_ptr_->CartToJnt(initial_joint_positions, desired_pose, result_joint_positions);
  return rc >= 0;
}

}
