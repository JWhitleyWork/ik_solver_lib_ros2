#ifndef IK_SOLVER_LIB__TRAC_IK__TRAC_IK_SOLVER_HPP_
#define IK_SOLVER_LIB__TRAC_IK__TRAC_IK_SOLVER_HPP_

#include "ik_solver_lib/base/ik_solver_base.hpp"

#include <kdl/chainfksolverpos_recursive.hpp>
#include <rclcpp/logging.hpp>

#include <memory>
#include <string>

namespace ik_solver_lib
{

class TracIKSolver : public IKSolverBase
{
public:
  TracIKSolver() = default;
  void initialize(
    const std::string & chain_start, const std::string & chain_end,
    const std::string & urdf_param, double timeout, double eps) override;
  bool solveIK(const KDL::Frame & desired_pose, KDL::JntArray & result_joint_positions) override;

private:
  std::shared_ptr<TRAC_IK::TRAC_IK> ik_solver_ptr_;
  KDL::Chain chain_;
  KDL::JntArray ll_, ul_;  // Lower and upper joint limits
  rclcpp::Logger logger_ = rclcpp::get_logger("ik_solver_lib::TracIKSolver");
};

}

#endif // IK_SOLVER_LIB__TRAC_IK__TRAC_IK_SOLVER_HPP_
