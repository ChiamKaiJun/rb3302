// SPDX-License-Identifier: Apache-2.0

#include "nav2_core/progress_checker.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace planning_rpp
{

// Disable progress timeouts while retaining Nav2's progress-checker interface.
class NoProgressChecker : public nav2_core::ProgressChecker
{
public:
  void initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr &,
    const std::string &) override {}

  bool check(geometry_msgs::msg::PoseStamped &) override {return true;}

  void reset() override {}
};

}  // namespace planning_rpp

PLUGINLIB_EXPORT_CLASS(planning_rpp::NoProgressChecker, nav2_core::ProgressChecker)
