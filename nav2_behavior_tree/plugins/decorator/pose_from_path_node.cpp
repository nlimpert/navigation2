// Copyright (c) 2022 Nicolas Limpert
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <string>
#include <memory>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "behaviortree_cpp_v3/decorator_node.h"

#include "nav2_behavior_tree/plugins/decorator/pose_from_path_node.hpp"

#include "rclcpp/rclcpp.hpp"

namespace nav2_behavior_tree
{

using std::placeholders::_1;

PoseFromPath::PoseFromPath(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::DecoratorNode(name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
}

inline BT::NodeStatus PoseFromPath::tick()
{
  nav_msgs::msg::Path path;
  int pose_index;
  geometry_msgs::msg::PoseStamped goal;

  const BT::NodeStatus child_state = child_node_->executeTick();
  bool success = false;

  switch (child_state) {
    case BT::NodeStatus::RUNNING:
      return BT::NodeStatus::RUNNING;

    case BT::NodeStatus::SUCCESS:
      success = true;
      break;

    case BT::NodeStatus::FAILURE:
      success = false;
      break;

    default:
      success = false;
      break;
  }

  if (success == false) {
    RCLCPP_WARN(node_->get_logger(), "No successful planning");
    return BT::NodeStatus::FAILURE;
  }

  getInput("input_path", path);
  getInput("pose_index", pose_index);

  RCLCPP_INFO(node_->get_logger(), "index: %li size: %li", static_cast<size_t>(pose_index), path.poses.size());
  if (static_cast<size_t>(pose_index) >= path.poses.size()) {
    pose_index = 0;
  }
//  pose_index = pose_index ? static_cast<size_t>(pose_index) < path.poses.size() -1 : 0;
  if (path.poses.size() == 0) {
    return BT::NodeStatus::FAILURE;
  }

  //TODO: use last pose in case the index is -1
  goal = path.poses.at(pose_index);

  RCLCPP_INFO(node_->get_logger(), "selected pose: %f %f", goal.pose.position.x, goal.pose.position.y);

  setOutput("output_pose", goal);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::PoseFromPath>("PoseFromPath");
}
