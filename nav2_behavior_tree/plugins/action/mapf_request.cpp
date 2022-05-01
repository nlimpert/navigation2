#include <string>
#include <memory>

#include <chrono>

#include <cstdlib>

#include "nav2_mapf_planner/mapf_planner.hpp"
#include "nav2_util/node_utils.hpp"
using namespace std::chrono_literals;
using rcl_interfaces::msg::ParameterType;
namespace nav2_mapf_planner
{

    MapfPlanner::MapfPlanner() : node(std::make_shared<rclcpp::Node>("mapf_client"))
    {
    }

    void MapfPlanner::configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
        std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
    {

        auto node = parent.lock();
        node_ = node;
        tf_ = tf;
        name_ = name;
        costmap_ = costmap_ros->getCostmap();
        global_frame_ = costmap_ros->getGlobalFrameID();
//        node = ;
//        node = rclcpp::Node::make_shared("minimal_client");
        nav2_util::declare_parameter_if_not_declared(node_, name_ + ".id", rclcpp::ParameterValue(1));
        node_->get_parameter(name_ + ".id", id);

        RCLCPP_INFO(node_->get_logger(), " Configure Navigation2 Mapf Planner", name_.c_str());
        // add parameters
        client_ = node_->create_client<mapf_actions::srv::Mapf>("/off_field/mapf_plan");
    }

    void MapfPlanner::cleanup()
    {
        RCLCPP_INFO(node_->get_logger(), "Cleaning up Mapf Planner", name_.c_str());
    }

    void MapfPlanner::activate()
    {
        RCLCPP_INFO(node_->get_logger(), "Activating Mapf Planner", name_.c_str());
    }
    void MapfPlanner::deactivate()
    {
        RCLCPP_INFO(node_->get_logger(), "Deactivating plugin %s of type nav2_mapf_planner", name_.c_str());
    }

    nav_msgs::msg::Path MapfPlanner::createPlan(
        const geometry_msgs::msg::PoseStamped &start,
        const geometry_msgs::msg::PoseStamped &goal)
    {
        RCLCPP_INFO(rclcpp::get_logger("mapf_planner"), "Got a plan request");
        auto request = std::make_shared<mapf_actions::srv::Mapf::Request>();
        request->goal = goal;
        request->start = start;
        request->robotino_id = id;
        request->timestamp = node_->get_clock().get()->now();
        RCLCPP_INFO(rclcpp::get_logger("mapf_planner"), "ID: %d", id);
        auto result = client_->async_send_request(request);

        rclcpp::FutureReturnCode returncode = rclcpp::spin_until_future_complete(node, result, 40ms);

        if (returncode ==
             rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(rclcpp::get_logger("mapf_planner"), "Successfully calculated a path");
        }
        else if (returncode ==
                  rclcpp::FutureReturnCode::TIMEOUT)
        {
            RCLCPP_INFO(rclcpp::get_logger("mapf_planner"), "Timeout when requesting Mapf-Server");
        }
        else
        {
            RCLCPP_INFO(rclcpp::get_logger("mapf_planner"), "Mapf failed.");
        }
        return result.get()->path;
    }

}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_mapf_planner::MapfPlanner, nav2_core::GlobalPlanner)
