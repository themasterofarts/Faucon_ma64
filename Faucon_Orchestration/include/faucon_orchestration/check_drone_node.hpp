#ifndef CHECK_DRONE_NODE_HPP
#define CHECK_DRONE_NODE_HPP

#include <behaviortree_cpp/bt_factory.h>
#include "rclcpp/rclcpp.hpp"
#include <behaviortree_cpp/condition_node.h>



namespace faucon
{

class CheckDroneNode : public BT::ConditionNode
{
public:
CheckDroneNode(const std::string &name, const BT::NodeConfiguration &config,
                  rclcpp::Node::SharedPtr node)
      : BT::ConditionNode(name, config), node_(node)
  {}  
  
  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;

  };
}

#endif  // CHECK_DRONE_NODE_HPP