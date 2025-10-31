#ifndef CHECK_SYSTEM_NODE_HPP
#define CHECK_SYSTEM_NODE_HPP

#include <behaviortree_cpp/bt_factory.h>
#include "rclcpp/rclcpp.hpp"
#include <behaviortree_cpp/condition_node.h>



namespace faucon
{

class CheckSystemNode : public BT::ConditionNode
{
public:
  CheckSystemNode(const std::string &name, const BT::NodeConfiguration &config,
                  rclcpp::Node::SharedPtr node)
      : BT::ConditionNode(name, config), node_(node)
  {}  
  
  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;

  };
}

#endif  // CHECK_SYSTEM_NODE_HPP