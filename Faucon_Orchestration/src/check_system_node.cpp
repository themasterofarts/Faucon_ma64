#include "faucon_orchestration/check_system_node.hpp"


namespace faucon
{

BT::PortsList CheckSystemNode::providedPorts()
{
  return {};
}   

BT::NodeStatus CheckSystemNode::tick()
{
  RCLCPP_INFO(node_->get_logger(), "Vérification du système...");

bool system_ok{};
node_->get_parameter("system_status", system_ok); 

  
  //bool system_ok = true; 

  if (system_ok)
  {
    RCLCPP_INFO(node_->get_logger(), "Système OK.");
    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    RCLCPP_ERROR(node_->get_logger(), "Problème détecté dans le système !");
    return BT::NodeStatus::FAILURE;
  }
}  
}  // namespace faucon