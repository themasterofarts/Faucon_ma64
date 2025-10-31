#include "faucon_orchestration/check_drone_node.hpp"


namespace faucon
{

BT::PortsList CheckDroneNode::providedPorts()
{
  return {};
}   

BT::NodeStatus CheckDroneNode::tick()
{
  RCLCPP_INFO(node_->get_logger(), "Vérification du drone...");

bool drone_ok{};
node_->get_parameter("drone_status", drone_ok); 


  if (drone_ok)
  {
    RCLCPP_INFO(node_->get_logger(), "Drone OK.");
    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    RCLCPP_ERROR(node_->get_logger(), "Drone non disponible !");
    return BT::NodeStatus::FAILURE;
  }
}  
}  // namespace faucon