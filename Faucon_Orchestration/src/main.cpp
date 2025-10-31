

#include "ament_index_cpp/get_package_share_directory.hpp"
#include <filesystem>
#include "rclcpp/rclcpp.hpp"
#include <behaviortree_cpp/bt_factory.h>
#include "faucon_orchestration/check_system_node.hpp"
#include "faucon_orchestration/check_drone_node.hpp"
// #include "faucon_orchestration/navigation_node.hpp"
// #include "faucon_orchestration/detect_event_node.hpp"
// #include "faucon_orchestration/dispatch_uav_node.hpp"
// #include "faucon_orchestration/ingest_result_node.hpp"

using namespace faucon;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("orchestrator_node");

  node-> declare_parameter<bool>("system_status", true);
  node-> declare_parameter<bool>("drone_status", true);

  std::string package_share_dir =
      ament_index_cpp::get_package_share_directory("faucon_orchestration");

  BT::BehaviorTreeFactory factory;

  // Enregistre chaque nœud avec le contexte ROS2
  factory.registerNodeType<CheckSystemNode>("CheckSystem", node);
  factory.registerNodeType<CheckDroneNode>("CheckDrone", node);
  //factory.registerNodeType<NavigationNode>("Navigation", node);
  // factory.registerNodeType<DetectEventNode>("DetectEvent", node);
  // factory.registerNodeType<DispatchUAVNode>("DispatchUAV", node);    
  // factory.registerNodeType<IngestResultNode>("IngestResult", node); 

  
  std::filesystem::path tree_path =
      std::filesystem::path(package_share_dir) / "config" / "orchestrator.xml";

  // création de l'arbre
  auto tree = factory.createTreeFromFile(tree_path.string());

 
  RCLCPP_INFO(node->get_logger(), "Orchestrateur BT lancé.");
  rclcpp::Rate rate(10);
  while (rclcpp::ok())
  {
    //tree.tickRoot();
    tree.tickWhileRunning();
    rclcpp::spin_some(node);
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
