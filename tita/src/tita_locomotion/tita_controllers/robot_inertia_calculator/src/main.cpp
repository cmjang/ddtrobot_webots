
#include "robot_inertia_calculator/robot_inertia_calculator.hpp"

int main(int argc, char ** argv)
{
  // 初始化ROS 2节点
  rclcpp::init(argc, argv);
  auto options = rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<tita_locomotion::RobotMassInertiaCalculator>(options);
  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

  executor->add_node(node);
  executor->spin();
  rclcpp::shutdown();
  return 0;
}
