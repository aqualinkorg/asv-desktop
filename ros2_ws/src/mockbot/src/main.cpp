#include <mockbot/mockbot_node.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init( argc, argv );
  
  spdlog::info("========================================");
  spdlog::info("STARTING mockbot\n");

  auto node = std::make_shared<MockbotNode>();

  rclcpp::spin( node );  

  spdlog::info("ENDING mockbot\n");
  return 0;
}