#include <functional>
#include <csignal>
#include <thread>
#include <string>

#include <boost/program_options.hpp>
#include <spdlog/spdlog.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int64.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/battery_state.hpp>

#include <mockbot/periodic_scheduler.hpp>

class MockbotNode : public rclcpp::Node
{
public:
  MockbotNode();
  virtual ~MockbotNode() = default;

private:
 
};
