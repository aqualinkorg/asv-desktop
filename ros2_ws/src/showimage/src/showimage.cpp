// Copyright 2015 Open Source Robotics Foundation, Inc.
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

#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"

#include <boost/program_options.hpp>
#include <spdlog/spdlog.h>

class ShowImage : public rclcpp::Node
{
public:
  explicit ShowImage( const std::string& topic_name )
  : Node( "showimage" )
  , topic_{ "camera_" + topic_name + "/image_raw/compressed" }
  {
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    initialize();
  }

private:
  void initialize()
  {
    if (show_image_) {
      // Initialize an OpenCV named window called "showimage".
      cv::namedWindow("showimage", cv::WINDOW_AUTOSIZE);
      cv::waitKey(1);
    }
    // Set quality of service profile based on command line options.
    auto qos = rclcpp::QoS(
      rclcpp::QoSInitialization(
        // The history policy determines how messages are saved until taken by
        // the reader.
        // KEEP_ALL saves all messages until they are taken.
        // KEEP_LAST enforces a limit on the number of messages that are saved,
        // specified by the "depth" parameter.
        history_policy_,
        // Depth represents how many messages to store in history when the
        // history policy is KEEP_LAST.
        depth_
    ));
    // The reliability policy can be reliable, meaning that the underlying transport layer will try
    // ensure that every message gets received in order, or best effort, meaning that the transport
    // makes no guarantees about the order or reliability of delivery.
    qos.reliability(reliability_policy_);
    auto callback = [this](const sensor_msgs::msg::CompressedImage::SharedPtr msg)
      {
        process_image(msg, show_image_, this->get_logger());
      };

    RCLCPP_INFO(this->get_logger(), "Subscribing to topic '%s'", topic_.c_str());
    sub_ = create_subscription<sensor_msgs::msg::CompressedImage>(topic_, qos, callback);
  }

  /// Convert the ROS Image message to an OpenCV matrix and display it to the user.
  // \param[in] msg The image message to show.
  void process_image(
    const sensor_msgs::msg::CompressedImage::SharedPtr msg, bool show_image, rclcpp::Logger logger)
  {
    RCLCPP_INFO(logger, "Received image #%s", msg->header.frame_id.c_str());
    std::cerr << "Received image #" << msg->header.frame_id.c_str() << std::endl;

    if (show_image) {

      // Create a Size(1, nSize) Mat object of 8-bit, single-byte elements
      cv::Mat rawData( 1, msg->data.size(), CV_8UC1, (void*)msg->data.data() );

      cv::Mat decodedImage = imdecode( rawData, 1 /*, flags */ );
      if ( decodedImage.data == NULL )   
      {
          // Error reading raw image data
          return;
      }

      // // Show the image in a window called "showimage".
      cv::imshow("showimage", decodedImage);

      // Draw the screen and wait for 1 millisecond.
      cv::waitKey(1);
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_;
  size_t depth_                                     = 1;
  rmw_qos_reliability_policy_t reliability_policy_  = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
  rmw_qos_history_policy_t history_policy_          = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
  bool show_image_ = true;
  std::string topic_ = "camera_depth/image_raw/compressed";
};

namespace po = boost::program_options;

int main(int argc, char ** argv)
{
  // Program arguments
  po::options_description options( "Options:" );
  po::variables_map args;

  // Setup options
  options.add_options()
    ( "help,h",  "Show help" )
    ( "topic,t", po::value<std::string>()->required(), "Camera name ('left' or 'depth')" );

  // Parse command line options
  po::store( po::parse_command_line( argc, argv, options ), args );
  po::notify( args );

  if( args.count( "help" ) )
  {
    // Print help string
    std::cout << options << std::endl;
    throw std::runtime_error( "" );
  }

  rclcpp::init( argc, argv );

  rclcpp::executors::SingleThreadedExecutor exec;
  
  spdlog::info("========================================");
  spdlog::info("STARTING recorder\n");

  auto node = std::make_shared<ShowImage>( args[ "topic" ].as<std::string>() );

  exec.add_node( node );

  while( rclcpp::ok() )
  {
    exec.spin();
  }
  
  spdlog::info("ENDING recorder\n");
  return 0;
}
