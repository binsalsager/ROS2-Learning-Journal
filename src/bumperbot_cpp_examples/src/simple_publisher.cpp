// Copyright 2024 Sager Binsal Scariah
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

// 1. ---- IMPORTS ----
// The core ROS 2 C++ library
#include "rclcpp/rclcpp.hpp"
// The specific message type we want to use (String)
#include "std_msgs/msg/string.hpp"

// Includes for using timers and binding member functions
#include <chrono>
#include <functional>

// A handy way to use time literals like `1s` for seconds
using namespace std::chrono_literals;

// 2. ---- CLASS DEFINITION (THE BLUEPRINT) ----
class SimplePublisher : public rclcpp::Node
{
public:
  // 3. ---- CONSTRUCTOR (THE SETUP) ----
  // This runs once when the object is created
  SimplePublisher()
  : Node("simple_publisher"), counter_(0)  // FIX: Use double quotes for strings
  {
    // Create the publisher. Note the message type is a template argument <...>
    pub_ = create_publisher<std_msgs::msg::String>("chatter", 10);

    // Create a wall timer that triggers every 1 second
    // std::bind is the C++ way to tell the timer to call a member function of this object
    timer_ = create_wall_timer(1s, std::bind(&SimplePublisher::timerCallback, this));

    // Log a message to the console
    RCLCPP_INFO(get_logger(), "Publishing at 1Hz");
  }

private:
  // --- Member Variables ---
  unsigned int counter_;  // FIX: Missing semicolon
  // A "Shared Pointer" to the publisher object. C++ uses pointers heavily.
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_; // FIX: Typo `SharedPtr`
  rclcpp::TimerBase::SharedPtr timer_;

  // 4. ---- CALLBACK (THE ACTION) ----
  void timerCallback()
  {
    // Create a new message object
    auto message = std_msgs::msg::String();
    // Set its data field. `std::to_string` converts our counter number to a string.
    message.data = "Hello ROS 2 - counter: " + std::to_string(counter_++); // FIX: Missing semicolon

    // Publish the message
    pub_->publish(message); // FIX: Was a colon, should be a semicolon
  }
};


// 5. ---- MAIN EXECUTION (THE "ON" SWITCH) ----
int main(int argc, char * argv[])
{ // FIX: Must be curly braces, not square brackets
  // Initialize the ROS 2 C++ client library
  rclcpp::init(argc, argv);

  // Create a shared pointer to a new SimplePublisher object
  auto node = std::make_shared<SimplePublisher>(); // FIX: Typo in class name

  // Spin the node, keeping it alive to process callbacks (like the timer)
  rclcpp::spin(node);

  // Shut down the ROS 2 system after the spin is interrupted (e.g., by Ctrl+C)
  rclcpp::shutdown();
  return 0;
} // FIX: Must be curly braces
