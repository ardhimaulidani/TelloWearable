#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */
      float a = 0;
      float b = 0;

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0)
    {
      publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/wearable_angle", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      auto message = std_msgs::msg::Float32MultiArray();
      message.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
      message.layout.dim[0].size = 2;
      message.layout.dim[0].stride = 1;
      message.layout.dim[0].label = "bla";
      message.data.resize(2);

      //RCLCPP_INFO(this->get_logger(), "Init msg");
      message.data[0] = a++;
      message.data[1] = b--;

      //RCLCPP_INFO(this->get_logger(), "Finished init");
      RCLCPP_INFO(this->get_logger(), "Publishing: '%.3f    %.3f'", message.data[0], message.data[1]);

      publisher_->publish(message);
      //RCLCPP_INFO(this->get_logger(), "Published");
    }
    
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
