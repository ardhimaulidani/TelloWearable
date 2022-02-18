#include <memory>
#include <chrono>
#include <iostream>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/u_int16_multi_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tello_msgs/srv/tello_action.hpp"

using ros_clock = std::chrono::system_clock;
using ms = std::chrono::duration<double, std::milli>;
double vx, vy, vz = 0.0, wz = 0.0;
double finger_1, finger_2, finger_3;
auto teleop = geometry_msgs::msg::Twist();
bool status = false;

static void AngleCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
  // const auto before = ros_clock::now();
  vx = msg->data[0];
  vy = msg->data[1];

  teleop.linear.z = vz;
  teleop.angular.z = wz;
}

static void PotCallback(const std_msgs::msg::UInt16MultiArray::SharedPtr msg_pot)
{
  // const auto before = ros_clock::now();
  finger_1 = msg_pot->data[0];
  finger_2 = msg_pot->data[1];
  finger_3 = msg_pot->data[2];

  finger_1 = finger_1/4095;
  finger_2 = finger_2/4095;
  finger_3 = finger_3/4095;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("tello_teleop_node");
  rclcpp::WallRate loop_rate(10);

  auto angle_sub = node->create_subscription<std_msgs::msg::Float32MultiArray>(
    "/wearable_angle", rclcpp::SensorDataQoS(), AngleCallback);
  auto pot_sub = node->create_subscription<std_msgs::msg::UInt16MultiArray>(
    "/wearable_pot", rclcpp::SensorDataQoS(), PotCallback);
  auto cmd_vel_pub = node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  auto tello_client_ = node->create_client<tello_msgs::srv::TelloAction>("tello_action");


  std::cout << "DJI Tello is idling ..." << std::endl;

  while (rclcpp::ok())
  { 
    // std::cout << "Masuk while pertama..." << std::endl;
    // std::cout << "Status: " << status << std::endl;
    // std::cout << "Finger_3: " << finger_3 << std::endl;
    if(finger_3 >= 0.94 && status == false)
    {
      auto request = std::make_shared<tello_msgs::srv::TelloAction::Request>();
      request->cmd = "takeoff";
      auto result_future = tello_client_->async_send_request(request);
      // if (rclcpp::spin_until_future_complete(node, result_future) !=
      //   rclcpp::FutureReturnCode::SUCCESS)
      // {
      //   RCLCPP_ERROR(node->get_logger(), "service call failed :(");
      //   // tello_client_->remove_pending_request(result_future);
      //   return 1;
      // }
      std::cout << "TAKE-OFF !!!" << std::endl;
      status = true;
    }
    
    while (rclcpp::ok() && status == true)
    {
      // std::cout << "Masuk while kedua" << std::endl;
      // std::cout << "Status: " << status << std::endl;
      // std::cout << "Finger_3: " << finger_3 << std::endl;
      
      if(finger_3 < 0.94)
      {
        auto request = std::make_shared<tello_msgs::srv::TelloAction::Request>();
        request->cmd = "land";
        auto result_future = tello_client_->async_send_request(request);
        if (rclcpp::spin_until_future_complete(node, result_future) !=
          rclcpp::FutureReturnCode::SUCCESS)
        {
          RCLCPP_ERROR(node->get_logger(), "service call failed :(");
          // tello_client_->remove_pending_request(result_future);
          return 1;
        }
        std::cout << "LANDING !!!" << std::endl;
        status = false;
        break;
      }
      if(finger_2 < 0.85)
      {
        std::cout << "Mode: Roll Pitch" << std::endl;
        if((vx <= 15) && (vx >= -15))
        {
          teleop.linear.x = 0.0;
        }
        else if (vx > 15)
        {
          teleop.linear.x = -0.4;
        }
        else if (vx < -15)
        {
          teleop.linear.x = 0.4;
        }    
        
        if((vy <= 15) && (vy >= -15))
        {
          teleop.linear.y = 0.0;
        }
        else if (vy > 15)
        {
          teleop.linear.y = -0.4;
        }
        else if (vy < -15)
        {
          teleop.linear.y = 0.4;
        }
      }

      else if(finger_2 >= 0.85)
      {
        std::cout << "Mode: Yaw & Height" << status << std::endl;
        teleop.linear.x = 0.0;
        teleop.linear.y = 0.0;
        if((vx <= 15) && (vx >= -15))
        {
          teleop.linear.z = 0.0;
        }
        else if (vx > 15)
        {
          teleop.linear.z = 0.3;
        }
        else if (vx < -15)
        {
          teleop.linear.z = -0.3;
        }  

        if((vy <= 15) && (vy >= -15))
        {
          teleop.angular.z = 0;
        }
        else if (vy > 15)
        {
          teleop.angular.z = -0.5;
        }
        else if (vy < -15)
        {
          teleop.angular.z = 0.5;
        }  
      }
      if((finger_2 >= 0.85) && (finger_1 >= 0.85))
      {
        std::cout << "Mode: Flip" << status << std::endl;
        if((vx <= 45) && (vx >= -45))
        {
          teleop.linear.x = 0.0;
        }
        else if (vx > 45)
        {
          auto request = std::make_shared<tello_msgs::srv::TelloAction::Request>();
          request->cmd = "flip f";
          auto result_future = tello_client_->async_send_request(request);
          if (rclcpp::spin_until_future_complete(node, result_future) !=
            rclcpp::FutureReturnCode::SUCCESS)
          {
            RCLCPP_ERROR(node->get_logger(), "service call failed :(");
            // tello_client_->remove_pending_request(result_future);
            return 1;
          }
        }
        else if (vx < -45)
        {
          auto request = std::make_shared<tello_msgs::srv::TelloAction::Request>();
          request->cmd = "flip b";
          auto result_future = tello_client_->async_send_request(request);
          if (rclcpp::spin_until_future_complete(node, result_future) !=
            rclcpp::FutureReturnCode::SUCCESS)
          {
            RCLCPP_ERROR(node->get_logger(), "service call failed :(");
            // tello_client_->remove_pending_request(result_future);
            return 1;
          }
        }    
        
        if((vy <= 45) && (vy >= -45))
        {
          teleop.linear.y = 0.0;
        }
        else if (vy > 45)
        {
          auto request = std::make_shared<tello_msgs::srv::TelloAction::Request>();
          request->cmd = "flip l";
          auto result_future = tello_client_->async_send_request(request);
          if (rclcpp::spin_until_future_complete(node, result_future) !=
            rclcpp::FutureReturnCode::SUCCESS)
          {
            RCLCPP_ERROR(node->get_logger(), "service call failed :(");
            // tello_client_->remove_pending_request(result_future);
            return 1;
          }
        }
        else if (vy < -45)
        {
          auto request = std::make_shared<tello_msgs::srv::TelloAction::Request>();
          request->cmd = "flip r";
          auto result_future = tello_client_->async_send_request(request);
          if (rclcpp::spin_until_future_complete(node, result_future) !=
            rclcpp::FutureReturnCode::SUCCESS)
          {
            RCLCPP_ERROR(node->get_logger(), "service call failed :(");
            // tello_client_->remove_pending_request(result_future);
            return 1;
          }
        }
      }
      std::cout << "Receiving: (" << vx << ", " << vy << ")" << std::endl;
      std::cout << "Receiving: (" << finger_1 << ", " << finger_2 << ", " << finger_3 << ")" << std::endl;
      std::cout << "Publishing: (" << teleop.linear.x << ", " << teleop.linear.y << ", " << teleop.linear.z << ", " << teleop.angular.z << ")" << std::endl;

      cmd_vel_pub->publish(teleop);     
      // const ms duration = ros_clock::now() - before;
      // std::cout << "Callback called in " << duration.count() << "s" << std::endl;
      rclcpp::spin_some(node);
      loop_rate.sleep();
    }
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  return 0;
}