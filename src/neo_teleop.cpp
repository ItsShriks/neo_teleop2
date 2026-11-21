/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, Neobotix GmbH
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Neobotix nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cmath>
#include <memory>
#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class NeoTeleop : public rclcpp::Node
{
public:
  NeoTeleop()
  : Node("neo_teleop2_node")
  {
    // declare Parameters
    this->declare_parameter<double>("scale_linear_x", 0.4);
    this->declare_parameter<double>("scale_linear_y", 0.4);
    this->declare_parameter<double>("scale_angular_z", 0.6);
    this->declare_parameter<int>("axis_linear_x", 1);
    this->declare_parameter<int>("axis_linear_y", 0);
    this->declare_parameter<int>("axis_angular_z", 2);
    this->declare_parameter<double>("smooth_factor", 0.2);
    this->declare_parameter<int>("deadman_button", 5);
    this->declare_parameter<double>("joy_timeout", 1.);
    // Logitech F710: the X/D switch on its back changes the axis layout. D (DirectInput): 6 axes,
    // the right stick X is axis 2. X (XInput): 8 axes, axis 2 is the LEFT TRIGGER (rests at +-1 ->
    // full-speed rotation), the right stick X is axis 3. Used when /joy has xinput_num_axes axes.
    this->declare_parameter<int>("axis_angular_z_xinput", 3);
    this->declare_parameter<int>("xinput_num_axes", 8);
    // stop if /joy goes silent while the deadman is held (joy_node repeats at 20 Hz; a wireless
    // dropout or a re-enumerating receiver must not keep the last command going) [s]
    this->declare_parameter<double>("joy_stale_timeout", 0.3);
    // the sticks must be within this of centre when the deadman is pressed, else nothing moves
    // until they are (a pad waking up can report full-scale axes for a moment)
    this->declare_parameter<double>("engage_max_axis", 0.2);

    // Get Paramters
    this->get_parameter("scale_linear_x", linear_scale_x);
    this->get_parameter("scale_linear_y", linear_scale_y);
    this->get_parameter("scale_angular_z", angular_scale_z);
    this->get_parameter("axis_linear_x", axis_linear_x);
    this->get_parameter("axis_linear_y", axis_linear_y);
    this->get_parameter("axis_angular_z", axis_angular_z);
    this->get_parameter("smooth_factor", smooth_factor);
    this->get_parameter("deadman_button", deadman_button);
    this->get_parameter("joy_timeout", joy_timeout);
    this->get_parameter("axis_angular_z_xinput", axis_angular_z_xinput);
    this->get_parameter("xinput_num_axes", xinput_num_axes);
    this->get_parameter("joy_stale_timeout", joy_stale_timeout);
    this->get_parameter("engage_max_axis", engage_max_axis);

    vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
    joy_sub = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 1,
      std::bind(&NeoTeleop::joy_callback, this, _1));
  }

  void send_cmd();

protected:
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy);

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;
  geometry_msgs::msg::Twist cmd_vel;

  double linear_scale_x = 0;
  double linear_scale_y = 0;
  double angular_scale_z = 0;
  double smooth_factor = 1;
  double joy_timeout = 0;
  int axis_linear_x = -1;
  int axis_linear_y = -1;
  int axis_angular_z = -1;
  int deadman_button = -1;
  int axis_angular_z_xinput = -1;
  int xinput_num_axes = -1;
  double joy_stale_timeout = 0;
  double engage_max_axis = 1;

  rclcpp::Time last_joy_time;
  rclcpp::Time last_msg_time;
  bool have_msg = false;
  bool is_engaged = false;  // deadman pressed with the sticks centred; false until they are
  int last_num_axes = -1;

  double joy_command_x = 0;
  double joy_command_y = 0;
  double joy_command_z = 0;

  bool is_active = false;
  bool is_deadman_pressed = false;
};


static double axis(const sensor_msgs::msg::Joy & joy, int i)
{
  return (i >= 0 && i < static_cast<int>(joy.axes.size())) ? joy.axes[i] : 0.0;
}

void NeoTeleop::joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy)
{
  const int num_axes = static_cast<int>(joy->axes.size());
  const bool xinput = num_axes == xinput_num_axes;
  if (num_axes != last_num_axes) {
    RCLCPP_WARN(get_logger(), "joystick has %d axes: %s mode, angular z on axis %d", num_axes,
      xinput ? "XInput (F710 switch X)" : "DirectInput (F710 switch D)",
      xinput ? axis_angular_z_xinput : axis_angular_z);
    last_num_axes = num_axes;
    is_engaged = false;  // layout changed under a held deadman: re-engage with centred sticks
  }
  last_msg_time = rclcpp::Clock().now();
  have_msg = true;

  const bool was_pressed = is_deadman_pressed;
  if (deadman_button >= 0 && deadman_button < static_cast<int>(joy->buttons.size())) {
    is_deadman_pressed = static_cast<bool>(joy->buttons[deadman_button]);
  } else {
    is_deadman_pressed = false;
  }

  const double x = axis(*joy, axis_linear_x);
  const double y = axis(*joy, axis_linear_y);
  const double z = axis(*joy, xinput ? axis_angular_z_xinput : axis_angular_z);

  if (!is_deadman_pressed) {
    is_engaged = false;
  } else if (!is_engaged) {
    if (std::abs(x) <= engage_max_axis && std::abs(y) <= engage_max_axis &&
      std::abs(z) <= engage_max_axis)
    {
      is_engaged = true;
    } else if (!was_pressed) {
      RCLCPP_WARN(get_logger(), "deadman pressed with the sticks off centre (%.2f %.2f %.2f): "
        "not moving until they are centred", x, y, z);
    }
  }
  if (is_engaged) {
    is_active = true;
    last_joy_time = last_msg_time;
  }

  joy_command_x = is_engaged ? linear_scale_x * x : 0.0;
  joy_command_y = is_engaged ? linear_scale_y * y : 0.0;
  joy_command_z = is_engaged ? angular_scale_z * z : 0.0;
}

void NeoTeleop::send_cmd()
{
  if (is_engaged && have_msg &&
    (rclcpp::Clock().now() - last_msg_time).seconds() > joy_stale_timeout)
  {
    RCLCPP_ERROR(get_logger(), "no /joy for %.1f s with the deadman held (wireless dropout?): "
      "stopping", (rclcpp::Clock().now() - last_msg_time).seconds());
    is_deadman_pressed = false;
    is_engaged = false;
    is_active = false;
    cmd_vel = geometry_msgs::msg::Twist();
    vel_pub->publish(cmd_vel);
    return;
  }
  if (is_engaged) {
    // smooth inputs
    cmd_vel.linear.x = joy_command_x * smooth_factor + cmd_vel.linear.x * (1 - smooth_factor);
    cmd_vel.linear.y = joy_command_y * smooth_factor + cmd_vel.linear.y * (1 - smooth_factor);
    cmd_vel.angular.z = joy_command_z * smooth_factor + cmd_vel.angular.z * (1 - smooth_factor);

    // publish
    vel_pub->publish(cmd_vel);
  } else if (is_active) {
    if ((rclcpp::Clock().now() - last_joy_time).seconds() > joy_timeout) {
      cmd_vel = geometry_msgs::msg::Twist();      // set to all zero
      is_active = false;
    } else {
      // smooth towards zero
      cmd_vel.linear.x = cmd_vel.linear.x * (1 - smooth_factor);
      cmd_vel.linear.y = cmd_vel.linear.y * (1 - smooth_factor);
      cmd_vel.angular.z = cmd_vel.angular.z * (1 - smooth_factor);
    }
    // publish
    vel_pub->publish(cmd_vel);
  }
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto nh = std::make_shared<NeoTeleop>();
  double control_rate = 50;
  rclcpp::Rate loop_rate(control_rate);

  while (rclcpp::ok()) {
    nh->send_cmd();

    loop_rate.sleep();

    rclcpp::spin_some(nh);
  }

  return 0;
}
