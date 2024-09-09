// Copyright (c) 2024 PAL Robotics S.L. All rights reserved.
//
// Unauthorized copying of this file, via any medium is strictly prohibited,
// unless it was supplied under the terms of a license agreement or
// nondisclosure agreement with PAL Robotics SL. In this case it may not be
// copied or disclosed except in accordance with the terms of that agreement.

#ifndef ROBOT_GUARDIAN__ROBOT_GUARDIAN_HPP_
#define ROBOT_GUARDIAN__ROBOT_GUARDIAN_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "hri_msgs/msg/ids_list.hpp"
#include "hri_msgs/msg/expression.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "hri/hri.hpp"

using std::placeholders::_1;

namespace robot_guardian
{

class RobotGuardian : public rclcpp::Node
{
public:
  explicit RobotGuardian(const rclcpp::NodeOptions & options);
  ~RobotGuardian();
  void init();

private:
  void personListCallback(const hri_msgs::msg::IdsList & person_list);
  void updateExpression();

private:
  rclcpp::Publisher<hri_msgs::msg::Expression>::SharedPtr expression_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr position_pub_;

  hri_msgs::msg::Expression last_expression_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  std::shared_ptr<hri::HRIListener> hri_listener_;
  std::shared_ptr<hri::Person> first_seen_person_;
};

}  // namespace robot_guardian

#endif  // ROBOT_GUARDIAN__ROBOT_GUARDIAN_HPP_
