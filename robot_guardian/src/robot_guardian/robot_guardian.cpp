// Copyright (c) 2024 PAL Robotics S.L. All rights reserved.
//
// Unauthorized copying of this file, via any medium is strictly prohibited,
// unless it was supplied under the terms of a license agreement or
// nondisclosure agreement with PAL Robotics SL. In this case it may not be
// copied or disclosed except in accordance with the terms of that agreement.

#include "rclcpp/rclcpp.hpp"
#include "robot_guardian/robot_guardian.hpp"


using namespace std::chrono_literals;
namespace robot_guardian
{

using std::placeholders::_1;

RobotGuardian::RobotGuardian(const rclcpp::NodeOptions & options)
: Node("robot_guardian", options)
{
  tf_buffer_ =
    std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ =
    std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  expression_pub_ = this->create_publisher<hri_msgs::msg::Expression>("/robot_face/expression", 10);
  position_pub_ =
    this->create_publisher<geometry_msgs::msg::PointStamped>("/robot_face/look_at", 10);

  timer_ = create_wall_timer(500ms, [this]() {return this->updateExpression();});
}

RobotGuardian::~RobotGuardian()
{
}

void RobotGuardian::init()
{
  // "shared_from_this()" cannot be used in the constructor!
  hri_listener_ = hri::HRIListener::create(shared_from_this());
}

void RobotGuardian::updateExpression()
{
  last_expression_.expression = hri_msgs::msg::Expression::SAD;
  auto tracked_people_ = hri_listener_->getTrackedPersons();

  if (!tracked_people_.empty()) {

    for (const auto & [person_id, person]: tracked_people_) {

      if (!first_seen_person_) {
        if (person_id.find("anonymous") == std::string::npos) {
          first_seen_person_ = std::move(person);
          RCLCPP_INFO(
            this->get_logger(), "The first seen person is: %s",
            person_id.c_str());
        }
      } else if (person_id == first_seen_person_->id()) {
        first_seen_person_ = std::move(person); // Update the person (the faces may change)
        RCLCPP_INFO(this->get_logger(), "Goal person detected");
        last_expression_.expression = hri_msgs::msg::Expression::HAPPY;
      }

      RCLCPP_INFO(
        this->get_logger(), "I see a person: %s",
        person->id().c_str());
    }

    if (first_seen_person_) {
      std::string fromFrameRel = "sellion_link";
      auto person_face_ = first_seen_person_->face();

      RCLCPP_INFO(
        this->get_logger(), "Goal person id: %s", first_seen_person_->id().c_str());

      if (person_face_) {

        std::string toFrameRel = first_seen_person_->face()->frame();

        RCLCPP_INFO(
          this->get_logger(), "Face frame: %s", toFrameRel.c_str()
        );
        geometry_msgs::msg::TransformStamped t;
        try {
          t = tf_buffer_->lookupTransform(
            toFrameRel, fromFrameRel,
            tf2::TimePointZero);
        } catch (const tf2::TransformException & ex) {
          RCLCPP_INFO(
            this->get_logger(), "Could not transform %s to %s: %s",
            toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
          return;
        }

        geometry_msgs::msg::PointStamped head_position;
        head_position.header.frame_id = fromFrameRel;
        head_position.point.x = -t.transform.translation.x;
        head_position.point.y = -t.transform.translation.y;
        head_position.point.z = t.transform.translation.z;

        position_pub_->publish(head_position);

        RCLCPP_INFO(
          this->get_logger(), "Transform x: %f", t.transform.translation.x
        );
      }
    }

  }
  expression_pub_->publish(last_expression_);
}

}  // namespace robot_guardian

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(robot_guardian::RobotGuardian)
