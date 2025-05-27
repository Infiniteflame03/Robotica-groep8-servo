#include "PositionNode.h"

static std::string TOPIC = "servo";

using namespace std::chrono_literals;
using std::placeholders::_1;


PositionNode::PositionNode(ServoControl& servo_control) : Node("position_node"), servoControl_(servo_control) {
    std::stringstream ss;
    ss << TOPIC << "/current_pos";
    publisher_ = this->create_publisher<servo::msg::Position>(ss.str(), 1);
    auto timer_callback =
        [this]() -> void {
            publish(this->servoControl_.getAngle(), this->servoControl_.getHeight(), this->servoControl_.getDistance(), this->servoControl_.getGripperAngle(), this->servoControl_.getClawAngle());
        };
    timer_ = this->create_wall_timer(100ms, timer_callback);

    // Reset stringstream for subscription topic
    ss.str(std::string());
    ss << TOPIC << "/target_pos";
    subscription_ = this->create_subscription<servo::msg::Position>(ss.str(), 1, std::bind(&PositionNode::callback, this, _1));
}

void PositionNode::publish(int angle, int height, int distance, int gripper_angle, int claw_angle) {
    auto msg = servo::msg::Position();
    msg.angle = angle;
    msg.height = height;
    msg.distance = distance;
    msg.gripper_angle = gripper_angle;
    msg.claw_angle = claw_angle;
    std::stringstream ss;
    ss << angle << ',' << height << ',' << distance << ',' << gripper_angle << ',' << claw_angle;
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", ss.str().c_str());
    this->publisher_->publish(msg);
}

void PositionNode::callback(const servo::msg::Position::UniquePtr msg) {
    std::stringstream ss;
    ss << msg->angle << ',' << msg->height << ',' << msg->distance << ',' << msg->gripper_angle << ',' << msg->claw_angle;
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", ss.str().c_str());
    servoControl_.setAngle(msg->angle);
    servoControl_.setHeight(msg->height);
    servoControl_.setDistance(msg->distance);
    servoControl_.setGripperAngle(msg->gripper_angle);
    servoControl_.setClawAngle(msg->claw_angle);
}