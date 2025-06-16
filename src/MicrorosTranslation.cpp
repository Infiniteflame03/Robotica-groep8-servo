#include "MicrorosTranslation.h"

using std::placeholders::_1;

MicrorosTranslation::MicrorosTranslation()
: Node("microros_translation") {
    RCLCPP_INFO(this->get_logger(), "Translator created");
    modeSubscription_ = this->create_subscription<std_msgs::msg::Int32>("microros/mode", 1, std::bind(&MicrorosTranslation::modeTopicCallback, this, _1));
    speedSubscription_ = this->create_subscription<std_msgs::msg::Int32MultiArray>("microros/speed", 1, std::bind(&MicrorosTranslation::speedTopicCallback, this, _1));

    modePublisher_ = this->create_publisher<servo::msg::Mode>("servo/mode", 1);
    speedPublisher_ = this->create_publisher<servo::msg::Speed>("servo/speed", 1);
    RCLCPP_INFO(this->get_logger(), "Subscribers + Publishers created");
}

void MicrorosTranslation::modeTopicCallback(const std_msgs::msg::Int32::UniquePtr msg) {
    RCLCPP_INFO(this->get_logger(), "Mode received and published");
    servo::msg::Mode mode_msg;
    mode_msg.mode = static_cast<int>(msg->data);
    modePublisher_->publish(mode_msg);
}

void MicrorosTranslation::speedTopicCallback(const std_msgs::msg::Int32MultiArray::UniquePtr msg) {

    //RCLCPP_INFO(this->get_logger(), "Speed array size: %ld", msg->data.size());
    if (msg->data.size() != 6) {
        RCLCPP_WARN(this->get_logger(), "Received array with insufficient data");
        return;
    }

    servo::msg::Speed speed_msg;
    speed_msg.angle_speed = static_cast<int16_t>(msg->data[0]);
    speed_msg.height_speed = static_cast<int16_t>(msg->data[1]);
    speed_msg.distance_speed = static_cast<int16_t>(msg->data[2]);
    speed_msg.gripper_yaw = static_cast<float>(msg->data[3]);
    speed_msg.gripper_pitch = static_cast<float>(msg->data[4]);
    speed_msg.claw_angle = static_cast<float>(msg->data[5]);//(msg->data[5]) ? 90.f: 0.f;

    speedPublisher_->publish(speed_msg);
    //RCLCPP_INFO(this->get_logger(), "Speed received and published");
}