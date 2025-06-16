#include "PositionNode.h"

using namespace std::chrono_literals;
using std::placeholders::_1;

const std::string TOPIC = "servo";


PositionNode::PositionNode(ServoControl& servo_control) : Node("position_node"), servoControl_(servo_control) {
    std::stringstream ss;
    ss << TOPIC << "/current_pos";
    publisher_ = this->create_publisher<servo::msg::Position>(ss.str(), 1);
    auto timer_callback =
        [this]() -> void {
            publish(this->servoControl_.getAngle(), this->servoControl_.getHeight(), this->servoControl_.getDistance(), this->servoControl_.getGripperPitch(), this->servoControl_.getGripperYaw(), this->servoControl_.getClawAngle());
        };
    positionTimer_ = this->create_wall_timer(100ms, timer_callback);

    // Reset stringstream for subscription topic
    ss.str(std::string());
    ss << TOPIC << "/mode";
    modeSubscription_ = this->create_subscription<servo::msg::Mode>(ss.str(), 1, std::bind(&PositionNode::modeCallback, this, _1));
    speedSubscription_ = this->create_subscription<servo::msg::Speed>("servo/speed", 1, std::bind(&PositionNode::speedCallback, this, _1));
    speedTimer_ = this->create_wall_timer(50ms, std::bind(&PositionNode::speedTimerCallback, this));
    //    std::chrono::milliseconds(10),
    //    std::bind(&GripperSequenceNode::timerCallback, this)
    //);
}

void PositionNode::publish(float angle, float height, float distance, float gripper_pitch, float gripper_yaw, float claw_angle) {
    auto msg = servo::msg::Position();
    msg.angle = angle;
    msg.height = height;
    msg.distance = distance;
    msg.gripper_pitch = gripper_pitch;
    msg.gripper_yaw = gripper_yaw;
    msg.claw_angle = claw_angle;
    std::stringstream ss;
    ss << angle << ',' << height << ',' << distance << ',' << gripper_pitch << ',' << gripper_yaw << ',' << claw_angle;
    //RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", ss.str().c_str());
    this->publisher_->publish(msg);
}

void PositionNode::modeCallback(const servo::msg::Mode::UniquePtr msg) {
    std::stringstream ss;
    ss << TOPIC;
    positionSubscription_.reset();
    speedSubscription_.reset();
    speedTimer_->cancel();
    switch(msg->mode) {
        case 0:
            ss << "/speed";
            positionSubscription_.reset();
            speedSubscription_ = this->create_subscription<servo::msg::Speed>("servo/speed", 1, std::bind(&PositionNode::speedCallback, this, _1));
            speedTimer_->reset();
            break;
        case 1:
        case 2:
        case 3:
            ss << "/target_pos";
            speedSubscription_.reset();
            positionSubscription_ = this->create_subscription<servo::msg::Position>("servo/mode", 1, std::bind(&PositionNode::positionCallback, this, _1));
            break;
        default:
            positionSubscription_.reset();
            speedSubscription_.reset();
            break;
    }
}

void PositionNode::positionCallback(const servo::msg::Position::UniquePtr msg) {
    std::stringstream ss;
    ss << msg->angle << ',' << msg->height << ',' << msg->distance << ',' << msg->gripper_pitch << ',' << msg->gripper_yaw << ',' << msg->claw_angle;
    RCLCPP_INFO(this->get_logger(), "Position: I heard: '%s'", ss.str().c_str());
    servoControl_.setAngle(msg->angle);
    servoControl_.setHeight(msg->height);
    servoControl_.setDistance(msg->distance);
    servoControl_.setGripperPitch(msg->gripper_pitch);
    servoControl_.setGripperYaw(msg->gripper_yaw);
    servoControl_.setClawAngle(msg->claw_angle);
}

void PositionNode::speedCallback(const servo::msg::Speed::UniquePtr msg) {
    //std::stringstream ss;
    //ss << msg->angle_speed << ',' << msg->height_speed << ',' << msg->distance_speed << ',' << msg->gripper_pitch << ',' << msg->gripper_yaw << ',' << msg->claw_angle;
    //RCLCPP_INFO(this->get_logger(), "Speed: I heard: '%s'", ss.str().c_str());
    this->angleSpeed_ = msg->angle_speed;
    this->heightSpeed_ = msg->height_speed;
    this->distanceSpeed_ = msg->distance_speed;
    this->gripperPitch_ = msg->gripper_pitch;
    this->gripperYaw_ = msg->gripper_yaw;
    this->clawAngle_ = msg->claw_angle;
}

void PositionNode::speedTimerCallback() {
    RCLCPP_INFO(this->get_logger(), "Speed: %d,%d,%d,%d,%d,%d", angleSpeed_, heightSpeed_, distanceSpeed_, gripperPitch_, gripperYaw_, clawAngle_);
    if (previousAngleSpeed_ != angleSpeed_) {
        servoControl_.setAngleSpeed(this->angleSpeed_);
        previousAngleSpeed_ = angleSpeed_;
    }
    if (previousHeightSpeed_ != heightSpeed_) {
        servoControl_.setHeightSpeed(this->heightSpeed_);
        previousHeightSpeed_ = heightSpeed_;
    }
    if (previousDistanceSpeed_ != distanceSpeed_) {
        servoControl_.setDistanceSpeed(this->distanceSpeed_);
        previousDistanceSpeed_ = distanceSpeed_;
    }
    if (previousGripperPitch_ != gripperPitch_) {
        servoControl_.setGripperPitch(this->gripperPitch_);
        previousGripperPitch_ = gripperPitch_;
    }
    if (previousGripperYaw_ != gripperYaw_) {
        servoControl_.setGripperYaw(this->gripperYaw_);
        previousGripperYaw_ = gripperYaw_;
    }
    if (previousClawAngle_ != clawAngle_) {
        servoControl_.setClawAngle(this->clawAngle_);
        previousClawAngle_ = clawAngle_;
    }
}