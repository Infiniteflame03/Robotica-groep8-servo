#ifndef POSITIONNODE_H
#define POSITIONNODE_H

#include "rclcpp/rclcpp.hpp"
#include "servo/msg/mode.hpp"
#include "servo/msg/position.hpp"
#include "servo/msg/speed.hpp"
#include "ServoControl.h"

#include <chrono>
#include <cstdlib>
#include <memory>
#include <string>

class PositionNode : public rclcpp::Node {
public:
    PositionNode(ServoControl& servo_control);

private:
    /**
     *    Publishes the angle, height, distance, gripper angle and claw angle
     *    @param angle
     *    @param height
     *    @param distance
     *    @param gripper_angle
     *    @param claw_angle
     */
    void publish(float angle, float height, float distance, float gripper_pitch, float gripper_yaw, float claw_angle);

    /**
     *    Callback function for mode subscription
     *    @param msg received message
     */
    void modeCallback(const servo::msg::Mode::UniquePtr msg);
    /**
     *    Callback function for position subscription
     *    @param msg received message
     */
    void positionCallback(const servo::msg::Position::UniquePtr msg);
    /**
     *    Callback function for speed subscription
     *    @param msg received message
     */
    void speedCallback(const servo::msg::Speed::UniquePtr msg);
    /**
     *    Callback function for sending speed control commands to servo's
     */
    void speedTimerCallback();

    rclcpp::TimerBase::SharedPtr positionTimer_;
    rclcpp::TimerBase::SharedPtr speedTimer_;
    rclcpp::Publisher<servo::msg::Position>::SharedPtr publisher_;
    rclcpp::Subscription<servo::msg::Mode>::SharedPtr modeSubscription_;
    rclcpp::Subscription<servo::msg::Position>::SharedPtr positionSubscription_;
    rclcpp::Subscription<servo::msg::Speed>::SharedPtr speedSubscription_;
    size_t count_;
    ServoControl& servoControl_;

    int angleSpeed_;
    int heightSpeed_;
    int distanceSpeed_;
    int gripperPitch_;
    int gripperYaw_;
    int clawAngle_;

    int previousAngleSpeed_;
    int previousHeightSpeed_;
    int previousDistanceSpeed_;
    int previousGripperPitch_;
    int previousGripperYaw_;
    int previousClawAngle_;
};


#endif //POSITIONNODE_H
