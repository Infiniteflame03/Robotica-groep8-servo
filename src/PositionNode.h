#ifndef POSITIONNODE_H
#define POSITIONNODE_H

#include "rclcpp/rclcpp.hpp"
#include "servo/msg/position.hpp"
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
    void publish(int angle, int height, int distance, int gripper_angle, int claw_angle);

    /**
     *    Callback function for subscription
     *    @param msg to publish
     */
    void callback(const servo::msg::Position::UniquePtr msg);

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<servo::msg::Position>::SharedPtr publisher_;
    rclcpp::Subscription<servo::msg::Position>::SharedPtr subscription_;
    size_t count_;
    ServoControl& servoControl_;
};


#endif //POSITIONNODE_H
