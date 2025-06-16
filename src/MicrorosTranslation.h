#ifndef MICROROSTRANSLATION_H
#define MICROROSTRANSLATION_H

#include "rclcpp/rclcpp.hpp"
#include "servo/msg/mode.hpp"
#include "servo/msg/speed.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

class MicrorosTranslation : public rclcpp::Node {
public:
    MicrorosTranslation();

private:
    void modeTopicCallback(const std_msgs::msg::Int32::UniquePtr msg);
    void speedTopicCallback(const std_msgs::msg::Int32MultiArray::UniquePtr msg);

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr modeSubscription_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr speedSubscription_;
    rclcpp::Publisher<servo::msg::Mode>::SharedPtr modePublisher_;
    rclcpp::Publisher<servo::msg::Speed>::SharedPtr speedPublisher_;
};



#endif //MICROROSTRANSLATION_H
