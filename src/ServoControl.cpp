#include "ServoControl.h"

#define CW  0
#define CCW 1

ServoControl::ServoControl(int angleId, int heightId, int distanceId, int gripperAngleId, int clawAngleId) :
serial_(UART_DEVICE),
angleId_(angleId),
heightId_(heightId),
distanceId_(distanceId),
gripperAngleId_(gripperAngleId),
clawAngleId_(clawAngleId)
{
    if (!serial_.begin(1000000)) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Serial port failed!");
        throw 1;
    }

    ax12a.begin(BAUDRATE, DIR_PIN, &serial_);

    // Checks if all servo's are connected/found
    auto ids = {angleId_, heightId_, distanceId_, gripperAngleId_, clawAngleId_};
    bool fail = false;
    for (auto id : ids) {
        if (ax12a.ping(id)) {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Servo not found: %d", id);
            fail = true;
        }
    }
    if (fail) throw 2;

    // Configure all servo's to the appropriate mode
    ax12a.setEndless(angleId_, true);
    ax12a.setEndless(heightId_, true);
    ax12a.setEndless(distanceId_, true);
    ax12a.setEndless(gripperAngleId_, false);
    ax12a.setAngleLimit(gripperAngleId_, 0, 1023);
    ax12a.setEndless(clawAngleId_, false);
    ax12a.setAngleLimit(clawAngleId_, 0, 1023);
}

ServoControl::~ServoControl() {
    ax12a.end();
}

void ServoControl::setAngle(float angle) {
    if (angle == angle_) return;
    // TODO read angle and set appropriate direction
    bool dir = CW;
    turn(angleId_, dir);
    angle_ = angle;
}

void ServoControl::setHeight(float height) {
    if (height == height_) return;
    // TODO read height and set appropriate direction
    bool dir = CW;
    turn(heightId_, dir);
    height_ = height;
}

void ServoControl::setDistance(float distance) {
    if (distance == distance_) return;
    // TODO read distance and set appropriate direction
    bool dir = CW;
    turn(distanceId_, dir);
    distance_ = distance;
}

void ServoControl::setGripperAngle(float gripper_angle) {
    if (gripper_angle == gripperAngle_) return;
    setGoalPosition(gripperAngleId_, gripper_angle);
    gripperAngle_ = gripper_angle;
}

void ServoControl::setClawAngle(float claw_angle) {
    if (claw_angle == clawAngle_) return;
    setGoalPosition(clawAngleId_, claw_angle);
    clawAngle_ = claw_angle;
}

float ServoControl::getAngle(void) {
    return 0.f; // TODO
}

float ServoControl::getHeight(void) {
    return 0.f; // TODO
}

float ServoControl::getDistance(void) {
    return 0.f; // TODO
}

float ServoControl::getGripperAngle(void) {
    return positionToAngle(getCurrentPosition(gripperAngleId_));
}

float ServoControl::getClawAngle(void) {
    return positionToAngle(getCurrentPosition(clawAngleId_));
}

float ServoControl::positionToAngle(int position) {
    return static_cast<float>(position)*1023.f/300.f;
}

int ServoControl::angleToPosition(float angle) {
    return static_cast<int>(round(static_cast<float>(angle)/300.f*1023.f));
}

void ServoControl::setGoalPosition(int id, int position) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Moving servo %d to position %d", id, position);
    ax12a.moveSpeed(id, position, 512);
}

void ServoControl::turn(int id, bool direction) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Turning servo %d %s", id, direction ? "CW" : "CCW");
    ax12a.turn(id, direction, 512);
}

void ServoControl::stop(int id) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Stopped servo %d", id);
    ax12a.turn(id, false, 0);
}

int ServoControl::getCurrentPosition(int id) {
    return ax12a.readPosition(id);
}
/*
float ServoControl::getGoalPosition(int id) {
    int position = ax12a.readRegister(id, 30, 2);
    return positionToAngle(position);
}*/