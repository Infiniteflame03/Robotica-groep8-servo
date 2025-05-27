#ifndef SERVOCONTROL_H
#define SERVOCONTROL_H

#include "AX12A/AX12A.h"
#include "rclcpp/rclcpp.hpp"
#include <cmath>

#define UART_DEVICE     "/dev/ttyAMA0"
#define BAUDRATE        1000000
#define DIR_PIN         23              // GPIO 23

class ServoControl {
public:
    ServoControl(int angleId, int heightId, int distanceId, int gripperAngleId, int clawAngleId);
    ~ServoControl();

    void setAngle(float angle);
    void setHeight(float height);
    void setDistance(float distance);
    void setGripperAngle(float gripperAngle);
    void setClawAngle(float clawAngle);

    float getAngle(void);
    float getHeight(void);
    float getDistance(void);
    float getGripperAngle(void);
    float getClawAngle(void);

private:
    static float positionToAngle(int position);
    static int angleToPosition(float angle);
    /**
     *    Sets the goal position of a servo
     *    @param id of servo
     *    @param angle (0-1023)
     *    !!! Only use servo_id's that have been configured to use angles (not infinite rotation)
     */
    void setGoalPosition(int id, int position);
    /**
     *    Turns a servo in a direction
     *    @param id of servo
     *    @param direction true = CW, false = CCW
     *    !!! Only use servo_id's that have been configured to use infinite rotation (not angles)
     */
    void turn(int id, bool direction);
    /**
     *    Stops a servo from turning
     *    @param id of servo
     *    !!! Only use servo_id's that have been configured to use infinite rotation (not angles)
     */
    void stop(int id);
    /**
     *    Gets the current position of the servo with specified id
     *    @param id of servo
     *    @return current angle (0-1023)
     */
    int getCurrentPosition(int id);
    /**
     *    Gets the goal position of the servo with specified id
     *    @param id of servo
     *    @return goal angle (0-1023)
     */

    HardwareSerialRPi serial_;

    // Servo id's
    int angleId_;
    int heightId_;
    int distanceId_;
    int gripperAngleId_;
    int clawAngleId_;

    // Current (target) values
    float angle_;
    float height_;
    float distance_;
    float gripperAngle_;
    float clawAngle_;
};



#endif //SERVOCONTROL_H
