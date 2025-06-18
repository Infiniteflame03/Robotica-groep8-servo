#ifndef SERVOCONTROL_H
#define SERVOCONTROL_H

#include "rclcpp/rclcpp.hpp"
#include "AX12A/AX12A.h"
#include "TCA9548A.h"
#include "AS5600Sensor.h"
#include "VL53L1X.hpp"
#include <cmath>
#include <fcntl.h>  // voor O_RDWR en open()
#include <unistd.h>   // close()

const std::string   UART_DEVICE        = "/dev/ttyAMA0";
const long          BAUDRATE           = 1000000;
const unsigned char DIR_PIN            = 23;             // GPIO 23
const bool          CW                 = false;
const bool          CCW                = true;
const float         DISTANCE_MARGIN    = 100.f;
const float         ANGLE_MARGIN       = 10.f;
const float         ANGLE_OFFSET       = 166.6f;
const float         HEIGHT_MIN         = 394.f;
const float         HEIGHT_MIN_ANGLE   = 324.f;
const float         HEIGHT_MAX         = 114.f;
const float         HEIGHT_MAX_ANGLE   = 86.f;
const float         DISTANCE_MIN       = 380.f;
const float         DISTANCE_MIN_ANGLE = 302.7f;
const float         DISTANCE_MAX       = 40.f;
const float         DISTANCE_MAX_ANGLE = 64.5f;
// angle per mm for distance = 1,78

class ServoControl {
public:
    ServoControl(int angleId, int heightId, int distanceId, int gripperYawId, int gripperPitchId, int clawAngleId, const char* i2cDevice, int multiplexerAddress);
    ~ServoControl();

    void setAngleSpeed(int speed);
    void setHeightSpeed(int speed);
    void setDistanceSpeed(int speed);
    void setGripperYawSpeed(int speed);
    void setGripperPitchSpeed(int speed);
    void setClawSpeed(int speed);

    void setAngle(float angle);
    void setHeight(float height);
    void setDistance(float distance);
    void setGripperYaw(float gripperYaw);
    void setGripperPitch(float gripperPitch);
    void setClawAngle(float clawAngle);

    float getAngle(void);
    float getHeight(void);
    float getDistance(void);
    float getGripperYaw(void);
    float getGripperPitch(void);
    float getClawAngle(void);

    void setupServos(int mode);
    /**
     *    Enables/Disables torque for a servo
     *    @param id of servo
     *    @param enable
     */
    void torque(int id, bool enable);
    /**
     *    Turns a servo in a direction
     *    @param id of servo
     *    @param direction true = CW, false = CCW
     *    !!! Only use servo_id's that have been configured to use infinite rotation (not angles)
     */
    void turn(int id, bool direction, int speed = 512);
    /**
     *    Stops a servo from turning
     *    @param id of servo
     *    !!! Only use servo_id's that have been configured to use infinite rotation (not angles)
     */
    void stop(int id);

private:
    void readAngle(int fd, int channel, float& angle);
    /**
     *    Measures the distance of a TOF sensor
     *    @param fd filedescriptor of multiplexer
     *    @param channel of TOF sensor
     *    @param distance result variable
     */
    void readDistance(int fd, int channel, float& distance);
    /**
     *    Converts a servo position to an angle in °
     *    @param position to convert
     *    @return angle in °
     */
    static float positionToAngle(int position);
    /**
     *    Converts an angle in ° to a servo position
     *    @param angle in ° to convert
     *    @return position
     */
    static int angleToPosition(float angle);
    /**
     *    Sets the goal position of a servo
     *    @param id of servo
     *    @param angle (0-1023)
     *    !!! Only use servo_id's that have been configured to use angles (not infinite rotation)
     */
    void setGoalPosition(int id, int position, int speed = 512);
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
    TCA9548A mux;

    // Servo id's
    int angleId_;
    int heightId_;
    int distanceId_;
    int gripperYawId_;
    int gripperPitchId_;
    int clawAngleId_;

    // Target values
    float targetAngle_ =-1;
    float targetHeight_ = -1;
    float targetDistance_ = -1;
    float targetGripperYaw_ = -1;
    float targetGripperPitch_ = -1;
    float targetClawAngle_ = -1;

    // Current values
    float angle_ = -1;
    float height_ = -1;
    float distance_ = -1;
    float gripperYaw_ = -1;
    float gripperPitch_ = -1;
    float clawAngle_ = -1;

    std::thread angle_read_thread_;
    std::thread angle_thread_;
    std::thread height_thread_;
    std::thread distance_thread_;
};



#endif //SERVOCONTROL_H
