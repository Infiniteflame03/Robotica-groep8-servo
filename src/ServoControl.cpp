#include "ServoControl.h"

ServoControl::ServoControl(int angleId, int heightId, int distanceId, int gripperAngleId, int clawAngleId, const char* i2cDevice, int multiplexerAddress) :
serial_(UART_DEVICE),
mux(i2cDevice, multiplexerAddress),
angleId_(angleId),
heightId_(heightId),
distanceId_(distanceId),
gripperAngleId_(gripperAngleId),
clawAngleId_(clawAngleId)
{
    if (!serial_.begin(1000000)) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Serial port failed!");
        throw 1;
    }

    ax12a.begin(BAUDRATE, DIR_PIN, &serial_);

    // Checks if all servo's are connected/found
    auto ids = {angleId_, heightId_, distanceId_, gripperAngleId_, clawAngleId_};
    bool fail = false;
    for (auto id : ids) {
        if (ax12a.ping(id)) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Servo not found: %d", id);
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
    ax12a.setMaxTorque(clawAngleId_, 1023);
    ax12a.setCMargin(clawAngleId_, 0, 0);
    ax12a.setCSlope(clawAngleId_, 0, 0);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Claw angle: %f° (%d)", getClawAngle(), getCurrentPosition(clawAngleId_));

    if (!mux.begin()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to initialize multiplexer");
        throw 2;
    }

    angle_read_thread_ = std::thread([this, i2cDevice]() {
        try {
            while (true) {
                int fd = open(i2cDevice, O_RDWR);
                if (fd < 0) {
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to open i2c port");
                    return;
                }
                int channel = 3;
                readAngle(fd, channel, angle_);
                //readDistance(fd, channel, height_);
                close(fd);
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Exception in thread: %s", e.what());
        } catch (...) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Unknown exception in thread");
        }
    });
}

ServoControl::~ServoControl() {
    ax12a.end();
    if (angle_read_thread_.joinable()) {
        angle_read_thread_.join();
    }
}

void ServoControl::readAngle(int fd, int channel, float& angle) {
    if (!mux.selectChannel(channel)) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to select channel %d", channel);
        close(fd);
        return;
    }

    AS5600Sensor sensor;
    if (!sensor.beginWithFile(fd)) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to initialize sensor on channel %d", channel);
        close(fd);
        return;
    }

    float temp = 0.f;
    float read_out = 0.f;
    int success_count = 0;
    for (int i = 0; i < 10; i++) {
        bool ok = sensor.readAngleDeg(read_out);
        if (!ok || read_out == 0) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Sensor on channel %d: Invalid readout", channel);
        } else {
            temp += read_out;
            success_count++;
        }
    }
    temp /= success_count;

    float difference = 180 - std::abs(std::abs(angle_ - temp) - 180);
    if (difference > 20 && angle_ != -1)
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Sensor on channel %d: Ignoring output difference to great (%f)", channel, difference);
    else {
        //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sensor on channel %d: %f°", channel, temp);
        angle = temp;
    }
}

void ServoControl::setAngle(float angle) {
    if (angle == targetAngle_) return;
    targetAngle_ = angle;
    float diff = fmodf((targetAngle_ - angle_ + 360.0f), 360.0f);
    bool dir = diff > 180.0f;
    turn(angleId_, dir);
    std::thread([this, angle]() {
        while (std::abs(this->angle_ - angle) > ANGLE_MARGIN) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        stop(this->angleId_);
    }).detach();
}

void ServoControl::setHeight(float height) {
    if (height == targetHeight_) return;
    // TODO read height and set appropriate direction
    bool dir = CW;
    turn(heightId_, dir);
    targetHeight_ = height;
}

void ServoControl::setDistance(float distance) {
    if (distance == targetDistance_) return;
    targetDistance_ = distance;
    /*std::thread t(() {
        float travel_distance = current_distance-distance;
        // TODO set appropriate direction (depends on gearing)
        bool dir = std::abs(current_distance-distance) <= 0;
        int speed = 512;
        turn(distanceId_, dir, speed);
        // TODO read current distance from TOF sensor
        float current_distance = 0.f;
        while (current_distance < distance) {
            if (std::abs(current_distance-distance) < DISTANCE_MARGIN) {
                stop(distanceId_);
                return;
            }
        }
    });*/
}

void ServoControl::setGripperAngle(float gripper_angle) {
    if (gripper_angle == targetGripperAngle_) return;
    setGoalPosition(gripperAngleId_, angleToPosition(gripper_angle), 64);
    targetGripperAngle_ = gripper_angle;
}

void ServoControl::setClawAngle(float claw_angle) {
    if (claw_angle == targetClawAngle_) return;
    ax12a.setEndless(clawAngleId_, false);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    setGoalPosition(clawAngleId_, angleToPosition(claw_angle), 128);
    if (claw_angle >= 90) {
        //setGoalPosition(clawAngleId_, angleToPosition(claw_angle), 64);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        ax12a.setEndless(clawAngleId_, true);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        turn(clawAngleId_, false, 700);
    }
    targetClawAngle_ = claw_angle;
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
    return static_cast<float>(getCurrentPosition(clawAngleId_));
    //return positionToAngle(getCurrentPosition(clawAngleId_));
}

float ServoControl::positionToAngle(int position) {
    return static_cast<float>(position)*1023.f/300.f;
}

int ServoControl::angleToPosition(float angle) {
    return static_cast<int>(round(static_cast<float>(angle)/300.f*1023.f));
}

void ServoControl::setGoalPosition(int id, int position, int speed) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Moving servo %d to position %d", id, position);
    ax12a.moveSpeed(id, position, speed);
}

void ServoControl::turn(int id, bool direction, int speed) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Turning servo %d %s", id, direction ? "CW" : "CCW");
    ax12a.turn(id, direction, speed);
}

void ServoControl::stop(int id) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Stopped servo %d", id);
    ax12a.turn(id, false, 0);
}

int ServoControl::getCurrentPosition(int id) {
    return ax12a.readPosition(id);
}

void ServoControl::torque(int id, bool enable) {
    ax12a.torqueStatus(id, enable);
}
/*
float ServoControl::getGoalPosition(int id) {
    int position = ax12a.readRegister(id, 30, 2);
    return positionToAngle(position);
}*/