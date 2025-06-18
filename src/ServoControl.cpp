#include "ServoControl.h"
using namespace std::chrono_literals;

ServoControl::ServoControl(int angleId, int heightId, int distanceId, int gripperYawId, int gripperPitchId, int clawAngleId, const char* i2cDevice, int multiplexerAddress) :
serial_(UART_DEVICE),
mux(i2cDevice, multiplexerAddress),
angleId_(angleId),
heightId_(heightId),
distanceId_(distanceId),
gripperYawId_(gripperYawId),
gripperPitchId_(gripperPitchId),
clawAngleId_(clawAngleId)
{
    if (!serial_.begin(1000000)) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Serial port failed!");
        throw 1;
    }

    ax12a.begin(BAUDRATE, DIR_PIN, &serial_);

    // Checks if all servo's are connected/found
    auto ids = {angleId_, heightId_, distanceId_, gripperYawId_, gripperPitchId_, clawAngleId_};
    bool fail = false;
    for (auto id : ids) {
        if (ax12a.ping(id)) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Servo not found: %d", id);
            fail = true;
        }
    }
    //if (fail) throw 2;

    // Configure all servo's to the appropriate mode
    setupServos(0);

    //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Claw angle: %f° (%d)", getClawAngle(), getCurrentPosition(clawAngleId_));


    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Initializing multiplexer");
    if (!mux.begin()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to initialize multiplexer");
        throw 2;
    }

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Initializing sensor thread");
    angle_read_thread_ = std::thread([this, i2cDevice]() {
        try {
            while (true) {
                int fd = open(i2cDevice, O_RDWR);
                if (fd < 0) {
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to open i2c port");
                    return;
                }
                //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Reading sensors...");
                float heightAngle_ = -1;
                float distanceAngle_ = -1;
                readAngle(fd, 0, angle_);
                readDistance(fd, 3, height_);
                readAngle(fd, 2, heightAngle_);
                readDistance(fd, 4, distance_);
                readAngle(fd, 1, distanceAngle_);
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Angle: %f, Height: %f, Height Angle: %f, Distance: %f, Distance Angle: %f", getAngle(), height_, heightAngle_, distance_, distanceAngle_);
                close(fd);
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Exception in thread: %s", e.what());
        } catch (...) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Unknown exception in thread");
        }
    });
    angle_read_thread_.detach();
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Thread created");
}

ServoControl::~ServoControl() {
    ax12a.end();
    if (angle_read_thread_.joinable()) {
        angle_read_thread_.join();
    }
}

void ServoControl::setupServos(int mode) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Mode: %d", mode);
    switch (mode) {
        case 1:
        case 2:
        case 3:
            ax12a.setEndless(angleId_, true);
            ax12a.setEndless(heightId_, true);
            ax12a.setEndless(distanceId_, true);
            ax12a.setEndless(gripperPitchId_, false);
            ax12a.setAngleLimit(gripperPitchId_, 0, 1023);
            ax12a.setEndless(gripperYawId_, false);
            ax12a.setAngleLimit(gripperYawId_, 0, 1023);
            ax12a.setEndless(clawAngleId_, false);
            ax12a.setAngleLimit(clawAngleId_, 0, 1023);
            ax12a.setMaxTorque(clawAngleId_, 1023);
            ax12a.setCMargin(clawAngleId_, 0, 0);
            ax12a.setCSlope(clawAngleId_, 0, 0);
            break;
        default:
            ax12a.setEndless(angleId_, true);
            ax12a.setEndless(heightId_, true);
            ax12a.setEndless(distanceId_, true);
            ax12a.setEndless(gripperPitchId_, true);
            ax12a.setEndless(gripperYawId_, true);
            ax12a.setEndless(clawAngleId_, false);
            ax12a.setAngleLimit(clawAngleId_, 0, 1023);
            ax12a.setMaxTorque(clawAngleId_, 1023);
            ax12a.setCMargin(clawAngleId_, 0, 0);
            ax12a.setCSlope(clawAngleId_, 0, 0);
            break;
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
    for (int i = 0; i < 1; i++) {
        bool ok = sensor.readAngleDeg(read_out);
        if (!ok || read_out == 0) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Sensor on channel %d: Invalid readout", channel);
        } else {
            temp += read_out;
            success_count++;
        }
    }
    temp /= success_count;

    //float difference = 180 - std::abs(std::abs(angle_ - temp) - 180);
    /*if (difference > 20 && angle_ != -1)
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Sensor on channel %d: Ignoring output difference to great (%f)", channel, difference);
    else {*/
        //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sensor on channel %d: %f°", channel, temp);
        angle = temp;
    //}
}

void ServoControl::readDistance(int fd, int channel, float& distance) {
    if (!mux.selectChannel(channel)) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to select channel %d", channel);
        close(fd);
        return;
    }

    auto i2c = I2CBus::makeShared("/dev/i2c-1");
    VL53L1X sensor(i2c);
    sensor.setDistanceMode(VL53L1X::DISTANCE_MODE_SHORT);
    sensor.setTimingBudget(VL53L1X::TIMING_BUDGET_20_MS);
    sensor.initialize(); // TODO Fails
    std::this_thread::sleep_for(2ms);

    float temp = 0.f;
    int read_out;
    int success_count = 0;
    sensor.startRanging();
    for (int i = 0; i < 1; i++) {
        read_out = sensor.getDistance();
        if (read_out >= 16384) continue;
        temp += read_out;
        success_count++;
    }
    sensor.stopRanging();
    distance = temp / success_count;
}


void ServoControl::setAngleSpeed(int speed) {
    if (speed == 0) stop(angleId_);
    else turn(angleId_, speed > 0, speed);
}

void ServoControl::setHeightSpeed(int speed) {
    if (speed == 0) stop(heightId_);
    else turn(heightId_, speed > 0, speed);
}

void ServoControl::setDistanceSpeed(int speed) {
    if (speed == 0) stop(distanceId_);
    else turn(distanceId_, speed > 0, speed);
}

void ServoControl::setGripperYawSpeed(int speed) {
    if (speed == 0) stop(gripperYawId_);
    else turn(gripperYawId_, speed > 0, speed);
}

void ServoControl::setGripperPitchSpeed(int speed) {
    if (speed == 0) stop(gripperPitchId_);
    else turn(gripperPitchId_, speed > 0, speed);
}

void ServoControl::setClawSpeed(int speed) {
    if (speed == 0) stop(clawAngleId_);
    else turn(clawAngleId_, speed > 0, speed);
}

void ServoControl::setAngle(float angle) {
    if (angle == targetAngle_) return;
    targetAngle_ = angle;
    float diff = fmodf((targetAngle_ - this->getAngle() + 360.0f), 360.0f);
    bool dir = diff > 180.0f;
    turn(angleId_, dir);
    angle_thread_ = std::thread([this, angle]() {
        while (std::abs(this->getAngle() - angle) > ANGLE_MARGIN) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        stop(this->angleId_);
        targetAngle_ = -1;
    });
}

void ServoControl::setHeight(float height) {
    if (height == targetHeight_) return;
    targetHeight_ = height;
    height_thread_ = std::thread([this, height]() {
        float travel_distance = height_-height;
        // TODO set appropriate direction (depends on gearing)
        bool dir = travel_distance <= 0;
        int speed = 1023;
        turn(heightId_, dir, speed);
        while (std::abs(travel_distance) > DISTANCE_MARGIN) {
            std::cout << "Difference: " << std::abs(travel_distance) << "\tCurrent height:" << height_ << std::endl;
        }
        stop(distanceId_);
        targetHeight_ = -1;
    });
}

void ServoControl::setDistance(float distance) {
    if (distance == targetDistance_) return;
    targetDistance_ = distance;
    std::thread distance_thread_ = std::thread([this, distance]() {
        float travel_distance = distance_-distance;
        // TODO set appropriate direction (depends on gearing)
        bool dir = travel_distance <= 0;
        int speed = 512;
        turn(distanceId_, dir, speed);
        while (std::abs(travel_distance) > DISTANCE_MARGIN) {
            std::cout << "Difference: " << std::abs(travel_distance) << "\tCurrent distance:" << distance_ << std::endl;
        }
        stop(distanceId_);
        targetDistance_ = -1;
    });
}

void ServoControl::setGripperYaw(float gripper_yaw) {
    if (gripper_yaw == targetGripperYaw_ || gripper_yaw > 180 || gripper_yaw < 0) return;
    setGoalPosition(gripperYawId_, angleToPosition(gripper_yaw), 128);
    targetGripperYaw_ = gripper_yaw;
}

void ServoControl::setGripperPitch(float gripper_pitch) {
    if (gripper_pitch == targetGripperPitch_ || gripper_pitch > 180 || gripper_pitch < 0) return;
    setGoalPosition(gripperPitchId_, angleToPosition(gripper_pitch), 128);
    targetGripperPitch_ = gripper_pitch;
}

void ServoControl::setClawAngle(float claw_angle) {
    if (claw_angle == targetClawAngle_) return;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Moving claw to angle %f", claw_angle);
    ax12a.setEndless(clawAngleId_, false);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    claw_angle += 20;
    setGoalPosition(clawAngleId_, angleToPosition(claw_angle), 192);
    if (claw_angle >= 90) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        ax12a.setEndless(clawAngleId_, true);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        turn(clawAngleId_, false, 600);
    }
    targetClawAngle_ = claw_angle;
}

float ServoControl::getAngle(void) {
    return fmodf(angle_ + 360.f - ANGLE_OFFSET, 360.f);
}

float ServoControl::getHeight(void) {
    return height_; // TODO
}

float ServoControl::getDistance(void) {
    return distance_; // TODO
}

float ServoControl::getGripperYaw(void) {
    return positionToAngle(getCurrentPosition(gripperYawId_));
}

float ServoControl::getGripperPitch(void) {
    return positionToAngle(getCurrentPosition(gripperPitchId_));
}

float ServoControl::getClawAngle(void) {
    //return static_cast<float>(getCurrentPosition(clawAngleId_));
    return positionToAngle(getCurrentPosition(clawAngleId_));
}

float ServoControl::positionToAngle(int position) {
    return static_cast<float>(position)*300.f/1023.f;
}

int ServoControl::angleToPosition(float angle) {
    return static_cast<int>(round(static_cast<float>(angle)/300.f*1023.f));
}

void ServoControl::setGoalPosition(int id, int position, int speed) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Moving servo %d to position %d", id, position);
    ax12a.moveSpeed(id, position, speed);
}

void ServoControl::turn(int id, bool direction, int speed) {
    if (speed == 0) return;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Turning servo %d %s", id, direction ? "CW" : "CCW");
    ax12a.turn(id, direction, speed >= 0 ? speed : speed*-1);
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