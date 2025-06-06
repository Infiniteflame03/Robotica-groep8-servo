#ifndef AS5600SENSOR_H
#define AS5600SENSOR_H

class AS5600Sensor {
private:
    int file;
    const char* i2cDevice;
    bool ownsFile = false;
public:
    int address;
    AS5600Sensor(const char* device = "/dev/i2c-1", int addr = 0x36);
    ~AS5600Sensor();

    bool begin();
    bool beginWithFile(int openFile); // belangrijk voor multiplexer
    bool readRawAngle(int& rawAngle);
    bool readAngleDeg(float& angleDeg);
    
};

#endif // AS5600SENSOR_H
