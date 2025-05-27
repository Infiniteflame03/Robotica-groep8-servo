#ifndef HARDWARESERIALRPI_H
#define HARDWARESERIALRPI_H

#include <string>
#include <cstdint>

class HardwareSerialRPi {
public:
    HardwareSerialRPi(const std::string& device = "/dev/serial0");
    ~HardwareSerialRPi();

    bool begin(int baudrate);
    void end();

    int available();
    int read();
    int peek(); // Not supported yet
    void flush();
    ssize_t write(const uint8_t* data, size_t len);
    ssize_t write(uint8_t byte);

private:
    int fd;
    std::string devicePath;
    uint8_t buffer;
    bool bufferAvailable = false;
};

#endif
