#include "HardwareSerialRPi.h"
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <cstring>
#include <iostream>

HardwareSerialRPi::HardwareSerialRPi(const std::string& device)
    : fd(-1), devicePath(device) {}

HardwareSerialRPi::~HardwareSerialRPi() {
    end();
}

bool HardwareSerialRPi::begin(int baudrate) {
    fd = open(devicePath.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) {
        perror("Unable to open serial port");
        return false;
    }

    struct termios options;
    tcgetattr(fd, &options);

    speed_t brate;
    switch (baudrate) {
        case 9600: brate = B9600; break;
        case 19200: brate = B19200; break;
        case 38400: brate = B38400; break;
        case 57600: brate = B57600; break;
        case 115200: brate = B115200; break;
        case 1000000: brate = B1000000; break;
        default:
            std::cerr << "Unsupported baudrate\n";
            return false;
    }

    cfsetispeed(&options, brate);
    cfsetospeed(&options, brate);

    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;

    //options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    //options.c_iflag &= ~(IXON | IXOFF | IXANY);
    //options.c_oflag &= ~OPOST;

    options.c_lflag = 0;
    options.c_iflag = 0;
    options.c_oflag = 0;

    tcsetattr(fd, TCSANOW, &options);
    return true;
}

void HardwareSerialRPi::end() {
    if (fd != -1) {
        close(fd);
        fd = -1;
    }
}

int HardwareSerialRPi::available() {
    if (fd == -1) return 0;
    int count;
    ioctl(fd, FIONREAD, &count);
    return count;
}
/*
int HardwareSerialRPi::read() {
    if (fd == -1) return -1;
    uint8_t byte;
    if (::read(fd, &byte, 1) == 1) return byte;
    return -1;
}

int HardwareSerialRPi::peek() {
    // Not implemented: would require internal buffering
    std::cerr << "peek() not supported\n";
    return -1;
}*/

int HardwareSerialRPi::read() {
    if (fd == -1) return -1;

    if (bufferAvailable) {
        bufferAvailable = false;
        return buffer;
    }

    uint8_t byte;
    if (::read(fd, &byte, 1) == 1) return byte;
    return -1;
}


int HardwareSerialRPi::peek() {
    if (fd == -1) return -1;

    if (bufferAvailable) {
        return buffer;
    }

    uint8_t byte;
    if (::read(fd, &byte, 1) == 1) {
        buffer = byte;
        bufferAvailable = true;
        return buffer;
    }

    return -1;
}

void HardwareSerialRPi::flush() {
    if (fd != -1) tcdrain(fd);
}

ssize_t HardwareSerialRPi::write(const uint8_t* data, size_t len) {
    if (fd == -1) return -1;
    return ::write(fd, data, len);
}

ssize_t HardwareSerialRPi::write(uint8_t byte) {
    return write(&byte, 1);
}