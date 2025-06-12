#include "AS5600Sensor.h"
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <iostream>
#include <cstdint>

AS5600Sensor::AS5600Sensor(const char* device, int addr)
    : file(-1), i2cDevice(device), address(addr) {}

AS5600Sensor::~AS5600Sensor() {
    if (ownsFile && file >= 0) {
        close(file);
    }
}

bool AS5600Sensor::begin() {
    file = open(i2cDevice, O_RDWR);
    if (file < 0) {
        std::cerr << "Fout bij openen van I2C apparaat\n";
        return false;
    }

    if (ioctl(file, I2C_SLAVE, address) < 0) {
        std::cerr << "Kan geen verbinding maken met AS5600\n";
        close(file);
        file = -1;
        return false;
    }

    ownsFile = true; // <-- deze fd moeten we zelf sluiten
    return true;
}

bool AS5600Sensor::beginWithFile(int openFile) {
    file = openFile;
    if (ioctl(file, I2C_SLAVE, address) < 0) {
        std::cerr << "Kan geen verbinding maken met AS5600 via multiplexer\n";
        return false;
    }

    ownsFile = false; // <-- extern fd, niet zelf sluiten
    return true;
}

bool AS5600Sensor::readRawAngle(int& rawAngle) {
    if (file < 0) return false;

    char reg = 0x0C;
    char data[2] = {0};

    if (write(file, &reg, 1) != 1) {
        std::cerr << "Fout bij schrijven naar register\n";
        return false;
    }

    if (read(file, data, 2) != 2) {
        std::cerr << "Fout bij lezen van data\n";
        return false;
    }

    rawAngle = ((data[0] << 8) | data[1]) & 0x0FFF;

    if (rawAngle == 0) {
        std::cerr << "Ongeldige uitlezing (0)\n";
        return false;
    }

    return true;
}

bool AS5600Sensor::readAngleDeg(float& angleDeg) {
    int raw;
    if (!readRawAngle(raw)) return false;

    angleDeg = (raw / 4096.f) * 360.f;
    return true;
}
