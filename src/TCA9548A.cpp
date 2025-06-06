#include "TCA9548A.h"
#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cstdint>  // voor uint8_t

int TCA9548A::getFileDescriptor() const {
    return file;
}

TCA9548A::TCA9548A(const char* device, int addr)
    : file(-1), i2cDevice(device), address(addr) {}

TCA9548A::~TCA9548A() {
    if (file >= 0) close(file);
}

bool TCA9548A::begin() {
    file = open(i2cDevice.c_str(), O_RDWR);
    if (file < 0) {
        perror("open i2c device failed");
        return false;
    }
    std::cout << "I2C device opened with fd: " << file << std::endl;

    if (ioctl(file, I2C_SLAVE, address) < 0) {
        perror("ioctl set slave address failed");
        close(file);
        file = -1;
        return false;
    }
    std::cout << "Multiplexer at address 0x" << std::hex << address << std::dec << " selected\n";
    return true;
}

bool TCA9548A::selectChannel(int channel) {
    if (file < 0) {
        std::cerr << "File descriptor ongeldig (mux niet geopend)\n";
        return false;
    }

    if (ioctl(file, I2C_SLAVE, address) < 0) {
        perror("ioctl fout bij instellen van mux-adres");
        return false;
    }

    uint8_t config = 1 << channel;
    /*std::cout << "Schrijf 0x" << std::hex << static_cast<int>(config)
              << " naar mux op kanaal " << std::dec << channel << ", fd: " << file << "\n";
    */
    if (write(file, &config, 1) != 1) {
        perror("write fout");
        return false;
    }

    return true;
}

