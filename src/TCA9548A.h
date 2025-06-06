#ifndef TCA9548A_H
#define TCA9548A_H

#include <string>

class TCA9548A {
private:
    int file;
    std::string i2cDevice;
    int address;

    
public:
    int getFileDescriptor() const;
    TCA9548A(const char* device = "/dev/i2c-1", int addr = 0x70);
    ~TCA9548A();

    bool begin();
    bool selectChannel(int channel);
};

#endif // TCA9548A_H
