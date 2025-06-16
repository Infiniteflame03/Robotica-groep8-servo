# Instructions
## INSTALL WIRINGPI LIBRARY  

    git clone https://github.com/WiringPi/WiringPi.git
    cd WiringPi
    ./build


## Enable UART0
1. 
        sudo nano boot/firmware/config.txt
2. add to the end:

        enable_uart=1
        dtoverlay=uart0
3. 
       sudo reboot

## Allow acces to GPIO
Parts of the program require GPIO access, but this requires certain permissions.  
Easiest **(but definitely not the safest)** way to fix this is:

    sudo chmod 777 /dev -R

## Startup
Startup requires two programs to run
### Micro Ros Agent

    source install/setup.sh
    ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888

### Main Program
    
    ros2 run servo main
