#INSTALL WIRINGPI LIBRARY
git clone https://github.com/WiringPi/WiringPi.git
cd WiringPi
./build

# Enable UART0
sudo nano boot/firmware/config.txt
voeg toe:
enable_uart=1
dtoverlay=uart0
dan:
sudo reboot
