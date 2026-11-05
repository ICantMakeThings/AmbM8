## How to build
```
# connect epaper module
# res pin  -> Pin 11
# busy pin -> Pin 18
# d/c pin  -> Pin 22
# cs pin   -> Pin 24
# sck/scl pin  -> Pin 23
# mosi/sda pin -> Pin 19

# enable spi
sudo raspi-config 
# Interfacing Options -> SPI -> Yes

# reboot
sudo reboot

# install wiringpi
git clone https://github.com/WiringPi/WiringPi
cd WiringPi
./build

# build epaper example
make clean
# 4.2 Inch Epaper Module Black/White/Red
make EPD_TYPE=EPD420_BWR
# 4.2 Inch Epaper Module Black/White
make EPD_TYPE=EPD420_BW

# run epaper example
./bin/epd
```