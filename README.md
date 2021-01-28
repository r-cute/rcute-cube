# rcute-cube

rcute-cube is an independent part of the [Cozmars project](https://github.com/r-cute/rcute-cozmars)

## upload

download rcute-cube.ino.generic.bin and spiffs.bin from release page, then

`python -m esptool --port [PORT] --chip esp8266 --baud 115200 write_flash 0x00 .\rcute-cube.ino.generic.bin 0x200000 .\spiffs.bin`

## make spiffs image

`.\mkspiffs.exe -c .\data\ -b 8192 -s 1835008 .\spiffs.bin`

mkspiffs:
https://github.com/igrr/mkspiffs/releases

## ref

mpu6050:
https://github.com/jrowberg/i2cdevlib

p5.js example:
https://github.com/gdsports/imu-wifi