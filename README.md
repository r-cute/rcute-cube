# rcute-cube

rcute-cube is an independent part of the [Cozmars project](https://github.com/r-cute/rcute-cozmars)

## Upload pre-built bin files

0. `pip install esptool`
1. download rcute-cube.ino.generic.bin and spiffs.bin from release page, then
2. `python -m esptool --port [PORT] --chip esp8266 --baud 115200 write_flash 0x00 .\rcute-cube.ino.generic.bin 0x200000 .\spiffs.bin`

## Electronic parts

* 3.7v 14500 battery
* 112D on/off power button
* MPU-6050
* ESP12F

![wiring](/wiring.png)

## Simple debug

On power up, the rgb led should blink white once, any subsequent color blinks indecate different errors:

* 2 red blinks: SPIFFS file system failed to start
* 1 green blink: Unable to connect to pre-configured wifi, this usually happens when you start cube for the fist time, then a hotspot will be provided named rcute-cube-xxxx, xxxx being the last 4 digit of cube's mac address.
* 1 blue blink: MPU6050 connection failed
* 2 blue blinks: MPU6050 initial memory load failed
* 3 blue blinks: MPU6050 DMP configuration updates failed

## License

This project is open sourced for educational purpose, Commercial usage is prohibited.

## Build from source with Arduino IDE

* install ["ESP8266 Sketch Data Upload" plugin](https://github.com/esp8266/arduino-esp8266fs-plugin)
* set [Tools] -> [Flash Size] -> [4MB(FS:2MB)]
* upload & upload sketch data

## Output build bin files

* set [Tools] -> [Flash Size] -> [4MB(FS:2MB)], then [Project] -> [Export bin files]
* make spiffs image
	1. download [mkspiffs](https://github.com/igrr/mkspiffs/releases), then
	2. `.\mkspiffs.exe -c .\data\ -b 8192 -s 1835008 .\spiffs.bin`

* or, output bin files from Arduino IDE, [link](https://www.kanda.com/blog/microcontrollers/avr-microcontrollers/find-arduino-hex-files-output-binaries/)

## Ref

* mpu6050 lib: https://github.com/jrowberg/i2cdevlib
* p5.js mpu6050 examples: https://github.com/gdsports/imu-wifi