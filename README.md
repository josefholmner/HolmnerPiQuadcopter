# HolmnerPiQuadcopter
Control system for quadcopters implemented on a Raspberry Pi.
Uses:
 * Raspberry Pi 3 Model B
 * Accelerometer / Gyro: MPU6050
 * Radio transmitter: Hobby King 2.4Ghz 6Ch Tx & Rx V2 (Mode 2)
 * Motors: 4x FC 28-22 (1200kv)
 * ESC: 4x Hobbyking SS Series 15-18A ESC
 * Barometer BMP280 (optional)

## Build
gcc -Wall -pthread -o "Quadcopter_Holmner_run" *.c -lwiringPi -lpigpio -lm

## Current status:
This project was discontinued before liftoff, but all necessary parts
are there, reading and filtering of sensor data, reading radio
controller stick inputs, performing PID control and controlling the
ESCs. Some manual tweaking would likely be neccesary before taking any
DIY quadcopter in the air. The barometer data is currently not used by
the control loop, but the data is read and available for anyone wanting
to use it.


 ## HOW TO USE
 1) Download and install the wiringPi library
 2) Download and install the pigpio library
 3) Make sure that I2C is activated on your Raspberry Pi (just google how)
 4) To increase the I2C baud rate to 400kHz (recommended), add dtparam=i2c1_baudrate=400000 to /boot/config.txt
 5) Go through the SETTINGS area in the main() function to setup GPIO pins etc.
 6) Build settings: gcc -Wall -pthread -o "Quadcopter_Holmner_run" *.c -lwiringPi -lpigpio -lm

## NOTE
This software was developed on the Raspberry Pi 3 Model B.
The linux distribution used was: Raspbian GNU/LINUX 8
The sofware used for writing and compiling the code was Geany 1.24.1

## HARDWARE
The following hardware was used for this project (using other hardware
may demand changing the code, especially for communication interfaces).

 * Raspberry Pi 3 Model B
 * Accelerometer / Gyro: MPU6050
 * Radio transmitter: Hobby King 2.4Ghz 6Ch Tx & Rx V2 (Mode 2)
 * Motors: 4x FC 28-22 (1200kv)
 * ESC: 4x Hobbyking SS Series 15-18A ESC
 * Barometer BMP280 (optional)

 ## CREDITS
 Code written by Josef Holmner 2018. It is free to use and modify. Happy flying :)

