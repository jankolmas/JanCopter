# JanCopter

JanCopter - Quadcopter firmware for Arduino Mega 2560 (Compatibility with other boards is not guaranteed)
   
## Files in the folder:
- JanCopter.ino:         Global variables, setup() and loop() functions
- Config.h               Preprocessor variables, constants and pin assignments
- MPU6050_DMP.h          Files to get angles and rates from the IMU
- MPU6050.cpp            
- MPU6050.h              
- helper_3dmath.h        Helper functions for dealing with Quaternions and Vectors
- FlightCtrl.ino:        Maps data from RX, combines with data from IMU, runs PID and sends result to motors
- Motor.ino:             Motor initialization and arming procedures
- PID.ino:               PID initialization and functions
- RX.ino:                Interrupt and timing functions for reading RX values
- Sensor.ino:            IMU initialization and DMP angle/rate reading
- debug.ino:             The debug routine, if enabled in Config.h, sends select variables to the computer through Serial

## Files in the Library folder:
Import these libraries to your computer using Edit->Import Library->Add Library...

- I2Cdev.cpp
- I2Cdev.h               Library for I2C communication (for the IMU)
- PIDCont.cpp
- PIDCont.h              Library for PID feedback loop
- FastServo.cpp
- FastServo.h            Library to update the ESCs at 100Hz (50Hz is default with Servo.h)

## Flight configuration:
JanCopter is set up for the X configuration. Motor numbers are shown in this top view:

 ```            front
           0       1
             \   /
   left        X      right   
             /   \
           3       2
              back
```

Roll is the angle around the front-back axis (X). Positive roll is defined to the right, clockwise if looking from the back.              
Pitch is the angle around the left-right axis (Y). Positive pitch is defined with the front pointing up, clockwise if looking from the left.
Yaw is the angle around the vertical axis (Z). Positive yaw is defined to the right, clockwise if looking from the top.

## Authors
Created by Jan Kolmas and Brandon Araki, 15 december 2013

Based on [BlueCopter](https://github.com/baselsw/BlueCopter) code by Basel Al-Rudainy, 6 april 2013.

MPU code by Jeff Rowberg <jeff@rowberg.net>, 20 may 2013

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.
This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.
