/*
  JanCopter - Quadcopter firmware for Arduino Mega
  
  Created by Jan Kolmas and Brandon Araki, 15 december 2013
  Based on BlueCopter code by Basel Al-Rudainy, 6 april 2013.
  MPU code by Jeff Rowberg <jeff@rowberg.net>, 20 may 2013
  
  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.
  
  This library was written for the Arudino Mega 2560. Compatibility with other boards is not guaranteed.
  
Files in the folder:
JanCopter.ino:         Global variables, setup() and loop() functions
Config.h               Preprocessor variables, constants and pin assignments
MPU6050_DMP.h          Files to get angles and rates from the IMU
MPU6050.cpp            |
MPU6050.h              |
helper_3dmath.h        Helper functions for dealing with Quaternions and Vectors
FlightCtrl.ino:        Maps data from RX, combines with data from IMU, runs PID and sends result to motors
Motor.ino:             Motor initialization and arming procedures
PID.ino:               PID initialization and functions
RX.ino:                Interrupt and timing functions for reading RX values
Sensor.ino:            IMU initialization and DMP angle/rate reading
debug.ino:             The debug routine, if enabled in Config.h, sends select variables to the computer through Serial

Files in the Library folder:
I2Cdev.cpp
I2Cdev.h               Library for I2C communication (for the IMU)
PIDCont.cpp
PIDCont.h              Library for PID feedback loop
FastServo.cpp
FastServo.h            Library to update the ESCs at 100Hz (50Hz is default with Servo.h)
Import these libraries to your computer using Edit->Import Library->Add Library...


Flight configuration:
JanCopter is set up for the X configuration. Motor numbers are shown in this top view:

             front
           0       1
             \   /
   left        X      right   
             /   \
           3       2
              back

Roll is the angle around the front-back axis (X). Positive roll is defined to the right, clockwise if looking from the back.              
Pitch is the angle around the left-right axis (Y). Positive pitch is defined with the front pointing up, clockwise if looking from the left.
Yaw is the angle around the vertical axis (Z). Positive yaw is defined to the right, clockwise if looking from the top.

*/

//=======================================================================================
// INITIALIZATION
//=======================================================================================


// Configuration, limit and tuning constants
#include "Config.h"
// I2C Communication libraries for talking to the IMU
#include <Wire.h>
#include <I2Cdev.h>
// Library for PID control
#include <PIDCont.h>
// Library for talking to the ESCs
#include <FastServo.h>
// Library for IMU Digital Motion Processing (DMP)
#include "MPU6050_DMP.h"
// Library to handle the RX interrupts
#include <avr/pgmspace.h>
// All the other .ino files in this folder are included automatically

// Define PID controller for angles and angular rates. Yaw is unknown because we are using a 6 DOF IMU.
PIDCont PIDroll,PIDpitch;
PIDCont PIDrollrate,PIDpitchrate,PIDyawrate;

// This is the Inertial Measurement Unit (IMU)
MPU6050 mpu;

// Electronic Speed Controllers (ESCs)
FastServo esc0;
FastServo esc1;
FastServo esc2;
FastServo esc3;

// Global sensor variables
float ypr[3] = {0.0,0.0,0.0};    // [yaw, pitch, roll] angles in radians
int gyro[3] = {0,0,0};    // [roll, pitch, yaw] angular rates in deg/s , NOTE THIS IS DIFFERENT FROM ABOVE
int acc[3] = {0,0,0}; // [x, y, z] acceleration for debugging purposes

// Outputs to the motors, ranging from 0 to 179
int m0_val = 0;
int m1_val = 0;
int m2_val = 0;
int m3_val = 0;

// Desired setpoints for PID controllers.
int setRoll = 0; // Desired roll, given by RX roll stick
int setPitch = 0; //Desired pitch, given by RX pitch stick
int setRollRate = 0; //Desired roll rate, calculated by the PIDroll controller
int setPitchRate = 0; //Desired pitch rate, calculated by the PIDpitch controller
int setYawRate = 0; //Desired yaw rate, given by RX yaw stick

// Result of PID control, these four sum together to give motor output values
int PIDyaw_val = 0;
int PIDroll_val = 0;
int PIDpitch_val = 0;
int throttle=MOTOR_ZERO_LEVEL; // Initialize throttle to mininum

//running averages - helps to know when to reset I
double roll_p = 0;
double pitch_p = 0;

// Receiver values, updated by external interrupts 
volatile int rxVal[5]={1500,1500,1500,0,1000}; // yaw,pitch,roll,aux1,throttle. Defined as volatile because they change rapidly

// Input variables for PID (you give these gyro and DMP outputs)
double roll_input = 0;
double pitch_input = 0;
double pitchRate_input = 0;
double rollRate_input = 0;
double yawRate_input = 0;

// LED
int LEDstate = 0;

// Flag set by the MPU external interrupt signalling that DMP data are available
volatile bool mpuInterrupt = false; 

// State variable. When false, motors will not move.
bool alive = false;


//=======================================================================================
// SETUP
//=======================================================================================

void setup() {

  // Begin serial at highest baud
  Serial.begin(115200);
  
  // Initialize the LED
  pinMode(LED_PIN,OUTPUT);
  digitalWrite(LED_PIN,0);
  
  // Initialize the IMU. Initialize this first so that FIFO buffer doesn't overflow
  MPUinit();
  
  // Initialize the ESCs
  motorInit();
  
  // Initialize the radio
  rxInit();
  
  // Reset the PID controllers
  PID_init(); 
  
  // Arm the motors, ready gor flight
  motorArm(); 
   
}

//=======================================================================================
// LOOP
//=======================================================================================

void loop() {
  
  // If new data are available, read and process them. This should be happening at a rate set by DMP
    if(mpuInterrupt==true){
      
      // Read new sensor data
      updateSensorVal();
      
      // Resolve three states: off, idle and alive
      if(alive==true){
       
        // If throttle stick is at the bottom, idle the motors. Once the stick moves a notch up, engage the stabilization and flight control
        if(rxVal[4]>THROTTLE_RMIN) FlightControl();
        else motorIdle();
        
        // Flick the throttle/yaw stick to the bottom left corner to kill motors
        if(rxVal[4]<=THROTTLE_RMIN && rxVal[0]<YAW_RMIN+5) alive = false;     
     }
     else{ 
       motorArm();
       
       // Flick the throttle/yaw stick to the bottom right corner to start motors
       if(rxVal[4]<=THROTTLE_RMIN && rxVal[0]>YAW_RMAX-5) alive = true; 
     }
  }
                    
  // Print out global variables defined in Config.h. Caution: enabling this slows down the program and can change its behavior
  #ifdef DEBUG
    debugProcess();
  #endif 

}

