/*
Sensor.ino

This file contains functions for reading angles and angular rates from the IMU.

*/

// This IMU DMP code is based on the following:
// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class using DMP (MotionApps v2.0)
// 6/21/2012 by Jeff Rowberg <jeff@rowberg.net>

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container


/*

Units:
Accelerometer sensitivity: 8192 LSB/g (where g is 9.8m/s^2)
The units of ypr[] coming from dmp are radians, because they come from a atan2 function
Units of gyro[] are assumed to be in deg/s. 

*/



// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

// This function gets called when the MPU triggers and external interrupt
void dmpDataReady() {
  // Set a flag for the loop() program that new data is available
   mpuInterrupt = true;
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void MPUinit() {
  
    Wire.begin();
    TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    
    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // Offsets are not in the same units as accelerometer and gyro values (if your gyro is showing 25 and should be
    // showing 0 then an offset of -25 will not work). Use trial and error or the Calibration program to get 
    //approximate values. The IMU will calibrate itself in about 20s from starting up, so make sure the quadcopter is level.
    mpu.setXGyroOffset(GYRO_X_OFFSET);
    mpu.setYGyroOffset(GYRO_Y_OFFSET);
    mpu.setZGyroOffset(GYRO_Z_OFFSET);
    mpu.setZAccelOffset(ACC_Z_OFFSET); 
    mpu.setXAccelOffset(ACC_X_OFFSET);
    mpu.setYAccelOffset(ACC_Y_OFFSET);
    
    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection   
       attachInterrupt(0, dmpDataReady, RISING);
       
        //mpuIntStatus = mpu.getIntStatus();
        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready"));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = 42;//mpu.dmpGetFIFOPacketSize();
        
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
}

// Update the global angle and rate variables. This gets called by the loop() if the MPU
// set a flag via the interrupt function dmpDataReady()
void updateSensorVal() {
  
  // Blink the LED to indicate that code is running and IMU is read properly
  if(LEDstate == 1) LEDstate = 0;
  else LEDstate = 1;  
  digitalWrite(LED_PIN,LEDstate);

  // reset interrupt flag
  mpuInterrupt = false;
     
    // Get current status
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    // this happens from time to time but is not a big deal
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        // Yaw, Pitch, Roll angles in radian
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        // Roll rate, pitch rate, yaw rate are in degrees/s. Note the order is different from above.
        mpu.dmpGetGyro(gyro,fifoBuffer);
        
        // Use the following for debug purposes
        //mpu.dmpGetAccel(acc,fifoBuffer);

    }
    
    
}
