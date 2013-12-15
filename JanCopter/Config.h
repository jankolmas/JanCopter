/*
Config.h

This file contains all constants, pin assignments and other preprocessor variables.

Contents:
  Debug
  PID gains and limits
  RX settings
  Sensor offsets
  ESC settings
  Pins


*/

//=======================================================================================
// Debug
//=======================================================================================
     
// Uncomment the next line to enable debug output
//#define DEBUG 

// Uncomment any of the following to select which variables to output. It is possible to do more at once.
//#define DEBUG_ANGLES
//#define DEBUG_ACC
//#define DEBUG_GYRO
//#define DEBUG_MOTOR
//#define DEBUG_PID
//#define PLOT_PID
//#define PLOT_YAW
//#define DEBUG_RX

//=======================================================================================
// PID gains and limits
//=======================================================================================

#define ROLL_PID_KP 0.12        // proportional gain
#define ROLL_PID_KI  0.23      // integral gain
#define ROLL_PID_KD  0.014          // derivative gain
#define ROLL_PID_MIN  -70.0     // minimum value of PID output
#define ROLL_PID_MAX  70.0      // maximum value of PID output

#define PITCH_PID_KP  0.11
#define PITCH_PID_KI  0.23
#define PITCH_PID_KD  0.012
#define PITCH_PID_MIN  -70.0
#define PITCH_PID_MAX  70.0

#define YAW_PID_KP  0.6
#define YAW_PID_KI  0.0
#define YAW_PID_KD  -0.01
#define YAW_PID_MIN  -35.0
#define YAW_PID_MAX  35.0

#define ROLL_KP 1.1
#define ROLL_KI 0.09
#define ROLL_KD 0.0 //-0.005
#define ROLL_MIN -100.0
#define ROLL_MAX 100.0

#define PITCH_KP 1.1 
#define PITCH_KI 0.09
#define PITCH_KD 0.0 //-0.005 
#define PITCH_MIN -100.0
#define PITCH_MAX 100.0

#define I_ANGLE 15  // Enable intergral controller when angle is smaller than this value
// Integral control is useful to eliminate a small constant offset but can cause instability when used at large angles


//=======================================================================================
// RX settings
//=======================================================================================

#define THROTTLE_RMIN  1000               // receiver minimum (use the DEBUG_RX to set these)
#define THROTTLE_RMAX  1950               // receiver maximum
#define THROTTLE_WMIN  MOTOR_IDLE_LEVEL   // map minimum motor output to idle level (not enough to lift off) once quad is alive
#define THROTTLE_WMAX  MOTOR_MAX_LEVEL    // map max receiver throttle to max motor output

#define ROLL_RMIN  1000 // receiver minimum
#define ROLL_RMAX  1950 // receiver maximum
#define ROLL_WMIN  -20  // minimum desired angle setpoint in degrees
#define ROLL_WMAX  20   // maximum desired angle setpoint in degrees

#define PITCH_RMIN  1100
#define PITCH_RMAX  1880
#define PITCH_WMIN  -20
#define PITCH_WMAX  20

#define YAW_RMIN  998
#define YAW_RMAX  1950
#define YAW_WMIN  -90
#define YAW_WMAX  90


#define DEAD_RMIN 1470 // define dead zone with no effect (so that random tiny stick oscillations when 
#define DEAD_RMAX 1520 // you're not touching the stick will be ignored)


//=======================================================================================
// Sensor offsets
//=======================================================================================

// You should tune these using trial and error or using the Calibration program.
// However, the MPU calibrates itself within about 20 s from startup, so I don't know if these are necessary.
#define ACC_X_OFFSET	-2293
#define ACC_Y_OFFSET	-1334
#define ACC_Z_OFFSET	1828

#define GYRO_X_OFFSET	45
#define GYRO_Y_OFFSET	0
#define GYRO_Z_OFFSET	-45


//=======================================================================================
// ESC settings
//=======================================================================================

//Motor PPM Levels
#define MOTOR_ZERO_LEVEL  0
#define MOTOR_ARM_LEVEL  50 
#define MOTOR_MAX_LEVEL  179
#define MOTOR_IDLE_LEVEL 100


//=======================================================================================
// Pins
//=======================================================================================

// RX pins
#define RX_PIN_ROLL  51     //PCINT2
#define RX_PIN_PITCH  50    //PCINT3
#define RX_PIN_YAW  52      //PCINT1 , all masked to PCINT0_vect. See RX.ino for more detail on interrupts
// Interrupt numbers
#define RX_PIN_THROTTLE  4  //ATMEGA INTERRUPT 4
#define RX_PIN_AUX1  5      //ATMEGA INTERRUPT 5

//Motor pins
#define MOTOR0  7
#define MOTOR1  6
#define MOTOR2  5
#define MOTOR3  4

// Misc pins
#define LED_PIN 37
