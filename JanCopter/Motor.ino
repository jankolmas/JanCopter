/*
Motor.ino

This file contains functions for controlling the motors through ESCs. The code that writes flight values to the motors 
is at the end of FlightCtrl.ino

*/

// Attach motors to motor pins and set them to zero level
void motorInit(){
  esc0.attach(MOTOR0);
  esc1.attach(MOTOR1);
  esc2.attach(MOTOR2);
  esc3.attach(MOTOR3);
  esc0.write(MOTOR_ZERO_LEVEL);
  esc1.write(MOTOR_ZERO_LEVEL);
  esc2.write(MOTOR_ZERO_LEVEL);
  esc3.write(MOTOR_ZERO_LEVEL);
}

// Arming the motors will tell the ESCs to release the motor brake
void motorArm(){
  esc0.write(MOTOR_ARM_LEVEL);
  esc1.write(MOTOR_ARM_LEVEL);
  esc2.write(MOTOR_ARM_LEVEL);
  esc3.write(MOTOR_ARM_LEVEL);
  
  // Reset the integral control that has accumulated
  PID_pitch_integral_off();
  PID_roll_integral_off();
}

// Spin the motors at a low setting that is not enough to lift the craft
void motorIdle(){
  esc0.write(MOTOR_IDLE_LEVEL);
  esc1.write(MOTOR_IDLE_LEVEL);
  esc2.write(MOTOR_IDLE_LEVEL);
  esc3.write(MOTOR_IDLE_LEVEL);

  // Reset the integral control that has accumulated
  PID_pitch_integral_off();
  PID_roll_integral_off();
}


