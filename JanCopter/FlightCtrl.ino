/*
FlightCtrl.ino

This file contains mapping of the receiver values, stabilization, PID control and motor output.
The single function in this file, FlightControl() gets called from JanCopter.ino when the craft is 
alive and throttled up.

*/

void FlightControl(){

  // Map throttle from input values to output values
  throttle=map(rxVal[4],THROTTLE_RMIN,THROTTLE_RMAX,THROTTLE_WMIN,THROTTLE_WMAX); 

  // Map yaw left and right. Positive yaw is defined to the right 
  if(rxVal[0]<DEAD_RMIN){
    setYawRate=map(rxVal[0],YAW_RMIN,DEAD_RMIN,YAW_WMIN,0);  // negative yaw
  }
  else if(rxVal[0]>DEAD_RMAX){
    setYawRate=map(rxVal[0],DEAD_RMAX,YAW_RMAX,0,YAW_WMAX); // positive yaw
  }
  else{
   setYawRate = 0.0; // If rxVal is within a dead zone, do nothing. You don't want the quad jerking around from tiny stick oscillations.
  }
  
  // Map roll left and right. Positive roll is defined to the right.
  if(rxVal[1]<DEAD_RMIN){
    setRoll=map(rxVal[1],ROLL_RMIN,DEAD_RMIN,ROLL_WMIN,0);
  }
  else if(rxVal[1]>DEAD_RMAX){
    setRoll=map(rxVal[1],DEAD_RMAX,ROLL_RMAX,0,ROLL_WMAX);
  }
  else{
   setRoll = 0.0; 
  }
  
  // Map pitch front and back. Positive pitch is defined up. If you look at the IMU coordinate system the pitch is defined down instead of up, hence the inversion
  if(rxVal[2]<DEAD_RMIN){
    setPitch=map(rxVal[2],PITCH_RMIN,DEAD_RMIN,PITCH_WMIN,0);
  }
  else if(rxVal[2]>DEAD_RMAX){
    setPitch=map(rxVal[2],DEAD_RMAX,PITCH_RMAX,0,PITCH_WMAX);
  }
  else{
   setPitch = 0.0; 
  }

  // desired pitch and roll rate depend on the scaled magnitude of the error (angular distance from setpoint)
  // yaw is not present, because the yaw stick controls rate rather than angle. aka you don't want the quad to reorient back once you let go of the yaw stick. You do want this with pitch and roll
  pitch_p = pitch_input;
  roll_p = roll_input;
  
  // ypr is in radians, convert to degrees
  pitch_input = ((float) ypr[1])*180.0/PI;
  roll_input = ((float) ypr[2])*180.0/PI;
  
  // Running average just for the integral decisions to prevent accidental I resets
  pitch_p = (pitch_p + pitch_input)/2;
  roll_p = (roll_p + roll_input)/2;
    
  // Enable integral control for small angles, exclude the center to prevent oscillations  
  double diffRoll = roll_p - setRoll;
  if(diffRoll > I_ANGLE || diffRoll < -I_ANGLE) PID_roll_integral_off();
  // Logic for resetting the integral:
  // When the desired angle is large (~20 degrees), then 5% (eg, setRoll/20) is relatively big
  // and the integral should be reset when the angle comes within +/- 5% of the desired angle
  // in order to prevent integral accumulation.
  // However, for small desired angles (such as ~3 degrees), 5% is too small, so we set the reset
  // angle to be 0.5 degrees instead. That explains the max(setRoll/20,0.5) business I hope.
  else{   
    if(diffRoll > min(-setRoll/20,-0.5) && diffRoll < max(setRoll/20,0.5)) PID_roll_integral_off();
    else PIDroll.SetI(ROLL_KI);
  }
  double diffPitch = pitch_p - setPitch;
  if(diffPitch > I_ANGLE || diffPitch < -I_ANGLE) PID_pitch_integral_off();
  else{
    if(diffPitch > min(-setPitch/20,-0.5) && diffPitch < max(setPitch/20,0.5)) PID_pitch_integral_off();
    else PIDpitch.SetI(PITCH_KI);
  }
  
  // Run the first stage of PID controllers (angle)
  setPitchRate= (double)PIDpitch.Compute((float)setPitch-pitch_input);
  setRollRate= (double)PIDroll.Compute((float)setRoll-roll_input);

  // if you are already moving at a velocity demanded by the roll and pitch PID controllers, output nothing and inertia will do its job
  // if rotation is too slow, increase acceleration = give power to the motors that increase roll/pitch/yaw by increasing PIDroll_val etc.
  yawRate_input = -(float) gyro[2]; // there is a minus sign because the gyro values coming from the IMU are inverted.
  pitchRate_input = - (float)gyro[1]; // there is a minus sign because the gyro values coming from the IMU are inverted.
  rollRate_input = (float)gyro[0];
  
  // Run the second stage of PID controllers (angular rate)
  PIDroll_val= (int)PIDrollrate.Compute((float)setRollRate-rollRate_input);
  PIDpitch_val= (int)PIDpitchrate.Compute((float)setPitchRate-pitchRate_input);
  PIDyaw_val= (int)PIDyawrate.Compute((float)setYawRate-yawRate_input);
  
  // Calculate output value for each motor
  // yaw, pitch, roll are positive up and right. 
  m0_val=throttle+PIDroll_val+PIDpitch_val-PIDyaw_val; // front left
  m1_val=throttle-PIDroll_val+PIDpitch_val+PIDyaw_val; // front right
  m2_val=throttle-PIDroll_val-PIDpitch_val-PIDyaw_val; // back left
  m3_val=throttle+PIDroll_val-PIDpitch_val+PIDyaw_val; // back right

  // Write output to motors. Do not allow to drop below idle level.
  esc0.write(max(m0_val,MOTOR_IDLE_LEVEL));
  esc1.write(max(m1_val,MOTOR_IDLE_LEVEL));
  esc2.write(max(m2_val,MOTOR_IDLE_LEVEL));
  esc3.write(max(m3_val,MOTOR_IDLE_LEVEL));

}
