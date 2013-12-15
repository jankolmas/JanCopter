/*

Debugging routines. Select what variables you want to see in Config.h

*/

// DebugProcess is called if DEBUG is defined in Config.h
void debugProcess(){

// Angles  
#ifdef DEBUG_ANGLES
  Serial.print("yaw:");
  Serial.print((float)(ypr[0]*180/PI));
  Serial.print('\t');
  Serial.print("pitch:");
  Serial.print((float)(ypr[1]*180/PI));
  Serial.print('\t');
  Serial.print("roll:");
  Serial.print((float)(ypr[2]*180/PI));
  Serial.print('\t');
#endif

// Angular rates
#ifdef DEBUG_GYRO
  Serial.print("d_yaw:");
  Serial.print(((float)gyro[2]));
  Serial.print('\t');
  Serial.print("d_pitch:");
  Serial.print(((float)gyro[1]));
  Serial.print('\t');
  Serial.print("d_roll:");
  Serial.print(((float)gyro[0]));
  Serial.print('\t');
#endif

// Motor outputs
#ifdef DEBUG_MOTOR
  Serial.print("M0:");
  Serial.print((float)(m0_val));
  Serial.print('\t');
  Serial.print("M1:");
  Serial.print((float)(m1_val));
  Serial.print('\t');
  Serial.print("M2:");
  Serial.print((float)(m2_val));
  Serial.print('\t');
  Serial.print("M3:");
  Serial.print((float)(m3_val));
  Serial.print('\t');
#endif

// Accelerometer values. You need to enable direct accelerometer reading at the bottom of Sensor.ino
#ifdef DEBUG_ACC
  Serial.print("X:");
  Serial.print(acc[0]);
  Serial.print('\t');
  Serial.print("Y:");
  Serial.print(acc[1]);
  Serial.print('\t');
  Serial.print("Z:");
  Serial.print(acc[2]);
  Serial.print('\t');
#endif

// Receiver values
#ifdef DEBUG_RX
  Serial.print(F("Yaw:"));
  Serial.print(rxVal[0]);
  Serial.print('\t');
  Serial.print(F("Roll:"));
  Serial.print(rxVal[1]);
  Serial.print('\t');
  Serial.print(F("Pitch:"));
  Serial.print(rxVal[2]);
  Serial.print('\t');
  Serial.print(F("AUX1:"));
  Serial.print(rxVal[3]);
  Serial.print('\t'); 
  Serial.print(F("Throttle:"));
  Serial.print(rxVal[4]);
  Serial.print('\t');    
#endif

// Outputs of the PID loops
#ifdef DEBUG_PID
  Serial.print("PIDroll:");
  Serial.print((float)(PIDroll_val));
  Serial.print('\t');
  Serial.print("PIDyaw:");
  Serial.print((float)(PIDyaw_val));
  Serial.print('\t');
  Serial.print("PIDpitch:");
  Serial.print((float)(PIDpitch_val));
  Serial.print('\t');
#endif

// Values through the PID loops for plotting. Rename to pitch if you want to record pitch
#ifdef PLOT_PID
  Serial.print((float)(roll_input));
  Serial.print(", ");
  Serial.print((float)(setRoll));
  Serial.print(", ");
  Serial.print((float)(setRollRate));
  Serial.print(", ");
  Serial.print((float)(PIDroll_val));
#endif

// Values through the yaw PID loop for plotting.
#ifdef PLOT_YAW
  Serial.print((float)(yawRate_input));
  Serial.print(", ");
  Serial.print((float)(setYawRate));
  Serial.print(", ");
  Serial.print((float)(PIDyaw_val));
#endif

  Serial.print('\n');

}
