/*
PID.ino

Initialization of all PID controllers and switching of integral contorl

*/

// Initialize all 5 loops, load parameters from Config.h into the controllers.
void PID_init(){
  PIDrollrate.ChangeParameters(ROLL_PID_KP,ROLL_PID_KI,ROLL_PID_KD,ROLL_PID_MIN,ROLL_PID_MAX);
  PIDpitchrate.ChangeParameters(PITCH_PID_KP,PITCH_PID_KI,PITCH_PID_KD,PITCH_PID_MIN,PITCH_PID_MAX);
  PIDyawrate.ChangeParameters(YAW_PID_KP,YAW_PID_KI,YAW_PID_KD,YAW_PID_MIN,YAW_PID_MAX);
  PIDroll.ChangeParameters(ROLL_KP,ROLL_KI,ROLL_KD,ROLL_MIN,ROLL_MAX);
  PIDpitch.ChangeParameters(PITCH_KP,PITCH_KI,PITCH_KD,PITCH_MIN,PITCH_MAX);
}

// Turn off integral for pitch (mainly for large angles)
void PID_pitch_integral_off(){
      PIDpitch.resetI();
      PIDpitch.SetI(0.0);
      PIDpitchrate.resetI();
      PIDpitchrate.SetI(0.0);
}

// Turn off integral for roll (mainly for large angles)
void PID_roll_integral_off(){
      PIDroll.resetI();
      PIDroll.SetI(0.0);
      PIDrollrate.resetI();
      PIDrollrate.SetI(0.0);
}
