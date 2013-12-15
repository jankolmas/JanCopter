/*
RX.ino

Functions for reading radio commands from the receiver. JanCopter uses a 6 channel radio,
bare minimum is 4 channels.

RX channels:
CH1: roll
CH2: throttle
CH3: landing gear - same as channel 6 (for some reason)
CH4: yaw
CH5: aux - attached to a switch  
CH6: pitch - same as channel 3 (for some reason)
The channels can vary from radio to radio. Adjust acccordingly.

 
The rx arrays have the following items:
  0:yaw
  1:roll
  2:pitch
  3:aux1
  4:throttle
  
rxVal[] - length of receiver pulse in microseconds (should be 0-2000)
rxPrev[] - time at which the pin went high in microseconds
rxState[] - state of each pin in rxVal array in the current loop (0 or 1)



Interrupts:
There is no function in the code that calls for reading of the receiver values. This code uses
external interrupts - when a signal gets sent from the receiver to the arduino on an interrupt pin,
code execution is paused and a pre-assigned function is executed. Then normal code execution 
continues.
Roll, pitch and yaw channels are attached to PCINT pins, which are "masked" to interrupt 0. This means
that every time any pin in this mask changes state, an interrupt routine is called.
Throttle and aux are directly attached to interrupt pins defined in Config.h. Check the Arduino Mega
pinout for location of interrupt pins.
This system allows fast and accurate reading of RX values.

*/


// Code the pin numbers in the flash memory so that they don't take up space in the SRAM
PROGMEM const byte rxPins[3]={RX_PIN_YAW,RX_PIN_ROLL,RX_PIN_PITCH};

// Make input bytes volatile because they will change frequently
volatile byte rxState[3]={0,0,0};
volatile int rxPrev[5]={0,0,0,0,0};

// Initialize the receiver
void rxInit(){
  
  // For yaw, pitch and roll
  for(byte i=0;i<3;i++){
    // Define as inputs
    pinMode(pgm_read_byte(&rxPins[i]),INPUT); 
    // Enable internal pull up resistors
    digitalWrite(pgm_read_byte(&rxPins[i]),HIGH);
  }
  
  // Attach ISR function to interrupt number 0
  PCICR |= (1 << PCIE0); 
  // Mask the following PCINT pins to interrupt 0
  PCMSK0 |= (1 << PCINT1)|(1 << PCINT2)|(1 << PCINT3); 
  // Enable interrupts
  sei();
  
  // Attach throttle and aux to individual interrupts
  attachInterrupt(RX_PIN_AUX1,rxGoesHigh0,RISING);  // call rxGoesHigh0() if signal on RX_PIN_AUX1 has a rising edge
  attachInterrupt(RX_PIN_THROTTLE,rxGoesHigh1,RISING);
}

// Signal went high. Start the timer and watch for the signal to go low
void rxGoesHigh0(){
  attachInterrupt(RX_PIN_AUX1,rxGoesLow0,FALLING);
  rxPrev[3]=micros();
}

// Signal went low. Collect the time in microseconds (PPM value) and watch for rising edge again
void rxGoesLow0(){
  attachInterrupt(RX_PIN_AUX1,rxGoesHigh0,RISING);
  rxVal[3]=micros()-rxPrev[3];  
}

// Signal went high. Start the timer and watch for the signal to go low
void rxGoesHigh1(){
  attachInterrupt(RX_PIN_THROTTLE,rxGoesLow1,FALLING);
  rxPrev[4]=micros();
}

// Signal went low. Collect the time in microseconds (PPM value) and watch for rising edge again
void rxGoesLow1(){
  attachInterrupt(RX_PIN_THROTTLE,rxGoesHigh1,RISING);
  rxVal[4]=micros()-rxPrev[4]; 
}

// A function that gets called whenever a digital PCINT pin included in the mask 0 changes
ISR(PCINT0_vect) 
{ 
  // We don't know which pin changed. Go through all of them.
  for(byte i=0;i<3;i++){
    
    // Read the pin state
    byte rxtemp=digitalRead(pgm_read_byte(&rxPins[i]));
    
    // If pin was low and now is high, start the timer (rising edge)
    if((rxState[i] == 0) & (rxtemp==1)){
      rxPrev[i]=micros();
      rxState[i]=1;
    }
    
    // If pin was high and now is low, stop the timer and store the time in microseconds (falling edge)
    else if((rxState[i] == 1) & (rxtemp==0)){
      rxVal[i]=micros()-rxPrev[i];
      rxState[i]=0;
    }
  }
}
