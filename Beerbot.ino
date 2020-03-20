#include "Motor.hpp"

//Creating objects
volatile uint8_t counter = 0;
volatile uint8_t timerAlert = LOW;

MOTOR motL;



void setup() {
  OCR0A = 0xAF;           // set up interrupt for 1 ms each time
  TIMSK0 |= _BV(OCIE0A);
  
  motL.motorSETUP(STEP_PIN_L, DIR_PIN_L);
}

SIGNAL(TIMER0_COMPA_vect){
  counter++;
  if (counter == 4)
  {
    timerAlert = HIGH;   // only fire every 4th time
    counter = 0;
  }
}

void loop() {

  if (timerAlert) {
    //Calculate thrust

    timerAlert = LOW;
    
  }
  
  motL.motorDrive(thrust);
}
