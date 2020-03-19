#include "Motor.hpp"

//Creating objects
MOTOR motL;


void setup() {
  motL.motorSETUP(STEP_PIN_L, DIR_PIN_L);
}

void loop() {
  motL.motorDrive();
  delay(1000);

}
