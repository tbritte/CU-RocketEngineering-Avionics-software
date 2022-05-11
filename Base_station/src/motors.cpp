#include "motors.h"

int motor_home(AccelStepper stepper,int home_pin){
  long init_home = -1;
  // waits for driver to wake up
  delay(5);
  // set max speed and acceleration for homing
  stepper.setMaxSpeed(100.0);
  stepper.setAcceleration(100.0);

  // starts the homing
  //moves until home_pin is logic LOW
  while(digitalRead(home_pin)){
    stepper.moveTo(init_home);
    init_home--;
    stepper.run();
    delay(5);
  }
  //sets current position to zero 
  stepper.setCurrentPosition(0);
  // resets the speed and acceleration
  stepper.setMaxSpeed(100.0);
  stepper.setAcceleration(100.0);
  init_home = 1;
  //moves stepper until home_pin is HIGH
  while(!digitalRead(home_pin)){
    stepper.moveTo(init_home);
    init_home++;
    stepper.run();
    delay(5);
  }
  return 0;
}
int motor_move(AccelStepper stepper, int move_distance){
  
}