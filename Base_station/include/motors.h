
// libs
#include <AccelStepper.h>
#include <MultiStepper.h>
#include <Arduino.h>

 #define STEP_1 1
 #define DIR_1 2
 #define STEP_2 3
 #define DIR_2 4
 #define HOME_1 5
 #define HOME_2 6
 
 // function macros
 int motor_home(AccelStepper stepper,int home_pin);
 int motor_move(AccelStepper stepper, int move_distance);

 // motor delcarations
 AccelStepper vert_stepper(AccelStepper::DRIVER,STEP_1,DIR_1);
 AccelStepper horz_stepper(AccelStepper::DRIVER,STEP_2,DIR_2);

 