#include "robot-config.h"
//fns.h -- use this file to make all functions that the drive program is dependent on
void spinFlaps(){
  flaps.spin(fwd, 100);
}
void reverseFlaps(){
  flaps.spin(vex::reverse, 65);
}
void stopFlaps(){
  flaps.stop(hold);
}

void raiseLift(){
  lift.spin(fwd, 100);
}
void dropLift(){
  lift.spin(vex::reverse, 100);
}
void stopLift(){
  lift.stop(hold);
}