/******************************
TODO:
  - Lift indices
  
*******************************/
#include "Updaters.h"
using namespace ClassFns;
using namespace vex;
competition Competition;
bool useVisionAlign = true;
bool isPressing(controller::button& btn){
  if(btn.pressing()){
    while(btn.pressing()){
      s(10);
    }
    return true;
  }
  
  return false;
}
bool isPressing(controller::axis& joystick, int mult){
  if(abs(joystick.value()) > 50 && joystick.value() * mult > 1){
    while(abs(joystick.value()) > 50 && joystick.value() * mult > 1){
      s(10);
    }
    //cout << "  ";
    return true;
  }
  
  return false;
}

void backwards(int t){
  Left.spin(vex::reverse, 60, pct);
  Right.spin(vex::reverse, 60, pct);
  s(t);
  Left.stop();
  Right.stop();
}
void backwardsSlow(int t, int sp = 40){
  Left.spin(vex::reverse, sp, pct);
  Right.spin(vex::reverse, sp, pct);
  s(t);
  Left.stop();
  Right.stop();
}
void fwds(int t){
  Left.spin(fwd, 60, pct);
  Right.spin(fwd, 60, pct);
  s(t);
  Left.stop(hold);
  Right.stop(hold);
  
}
void fwdsSlow(int t, double sp = 30){
  Left.spin(fwd, sp, pct);
  Right.spin(fwd, sp, pct);
  s(t);
  Left.stop(hold);
  Right.stop(hold);
  
}
void turnRight(int t){
  Left.spin(fwd, 60, pct);
  Right.spin(vex::reverse, 60, pct);
  s(t);
  Left.stop(hold);
  Right.stop(hold);
}
void turnLeft(int t){
  Left.spin(vex::reverse, 60, pct);
  Right.spin(fwd, 60, pct);
  s(t);
  Left.stop(hold);
  Right.stop(hold);
}
//Autonomous Stuff {

//Map order skillsOrSide - 3, centerGoals - 4, autonGoals - 2
void preventUntilGoal(){
  wc.pathEditor = [](BasicWheelController::PurePursuitController& l){
    if(l.isFullDone()){
      wc.pathEditor = [](PursuitController&){};
    }
    else {
      if(!frontCounter.pressing()){
        l.killBreak();
      }
      else {
        l.unKillBreak();
      }
    }
  };
}
void awp(){
}
void leftSide1GoalRush(){
  
}
void rightSide1GoalRush(){
  

}
void leftSide2GoalRush(){
  
}

void rightSideGoal(){
  
}
void addPids(){
  cout << "Turn P" << endl;
  while(1){
    if(isPressing(Greg.ButtonA)){
      wc.addTurnPid(0, 0, 0);
      break;
    }
    if(isPressing(Greg.ButtonB)){
      wc.addTurnPid(0.05, 0, 0.0);
      break;
    }
  }
  cout << "Turn D" << endl;
  while(1){
    if(isPressing(Greg.ButtonA)){
      break;
    }
    if(isPressing(Greg.ButtonB)){
      wc.addTurnPid(0.0, 0, 0.05);
      break;
    }
  }
  // cout << "Slave D" << endl;
  // while(1){
  //   if(isPressing(Greg.ButtonA)){
  //     break;
  //   }
  //   if(isPressing(Greg.ButtonB)){
  //     wc.addSlavePid(0.0, 0, 0.05);
  //     break;
  //   }
  // }
  cout << "Done" << endl;
}
void execute(){
  while(1){
    
  }
}
void skills(){}
void tentativeSkills(){
  //Lift Target: 0.74, 0, 0.55
  //Back Target: 0.74, 0, 0.35
  //Double Target: 0.54, 0, 0.2
  //Start: 0.74, 0, 0.2
  thread t = thread([](){
    s(15000);
    Greg.rumble("......");

    s(45000);
    Greg.rumble(".........");
  });
  //LAG Red
  // lowerBackLiftByOne();
  // lowerBackLiftByOne();
  // lowerBackLiftByOneWait();
  fwdsSlow(300, 20);
  clipGoal();
  // raiseBackLiftByOne();
  // raiseBackLiftByOneWait();
  //RAG Red
  //wc.addSlavePid(0.8, 0, 0.1);
  wc.addTurnPid(0, 0, 0.07);
  wc.addSlavePid(0, 0, 0.15);
  wc.driveTo(36.79, -40.09);
  
  wc.driveTo(36.79, 47.56);
  
  wc.popTopSlavePid();
  wc.popTopTurnPid();
  // lowerBackLiftByOne();
  alignGoalFront(RED2);
  wc.addTurnPid(-0.2, 0, 0);
  clipLiftGoal();
  s(300);
  raiseLiftByOne();
  //Platform RAG Red
  wc.backwardsFollow({PVector(33.12, 31.53)});
  wc.driveTo(27.13, 22.05);
  wc.speedLimit = 50;
  wc.driveTo(-24.15, 18.89);
  wc.speedLimit = 100;
  PVector LAGPos = wc.botPos();
  wc.popTopTurnPid();
  // lowerBackLiftByOneWait();
  unclipGoal();
  
  // raiseBackLiftByOne();
  // raiseBackLiftByOne();
  s(300);
  raiseLiftByOne();
  raiseLiftByOneWait();
  wc.driveTo(-46.19, -0.62);
  lowerLiftByOne();
  lowerLiftByOne();
  s(1000);
  unclipLiftGoal();
  raiseLiftByOne();
  raiseLiftByOne();
  s(300);
  //To LAG Red
  wc.afterTurn = [](){lowerLiftByOne();lowerLiftByOne();lowerLiftByOneWait();};
  wc.speedLimit = 40;
  wc.driveTo(LAGPos);
  wc.speedLimit = 100;
  alignGoalFront(RED2);
  clipLiftGoal();
  s(300);
  //Platform LAG
  raiseLiftByOne();
  raiseLiftByOne();
  wc.driveTo(-44.19, 5.47);
  unclipLiftGoal();
  s(300);
  // //To RNG
  // lowerBackLiftByOne();
  // lowerBackLiftByOne();
  // thread ok = thread([](){
  //   s(3000);
  //   lowerLiftByOne();
  //   lowerLiftByOne();
  // });
  
  // wc.backwardsFollow({PVector(-8.35, 32.31)});
  // alignGoalBack(NEUTRAL);
  // clipGoal();
  // s(300);

  
  //To RNG
  wc.afterTurn = [](){lowerLiftByOne();lowerLiftByOne();lowerLiftByOneWait();};
  wc.speedLimit = 40;
  wc.driveTo(-20.47, 30.78);
  wc.speedLimit = 100;
  alignGoalFront(YELLOW2);
  clipLiftGoal();
  s(300);
  //wc.estimateStartPos(PVector(-7.800346620450597, 12.403466204506064), 153.85788561525132)
  //Platform RNG
  raiseLiftByOne();
  raiseLiftByOne();
  raiseLiftByOne();
  wc.driveTo(-44.58, 3.8);
  
  unclipLiftGoal();
  wc.afterTurn = [](){lowerLiftByOne();lowerLiftByOne();lowerLiftByOneWait();};
  wc.driveTo(-24, 0);
  alignGoalFront(YELLOW2);
  clipLiftGoal();
  raiseLiftByOne();
  raiseLiftByOne();
  raiseLiftByOne();
  wc.driveTo(80, 0);
  lowerLiftByOne();
  lowerLiftByOne();
  s(500);
  unclipLiftGoal();


  // s(300);
  // raiseBackLiftByOne();
  // raiseBackLiftByOneWait();
  // wc.driveTo(-38.58, 6.8);
  // lowerLiftByOne();
  // s(1000);
  // unclipLiftGoal();
  // raiseLiftByOne();
  // wc.afterTurn = [](){lowerLiftByOne(); lowerLiftByOne();};
  
  // //Platform RNG
  // wc.driveTo(-33.2081, 3.11132);
  // alignGoalFront(YELLOW2);
  // clipLiftGoal();
  // s(300);
  // raiseLiftByOne();
  // raiseLiftByOne();
  // wc.driveTo(-36.30, 4.36);
  // unclipLiftGoal();

  // wc.afterTurn = [](){lowerLiftByOne();lowerLiftByOneWait();};
  // wc.driveTo(-9.31, -25.42);
  // alignGoalFront(YELLOW2);
  // clipLiftGoal();
  // s(300);
  // raiseLiftByOne();
  // wc.followPath({PVector(50.35, -49.83)});

  // raiseLiftByOneWait();
  // wc.driveTo(60, -41.81);
  // wc.driveTo(60, -36.1);
  // lowerLiftByOne();
  // lowerLiftByOneWait();
  // wc.driveTo(60, -9.14);
  // double last = tiltAngle;
  // double speed = 10;
  // while(1){
  //   // d/dx > 0
  //   if(tiltAngle - last > 0.01){
  //     backwardsSlow(10, 100);
  //     speed = 10;
  //   }

  //   //d/dx < 0
  //   else if(tiltAngle - last < -0.01){
  //     fwdsSlow(10, 100);
  //     speed = 10;
  //   }
  //   else if(tiltAngle > 5){
  //     speed -= 0.01;
  //     backwardsSlow(10, speed);
  //     if(speed < -10){
  //       speed = 5;
  //     }
  //   }
  //   else if(tiltAngle < -5){
  //     speed -= 0.01;
  //     fwdsSlow(10, speed);
  //     if(speed < -10){
  //       speed = 5;
  //     }
  //   }
  //   else {
  //     wc.hardBrake();
  //     s(10);
  //   }
  //   last = tiltAngle;
  // }


}
void autonInit(){
  //wc.followPath({PVector(0.0, -48.0), PVector(0.0, 0.0)});
  leftSide1GoalRush();


}
void autonomous(){
  cout << "Yeet" << endl;
  unclipGoal();
  unclipLiftGoal();
  conveyer.autonCtrl = true;
  conveyer.autonOut = 0;
  conveyer.countIn = 3;
  liftCtrllr.enable();
  cout << "Yeet" << endl;
  autonInit();
  // conveyer.autonCtrl = false;

  // waitForLiftFinish();
  // liftCtrllr.disable();
  s(15000);
}

//}
#define sensitivity 12

//Drivercontrol + automation {
class ButtonLatch {
  bool isPressing = false;
  int state = 0;
  int stateLim;
  controller::button& b;
public:
  ButtonLatch(controller::button& b, int stateLim = 2) : stateLim(stateLim), b(b){

  }
  ButtonLatch() = delete;
  bool pressing(){
    if(b.pressing() && !isPressing){
      state++;
      if(state == stateLim + 1){
        state = 1;
      }
      isPressing = true;
      return true;
    }
    else if(!b.pressing()){
      isPressing = false;
    }
    return false;
  }
  bool getState(){
    return state;
  }
};
void drivercontrol (){
  ButtonLatch R1Latch = ButtonLatch(Greg.ButtonR1);
  ButtonLatch R2Latch = ButtonLatch(Greg.ButtonR2);
  ButtonLatch BLatch = ButtonLatch(Greg.ButtonB);
  static bool driveReversed = true;
  //int wheelsMove = 0;
  liftCtrllr.disable();
  // backLiftCtrllr.disable();
  conveyer.noOut = goalHolder.value();
  KillThread::killAll();
  while(1){
    //Drive control, uses quotient/square for smoothness
    double Y1 = abs(Greg.Axis2.value()) > sensitivity ? Greg.Axis2.value() : 0;
    double Y2 = abs(Greg.Axis3.value()) > sensitivity ? Greg.Axis3.value() : 0;
    // Y1 /= 10;
    // Y1 *= Y1;
    // Y2 /= 10;
    // Y2 *= Y2;
    // Y1 *= Greg.Axis2.value() != 0 ? Greg.Axis2.value() / abs(Greg.Axis2.value()) : 1;
    // Y2 *= Greg.Axis3.value() != 0 ? Greg.Axis3.value() / abs(Greg.Axis3.value()) : 1;
    
    double s1 = Y2;
    double s2 = Y1;
    if(driveReversed){
      FL.spin(vex::reverse, s2, pct);
      ML.spin(vex::reverse, s2, pct);
      BL.spin(vex::reverse, s2, pct);
      FR.spin(vex::reverse, s1, pct);
      MR.spin(vex::reverse, s1, pct);
      BR.spin(vex::reverse, s1, pct);
    }
    else {
      FL.spin(fwd, s1, pct);
      ML.spin(fwd, s1, pct);
      BL.spin(fwd, s1, pct);
      FR.spin(fwd, s2, pct);
      MR.spin(fwd, s2, pct);
      BR.spin(fwd, s2, pct);
    }

    if(BLatch.pressing()){
      driveReversed = !driveReversed;
    }
    if(R1Latch.pressing()){
      conveyer.noOut = !goalHolder.value();
      liftGoalHolder.set(!liftGoalHolder.value());
      
    }
    if(R2Latch.pressing()){
      goalHolder.set(!goalHolder.value());
    }
    if(Greg.ButtonL2.pressing()){
      //Move the lift to the next index
      // liftCtrllr.addIndex();
      liftMot.spin(fwd);
    } else if(Greg.ButtonL1.pressing()){
      //Move the lift to the previous index
      // liftCtrllr.subIndex();
      liftMot.spin(vex::reverse);
    } else {
      //Allow the lift index to be changed
      // liftCtrllr.freePress();
      liftMot.stop(hold);
    }
    // if(Greg.ButtonA.pressing()){
    //   backLiftCtrllr.enable();
    //   for(int i = 0; i < 4; i++){
    //     lowerBackLiftByOne();
    //   }
    // }
    // else if(Greg.ButtonX.pressing()){
    //   backLiftCtrllr.enable();
    //   while(backLiftCtrllr.getIndex() != 2){
    //     raiseBackLiftByOne();
    //   }
    // }
    // else if(backLiftCtrllr.disabled()){
    //   flapConveyer.stop(hold);
    // }

    // //Manual conveyer override
    // if(Greg.ButtonUp.pressing()){
    //   conveyer.controller = true;
    //   spinFlaps();
    // }
    // else if(Greg.ButtonDown.pressing()){
    //   conveyer.controller = true;
    //   reverseFlaps();
    // }
    // else if(Greg.ButtonLeft.pressing()){
    //   conveyer.controller = true;
    //   stopFlaps();
    // }
    // else {
    //   //Return conveyer to program control
    //   stopFlaps();
    // }
    // cout << GPS.xPosition(inches) << ", " << GPS.yPosition(inches) << ", " << GPS.heading() << endl;
    s(100);
    
  }
}
/*void conveyerControl(){
  bool colorActive = false;
  // bool lineActive = false;
  s(1000);
  long int startTime = Brain.Timer.system();
  //conveyer.disabled = true;
  for(auto i : vector<int>(100)){
    topRingCounter.firstHit();
    i = 1;
  }
  while(1){
    if(conveyer.countIn < 0){
      conveyer.countIn = 0;
    }
    //cout << ringDetector.hue() << endl;
    //If there is a ring in the bottom, increment the ring count, 
    //  but don't do it more than once
    if((ringDetector.hue() > 100 && ringDetector.hue() < 370)){
      //cout << "Ring" << endl;
      if(!colorActive){
        conveyer.countIn++;
      }
      colorActive = true;
    }
    else {
      colorActive = false;
    }
    //If there is a ring leaving, enable the conveyer for a short time to 
    //  allow it to leave
    if(topRingCounter.firstHit()){
      //cout << "YEET" << endl;
      conveyer.autonOut--;
      conveyer.countIn--;
      if(conveyer.noOut){
        conveyer.disabled = true;
      }
      else {
        conveyer.enabledFor = 800;
      }
      //topRingCounter.reset();
      // lineActive = true;
    }
    // else {
    //   lineActive = false;
    // }
    //If there is a ring leaving and we don't want rings to leave, disable the conveyer
    if((topRingCounter.active() and conveyer.noOut)){
      conveyer.disabled = true;
    }
    else if(Brain.Timer.system() - startTime > 1000){
      conveyer.disabled = false;
    }

    //If there are rings on the bottom
    // groundRingDetector.takeSnapshot(Rings);
    // if(groundRingDetector.objectCount > 0 && groundRingDetector.largestObject.width > 30){
    //   conveyer.enabledFor = 500;
    // }
    if((ringDetectorLeft.installed() 
        && ringDetectorLeft.objectDistance(distanceUnits::in) < 6.0
        && ringDetectorLeft.isObjectDetected())
        || (ringDetectorRight.installed()
        && ringDetectorRight.objectDistance(distanceUnits::in) < 6.0
        && ringDetectorRight.isObjectDetected())){  
      // cout << ringDetectorRight.objectDistance(distanceUnits::in) << endl;
      // cout << ringDetectorLeft.objectDistance(distanceUnits::in) << endl;
      conveyer.enabledFor = 500;
    }
    //Run the conveyer
    if((!conveyer.disabled && not conveyer.controller)){
      if((conveyer.autonCtrl && conveyer.autonOut > 0) || !conveyer.autonCtrl){
        if(conveyer.countIn > 0 or conveyer.enabledFor > 0){
          //spinFlaps();
        }
        //Don't run the flaps if there isn't a reason to
        else {
          stopFlaps();
        }
      }
    }
    else if(not conveyer.controller){
      stopFlaps();
    }
    //Decrease the enabled for outside of the loop bc we don't want 
    //  conveyer running randomly after no longer being disabled
    conveyer.enabledFor -= 30;
    s(30);
  }
}*/
void liftControl(){
  int canMove = 20;
  LinkedList<double> positions;
  while(1){
    if(!liftCtrllr.disabled()){
      if(!liftCtrllr.isUp && liftMot.position(deg) < liftCtrllr.getPosition()){
        
        liftMot.spin(fwd);
        if(canMove < 0){
          if(abs(positions.getBase() - liftMot.position(deg)) < 10){
            // cout << "Transferring to High speed" << endl;
            liftMot.spin(fwd, 12, volt);
          }
        }

        liftCtrllr.done = false;
      }
      else if(liftCtrllr.isUp && liftMot.position(deg) > liftCtrllr.getPosition()){
        liftMot.spin(vex::reverse);
        if(canMove < 0){
          if(abs(positions.getBase() - liftMot.position(deg)) < 10){
            // cout << "Transferring to High speed" << endl;
            liftMot.spin(vex::reverse, 12, volt);
          }
        }
        liftCtrllr.done = false;
      }
      else {

        liftCtrllr.done = !liftCtrllr.prevented();;
        liftMot.stop(hold);
      }
    }
    // if(!backLiftCtrllr.disabled()){
    //   if(!backLiftCtrllr.isUp && flapConveyer.position(deg) < backLiftCtrllr.getPosition() - 30){
    //     flapConveyer.spin(fwd, 80, pct);
    //     backLiftCtrllr.done = false;
    //   }
    //   else if(backLiftCtrllr.isUp && flapConveyer.position(deg) > backLiftCtrllr.getPosition() + 30){
    //     flapConveyer.spin(vex::reverse, 80, pct);
    //     backLiftCtrllr.done = false;
    //   }
    //   else {

    //     backLiftCtrllr.done = !backLiftCtrllr.prevented();
    //     flapConveyer.stop(hold);
    //   }
    // }
    positions.push_back(liftMot.position(deg));
    if(canMove < 0){
      positions.popBase();
    }
    canMove--;
    //lift.spin(fwd, liftCtrllr.getPosition());
    liftCtrllr.sleep();
  }
}
//}

//APA {

void programWrite(bool start){
  int numArg = 0;
  bool threaded = false;
  //Set drive mode to auton to allow APA writing
  //driveMode = "auton";
  wc.coastBrake();
  if(start)
    autonInit();
  //Add some space
  cout << "\n\n" << endl;
  while(1){
    //Set the number args
    //left shift 2 bits, add 3 -- 0b11
    if(isPressing(Greg.ButtonUp)){
      numArg <<= 2;
      numArg += 3;
      cout << "// n:" << numArg << endl;
    } 
    //left shift 2 bits, add 2 -- 0b10
    else if(isPressing(Greg.ButtonRight)){
      numArg <<= 2;
      numArg += 2;
      cout << "// n:" << numArg << endl;
    }
    //left shift 2 bits, add 1 -- 0b01
    else if(isPressing(Greg.ButtonLeft)){
      numArg <<= 2;
      numArg += 1;
      cout << "// n:" << numArg << endl;
    }
    //left shift 2 bits, add 0 -- 0b00
    else if(isPressing(Greg.ButtonDown)){
      numArg <<= 2;
      numArg += 0;
      cout << "// n:" << numArg << endl;
    }
    //Resets numArg to 0
    else if(isPressing(Greg.Axis1, -1)){
      numArg = 0;
      cout << "// n:" << numArg << endl;
    }
    //Inverses the value of threaded: true => false, false => true
    else if(isPressing(Greg.ButtonA)){
      threaded = !threaded;
      cout << "// t:" << threaded << endl;
    }
    //Driving controls
    //driveTo:            Axis2 -           1
    //turnTo:             Axis1 -           1
    //backInto:           Axis2 -           -1
    //driveFwd:           Axis3 -           -1
    //followPath:         Axis3 -           1
    //backwardsFollow:    Axis1 -           -1
    else if(isPressing(Greg.Axis2, 1)){
      bool exit = false;
      while(!exit){
        exit = true;
        //Set driveTo
        if(isPressing(Greg.Axis2, 1)){
          cout << "wc.driveTo(" << VECT_XY(wc.botPos());
          if(wc.isOmniDir){
            cout << ", " << wc.botAngle() << ");" << endl;
          }
          else {
            cout << ");" << endl;
          }
        } 
        //Set turnTo
        else if(isPressing(Greg.Axis1, 1)){
          cout << "wc.turnTo(" << wc.botAngle() << ");" << endl;
        }
        //Set backInto
        else if(isPressing(Greg.Axis2, -1)){
          cout << "wc.backInto(" << VECT_XY(positioner.getPos());
          if(wc.isOmniDir){
            cout << ", " << wc.botAngle() << ");" << endl;
          }
          else {
            cout << ");" << endl;
          }
        }
        //Set driveFwd
        else if(isPressing(Greg.Axis3, -1)){

          cout << "//start" << endl;
          
          PVector startPos = wc.botPos();
          while(!isPressing(Greg.Axis2, -1)){
            s(100);
          }
          auto v = wc.botPos();
          double finalDist = startPos.dist2D(v);
          cout << "wc.driveFwd(" << finalDist << ");" << endl;
        }
        //Set followPath
        else if(isPressing(Greg.Axis3, 1)){
          cout << "//start" << endl;
          PVector startPos = wc.botPos();
          cout << "wc.followPath({PVector(" << VECT_XY(startPos) << ")";
          while(!isPressing(Greg.Axis2, -1)){
            if(isPressing(Greg.Axis2, 1)){
              cout << ", PVector(" << VECT_XY(wc.botPos()) << ")";
            }
            s(100);
          }
          if(wc.isOmniDir){
            
            
            cout << "}, " << wc.botAngle() << ");" << endl;
          } else {
            
            
            cout << "});" << endl;
          }
        }
        //Set backwardsFollow
        else if(isPressing(Greg.Axis1, -1)){
          cout << "//start" << endl;
          PVector startPos = wc.botPos();
          cout << "wc.backwardsFollow({PVector(" << VECT_XY(startPos) << ")";
          while(!isPressing(Greg.Axis2, -1)){
            if(isPressing(Greg.Axis2, 1)){
              cout << ", PVector(" << VECT_XY(wc.botPos()) << ")";
            }
            s(100);
          }
          if(wc.isOmniDir){
            
            
            cout << "}, " << wc.botAngle() << ");" << endl;
          } else {
            
            cout << "});" << endl;
          }
        }
        else {
          exit = false;
        }
        s(10);
      }
    }
    //All following functions are executed live, while writing
    
    //
    else if(isPressing(Greg.Axis2, -1)){
      
    }
    //
    else if(isPressing(Greg.Axis1, 1)){
      
    }
    //
    else if(isPressing(Greg.ButtonR1)){
      allowOutRings(numArg);
      numArg = 0;
    }
    //
    else if(isPressing(Greg.ButtonR2)){
      if(threaded){
        freeConveyer();
      }
      else {
        ctrlConveyer();
      }
    }
    //
    else if(isPressing(Greg.ButtonL1)){
      if(threaded){
        raiseLiftByOne();
      }
      else {
        raiseLiftByOneWait();
      }
    }
    //
    else if(isPressing(Greg.ButtonL2)){
      if(threaded){
        lowerLiftByOne();
      }
      else {
        lowerLiftByOneWait();
      }
    }
    //
    else if(isPressing(Greg.Axis3, -1)){
      
    }
    //
    else if(isPressing(Greg.Axis3, 1)){
      waitForLiftFinish();
    }
    //
    else if(isPressing(Greg.Axis4, 1)){

    }
    //
    else if(isPressing(Greg.Axis4, -1)){
      
    }
    //
    else if(isPressing(Greg.ButtonB)){
      
    }
    //Execute specified special function
    else if(isPressing(Greg.ButtonX)){
      specFns[numArg](true);
      numArg = 0;
    }
    //Add Comment
    else if(isPressing(Greg.ButtonY)){
      if(!start){
        return;
      }
    }
    //Sleep for a time to give the brain a break
    s(100);
  }
  
}
void programWrite(void* in){
  programWrite(*(bool*)in);
}


//}

//Brain Drawing Stuff {

void printVars() {
  Brain.Screen.waitForRefresh();
  Brain.Screen.clearScreen(black);
  Brain.Screen.setFillColor(black);
  Brain.Screen.printAt(10, 20, (string("glblBotAngle: ") + toCcp(glblBotAngle)).data());
  Brain.Screen.printAt(10, 40, (string("botAngles.y: ") + toCcp(botAngles.y)).data());
  Brain.Screen.printAt(10, 60, (string("botAngles.z: ") + toCcp(botAngles.z)).data());
  Brain.Screen.printAt(10, 80, (string("tiltAngle: ") + toCcp(tiltAngle)).data());
  Brain.Screen.printAt(10, 100, (string("wc.botPos().x: ") + toCcp(wc.botPos().x)).data());
  Brain.Screen.printAt(10, 120, (string("wc.botPos().y: ") + toCcp(wc.botPos().y)).data());
}
void displayTilt(){
  s(100);
  Brain.Screen.waitForRefresh();
  Brain.Screen.clearScreen(black);
  PVector start = PVector(160, 100);
  PVector v = PVector(0,80);
  v.rotate(tiltAngle);
  v += start;
  Brain.Screen.setFillColor(red);
  Brain.Screen.drawCircle(start.x, start.y, 5);
  Brain.Screen.drawCircle(v.x, v.y, 5);
}
void drawPath(){
  s(100);
  PVector off = PVector(100, 100);
  Brain.Screen.waitForRefresh();
  Brain.Screen.clearScreen(black);
  
  if(wc.drawArr){
    Brain.Screen.setFillColor(blue);
    for(auto v : wc.path){
      v *= 2.0;
      v += off;
      Brain.Screen.drawCircle(v.x, v.y, 2);
    }
  } else {
    Brain.Screen.setFillColor(black);
    Brain.Screen.printAt(40, 40, "No Path");
  }
}
const color grey = color(100, 100, 100);
const color lightGreen = color(100, 255, 100);

const color darkGreen = color(ClrDarkGreen);



void brainOS() {
  int state = 1;		
  int maxState = 3; 
  Button screenLeft = Button(Brain, 10, BRAIN_HEIGHT - 60, 30, 30, black, "<");		
  Button screenRight = Button(Brain, BRAIN_WIDTH - 40, BRAIN_HEIGHT - 60, 30, 30, black, ">");		
  while (1) {		
    switch(state) {
      case 1:
        printVars();
        break;
      case 2:
        displayTilt();
        break;
      case 3:
        drawPath();
        break;
      default:
        windowsLoader();
        break;
    }
    screenLeft.draw();		
    screenRight.draw();	
    if (screenLeft.clicked() && state > 0) {	
      state--;	
    }	
    else if (screenRight.clicked() && state < maxState) {	
      state++;	
    }	
  }	
}
//}
int main() {
  // testMotorConfiguration();
  unclipGoal();
  unclipLiftGoal();
  //useVisionAlign = true; 
  gyroInit(angler);
  //conveyer.noOut = true;
  gyroInit(GPS);
  updateBotAngle();
  startTilt = botAngles.z;

  KillThread updateGPSShare = thread(updateSharePos);
  //Make a thread to update the position so that it won't get blocked by other tasks
  KillThread updatePos = thread(trackPos);
  
  //thread conveyer = thread(conveyerControl);

  thread liftThread = thread(liftControl);

  //Make a thread to execute some auton tasks concurrently
  KillThread otherThreads = thread(executeThreads);
  
  //WINDOWS LOADER: YEET BURGER
  thread loader = thread(brainOS);
  
  cout << "Y " << endl;
  Competition.drivercontrol(drivercontrol);
  cout << "Yeet " << endl;
  Competition.autonomous(autonomous);
  
  while(1){
    cout << visionTest.estimatePos(RED2) << endl << endl;
    s(200);
  }
  while(1){
    
    s(300);
  }
}






