//#include "EPA_Tracker.h"
#include "Mogo.h"
#include "goalFront.h"
#include "goalDetector.h"

/*************************************

Motors

*************************************/
//Front Left Wheel (FL)
motor FL = motor(PORT11,gearSetting::ratio18_1,!false);
//Front Right Wheel (FR)
motor FR = motor(PORT19, gearSetting::ratio18_1,!true);
//Back Left Wheel (BL)
motor BL = motor(PORT14, gearSetting::ratio18_1,!false);
//Back Right Wheel (BR)
motor BR = motor(PORT18, gearSetting::ratio18_1,!true);
//Middle Left Wheel (ML)
motor ML = motor(PORT13, gearSetting::ratio18_1,!false);
//Middle Right Wheel (MR)
motor MR = motor(PORT17, gearSetting::ratio18_1,!true);

motor_group Left = motor_group(BL, ML, FL);
motor_group Right = motor_group(BR, MR, FR);

motor flapConveyer = motor(PORT8, gearSetting::ratio18_1, false);

motor liftMot = motor(PORT6, ratio36_1, true);

//New Motors, a few reasons for this: 
//    1 - less upfront code for stuff
//    2 - Simplified spin cmd
NewMotor<> wheels = NewMotor<>(FL, ML, BL, FR, MR, BR);
NewMotor<> leftWhls = NewMotor<>(BL, FL, ML);
NewMotor<> rghtWhls = NewMotor<>(BR, FR, MR);
NewMotor<> flaps = NewMotor<>(flapConveyer);
NewMotor<> lift = NewMotor<>(liftMot);

/*************************************

Pneumatics

*************************************/
pneumatics goalHolder = pneumatics(Brain.ThreeWirePort.H);
pneumatics liftGoalHolder = pneumatics(Brain.ThreeWirePort.G);

/*************************************

Sensors

*************************************/
//Front Goal Switch
LineCounter frontCounter(Brain.ThreeWirePort.E);
LineCounter backCounter(Brain.ThreeWirePort.F);

//Three wire expander
triport Expander = triport(PORT9);

//Inertial Sensor
inertial angler = inertial(PORT3);

//gps
gps GPS = gps(PORT7, -6.0, 0.0, inches, -90);
// Make a list of ports so Will can check them if they get messed up (make it a word doc)
Distance goalFront = Distance(PORT11);
Distance goalBack = Distance(PORT12);


//PotDials
PotDial skillsOrSide = PotDial(Brain.ThreeWirePort.C, 3);
PotDial centerGoals = PotDial(Brain.ThreeWirePort.D, 4);
PotDial autonGoals = PotDial(Brain.ThreeWirePort.E, 2);
PotDial::Controller<function<void()>> autonMap = 
    PotDial::Controller<function<void()>>(skillsOrSide, centerGoals, autonGoals);


/*************************************

Odometry

*************************************/

// VisionOdom visionTest = VisionOdom(goalFrontVision, {0, 0}, 12.5, 32, 0);

//Positioner init
typedef Positioner posTp;
posTp::xPortArr arrX = { };
posTp::yPortArr arrY = { Brain.ThreeWirePort.A };
//Make a positioner that measures x and y with smallest omni wheel rad
posTp positioner = posTp(arrX, arrY, 
                        { 1.0 }, { 1.0 }, { 1.0 }, { 1.0 },
                         0.0 ,  0.0 ,
                        1.375);

GPS_Share share = GPS_Share(positioner, GPS);

//Wheel controller
Omni_6Controller wc = Omni_6Controller(BL, BR, FL, FR, ML, MR, share);


/*************************************

Autonomous System Controllers

*************************************/

struct {
  bool noOut = false;
  bool disabled = false;
  int enabledFor = 0;
  int countIn = 0;
  bool controller = false;
  bool autonCtrl = false;
  int autonOut = 0;
} conveyer;



class {
  PID angleTarget = PID(1.0, 0.001, 0.1);
  int currentIndex = 0;
  bool lastPressing = false;
  vector<double> positions = { 0, -100, -480, -510 };
  int timesDone = 0;
  int sleepTime = 20;
  bool isDisabled = false;
  int prevent = 0;
public:
  bool done = false;
  void prev(){
    prevent = 5;
  }
  bool prevented(){
    return prevent-- > 0;
  }
  void disable(){
    isDisabled = true;
  }
  void enable(){
    isDisabled = false;
  }
  bool disabled(){
    return isDisabled;
  }
  bool isUp = false;
  void sleep(){
    s(sleepTime);
  }
  void freePress(){
    lastPressing = false;
  }
  void addIndex(){
    isUp = true;
    if(!lastPressing){
      currentIndex++;
      if(currentIndex >= positions.size()){
        currentIndex = positions.size() - 1;
      }
      angleTarget.setTarget(getPosition());
    }
    lastPressing = true;
  }
  void subIndex(){
    isUp = false;
    if(!lastPressing){
      currentIndex--;
      if(currentIndex < 0){
        currentIndex = 0;
      }
      angleTarget.setTarget(getPosition());
    }
    lastPressing = true;
  }
  double getPosition(){
    return positions[currentIndex];
  }
  double getSpeed(){
    if(abs(angleTarget.getError()) < 10.0){
      timesDone ++;
    }
    else {
      timesDone = 0;
    }
    return angleTarget.getVal(liftMot.position(rotationUnits::deg));
  }
  bool isDone(){
    return done;
  }
  void setIndex(int i){
    if(i > currentIndex){
      isUp = false;
      currentIndex = i;
    }
    if(i < currentIndex){
      isUp = true;
      currentIndex = i;
    }
  }
} liftCtrllr;

// class {
//   PID angleTarget = PID(1.0, 0.001, 0.1);
//   int currentIndex = 0;
//   bool lastPressing = false;
//   vector<double> positions = { 970, 750, 330, 0 };
//   int timesDone = 0;
//   int sleepTime = 20;
//   bool isDisabled = false;
//   int prevent = 0;
// public:
//   int getIndex(){
//     return currentIndex;
//   }
//   bool done = false;
//   void prev(){
//     prevent = 5;
//   }
//   bool prevented(){
//     return prevent-- > 0;
//   }
//   void disable(){
//     isDisabled = true;
//   }
//   void enable(){
//     isDisabled = false;
//   }
//   bool disabled(){
//     return isDisabled;
//   }
//   bool isUp = false;
//   void sleep(){
//     s(sleepTime);
//   }
//   void freePress(){
//     lastPressing = false;
//   }
//   void addIndex(){
//     isUp = true;
//     if(!lastPressing){
//       currentIndex++;
//       if(currentIndex >= positions.size()){
//         currentIndex = positions.size() - 1;
//       }
//       angleTarget.setTarget(getPosition());
//     }
//     lastPressing = true;
//   }
//   void subIndex(){
//     isUp = false;
//     if(!lastPressing){
//       currentIndex--;
//       if(currentIndex < 0){
//         currentIndex = 0;
//       }
//       angleTarget.setTarget(getPosition());
//     }
//     lastPressing = true;
//   }
//   double getPosition(){
//     return positions[currentIndex];
//   }
//   double getSpeed(){
//     if(abs(angleTarget.getError()) < 10.0){
//       timesDone ++;
//     }
//     else {
//       timesDone = 0;
//     }
//     return angleTarget.getVal(liftMot.position(rotationUnits::deg));
//   }
//   bool isDone(){
//     return done;
//   }
//   void setIndex(int i){
//     if(i > currentIndex){
//       isUp = !false;
//       currentIndex = i;
//     }
//     if(i < currentIndex){
//       isUp = !true;
//       currentIndex = i;
//     }
//   }
// } backLiftCtrllr;

#define TEST_MOT(m) cout << #m << endl; m.spin(fwd); s(1000); m.stop(); s(500);
void testMotorConfiguration(){
  TEST_MOT(FL)
  TEST_MOT(ML)
  TEST_MOT(BL)
  s(1000);
  TEST_MOT(FR)
  TEST_MOT(MR)
  TEST_MOT(BR)
}
/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void){
  // wheels.set(hold);
}
