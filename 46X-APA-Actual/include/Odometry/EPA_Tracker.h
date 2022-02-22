#include "Odometry/FieldCoord.h"
//EPA_Tracker.h -- Use this file to track the robot's absolute position on the field
//This file does so much math that it be like Beethoven 9 if it all works properly

//Make some basic conversion numbers
//To use, just multiply the number by the conversion
#define CM_TO_IN (1.0 / 2.54)
#define IN_TO_CM 2.54
#define DEG_TO_RAD (M_PI / 180.0)
#define RAD_TO_DEG (1.0 / DEG_TO_RAD)
typedef unsigned int uint;
//Shifts angle to range of [0, 360)
double baseAngle(double ang){
  while(ang >= 360.0){
    ang -= 360.0;
  }
  while(ang < 0.0){
    ang += 360.0;
  }
  return ang;
}
//Shifts an angle to a range of (-180, 180]
double posNeg180(double ang){
  double ret = baseAngle(ang);
  while(ret > 180.0){
    ret -= 360.0;
  }
  while(ret <= -180.0){
    ret += 360.0;
  }
  return ret;
}

void updateBotAngle(bool add = false);
//A class that stores a reference of something as a pointer
//  this is used so that I can put references in arrays
template<class tp>
struct Ref {
  tp* val;
  Ref(){}
  Ref(tp& v){
    val = &v;
  }
  tp& operator*(){
    return *val;
  }
  operator tp& (){
    return *val;
  }
  operator tp*(){
    return val;
  }
};
// //Use the template command for the amount of encoders used
// //Each encoder is used for each dimension
// //Positioner<2, 2> has four encoders total 2x, 2y
// template<uint encodersX, uint encodersY>
class Positioner {
//A few typedefs
public:
  typedef vector<encoder*> yEncoderArr; // Defines the type of the arrays
      //That the encoders will be stored in and names it encoderArr
  typedef vector<encoder*> xEncoderArr;
  typedef vector<Ref<vex::triport::port>> xPortArr; // Defines the type of the arrays
      //That the encoders will be stored in and names it encoderArr
  typedef vector<Ref<vex::triport::port>> yPortArr;
  //const double size = encodersX + encodersY;
//Private variables
private:

  double encXAmnt;// = (double) encodersX;
  double encYAmnt;// = (double) encodersY;
  typedef vector<double> xDoubleArr;
  typedef vector<double> yDoubleArr;
  xDoubleArr lastX;
  yDoubleArr lastY;
  xDoubleArr multX;
  yDoubleArr multY;
  xDoubleArr multNegX;
  yDoubleArr multNegY;
  double distFromCenterX;
  double distFromCenterY;

  xEncoderArr EncodersX; // Make the x encoder array
  yEncoderArr EncodersY;
  double lastLeft = 0.0, lastRight = 0.0;
  friend void waitForReset();
  void resetPos(PVector pos){
    this->pos = pos;
  }
public:
private:
  //array<double, encodersY> lastY;
  //array<double, encodersX> lastX;
  //PVector lastAngles = PVector(0.0, 0.0); // Make a vector that stores the last angles
  PVector pos = PVector(0.0, 0.0); // Make a vector to store the current position
  double wheelRad = 0.0; // A variable that stores the wheel radius in inches for
          // distance calculations later
  //Useless variables that I'm keeping just in case they become useful eventually
  //double updates_p_second = 500;
  //double& ups = updates_p_second;
  double speed = 0.0;
  uint64_t lastTime = Brain.Timer.systemHighResolution();
  //The main functions: constructor, updater...
  Positioner(xDoubleArr mX, yDoubleArr mY, xDoubleArr mNX, yDoubleArr mNY, double cDistX, double cDistY, double rad){
    wheelRad = rad;
    multX = mX;
    multY = mY;
    multNegX = mNX;
    multNegY = mNY;
    distFromCenterX = cDistX;
    distFromCenterY = cDistY;
  }
public:
  //The constructors

  //Accepts port array and radius
  Positioner(
    xPortArr xPorts, yPortArr yPorts, 
    xDoubleArr mX, yDoubleArr mY, 
    xDoubleArr mNX, yDoubleArr mNY, 
    double cDistX, double cDistY, 
    double rad
  ) : Positioner(mX, mY, mNX, mNY, cDistX, cDistY, rad)
  {
    int i = 0;
    //I know, I know, using 'new' is bad but, like, I mean, 
    // it's used only in the global scope, so all memory is deallocated by OS, not me
    for(auto& port : xPorts){
      //Allocate a completely new encoder from the heap and add it to the array
      EncodersX.push_back(new encoder(port));
      lastX.push_back(0);
      //Increase array access position
      i++;
    }

    i = 0;
    for(auto& port : yPorts){
      lastY.push_back(0);
      //Allocate a completely new encoder from the heap and add it to the array
      EncodersY.push_back(new encoder(port));
      //Increase array access position
      i++;
    }
    encXAmnt = EncodersX.size();
    encYAmnt = EncodersY.size();
    cout << EncodersY.size() << " sdf" << endl;
  }
  Positioner(){
  }
  //Function that updates the position
  //80+ lines of trig, vector math, and some sensor stuff
  PVector update(bool run, int microSec = 1){
    static double lostDist = 0.0;
    static PVector last = PVector(1, 1);
    //Vector of the wheel angles
    PVector angles = PVector();

    //Get encoder rotation as fast as possible
    //Use raw array for speed
    vector<double> rotX;
    vector<double> rotY;
    //Update bot angle as close to rotation access as possible
    // updateBotAngle(run);
    if(EncodersX.size() != 0){
      for(int i = 0; i < EncodersX.size(); i++){
        //Don't do any multiplication (for speed)
        rotX.push_back(EncodersX[i]->rotation(rotationUnits::deg));
      }
    }
    // cout << EncodersY.size() << endl;
    if(EncodersY.size() != 0){
      for(int i = 0; i < EncodersY.size(); i++){
        rotY.push_back(EncodersY[i]->rotation(rotationUnits::deg));
        // cout << rotY[i] << endl;
        // s(1);
      }
    }

    for(int i = 0; i < EncodersX.size(); i++){
      //Get the rotation in radians
      double rot = posNeg180(rotX[i] - lastX[i]) * DEG_TO_RAD;// * multX[i];
      //double turnExpected = deltaBotAngle * DEG_TO_RAD * distFromCenterX[i];

      if(rot < 0){
        
        rot *= multNegX[i];
      } else if(rot > 0){
        rot *= multX[i];
      }
      //add the change in rotation to angles.x
      angles.x += rot;
      //Reset the last rotation
      lastX[i] = rotX[i];
      
    }

    //Average angles.x

    //Same thing here    
    for(int i = 0; i < EncodersY.size(); i++){
      double rot = posNeg180(rotY[i] - lastY[i]) * DEG_TO_RAD;// * multY[i];

      //double turnExpected = deltaBotAngle * DEG_TO_RAD * distFromCenterY[i];
      if(rot < 0){
        rot *= multNegY[i];
      } else if(rot > 0){
        rot *= multY[i];
      }
      angles.y += rot;
      lastY[i] = rotY[i];
    }
    if(EncodersY.size() != 0){
      angles.y /= encYAmnt;
    }
    if(EncodersX.size() != 0){
      angles.x /= encXAmnt;
    }
    angles *= wheelRad;
    PVector deltaAngles;
    
    
    if(deltaBotAngle != 0.0){
      double deltaAngle = deltaBotAngle * DEG_TO_RAD;
      double sin2 = 2.0 * sin(deltaAngle / 2.0);
      double x = (angles.x / deltaAngle + distFromCenterX) * sin2;
      double y = (angles.y / deltaAngle + distFromCenterY) * sin2;
      deltaAngles = { x, y };
      deltaAngles.rotate(avgBotAngle);
    } else {
      deltaAngles = angles;
      deltaAngles.rotate(avgBotAngle);
    }

    //Get the change in the position
    PVector deltaPos = deltaAngles;
    if(deltaPos.dist2D() != 0){
      // cout << "ok" << endl;
    }
    //Random velocity stuff
    double time = microSec * 1.0e-6;
    speed = deltaPos.dist2D() / time;
    pos += deltaPos; // Add deltaPos to pos
    return pos; // Return pos so it can be used
  }
  PVector getPos(){
    return PVector(-pos.x, -pos.y, 0.0);
  }
  double xPosition(distanceUnits=inches){
    return getPos().x;
  }
  double yPosition(distanceUnits=inches){
    return getPos().y;
  }
  double heading(){
    return glblBotAngle;
  }
  FieldCoord fullPos(){
    return FieldCoord(getPos(), heading());
  }
  bool moving (){
    //If the velocity is greater than 0.01 in/s or 
    //its been less a second since the last call to clearMove
    return abs(speed) > 0.01 || 
           Brain.Timer.systemHighResolution() - lastTime < 1e6;
  }
  double velocity(){
    return speed;
  }
  void clearMove(){
    lastTime = Brain.Timer.systemHighResolution();
  }
};
