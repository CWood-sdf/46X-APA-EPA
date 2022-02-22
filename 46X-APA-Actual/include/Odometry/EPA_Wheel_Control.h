//EPA_WheelControl.h -- Use this file to control the wheel base
//     to go to specified positions

//A few major ideas working in this file:
//  1 - Bezier Curves - To make beautifully smooth paths
//  2 - Pure Pursuit - To direct the bot's motion along Bezier paths
//  3 - PID Controllers - To direct the bot's speed along the path

#include "Odometry/PID.h"
#include "Odometry/GPS_Share.h"
#include <deque>
#ifdef DEBUG
#warning "DEBUG already defined"
#else
#define DEBUG
#endif
//Makes an array of Vectors that can be efficiently modified at both ends
struct VectorArr {
  //Underlying storage type
  //Leave it public just because
  vector<PVector> arr = vector<PVector>();

  //Constructor
  VectorArr(std::initializer_list<PVector> list){
    for(PVector v : list){
      arr.push_back(v);
    }
  }

  //Default the other constructors and destructor
  VectorArr() = default;
  VectorArr(const VectorArr&) = default;
  VectorArr(VectorArr&&) = default;
  ~VectorArr() = default;

  //Use decltype because I don't care enough to find out the type

  //Iterators:
  decltype(arr.begin()) begin(){
    return arr.begin();
  }
  decltype(arr.end()) end(){
    return arr.end();
  }
  //Bracket access
  PVector& operator[](int i){
    return arr[i];
  }
  void pop(int i){
    arr.erase(arr.cbegin() + i, arr.cbegin()+ i+1);
  }
  // void popCurrentNext(){
  //   arr.popCurrentNext();
  // }
  void popBase(){
    arr.erase(arr.cbegin(), arr.cbegin() + 1);
  }
  void popEnd(){
    arr.erase(arr.cbegin() + arr.size() - 1, arr.cbegin() + arr.size());
  }
  //Get the size
  size_t size(){
    return arr.size();
  }
  //Add elements
  void push(PVector v){
    arr.push_back(v);
  }
  void push_front(PVector v){
    arr.insert(arr.cbegin(), v);
  }

  //Get last / first elements
  PVector& last(){
    return arr.back();
  }
  PVector& first(){
    return arr.front();
  }

  //operator=
  VectorArr& operator=(VectorArr& v){
    
    arr = v.arr;
    return *this;
  }
  //operator=
  VectorArr& operator=(VectorArr&& v){
    cout << "Ok" << endl;
    s(500);
    arr = v.arr;
    cout << "Ok" << endl;
    s(500);
    return *this;
  }
};

//Find a single point on a bezier curve with parameter t (t goes from 0 -> 1)
PVector bezierInterpolate(VectorArr ptArr, double t){
  //The array of the interpolated points
  VectorArr newPts = {};
  for(int i = 0; i < ptArr.size() - 1; i++){
    //Interpolate between current point and next
    PVector newPt = ptArr[i + 1] - ptArr[i];
    newPt.mult(t);
    newPt += ptArr[i];
    newPts.push(newPt);
  }
  //If interpolated point array still has multiple elements
  if(newPts.size() >= 2){
    //YAYYY RECURSION!!!!
    return bezierInterpolate(newPts, t);
  } else {
    //Otherwise return the only element
    return newPts.first();
  }
}
//Create a bezier curve
VectorArr bezierCurve(VectorArr ptArr, double inc = 1.0 / 50.0){
  double spacing = 1.0;
  //Return value
  VectorArr ret;
  //Go through multiple rounds of interpolation
  for(double i = 0; i <= 1; i += inc){
    PVector pt = bezierInterpolate(ptArr, i);
    ret.push(pt);
  }
  ret.push(ptArr.last());
  //PVector lastSafe = ret.first();
  //bool isSafe = true;
  // for(int i = 0; i < ret.size(); i++){
  //   auto& g = ret[i];
  //   if(!isSafe){
  //     if(g.dist2D(lastSafe) < spacing){
  //       ret.pop(i);
  //       i--;
  //     }
  //     else {
  //       lastSafe = ret[i];
  //     }
  //   } else {
  //     isSafe = false;
  //   }
  // }
  return ret;
}
VectorArr bezierDerivative(VectorArr ptArr, double inc = 1.0 / 50.0){
  VectorArr bezier = bezierCurve(ptArr, inc);
  VectorArr derivative;
  for(int i = 0; i < bezier.size() - 1; i++){
    derivative.push(bezier[i + 1] - bezier[i]);
  }
  derivative.push(derivative.last());
  return derivative;
}
VectorArr bezierAcc(VectorArr ptArr, double inc = 1.0 / 50.0){
  VectorArr derivative = bezierDerivative(ptArr, inc);
  VectorArr acc;
  for(int i = 0; i < derivative.size() - 1; i++){
    acc.push(derivative[i + 1] - derivative[i]);
  }
  acc.push(acc.last());
  return acc;
}
//Create a bezier curve
pair<VectorArr, VectorArr> bezierCurveNormalLR(VectorArr ptArr, double dist = 1.0, double inc = 1.0 / 50.0){
  double spacing = 1.0;
  //Return value
  pair<VectorArr, VectorArr> ret;
  VectorArr bezier = bezierCurve(ptArr, inc);
  
  VectorArr derivative = bezierDerivative(ptArr, inc);
  for(auto& i : derivative){
    i.normalize();
  }
  for(int i = 0; i < derivative.size(); i++){
    ret.first.push(derivative[i].get().rotate(-90).mult(dist).add(bezier[i]));
    ret.second.push(derivative[i].get().rotate(90).mult(dist).add(bezier[i]));
  }
  return ret;
}
vector<double> bezierCurvature(VectorArr ptArr, double inc = 1.0 / 50.0){
  vector<double> ret;
  VectorArr derivative = bezierDerivative(ptArr, inc);
  VectorArr acc = bezierAcc(ptArr, inc);
  for(int i = 0; i < derivative.size(); i++){
    Matrix<double, 2, 2> combo;
    combo[0][0] = derivative[i].x;
    combo[0][1] = derivative[i].y;
    combo[1][0] = acc[i].x;
    combo[1][1] = acc[i].y;
    double denominator = pow(derivative[i].mag(), 3);
    if(denominator != 0)
      ret.push_back(combo.determinant() / denominator);
    else
      ret.push_back(1e99);
  }
  return ret;
}
//The basic wheel controller 
class BasicWheelController {
protected: // PID variables + other random things
  // PosExtractor<gps> GPS;
  // PosExtractor Odom;
  static inline LinkedList<BasicWheelController*&> requestedInstances;
  static inline BasicWheelController* inst = NULL;
  void fillInstances(){
    for(auto& i : requestedInstances){
      i = this;
    }
    inst = this;
    
  }
  LinkedList<PidAdder> customPidsDrive;
  LinkedList<PidAdder> customPidsTurn;
  LinkedList<PidAdder> customPidsSlave;
  GPS_Share& pos;
  bool usingGPS = true;
  PID slaveCtrl = PID(3.1775, 0.001, 2.0);

  //Get ctrl kP up to 6.25
  PID ctrl = PID(4.91, 0, 2.4325);
  //Make derivative absolutely massive to force a hard stop
  //PID purePursuitCtrl = PID(5.5, 0.0, 3.4);
  PID turnCtrl = PID(0.67, 0, 0.1);

  PidAdder oneGoalTurn = PidAdder(0.05, 0.000001, -0.01);
  PidAdder oneGoalDrive = PidAdder(0.1, 0.000001, -0.01);
  PidAdder oneGoalSlave = PidAdder(0.05, 0.0000001, -0.005);
  PidAdder turnYeet = PidAdder(0.1, 0.0, 0.0);
  PidAdder driveYeet = PidAdder(0.1, 0.0, 0.0);
  map<double, std::function<void()>> distFns, oldFns;
  volatile bool continueFwdDrive = false;
  bool callingInDrive = false;
public: // Some variables
  static void requestInstance(BasicWheelController*& wc){
    if(inst != NULL){
      wc = inst;
    }
    else {
      requestedInstances.push_back(wc);
    }
  }
  
  //A public path for drawing
  VectorArr path;
  bool drawArr = false;
  bool isOmniDir = false;
  bool hasFn = false;

  //Function to be called between turning and driving
  std::function<void()> afterTurn = [](){};
  //Two motors just because 
  NewMotor<> Left;
  NewMotor<> Right; 
public: // Constructor
  BasicWheelController(vector<Ref<motor>> L, vector<Ref<motor>> R, GPS_Share& s) :
    pos(s)
  {
    fillInstances();
    Left = L;
    Right = R;
  }
public: // Some Functions
  void addDrivePid(PidAdder a){
    ctrl += a;
    customPidsDrive.push(a);
    
  }
  void addDrivePid(double p, double i, double d){
    addDrivePid(PidAdder(p, i, d));
  }
  void popTopDrivePid(){
    auto& a = customPidsDrive.getEnd();
    ctrl -= a;
    customPidsTurn.popEnd();
  }
  void addSlavePid(PidAdder a){
    slaveCtrl += a;
    customPidsSlave.push(a);
    cout << slaveCtrl << endl;
  }
  void addSlavePid(double p, double i, double d){
    addSlavePid(PidAdder(p, i, d));
  }
  void popTopSlavePid(){
    auto& a = customPidsSlave.getEnd();
    slaveCtrl -= a;
    customPidsSlave.popEnd();
  }
  void addTurnPid(PidAdder a){
    turnCtrl += a;
    customPidsTurn.push(a);
    cout << turnCtrl << endl;
    
  }
  void addTurnPid(double p, double i, double d){
    addTurnPid(PidAdder(p, i, d));
  }
  void popTopTurnPid(){
    auto& a = customPidsTurn.getEnd();
    turnCtrl -= a;
    customPidsTurn.popEnd();
  }
  void addTurn(){
    turnCtrl += turnYeet;
    
  }
  void addDrive(){
    ctrl += driveYeet;
    
  }
  vector<double> getDVals(){
    return ctrl.getDVals();
  }
  void driveTo(double x, double y, bool bad = false){
    if(reverseAuton){
      x *= -1.0;
    }
    driveTo(PVector(x, y));
  }
  void backInto(double x, double y){
    backInto(PVector(x, y));
  }
  void useGPS(){
    usingGPS = true;
  }
  void setGPS(bool usingGPS){
    this->usingGPS = usingGPS;
  }
  void noGPS(){
    usingGPS = false;
  }
  virtual double botAngle (){
    return pos.heading();
  }
  PVector& botPos(){
    return pos.position();
  }
  //Add a function to be called at a specified distance
  void addDistFn(double dist, std::function<void()> fn){
    distFns[dist] = fn;
  }
  //Reuse the old map
  void reuseDistFns(){
    distFns = oldFns;
  }
  void setFn(std::function<void()> fn){
    afterTurn = fn;
    hasFn = true;
  }
  void callFn(){
    if(hasFn){
      afterTurn();
    }
    hasFn = false;
  }
  void reuseFn(){
    hasFn = true;
  }

  //A hard brake
  virtual void hardBrake(){
    Left.stop(hold);
    Right.stop(hold);
  }
  void addGoal(){
    ctrl += oneGoalDrive;
    turnCtrl += oneGoalTurn;
    slaveCtrl += oneGoalSlave;
  }
  void removeGoal(){
    ctrl -= oneGoalDrive;
    turnCtrl -= oneGoalTurn;
    slaveCtrl -= oneGoalSlave;
  }
  double limit(double d, double l, double m){
    if(d < l){
      //return l;
    }
    if(d > m){
      //return m;
    }
    return d;
  }
public: // PID Stuff    
  PID getCtrl(){
    return ctrl;
  }
  void setOldDistFns(){
    oldFns = distFns;
  }
  void useDistFns(double dist){
    auto iterator = distFns.begin();
    for(; iterator != distFns.end(); ){
      auto& fn = *iterator;
      if(fn.first < dist){
        fn.second();
        distFns.erase(iterator);
      } else {
        ++iterator;
      }
    }
  }
private: // turnTo, with re-updating function
  virtual void turnTo(std::function<double()> angleCalc){
    //
    
    //If the auton is on the other side, turn to the opposite angle
    double angle = angleCalc();
    cout << angle << endl;
    if(!callingInDrive && reverseAuton){
      angle *= -1.0;
    }
    int timeIn = 0;
    int i = 0;
    //Get the normAngle
    double normAngle = posNeg180(angle - posNeg180(botAngle()));
    //init PID
    turnCtrl.setTarget(0);
    int sleepTime = 30;
    int minTimeIn = 150;
    double degRange = 4.0;
    int speedLimit = 60;
    
    // 
    // 
    // 
    
    if(normAngle < -degRange){
      //It is off by 4 degrees, because:
      //    A: if it is within 4 degrees, without the minSpeed 
      //          it is too slow; but, with minSpeed, it overshoots too much
      while(timeIn * sleepTime < minTimeIn){
        
        double s = turnCtrl.getVal(normAngle);
        turnLeft(s > speedLimit ? speedLimit : s);
        task::sleep(sleepTime);
        
        angle = angleCalc();
        
        normAngle = posNeg180(angle - botAngle());
        if(abs(normAngle) < degRange){
          timeIn++;
        }
        else {
          timeIn = 0;
        }
        // 
      }
      hardBrake();
    } else if(normAngle > degRange){
      while(timeIn * sleepTime < minTimeIn){
        
        double s = -turnCtrl.getVal(normAngle);
        turnRight(s > speedLimit ? speedLimit : s);
        task::sleep(sleepTime);
        
        angle = angleCalc();
        // 
        normAngle = posNeg180(angle - botAngle());
        if(abs(normAngle) < degRange){
          timeIn++;
        }
        else {
          timeIn = 0;
        }
      }
      hardBrake();
    }
    cout << normAngle << endl;
    // hardBrake();
    // s(500);
    // normAngle = posNeg180(angle - posNeg180(botAngle()));
    
  }
public: // TurnTo
  double speedLimit = 100;
  virtual void turnTo(double angle){
    turnTo([=](){ return angle; });
  }
  virtual void driveTo(PVector pos){
    
    VectorArr arr = {pos};
    
    
    followPath(arr);
  }
private: // followPath vars
  PVector lastTarget;
  double maxAcc = 20;
  double maxDAcc = 30;
  double kConst = 1.0;
  double exitDist = 0.0;
public: // followPath var editors
  void setKConst(double v){
    kConst = v;
  }
  void setExitDist(double v){
    exitDist = v;
  }
  PVector getLastTarget(){
    return lastTarget;
  }
private: // General path follower, keep it private so that the implementations use isNeg, not the autonomous
  
  //The beefiest function in this file
  virtual void followPath(VectorArr arr, bool isNeg){
    double purePursuitDist = 16.0; // Distance to pure pursuit target
    //Simple initialization and turn to first point
    // cout << 100 << endl;
    // s(500);
    VectorArr bezier;
    
    auto arrCopy = arr;
    {
      //Allow the array to be drawn
      this->drawArr = true;
      arr.push_front(botPos());
      //Construct the original bezier
      bezier = bezierCurve(arr);
      //Copy the array for future use
      arrCopy = arr;
      this->path = bezier;
      //Turn to the first point purePursuitDist away
      // get first point
      double dist = botPos().dist2D(bezier[0]);
      int i = 0;
      while(dist < purePursuitDist){
        dist += bezier[i].dist2D(bezier[i + 1]);
        i++;
      }
      turnTo(botPos().angleTo(bezier[i - 1]) + 180.0 * isNeg);
      cout << "Turn Done" << endl;
      afterTurn();
      afterTurn = [](){};
    }
    //Remake bezier, get curvatures, set target speeds
    vector<double> targetSpeeds;
    {
      //Get original arr
      arr = arrCopy;
      arr.push_front(botPos());
      bezier = bezierCurve(arr);
      
      vector<double> curvatures = bezierCurvature(arr);
      targetSpeeds = vector<double>();
      
      for(int i = 0; i < bezier.size(); i++){
        targetSpeeds.push_back(abs(min(speedLimit, kConst / curvatures[i])));
        if(abs(targetSpeeds.back()) > 100){
          targetSpeeds.back() = 100;
        }
      }
      //Smooth targetSpeeds
      //vf^2 = vi^2 + 2ad
      double startVel = 30;
      targetSpeeds[0] = startVel;
      targetSpeeds.back() = 40;
      for(int i = 1; i < bezier.size(); i++){
        //I think this math of converting inches to percent works
        double d = bezier[i].dist2D(bezier[i - 1]) / 0.567;
        double a = maxAcc;
        if(startVel < targetSpeeds[i]){
          startVel = targetSpeeds[i] = sqrt(startVel * startVel + 2.0 * a * d);
          if(startVel > targetSpeeds[i + 1]){
            startVel = targetSpeeds[i + 1];
          }
        }
        else if(startVel > targetSpeeds[i]){
          int startI = i;
          //Go backwards until reached speed
          do {
            startVel = targetSpeeds[i];
            i--;
            double a = maxDAcc;
            double d = bezier[i].dist2D(bezier[i + 1]) / 0.567;
            startVel = targetSpeeds[i] = sqrt(startVel * startVel + 2.0 * a * d);
          } while(startVel < targetSpeeds[i - 1]);
          i = startI;
          startVel = targetSpeeds[i];
        }
      }
    }
    //Because this is necessary for some stupid reason
    for(int i = 0; i < 2; i++){
      targetSpeeds.push_back(40);
    }

    //The index of the pursuit target
    int bezierIndex = 0;
    //The pursuit target
    PVector pursuit = bezier[bezierIndex];
    //Lots variables

    //Allow the array to be drawn
    this->drawArr = true;
    this->path = bezier;
    //The last dist
    double lastDist = 12 * 2 * 24;
    //A timer
    timer t = timer();
    int time = 0, //Counts the time in loop iterations
    timeIn = 0, // The amount of time spent near the target
    maxTimeIn = 20, // The time needed before exit
    sleepTime = 5, // The sleep time
    g = 0; // A debug output counter

    double normAngle = 0.0, //The normAngle
    //Going with an hopefully possible 1 in accuracy
    minAllowedDist = exitDist == 0.0 ? 4.0 : exitDist, // The maximum distance from target before starting timeIn count
    /*******      STUPID UGLY NONFIX FIX       *************/
    limitedSpeed = 20; // What the speed should be when it's limited
    // #undef DEBUG
    #ifdef DEBUG  
    struct {
      vector<double> outSpeeds, encSpeeds, targSpeeds, angles, cp, cd, sp, sd;
      vector<PVector> pos, pursuit;
      void add(double out, double enc, double targ, PVector p, double angle, double acp, double acd, double asp, double asd, PVector apursuit){
        outSpeeds.push_back(out);
        encSpeeds.push_back(enc);
        targSpeeds.push_back(targ);
        pos.push_back(p);
        angles.push_back(angle);
        cp.push_back(acp);
        cd.push_back(acd);
        sp.push_back(asp);
        sd.push_back(asd);
        pursuit.push_back(apursuit);
      }
    } realTime;
    #endif
    
    //Save the current distance fns
    setOldDistFns();
    bool speedLimited = false; // Is the speed limited?
    int timesStopped = 0;
    int q = 0;

    //Loop
    while(timeIn * sleepTime < maxTimeIn){
      //pathEditor(ctrllr);
      // Keep the Pure Pursuit target purePursuitDist inches away from the bot
      while(pursuit.dist2D(botPos()) < purePursuitDist && pursuit != bezier.last()){
        pursuit = bezier[bezierIndex];
        ++bezierIndex;
      }
      //Force slowspeed, I don't like this because it's a dumb fix for a problem that shouldn't exist
      if(botPos().dist2D(bezier.last()) < minAllowedDist){
        speedLimited = true;
      }
      //Near the target, increment timeIn
      if(botPos().dist2D(bezier.last()) < minAllowedDist && pursuit == bezier.last()){
        timeIn++;
        
      }
      else {
        timeIn = 0;
      }
      //The distance to the pursuit target
      double dist = botPos().dist2D(pursuit);

      //If the bot's not moving, and it's not currently accelerating
      if(pos.velocity() < 0.1 && t.time(timeUnits::msec) > 1000){
        timesStopped++;
      }
      else {
        timesStopped = 0;
      }
      //50 ms not moving -> exit
      if(timesStopped > 10){
        cout << "Stop Exit" << endl;
        break;
      }
      //Get the speed of the robot
      double speed = ctrl.getVal(dist) * (isNeg * 2.0 - 1.0);
      /***   UGLY CHEAP HACKY NONFIXY FIX   *****/
      //Pls remove eventually
      if(speedLimited){
        if(speed > limitedSpeed){
          speed = limitedSpeed;
        }
        if(speed < -limitedSpeed){
          speed = -limitedSpeed;
        }
      }
      
      //Use the distFns for the current dist
      useDistFns(botPos().dist2D(bezier.last()));
      //The point extended beyond the path to make sure normAngle doesn't balloon near the target
      PVector virtualPursuit = pursuit;
      if(botPos().dist2D(pursuit) < purePursuitDist && pursuit == bezier.last()){
        //A vector that is parallel wih last point
        PVector last = bezier.last() - bezier[bezier.size() - 4];
        //Distance to be added so that virtualPursuit is still purePursuitDist away from bot
        double addDist = purePursuitDist - botPos().dist2D(pursuit);
        //Make last to be proper size

        last *= purePursuitDist / last.mag();
        virtualPursuit += last;
      }
      //Angle of robot to target
      double angle = baseAngle(botPos().angleTo(virtualPursuit));
      

      //The angle that it needs to travel at
      double normAngle = posNeg180(angle - botAngle() + 180 * isNeg);


      /*** NOTHING HAPPENING IN NEXT TWO BLOCKS ***/
      //So that the robot can take tight turns, 
      //if the turn is too tight, then the robot direction of travel will flip
      //Also if it's basically at the end, 
      //  then the robot will take the more efficient path backwards to the target
      //  rather than turn around
      if(abs(normAngle) >= 150){
        //isNeg = !isNeg;
        // cout << botPos() << ", " << pursuit << endl;
        // cout << normAngle << endl;
        // cout << "Reverse Neg" << endl;
        //Decrease timeIn because we aren't doing any sleeping this round
        if(timeIn > 0){
          //timeIn--;
        }
        //Send it back up top to recalibrate the speeds without sleeping
        //continue;
      }
      if(g == 5){
        
        // cout << pursuit << ", " << botPos() << endl;
        // cout << angle << endl;
        // cout << "norm: " << normAngle << endl;
      }

      /**** IF THE DATE IS NOT FRIDAY, FEB 4, 2022, DON'T TOUCH THE NEXT LINE ***/
      //Get the turn speed and divide by 2 because it is being applied to both wheels
      double rightExtra = -slaveCtrl.getVal(normAngle) / 4.0;
      
      //normAngle is too big for program to handle -> exit
      if(abs(normAngle) > 70){
        cout << "Ok exit" << endl;
        break;
      }
      //Use the extra turn speed as given if the distance is greater than 6 inches, 
      //Otherwise divide by 10 so that the bot isn't over turning
      double extraSpeed = (dist > 6.0 ? rightExtra : rightExtra / 2.0);
      /***   ^^MIGHT NOT BE NECESSARY IF virtualPursuit DOES IT'S JOB    ****/

      if(g++ == 2){
        // cout << extraSpeed << ", " << normAngle << endl;
        // cout << speed << ", " << dist << endl;
        g = 0;
        cout << normAngle << endl;
      }
      //Slew speed
      if(abs(speed) > targetSpeeds[bezierIndex]){
        // cout << targetSpeeds[bezierIndex] << endl;
        double orgSpeed = speed;
        speed = targetSpeeds[bezierIndex];
        //Change extraSpeed to match original speed : extraSpeed ratio
        extraSpeed *= orgSpeed / speed / 2.0;
      }
      
      //Mindblowing lines right here
      //Move the robot
      moveRight(speed - extraSpeed);
      moveLeft(speed + extraSpeed);
      //Sleep (WOW, HE'S A GENIUS)
      s(sleepTime);
      
      #ifdef DEBUG
      realTime.add(speed, pos.velocity(), targetSpeeds[bezierIndex], botPos(), botAngle(), ctrl.p, ctrl.d, slaveCtrl.p, slaveCtrl.d, pursuit);
      #endif
    }
    
    //Stop drawing the path
    //De-init code
    {
      //Set the last target for external stuff
      lastTarget = bezier.last();
      //Stop the bot
      if(exitDist == 0.0){
        hardBrake();
      }
      this->drawArr = false;
      cout << "Path stop" << endl;
      //Print postion and target position
      cout << botPos() << ", " << bezier.last() << endl;
      exitDist = 0.0;
    }
    //Print all the lists
    #ifdef DEBUG
      s(1000);
      cout << endl << endl;
      cout << "p.frameRate(" << 1.0 / (double)sleepTime * 1000 << ");\n";
      cout << "main.inputData([";
      int line = 0;
      for(auto i : bezier){
        cout << "p.createVector(" << i << "), " << (line++ % 3 == 0 ? "\n" : "");
        s(20);

      }
      s(100);
      cout << "]);";
      cout << "\nmain.auxiliaryData.pos = [";
      for(auto i : realTime.pos){
        cout << "p.createVector(" << i << "), " << (line++ % 3 == 0 ? "\n" : "");
        s(20);
      }
      s(100);
      cout << "];\n\nmain.auxiliaryData.angle = [";
      for(auto i : realTime.angles){
        cout << i << ", " << (line++ % 3 == 0 ? "\n" : "");
        s(10);
      }
      s(100);
      cout << "];\n\nmain.auxiliaryData.pursuit = [";
      for(auto i : realTime.pursuit){
        cout << "p.createVector(" << i << "), " << (line++ % 3 == 0 ? "\n" : "");
        s(20);
      }
      s(100);
      cout << "];\n";
      cout << "main.resetI();main.setHighlight(0);\n";
      cout << "outputVel.inputData([" << flush;
      s(100);
      for(auto i : realTime.outSpeeds){
        cout << i << ", " << (line++ % 3 == 0 ? "\n" : "");
        s(20);
      }
      s(100);
      cout << "]);\n";
      cout << "encVel.inputData([";
      for(auto i : realTime.encSpeeds){
        cout << i << ", " << (line++ % 3 == 0 ? "\n" : "");
        s(20);
      }
      s(100);
      cout << "]);\n";
      cout << "targetVel.inputData([";
      for(auto i : realTime.targSpeeds){
        cout << i << ", " << (line++ % 3 == 0 ? "\n" : "");
        s(10);
      }
      s(100);
      cout << "]);\n";
      cout << "slaveP.inputData([";
      for(auto i : realTime.sp){
        cout << i << ", " << (line++ % 3 == 0 ? "\n" : "");
        s(10);
      }
      s(100);
      cout << "]);\n";
      cout << "slaveD.inputData([";
      for(auto i : realTime.sd){
        cout << i << ", " << (line++ % 3 == 0 ? "\n" : "");
        s(10);
      }
      s(100);
      cout << "]);\n";
      cout << "ctrlP.inputData([";
      for(auto i : realTime.cp){
        cout << i << ", " << (line++ % 3 == 0 ? "\n" : "");
        s(10);
      }
      s(100);
      cout << "]);\n";
      cout << "ctrlD.inputData([";
      for(auto i : realTime.cd){
        cout << i << ", " << (line++ % 3 == 0 ? "\n" : "");
        s(10);
      }
      s(100);
      cout << "]);\n";
      cout << "main.xRange = [48, -48]; main.yRange = [60, -60]; \nctrlD.customizeRange();\nctrlP.customizeRange();\nslaveP.customizeRange();\nslaveD.customizeRange();\ntargetVel.customizeRange();\nencVel.customizeRange();\noutputVel.customizeRange();";
      cout << endl;
    #endif
    // cout << botAngle() << endl;
    continueFwdDrive = true;
    

  }
public: // Path following implementations
  virtual void followPath(VectorArr arr){

    followPath(arr, false);

  }
  virtual void backwardsFollow(VectorArr arr){

    followPath(arr, true);

  }
  virtual void backInto(PVector pos){
    
    backwardsFollow({ pos });

  }
  virtual void driveFwd(double maxDist){
    PVector end = PVector(0.0, maxDist);
    end.rotate(botAngle());
    followPath({ end });
  }

  virtual void stopDriveFwd(){
    continueFwdDrive = false;
    hardBrake();
  }
public: // Functions that just move the wheels  

  virtual void moveLeft(int speed){
    Left.spinVolt(fwd, speed);
  }
  virtual void moveRight(int speed){
    Right.spinVolt(fwd, speed);
  }
  virtual void turnLeft(int speed = 100){
    Right.spinVolt(fwd, speed);
    Left.spinVolt(reverse, speed);
  }
  virtual void turnRight(int speed = 100){
    Left.spinVolt(fwd, speed);
    Right.spinVolt(reverse, speed);
  }
  virtual void coastBrake(){
    Left.stop(coast);
    Right.stop(coast);
  }
  
};
// typedef BasicWheelController::PurePursuitController PursuitController;
class OmniWheelController : public BasicWheelController {
public: // Import variables + add constructor
  OmniWheelController(vector<Ref<motor>> BL, vector<Ref<motor>> BR, GPS_Share& s) :
    BasicWheelController(BL, BR, s)
    //positioner(positioner)
  {
    
  }
  
  OmniWheelController(motor& BL, motor& BR, GPS_Share& s) :
    BasicWheelController({BL}, {BR}, s)
  {
    
  }
};

class Omni_4Controller : public BasicWheelController{
public: // Import variables, functions + add constructor

  Omni_4Controller(vector<Ref<motor>> BL, vector<Ref<motor>> BR, GPS_Share& s) :
    BasicWheelController(BL, BR, s)
    //positioner(positioner)
  {
    
  }
  Omni_4Controller(motor& BL, motor& BR, motor& FL, motor& FR, GPS_Share& s) :
    BasicWheelController({BL, FL}, {BR, FR}, s)
  {
    
  }
};

class MechWheelController : public Omni_4Controller{
public: // Import variables + constructor
  motor* BL;
  motor* BR;
  motor* FL;
  motor* FR;
  bool isOmniDir = true;
  //MechWheelController(motor& BL, motor& BR, posTp&, gps&) = delete;
  MechWheelController(motor& BL, motor& BR, motor& FL, motor& FR, GPS_Share& s) :
    Omni_4Controller(BL, BR, FL, FR, s) {
    this->BL = &BL;
    this->BR = &BR;
    this->FL = &FL;
    this->FR = &FR;
  }

public: // EPIC PID Things
  void moveAt(double angle, double speed, double turnSpeed){
    double Y1 = cos(angle * DEG_TO_RAD) * speed;
    double Y2 = cos(angle * DEG_TO_RAD) * speed;
    double X1 = sin(angle * DEG_TO_RAD) * speed;
    double X2 = sin(angle * DEG_TO_RAD) * speed;
    double FLS = Y1 + X1;
    double BLS = Y1 - X1;
    double FRS = Y2 - X2;
    double BRS = Y2 + X2;
    FLS += turnSpeed;
    BLS += turnSpeed;
    FRS -= turnSpeed;
    BRS -= turnSpeed;
    //Spin da motors
    BL->spin(vex::forward, BLS, velocityUnits::pct);
    BR->spin(vex::forward, BRS, velocityUnits::pct);
    FL->spin(vex::forward, FLS, velocityUnits::pct);
    FR->spin(vex::forward, FRS, velocityUnits::pct);
  }
  void followPath(VectorArr arr, double targetAngle){
    //Add the bot position to the front so that it has a smoother start
    arr.push_front(botPos());
    VectorArr bezier = bezierCurve(arr);
    this->drawArr = true;
    this->path = bezier;
    PVector pursuit = bezier.first();
    int i = 0;
    double lastDist = 12 * 2 * 24;
    timer t = timer();
    int time = 0;
    int speedLimit = 70;
    double angle = botAngle();
    int bezierIndex = 0;
    double normAngle = posNeg180(angle - posNeg180(botAngle()));
    double sign = 1.0;
    double startSign = normAngle > 0.0 ? 1.0 : -1.0;
    setOldDistFns();
    double lastTurnAngle = 0;
    int g = 0;
    while(pursuit != bezier.last() || botPos().dist2D(pursuit) > 2.0){
      //Maintain distance of 8 inches to Pure Pursuit target
      while(pursuit.dist2D(botPos()) < 8.0 && i < bezier.size()){
        
        pursuit = bezier[bezierIndex];
        
        ++bezierIndex;
      }
      double dist = botPos().dist2D(pursuit);
      if(abs(dist - lastDist) <= 0.4 && t.time(timeUnits::msec) > 700 && time++ * 10 > 700){
        cout << "Stop Exit" << endl;
        break;
      }
      double speed = -ctrl.getVal(dist);// * (bezier.size() - i) * 0.1;
      useDistFns(botPos().dist2D(bezier.last()));
      //Impose speed limit
      if(speed > speedLimit){
        speed = speedLimit;
      }
      //Angle between bot position and pursuit target
      angle = baseAngle(botPos().angleTo(pursuit));
      //The angle that it needs to travel at
      double normAngle = posNeg180(angle - botAngle());

      //The angle that it needs to turn to
      double normAngle2 = posNeg180(targetAngle - botAngle());

      double newSign = normAngle2 > 0.0 ? 1.0 : -1.0;
      // if(newSign / startSign == -1.0){
      //   slaveCtrl.setTarget(0.0);
      //   startSign = newSign;
      // }
      double turnSpeed = -turnCtrl.getVal(normAngle2);
      if(lastTurnAngle != 0 && abs(normAngle2) - 0.01 > abs(lastTurnAngle)){
        //turnSpeed *= 2.0;
      }
      
      lastTurnAngle = normAngle2;
      moveAt(normAngle, speed, turnSpeed / 4.0);
      s(1);

    }
    hardBrake();
    s(1500);
    this->drawArr = false;
    cout << botPos() << ", " << bezier.last() << endl;
    cout << botAngle() << endl;
    continueFwdDrive = true;

  }
  void driveTo(PVector pos, double finalAngle){
    followPath({ pos }, finalAngle);
  }
  void driveTo(double x, double y, double finalAngle){
    driveTo(PVector(x, y), finalAngle);
  }
  void backInto(double x, double y, double finalAngle){
    backInto(PVector(x, y), finalAngle);
  }
  void backInto(PVector pos, double finalAngle){
    backwardsFollow({ pos }, finalAngle);
  }
  virtual void backwardsFollow(VectorArr arr, double targetAngle){
    arr.push_front(botPos());
    VectorArr bezier = bezierCurve(arr);
    this->drawArr = true;
    this->path = bezier;
    PVector pursuit = bezier.first();
    int i = 0;
    double lastDist = 12 * 2 * 24;
    timer t = timer();
    int time = 0;
    int speedLimit = 70;
    double angle = botAngle();
    double normAngle = posNeg180(angle - posNeg180(botAngle()));
    double sign = 1.0;
    double startSign = normAngle > 0.0 ? 1.0 : -1.0;
    setOldDistFns();
    double lastTurnAngle = 0;
    int g = 0;
    int bezierIndex = 0;
    while(pursuit != bezier.last() || botPos().dist2D(pursuit) > 2.0){
      //Maintain distance of 8 inches to Pure Pursuit target
      while(pursuit.dist2D(botPos()) < 8.0 && i < bezier.size()){
        
        pursuit = bezier[bezierIndex];
        
        ++bezierIndex;
      }
      double dist = botPos().dist2D(pursuit);
      if(abs(dist - lastDist) <= 0.4 && t.time(timeUnits::msec) > 700 && time++ * 10 > 700){
        cout << "Stop Exit" << endl;
        break;
      }
      double speed = -ctrl.getVal(dist);// * (bezier.size() - i) * 0.1;
      useDistFns(botPos().dist2D(bezier.last()));
      //Impose speed limit
      if(speed > speedLimit){
        speed = speedLimit;
      }
      //Angle between bot position and pursuit target
      angle = baseAngle(botPos().angleTo(pursuit));
      //The angle that it needs to travel at
      double normAngle = posNeg180(angle - botAngle());

      //The angle that it needs to turn to
      double normAngle2 = posNeg180(targetAngle - botAngle());

      double newSign = normAngle2 > 0.0 ? 1.0 : -1.0;
      // if(newSign / startSign == -1.0){
      //   slaveCtrl.setTarget(0.0);
      //   startSign = newSign;
      // }
      double turnSpeed = -turnCtrl.getVal(normAngle2);
      if(lastTurnAngle != 0 && abs(normAngle2) - 0.01 > abs(lastTurnAngle)){
        //turnSpeed *= 2.0;
      }
      
      lastTurnAngle = normAngle2;
      moveAt(normAngle, -speed, turnSpeed / 4.0);
      s(1);

    }
    hardBrake();
    s(1500);
    this->drawArr = false;
    cout << botPos() << ", " << bezier.last() << endl;
    cout << botAngle() << endl;
    continueFwdDrive = true;


    
  }
};

class Omni_6Controller : public BasicWheelController{
public: // Import variables + constructor
  using BasicWheelController::botPos;
  bool isOmniDir = true;
  //Omni_6Controller(motor& BL, motor& BR, posTp&, gps&) = delete;
  //Omni_6Controller(motor& BL, motor& BR, motor&, motor&, posTp&, gps&) = delete;
  Omni_6Controller(vector<Ref<motor>> BL, vector<Ref<motor>> BR, GPS_Share& s) :
    BasicWheelController(BL, BR, s)
    //positioner(positioner)
  {
    
  }
  Omni_6Controller(motor& BL, motor& BR, motor& FL, motor& FR, motor& ML, motor& MR, GPS_Share& s) :
    BasicWheelController({BL, FL, ML}, {BR, FR, MR}, s)
  {
    
  }
};