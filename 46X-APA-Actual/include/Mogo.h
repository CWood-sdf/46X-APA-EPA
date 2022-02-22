#include "SensorClasses.h"

#define _sin(t) (t - t*t*t/6 + t*t*t*t*t/120 - t*t*t*t*t*t*t/5040 + t*t*t*t*t*t*t*t*t/362880)
#define _cos(t) (1 - t*t/2 + t*t*t*t/24 - t*t*t*t*t*t/720 + t*t*t*t*t*t*t*t/40320)
#define _tan(t) (_sin(t)/_cos(t))
class VisionOdom {
  vision* sensor;
  double mountHeight, mountAngle, mountRotation;
  PVector relPos;
  static inline BasicWheelController* wc;
  static constexpr double 
    screenWidth = 310,
    screenHeight = 210,
    widthAngle = 56 * DEG_TO_RAD,
    heightAngle = 46 * DEG_TO_RAD, 
    backDist = 0.5 * screenWidth / _tan(widthAngle / 2.0);
  PVector estimatePos(vision::object& object){
    cout << object.centerX << ", " << object.centerY << endl;
    PVector ret;
    cout << (screenHeight / 2.0 - object.centerY) << endl;
    double angleY = atan(-(screenHeight / 2.0 - object.centerY) / backDist);
    double angleX = atan((screenWidth / 2.0 - object.centerX) / backDist);
    cout << angleY * RAD_TO_DEG << ", " << angleX * RAD_TO_DEG << endl;
    double internalAngleY = M_PI_2 - mountAngle - angleY;
    cout << internalAngleY * RAD_TO_DEG << endl;
    double dy = tan(internalAngleY) * mountHeight;
    double h2 = sqrt(dy*dy + mountHeight * mountHeight);
    double dx = h2 * tan(angleX);
    ret = PVector(dx, dy);
    PVector rCopy = relPos;
    
    //ret
      //.rotate(mountRotation + wc->botAngle())
      //.add(wc->botPos())
      //.add(rCopy.rotate(wc->botAngle()));
    return ret;
  }
public:
  
  VisionOdom(vision& sensor, PVector relPos, double height, double mAngle, double mRotation){
    this->relPos = relPos;
    this->sensor = &sensor;
    mountHeight = height;
    mountAngle = mAngle * DEG_TO_RAD;
    mountRotation = mRotation;
    BasicWheelController::requestInstance(wc);
  }
  template<class Sig>
  PVector estimatePos(Sig& sig){
    sensor->takeSnapshot(sig);
    return estimatePos(sensor->largestObject);
  }
  template<class Sig>
  vector<PVector> estimateAllPos(Sig& sig){
    sensor->takeSnapshot(sig);
    vector<PVector> ret;
    for(int i = 0; i < sensor->objects.getLength(); i++){
      if(sensor->objects[i].exists){
        ret.push_back(estimatePos(sensor->objects[i]));
      }
    }
    return ret;
  }
  template<class Sig>
  PVector closest(Sig& sig){
    sensor->takeSnapshot(sig);
    vector<PVector> arr;
    for(int i = 0; i < sensor->objects.getLength(); i++){
      if(sensor->objects[i].exists){
        arr.push_back(estimatePos(sensor->objects[i]));
      }
    }
    PVector c;
    double dist = 100000;
    for(auto i : arr){
      if(i.dist2D(wc->botPos()) < dist){
        dist = i.dist2D(wc->botPos());
        c = i;
      }
    }
    return c;
  }
};
class Mogo {
  static GPS_Share* share;
  bool attachedBack = false;
  bool attachedFront = false;
  PVector estPos;
  //The place the robot would have to travel to to go to a mogo
  PVector closest(double dist, bool back){
    PVector botPos = share->position();
    PVector pos = estPos;
    double angle = pos.angleTo(botPos);
    PVector half = PVector(dist, 0);
    half.rotate(angle - half.heading2D()).add(pos);
    return half;
  }
  //The place the robot would have to travel to to go to a mogo
  PVector setNew(double dist, bool back){
    PVector botPos = share->position();
    double angle = share->heading() + 180.0 * back;
    PVector newPos = PVector(dist, 0);
    newPos.rotate(angle - newPos.heading2D()).add(botPos);
    estPos = newPos;
    return newPos;
  }
public:
  Mogo(PVector est){
    estPos = est;
  }
  Mogo(GPS_Share& g, PVector est) : Mogo(est){
    init(g);
  }
  bool isAttachedBack(){
    return attachedBack;
  }
  bool isAttachedFront(){
    return attachedFront;
  }
  void attachFront(){
    attachedFront = true;
  }
  void detachBack(){
    setNew(21, true);
    attachedBack = false;
  }
  void attachBack(){
    attachedBack = true;
  }
  void detachFront(){
    setNew(21, false);
    attachedFront = false;
  }
  PVector closestBack(){
    
    return closest(14, true);
  }
  PVector closestFront(){ // Same as closestBack() for now, but dist might change
    return closest(14, false);
  }
  
  static void init(GPS_Share& g){
    Mogo::share = &g;
  }
};