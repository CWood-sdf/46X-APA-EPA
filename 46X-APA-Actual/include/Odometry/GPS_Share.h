#include "Odometry/EPA_Tracker.h"

//template <class Odom>
class GPS_Share {
  Positioner& odom;
  gps& GPS;
  LinkedList<FieldCoord> gpsReadings;
  LinkedList<FieldCoord> odomReadings;
  FieldCoord pos;
  FieldCoord lastOdom;
  double speed = 0.0;
  bool lastBad = false;
  bool isFirstBad = false;
  static const int sleepTime = 4;
  static const int badTime = 200;
  bool firstBad(){
    if(isFirstBad){
      isFirstBad = false;
      return true;
    }
    return false;
  }
  bool readingBad(){
    FieldCoord startPos = gpsReadings.getBase();
    if(!lastBad){
      isFirstBad = true;
      lastBad = true;
    }
    else {
      lastBad = false;
    }
    for(auto p : gpsReadings){
      if(p != startPos){
        return false;
      }
    }
    return true;
  }
public:
  GPS_Share(Positioner& o, gps& g) : odom(o), GPS(g) {

  }
  
  FieldCoord& fullPos(){
    return pos;
  }
  PVector& position(){
    return pos.pos;
  }
  double heading(){
    return pos.angle;
  }
  double velocity(){
    return speed;
  }
  bool gpsBad(){
    return GPS.quality() != 100;
  }
  void update(){
    static int g = 0;

    FieldCoord currentOdom = odom.fullPos();
    
    FieldCoord deltaOdom = currentOdom - lastOdom;
    lastOdom = currentOdom;
    speed = deltaOdom.pos.mag() / (double)sleepTime * 1000.0;
    FieldCoord gpsCoord = FieldCoord(PVector(GPS.xPosition(inches), GPS.yPosition(inches)), GPS.heading());
    gpsReadings.push_back(gpsCoord);
    odomReadings.push_back(currentOdom);

    while(odomReadings.size() > badTime / sleepTime){
      odomReadings.popBase();
    }
    while(gpsReadings.size() > badTime / sleepTime){
      gpsReadings.popBase();
    }
    //cout << GPS.quality() << endl;
    if(gpsBad()){
      //   FieldCoord firstOdom = odomReadings.getBase();
      //   deltaOdom = currentOdom - firstOdom;

      // }
      //cout << "Using Odom" << endl;
      pos.pos += deltaOdom.pos;
      pos.angle += deltaOdom.angle;
    }
    else {
      pos = gpsCoord;
    }
    g++;
    if(g == 26){
      g = 0;
    }
    s(sleepTime);
  }
};