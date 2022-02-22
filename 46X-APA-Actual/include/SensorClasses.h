#include "vex.h"
#include "Odometry/EPA_Wheel_Control.h"
#include <list>

#define TO_4(x) x*x*x*x
#define TO_3(x) x*x*x
#define TO_2(x) x*x

//SensorClasses.h -- Use this file to make helper classes for sensors
extern GPS_Share share;
struct KillThread {
  static inline vector<thread*> instances;
  thread* t;
  
  KillThread(void(*callback)(void) ) : t(new thread(callback)) {
    instances.push_back(t);
  }
  KillThread(thread&& t) : t(&t){
    instances.push_back(this->t);
  }
  KillThread& operator=(thread&& t){
    this->t = &t;
    instances.push_back(&t);
    return *this;
  }
  static void killAll(){
    cout << "KAT" << endl;
    for(auto i : instances){
      i->interrupt();
      i->detach();
    }
    instances.clear();
  }
};
class Distance {
  static constexpr double deltaT = 0.005; // Update loop every 5ms
  double sensorSigma = 15; // The pathetic +/- 15mm accuracy
  double sigmaA = 10; // Random variance in accelleration
  Matrix<double, 3, 1> estimate = {{0}, {0}, {0}};
  Matrix<double, 3, 1> prediction = {{0}, {0}, {0}};
  Matrix<double, 3, 1> Gn = {{deltaT}, {1.0}, {1.0 / deltaT}};
  Matrix<double, 3, 3> F;

  Matrix<double, 3, 3> Q;
  //{ sensorSigma * sensorSigma }
  Matrix<double, 1, 1> Rn;
  // { 1, 0, 0}
  Matrix<double, 1, 3> H;
  
  //Init
  Matrix<double, 3, 3> uncertainty;
  Matrix<double, 3, 3> predictionUncertainty;
  Matrix<double, 3, 3> Id3 = Matrix<double, 3, 3>::Identity();
  vex::distance sensor;
  bool lastOff = false;
  public:
  Distance(const int32_t& port) : sensor(vex::distance(port)){
    //Construct process noise matrix
    Q = {
      {TO_4(deltaT) / 4, TO_3(deltaT) / 2, deltaT*deltaT / 2},
      {TO_3(deltaT) / 2,  deltaT * deltaT,            deltaT},
      {deltaT*deltaT/ 2,           deltaT,                 1}
    };
    Q *= sigmaA * sigmaA;

    //Construct state update equations
    F = {
      {1, deltaT, 0.5 * deltaT * deltaT},
      {0,      1,                deltaT},
      {0,      0,                     1}
    };

    H = {{1,0,0}};
  }
  double sensorVariance(){
    if(sensor.objectDistance(mm) > 200){
      //Over 200 mm, accuracy is +/- 5%
      return sensor.objectDistance(mm) * sensor.objectDistance(mm) * 0.05 * 0.05;
    }
    else {
      return sensorSigma * sensorSigma;
    }
  }
  void initLoop(){
    prediction = estimate = {{sensor.objectDistance(mm)}, {0}, {0}};
    uncertainty = {
      {sensorVariance(), 0,   0},
      {               0, 100, 0},
      {               0, 0, 200}
    };
    predictionUncertainty = uncertainty;
    
  }
  void test(){
    if(sensor.isObjectDetected() && sensor.objectDistance(inches) < 20){
      if(!lastOff){
        initLoop();
      }
      TOP:
      Rn[0][0] = sensorVariance();
      auto Kn = predictionUncertainty * H.transpose() * (H * predictionUncertainty * H.transpose() + Rn).inverse();
      Matrix<double, 1, 1> zn;
      zn[0][0] = sensor.objectDistance(mm);
      //Reset filter there is a high velocity maneuver
      if(abs(sensor.objectDistance(mm) - prediction[0][0]) > 80 /*(about 2 inches)*/){
        double prevEst = prediction[0][0];
        initLoop();

        //Give the new filter the best guess for velocity and accelleration
        estimate[1][0] = prediction[1][0] = (prevEst - zn[0][0]) / deltaT;
        estimate[2][0] = prediction[2][0] = 1.0 / 2.0 * (prevEst - zn[0][0]) / (deltaT * deltaT);
        goto TOP;
      }
      estimate = prediction + Kn * (zn - H * prediction);
      uncertainty = (Id3 - Kn * H) * predictionUncertainty * (Id3 - Kn * H).transpose() + Kn * Rn * Kn.transpose();
      //Use robot velocity as control input, bc we are assuming that the goals aren't moving during auton
      //Making the assumption that we are moving roughly towards the goal +/- 5 degrees
      //Could have a negative process noise for auton (because the goals are being taken away)
      double ctrlInput = -share.velocity();
      prediction = F * estimate + Gn * ctrlInput;
      predictionUncertainty = F * uncertainty * F.transpose() + Q;
      lastOff = true;
    }
    else {
      //Force re-initialization next time object detected
      lastOff = false;
    }
  }
  void endFiltering(){
    lastOff = false;
  }
  double rawDist(distanceUnits d){
    return sensor.objectDistance(d);
  }
  double objectDistance(distanceUnits d){
    if(!lastOff){
      return 0;
    }
    if(d == distanceUnits::in){
      return systemEstimate() / 25.4;
    }
    if(d == distanceUnits::cm){
      return systemEstimate() / 10;
    }
    else {
      return systemEstimate();
    }
  }
  void sleep(){
    s(deltaT * 1000.0);
  }
  bool isObjectDetected(){
    return lastOff;
  }
  double systemEstimate(){
    if(!lastOff){
      return 0;
    }
    return estimate(0, 0);
  }
  bool installed(){
    return sensor.installed();
  }
};
typedef triport::port port;


class LineCounter {
  
  static const KillThread updater;
  friend class LineGroup;
  line* sensor;
  bool isActive = false;
  int threshold = startThreshold;
  static const int startThreshold = 15;
  //May have to increase lowThreshold to prevent 2 balls in a row from counting as one
  static const int lowThreshold = 10;
  bool wasActiveLast = false;
  unsigned int countOut = 0;
  unsigned int countIn = 0;
  bool fHit = false;
public:
  static vector<LineCounter*> instances;
  LineCounter(line& se) : sensor(&se){
    ADD_THIS;
  }
  LineCounter(LineCounter& counter) : sensor(counter.sensor){
    isActive = counter.isActive;
    threshold = counter.threshold;
    ADD_THIS;
  }
  
  LineCounter(triport::port& p) : sensor(new line(p)) {
    ADD_THIS;
  }
  bool firstHit(){
    if(fHit){
      fHit = false;
      return true;
    }
    return false;
  }
  bool active(){
    return isActive;
  }
  void update(bool out = false){
    if(out)
    cout << sensor->reflectivity() << endl;
    
    bool isActive = pressing();
    if(wasActiveLast){
      if(!isActive){
        countOut++;
      }
    } else {
      if(isActive){
        countIn++;
        fHit = true;
      }
    }
    wasActiveLast = isActive;
  }
  void reset(){
    countIn = 0;
    countOut = 0;
    if(active()){
      countIn = 1;
      countOut = 0;
    }
  }
  int getCountOut(){
    return countOut;
  }
  int getCountIn(){
    return countIn;
  }
  bool pressing(){
    
    isActive = threshold <= sensor->reflectivity();
    if(isActive){
      threshold = lowThreshold;
    } else  {
      threshold = startThreshold;
    }
    return isActive;
  }
};
void microWait(uint time);
vector<LineCounter*> LineCounter::instances = {};
const KillThread LineCounter::updater = thread([](){
  if(LineCounter::instances.size() == 0){
    cout << "No Line Counter instances exist, exiting thread" << endl;\
    return;
  }
  while(1){
    for(auto t : LineCounter::instances){
      t->update();
    }
    microWait(2000);
  }
});


class PotDial {
  const int ticks;
  pot* sensor;
  double baseVal = 0.0;
  int range = 250;
  PotDial(int tickAmnt) : ticks(tickAmnt) {

  }
  PotDial(int tickAmnt, int rng) : PotDial(tickAmnt) {
    range = rng;
  }
  PotDial(int tickAmnt, int rng, double baseVal) : PotDial(tickAmnt, rng) {
    this->baseVal = baseVal;
  }
public:
  template<class StorageType>
  class Controller {
    friend class PotDial;
    vector<PotDial*> boundDials;
    vector<int> possibleTicks;
    map<vector<int>, StorageType> returnVals;
  public:
    template<class ... Args>
    Controller(Args&... potDials){
      vector<Ref<PotDial>> dials = { potDials... };
      for(auto& d : dials){
        d.val->addController(*this);
      }
    }
    Controller(){

    }
    void addVal(vector<int> inputs, StorageType out){
      if(inputs.size() > possibleTicks.size()){
        cout << "Invalid array size (too big)" << endl;
      }
      else if(inputs.size() < possibleTicks.size()){
        cout << "Invalid array size (too small)" << endl;
      }
      else {
        returnVals[inputs] = out;
      }
    }
    StorageType getVal(vector<int> input){
      if(input.size() > possibleTicks.size()){
        cout << "Invalid array size (too big)" << endl;
      }
      else if(input.size() < possibleTicks.size()){
        cout << "Invalid array size (too small)" << endl;
      }
      return returnVals[input];
    }
    StorageType getVal(){
      vector<int> vals = {};
      for(auto l : boundDials){
        vals.push_back(l->getAmnt());
      }
      return getVal(vals);
    }
  };
public:
  template<typename ... Args>
  PotDial(pot& sensor, Args... otherArgs) : PotDial(otherArgs...){
    this->sensor = &sensor;
  }
  template<typename... Args>
  PotDial(port& p, Args... args) : PotDial(args...){
    sensor = new pot(p);
  }
  template<class T>
  PotDial& addController(PotDial::Controller<T>& ctrllr){
    ctrllr.possibleTicks.push_back(ticks);
    ctrllr.boundDials.push_back(this);
    return *this;
  }
  // template<typename ... Args>
  // PotDial(vex::triport::port& p, Args... otherArgs) : PotDial(otherArgs...) {
  //   sensor = new pot(p);
  // }
  int getAmnt(){
    double angle = sensor->angle(deg);
    if(angle <= baseVal){
      return 1;
    } else if(angle >= baseVal + range){
      return ticks;
    } else {
      angle -= baseVal;
      angle /= (double)range;
      angle *= (double)ticks - 1;
      return round(angle) + 1;
    }
  }
};
