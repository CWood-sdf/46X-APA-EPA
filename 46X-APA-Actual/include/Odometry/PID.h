#include "vex.h"
//PID.h -- use this file for a PID class
class PID;
//Stores numbers to add to a PID's KVals
class PidAdder {
  friend class PID;
  double p, i, d;
public:
  //Constructor
  //PidAdder(0.1, 0.2, 0.1);
  PidAdder(double p, double i, double d){
    this->p = p;
    this->i = i;
    this->d = d;
  }
  PidAdder(){
    
  }
};
//Some nice simple math
class PID {
  public:
  //The values to multiply the K values by
  double p = 0.0, i = 0.0, d = 0.0;
  vector<double> dVals;
  //A simple struct that stores the multiplication values
  struct KVals {
    //The variables
    double p, i, d;
    //Set the values
    KVals& operator=(KVals& vals){
      p = vals.p;
      i = vals.i;
      d = vals.d;
      return *this;
    }
    //Constructor with initializer_list
    //KVals({0.1, 0.2, 0.3});
    KVals(std::initializer_list<double> vals){
      double* ths = (double*) this;
      for(double val : vals){
        *ths = val;
        ++ths;
      }
    }
    //Add a similar KVal class (Like PidAdder) to the values
    template<class T>
    KVals& operator+=(T a){
      p += a.p;
      i += a.i;
      d += a.d;
      return *this;
    }
    //Subtract
    template<class T>
    KVals& operator-=(T a){
      p -= a.p;
      i -= a.i;
      d -= a.d;
      return *this;
    }
    
  };
  //Stores the multiplication values under k
  KVals k;

  //Variables
  double target = 0.0, error = 0.0, lastError = 0.0, iCap, iGrowth, iZero;
public:
  //Constructors
  PID(KVals vals, double iCap = 0.0, double iGrowthRange = 0.0, double iZeroRange = 0.0) : k(vals){
    this->iCap = iCap;
    iGrowth = iGrowthRange;
    iZero = iZeroRange;
  }
  PID(double p, double i, double d, double iCap = 0.0, double iGrowthRange = 0.0, double iZeroRange = 0.0) : PID(KVals({ p, i, d })) {
    this->iCap = iCap;
    iGrowth = iGrowthRange;
    iZero = iZeroRange;
  }
  //Get the error
  double getError(){
    return error;
  }
  //Clear out the previous PID usage
  void resetVals(){
    error = 0.0;
    dVals.clear();
    p = i = d = 0.0;
  }
  //Set the target value of the PID
  void setTarget(double val){
    resetVals();
    target = val;
  }
  //Apply the error
  void incVals(double sensorVal){
    lastError = error;
    error = target - sensorVal;
    p = error;
    if(abs(error) <= iGrowth && iGrowth != 0.0){
      i += error;
    }
    if(i > iCap && iCap != 0.0){
      i = iCap;
    }
    if(abs(error) <= iZero && iZero != 0.0){
      i = 0.0;
    }
    d = error - lastError;
    dVals.push_back(d);
  }
  //Get the speed value given that error has already been applied
  double getVal(){
    double pInc = k.p * p;
    double iInc = k.i * this->i;
    double dInc = k.d * this->d;
    return pInc + iInc + dInc;
  }
  //Apply error, then return getVal()
  double getVal(double sensorVal){
    
    this->incVals(sensorVal);
    return getVal();
  }
  //Get the previous DVals
  vector<double> getDVals(){
    return dVals;
  }
  //Add a PidAdder
  PID& operator+= (PidAdder a){
    k += a;
    return *this;
  }
  //Subtract a PidAdder
  PID& operator-= (PidAdder a){
    k -= a;
    return *this;
  }
};


class MotionProf;
//Stores numbers to add to a PID's KVals
class MPAdder {
  friend class MotionProf;
  double v, a, p;
public:
  //Constructor
  //PidAdder(0.1, 0.2, 0.1);
  MPAdder(double v, double a, double p){
    this->v = v;
    this->a = a;
    this->p = p;
  }
  MPAdder(){
    
  }
};
//Some nice simple math
class MotionProf {
public:
  //A simple struct that stores the multiplication values
  struct KVals {
    //The variables
    double v, a, p;
    //Set the values
    KVals& operator=(KVals& vals){
      p = vals.p;
      a = vals.a;
      v = vals.v;
      return *this;
    }
    //Constructor with initializer_list
    //KVals({0.1, 0.2, 0.3});
    KVals(std::initializer_list<double> vals){
      double* ths = (double*) this;
      for(double val : vals){
        *ths = val;
        ++ths;
      }
    }
    //Add a similar KVal class (Like PidAdder) to the values
    template<class T>
    KVals& operator+=(T a){
      p += a.p;
      this->a += a.a;
      v += a.v;
      return *this;
    }
    //Subtract
    template<class T>
    KVals& operator-=(T a){
      p -= a.p;
      this->a -= a.a;
      v -= a.v;
      return *this;
    }
    
  };
  //Stores the multiplication values under k
  KVals k;
  vector<double> targetSpeeds;

public:
  //Constructors
  MotionProf(KVals vals) : k(vals){

  }
  MotionProf(double v, double a, double p) : MotionProf(KVals({ v, a, p })) {
    
  }
  
  //Get the speed value given that error has already been applied
  double getVal(double tv, double ta, double err){

    return k.v * tv + k.a * ta + k.p * err;
  }
  void loadTargetSpeeds(vector<double> a){
    targetSpeeds = a;
  }
  void smoothTargetSpeeds(double startVel, double maxAcc, double maxdAcc){
    // for(int i = 1; i < targetSpeeds.size(); i++){
    //   double d = bezier[i].dist2D(bezier[i - 1]) / 0.567;
    //   double a = maxAcc;
    //   if(startVel < targetSpeeds[i]){
    //     startVel = targetSpeeds[i] = sqrt(startVel * startVel + 2.0 * a * d);
    //     if(startVel > targetSpeeds[i + 1]){
    //       startVel = targetSpeeds[i + 1];
    //     }
    //   }
    //   else if(startVel > targetSpeeds[i]){
    //     int startI = i;
    //     //Go backwards until reached speed
    //     do {
    //       startVel = targetSpeeds[i];
    //       i--;
    //       double a = maxdAcc;
    //       double d = bezier[i].dist2D(bezier[i + 1]) / 0.567;
    //       startVel = targetSpeeds[i] = sqrt(startVel * startVel + 2.0 * a * d);
    //     } while(startVel < targetSpeeds[i - 1]);
    //     i = startI;
    //     startVel = targetSpeeds[i];
    //   }
    // }
  }
  void smoothTargetSpeeds(double startVel, double maxAcc){
    smoothTargetSpeeds(startVel, maxAcc, maxAcc);
  }
  //Add a MPAdder
  MotionProf& operator+= (MPAdder a){
    k += a;
    return *this;
  }
  //Subtract a MPAdder
  MotionProf& operator-= (MPAdder a){
    k -= a;
    return *this;
  }
};
ostream& operator<<(ostream& cout, PID& p){
  cout << p.k.p << ", " << p.k.i << ", " << p.k.d << endl;
  return cout;
}