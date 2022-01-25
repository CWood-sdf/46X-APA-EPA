#include "SpecialFunctions.h"


//Update the bot angle in all three dimensions
void updateBotAngle(bool add){
  //
  static double lastAngle = angler.orientation(orientationType::yaw, rotationUnits::deg);
  
  static double lastAngleYZ = angler.orientation(orientationType::pitch, rotationUnits::deg);
  static double lastAngleXZ = angler.orientation(orientationType::roll, rotationUnits::deg);
  //
  double err = 360.0 / 353.0;
  double errNeg = 360.0 / 354.0;

  double errYZ = 1.0;
  double errXZ = 1.0;
  static int i = 0;

  //
  double angle = angler.orientation(orientationType::yaw, rotationUnits::deg);
  double gain = posNeg180(angle - lastAngle);
  gain *= (gain > 0 ? err : errNeg);
  avgBotAngle = botAngles.x + gain / 2.0;
  botAngles.x += gain;
  deltaBotAngle = gain;
  lastAngle = angle;

  angle = angler.orientation(orientationType::pitch, rotationUnits::deg);
  gain = posNeg180(angle - lastAngleYZ);
  botAngles.y += gain * errYZ;
  lastAngleYZ = angle;

  angle = angler.orientation(orientationType::roll, rotationUnits::deg);
  gain = posNeg180(angle - lastAngleXZ);
  botAngles.z += gain * errXZ;
  lastAngleXZ = angle;

  tiltAngle = botAngles.z - startTilt;
  //Start off with 1 in y to mimic field setup
  // PVector angleObtainer = PVector(0.0, 10.0, 0.0);
  // //Rotate by basic field angle

  
  // angleObtainer.rotateXY(botAngles.x + 90.0);
  // angleObtainer.rotateYZ(botAngles.y);
  // angleObtainer.rotateXZ(botAngles.z);
  // tiltAngle = angleObtainer.headingYZ();
  if(i++ == 20){
    //cout << glblBotAngle << endl;
    i = 0;
  }

}
void microWait(uint time);
// LinkedList<pair<PVector, double>> gpsPostns;
// mutex gpsPosShare;
// LinkedList<pair<PVector, double>> odomPostns; 
// mutex odomPosShare;
// bool useOut = false;
// thread addToStream = thread([](){
//   ofstream sdCard ("DEBUG1.txt");
//   if(!sdCard.is_open()){
//     useOut = true;
//     sdCard << "var arr = [";
//     while(1){
//       if(gpsPostns.size() > 0 && odomPostns.size() > 0){
//         sdCard << flush;
//         while(!gpsPosShare.try_lock()){

//         }
//         gpsPosShare.lock();
//         sdCard << "[[new PVector(" << gpsPostns.getBase()->first.x << ", "
//           << gpsPostns.getBase()->first.y << "), " << gpsPostns.getBase()->second << "], ";
//         gpsPostns.popBase();
//         gpsPosShare.unlock();
//         while(!odomPosShare.try_lock()){

//         }
//         odomPosShare.lock();
//         sdCard << "[new PVector(" << odomPostns.getBase()->first.x << ", "
//           << odomPostns.getBase()->first.y << "), " << odomPostns.getBase()->second << "]],";
//         odomPostns.popBase();
//         odomPosShare.unlock();
//         s(1);
//       }
//     }
//   }
// });
void trackPos (){
  int times = 0;
  while(1){
    //Pause for 2 milliseconds
    int waitTime = 2000;
    positioner.update(times == 10, waitTime);
    
    microWait(waitTime);
    //Print the position every 5 iterations (0.01 s)
    // if(times++ == 5){
    //   while(!gpsPosShare.try_lock()){

    //   }
    //   gpsPosShare.lock();
    //   gpsPostns.push_back({{GPS.xPosition(), GPS.yPosition()}, GPS.heading()});
    //   gpsPosShare.unlock();
    //   while(!odomPosShare.try_lock()){

    //   }
    //   odomPosShare.lock();
    //   odomPostns.push_back({positioner.getPos(), positioner.heading()});
    //   odomPosShare.unlock();
    //   times = 0;
    //   //cout << wc.botPos() << endl;
    // }
  }
}
void programWrite(bool start = true);

//Allows a program reset at a new spot if there is limited tile space
void waitForReset(){
  PVector pos = wc.botPos();
  cout << "start" << endl;
  double startAng = wc.botAngle();
  while(!Greg.ButtonA.pressing()){
    s(100);
    
  }
  cout << "switch" << endl;
  while(Greg.ButtonA.pressing()){
    s(500);
    cout << "Org: " << startAng << " current: " << wc.botAngle() << endl;
  }
  positioner.resetPos(pos);
  cout << "done" << endl;
}

void microWait(uint time){
  auto startTime = Brain.Timer.systemHighResolution();
  //This volatile int is just there to prevent annoying compiler optimizations
  //That mess up the timing of everything
  volatile int i = 0;
  while(Brain.Timer.systemHighResolution() - startTime < time || i < -10){
    i++;
  }
}
//Suppress bothersome unused variable warning
//#pragma GCC diagnostic ignored "-Wunused-variable"


// void updateCount(){
//   //Update the line trackers
//   while(1){

//     microWait(10);
//   }
// }

void executeThreads(){
  //Execute autonomous functions
  while(1){
    if(threadFns.size() > 0){
      threadFns.front()();
      threadFns.pop_front();
    }
    else {
      this_thread::sleep_for(100);
    }
  }
}
void updateSharePos(){
  while(1){
    share.update();
  }
}