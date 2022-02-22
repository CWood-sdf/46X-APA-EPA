#include "SpecialFunctions.h"


//Update the bot angle in all three dimensions
void updateBotAngle(bool add){
  //
  bool gpsFirst = false;
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
  if(!share.gpsBad()/* && !gpsFirst*/){
    // cout << "Share not bad: " << share.heading() << endl;
    botAngles.x = share.heading();
    gpsFirst = true;
  }
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

void trackPos (){
  int times = 0;
  while(1){
    //Pause for 2 milliseconds
    int waitTime = 2000;
    updateBotAngle();
    positioner.update(times == 10, waitTime);
    // cout << "Yeet" << endl;
    s(2);
    // microWait(waitTime);
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