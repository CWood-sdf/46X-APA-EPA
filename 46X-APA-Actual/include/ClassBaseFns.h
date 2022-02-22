#include "fns.h"
//ClassBaseFns.h -- use this file for all auton functions
list<function<void()>> threadFns;

//Sleeps the current thread
void ts (int time){
  this_thread::sleep_for(time);
}
#define toStr(a)  #a

//This macro does a lot
//First it makes a prototype function with the passed name and accepting 2 arguments:
//        an int and a boolean that defaults to true
//It then defines the function with a beginning body that writes to the APA 
//      if the 'write' arg is true
string parseInt(vision::signature& c){
  return "";
}
#define FN_WITH_APA_SIG(name, tp1) \
  void name (tp1, bool = true);       \
  void name (tp1 amnt, bool write) {       \
    string msg = #name;       \
    msg += "(";                 \
    if(write)                             \
    cout << msg << parseInt(amnt) << string(");\n");

#define FN_WITH_APA_SIG_ARG(name, tp1, arg, str) \
  void name (tp1, arg, bool = true);       \
  void name (tp1 amnt, arg, bool write) {       \
    string msg = #name;       \
    msg += "(";                 \
    string msg2 = parseInt(amnt) + ", ";             \
    msg2 += str;                  \
    if(write)                             \
    cout << msg << msg2 << string(");\n");
#define FN_WITH_APA_SIG_NO_ARG(name) \
  void name (bool = true);       \
  void name (bool write) {       \
    string msg = #name;       \
    if(write)                             \
    cout << msg << string("();\n");
//


//All the functions that are based on classes
namespace ClassFns {
  FN_WITH_APA_SIG_NO_ARG(allowVisionAlign)
    wc.setExitDist(24.0);
  }
  FN_WITH_APA_SIG(visionAlign, vision::signature&)
    static VisionOdom basicAlign = VisionOdom(goalFrontVision, PVector(0, 6), 14, 0, 0);
    //Highest speed desired == 30
    //Highest err ~= 12
    PID align = PID(2.5, 0, 1.666);
    PID driveFwd = wc.getCtrl();
    align.setTarget(0);
    driveFwd.setTarget(0);
    while(!true){
      
      double fwd = driveFwd.getVal(wc.botPos().dist2D(wc.getLastTarget()));
      PVector l = basicAlign.closest(amnt);
      double alignVal = align.getVal(l.x);
      wc.moveLeft(fwd + alignVal);
      wc.moveRight(fwd - alignVal);
    }
  }
  //Useful for depositing pre-loads on two different goals
  FN_WITH_APA_SIG(allowOutRings, int)
    conveyer.autonCtrl = true;
    conveyer.autonOut = amnt;
  }
  FN_WITH_APA_SIG_NO_ARG(freeConveyer)
    conveyer.autonCtrl = false;
  }
  FN_WITH_APA_SIG_NO_ARG(ctrlConveyer)
    conveyer.autonCtrl = true;
  }

  FN_WITH_APA_SIG_NO_ARG(raiseLiftByOne)
    liftCtrllr.done = false;
    liftCtrllr.freePress();
    liftCtrllr.addIndex();
  }
  FN_WITH_APA_SIG_NO_ARG(lowerLiftByOne)
    liftCtrllr.done = false;
    liftCtrllr.freePress();
    liftCtrllr.subIndex();
  }
  FN_WITH_APA_SIG_NO_ARG(waitForLiftFinish)
    //cout << liftCtrllr.isDone() << endl;
    while(!liftCtrllr.isDone()){
      //cout << liftCtrllr.isDone() << endl;
      s(10);
    }
  }
  FN_WITH_APA_SIG_NO_ARG(raiseLiftByOneWait)
    liftCtrllr.done = false;
    liftCtrllr.freePress();
    liftCtrllr.addIndex();
    waitForLiftFinish(false);
  }
  FN_WITH_APA_SIG_NO_ARG(lowerLiftByOneWait)
    liftCtrllr.done = false;
    liftCtrllr.freePress();
    liftCtrllr.subIndex();
    waitForLiftFinish(false);
  }
  // FN_WITH_APA_SIG_NO_ARG(raiseBackLiftByOne)
  //   backLiftCtrllr.done = false;
  //   backLiftCtrllr.freePress();
  //   backLiftCtrllr.addIndex();
  // }
  // FN_WITH_APA_SIG_NO_ARG(lowerBackLiftByOne)
  //   backLiftCtrllr.freePress();
  //   backLiftCtrllr.done = false;
  //   backLiftCtrllr.subIndex();
  // }
  // FN_WITH_APA_SIG_NO_ARG(waitForBackLiftFinish)
  //   //cout << liftCtrllr.isDone() << endl;
  //   while(!backLiftCtrllr.isDone()){
  //     //cout << liftCtrllr.isDone() << endl;
  //     s(10);
  //   }
  // }
  // FN_WITH_APA_SIG_NO_ARG(raiseBackLiftByOneWait)
  //   backLiftCtrllr.freePress();
  //   backLiftCtrllr.done = false;
  //   backLiftCtrllr.addIndex();
  //   waitForBackLiftFinish(false);
  // }
  // FN_WITH_APA_SIG_NO_ARG(lowerBackLiftByOneWait)
  //   backLiftCtrllr.freePress();
  //   backLiftCtrllr.subIndex();
  //   backLiftCtrllr.done = false;
  //   waitForBackLiftFinish(false);
  // }
  FN_WITH_APA_SIG_NO_ARG(clipGoal)
    goalHolder.close();
    //wc.addGoal();
  }
  FN_WITH_APA_SIG_NO_ARG(unclipGoal)
    goalHolder.open();
    //wc.removeGoal();
  }
  FN_WITH_APA_SIG_NO_ARG(clipLiftGoal)
    liftGoalHolder.open();
  }
  FN_WITH_APA_SIG_NO_ARG(unclipLiftGoal)
    liftGoalHolder.close();
  }
  //Really old functions
  // FN_WITH_APA_SIG_NO_ARG(alignGoalFront)
  //   //Maybe put a PD loop in here
  //   goalFront.test();
  //   goalBack.test();
  //   double estimateDif = 0.0;
  //   double lastEst = 0.0;
  //   cout << goalFront.objectDistance(inches) << endl;
  //   cout << goalFront.objectDistance(inches) << endl;
  //   while(goalFront.objectDistance(mm) > 55 || goalBack.objectDistance(mm) > 55 || !goalFront.isObjectDetected() || !goalBack.isObjectDetected()){
  //     wc.moveLeft(-30);
  //     wc.moveRight(-30);
  //     goalFront.test();
  //     goalBack.test();
  //     bool frontDetect = goalFront.isObjectDetected() && goalFront.objectDistance(inches) < 12.0;
  //     bool backDetect = goalBack.isObjectDetected() && goalBack.objectDistance(inches) < 12.0;
  //     double dif = goalFront.objectDistance(mm) - goalBack.objectDistance(mm);
  //     if(goalFront.isObjectDetected() && goalBack.isObjectDetected()){

  //       if(dif > 10){ // goalBack (right) is closer
  //         wc.moveRight(-30 + dif * 0.75);
  //       }
  //       if(dif < -10){ // goalBack (right) is closer
  //         wc.moveLeft(-30 + -dif * 0.75);
  //       }
  //     }
  //     while(!frontDetect && backDetect){
  //       wc.moveLeft(-20);
  //       goalFront.test();
  //       goalBack.test();
  //       frontDetect = goalFront.isObjectDetected() && goalFront.objectDistance(inches) < 12.0;
  //       backDetect = goalBack.isObjectDetected() && goalBack.objectDistance(inches) < 12.0;
  //       goalFront.sleep();
  //     }
  //     wc.moveLeft(-30);
  //     wc.moveRight(-30);
  //     while(!backDetect && frontDetect){
  //       wc.moveLeft(-20);
  //       goalFront.test();
  //       goalBack.test();
  //       frontDetect = goalFront.isObjectDetected() && goalFront.objectDistance(inches) < 12.0;
  //       backDetect = goalBack.isObjectDetected() && goalBack.objectDistance(inches) < 12.0;
  //       goalFront.sleep();
  //     }
  //     wc.moveLeft(-30);
  //     wc.moveRight(-30);
  //     while(!backDetect && !frontDetect){
  //       wc.moveLeft(-10);
  //       wc.moveRight(10);
  //       goalFront.test();
  //       goalBack.test();
  //       frontDetect = goalFront.isObjectDetected() && goalFront.objectDistance(inches) < 12.0;
  //       backDetect = goalBack.isObjectDetected() && goalBack.objectDistance(inches) < 12.0;
  //       goalFront.sleep();
  //     }
  //     goalFront.sleep();
  //     cout << goalFront.objectDistance(mm) << ", " << goalBack.objectDistance(mm) << endl;
  //   }
  //   wc.moveLeft(-30);
  //   wc.moveRight(-30);
  //   s(300);
  //   wc.coastBrake();
  //   goalFront.endFiltering();
  // }

  //Functions I'm trying to replace
  // FN_WITH_APA_SIG(alignGoalFront, vision::signature&)
  //   vision::signature& code = amnt;
  //   goalFrontVision.takeSnapshot(code);
  //   vision::object* largestObject = &goalFrontVision.largestObject;
  //   cout << largestObject->height << endl;
  //   while(!largestObject->exists){
  //     goalFrontVision.takeSnapshot(code);
  //     largestObject = &goalFrontVision.largestObject;
  //     wc.moveLeft(10);
  //     wc.moveRight(-10);
  //     s(10);
  //   }
  //   s(700);
  //   vision::object* secondLargest;
  //   if(goalFrontVision.objectCount > 1 && goalFrontVision.objects[1].height > 60){
  //     secondLargest = &goalFrontVision.objects[1];
  //   }
  //   else {
  //     secondLargest = largestObject;
  //   }
  //   goalFront.test();
  //   goalBack.test();
  //   double lastFront = 100;
  //   double lastBack = 100;
  //   while(largestObject->exists){
  //     wc.moveLeft(-20);
  //     wc.moveRight(-20);
  //     if(largestObject == secondLargest){
  //       //Largest is on left side
  //       if(largestObject->centerX > 150){
  //         wc.moveRight(0);
  //       }
  //       //Largest is on right side
  //       else if(largestObject->centerX < 150){
  //         wc.moveLeft(0);
  //       }
  //     }
  //     else if (secondLargest->centerX < 245 && secondLargest->centerX > 180){
  //       wc.moveRight(-20 - (245 - secondLargest->centerX) / 10.0);
  //     }
  //     else if(secondLargest->centerX > 50 && secondLargest->centerX < 120){
  //       wc.moveLeft(-20 - (secondLargest->centerX - 50) / 10.0);
  //     }
      
  //     else if(largestObject->centerX > 245){
  //       wc.moveRight(-20 - (largestObject->centerX - 240) / 10.0);
  //     }
  //     else if(largestObject->centerX < 50){
  //       wc.moveLeft(-20 - (50 - largestObject->centerX) / 10.0);
  //     }
  //     // else if(!goalFront.isObjectDetected() || goalFront.objectDistance(inches) > 8.0){
  //     //   cout << "Left dist no obj" << endl;
  //     //   wc.moveLeft(0);
  //     // }
  //     // else if(!goalBack.isObjectDetected() || goalBack.objectDistance(inches) > 8.0){
  //     //   cout << "Right dist no obj" << endl;
  //     //   wc.moveRight(0);
  //     // }
  //     // else {
  //     //   cout << "Changing Based on dists" << endl;
  //     //   wc.moveLeft(-10 + (goalFront.objectDistance(mm) - goalBack.objectDistance(mm)));
  //     //   wc.moveRight(-10 - (goalFront.objectDistance(mm) - goalBack.objectDistance(mm)));
  //     // }
  //     goalFront.test();
  //     goalBack.test();
  //     if(goalFront.objectDistance(mm) < 55 && goalBack.objectDistance(mm) < 55 && goalFront.isObjectDetected() && goalBack.isObjectDetected()){
  //       cout << "Near Front: " << goalFront.objectDistance(mm) << ", " << goalBack.objectDistance(mm) << endl;
  //       break;
  //     }
  //     // if(goalFront.isObjectDetected() && goalFront.objectDistance(mm) < 55 && !goalBack.isObjectDetected() && lastBack < 65){
  //     //   break;
  //     // }
  //     // if(goalBack.isObjectDetected() && goalBack.objectDistance(mm) < 55 && !goalFront.isObjectDetected() && lastFront < 65){
  //     //   break;
  //     // }
  //     if(frontCounter.pressing()){
  //       cout << "Goal Found" << endl;
  //       break;
  //     }
  //     if(goalBack.isObjectDetected()){
  //       lastBack = goalBack.objectDistance(mm);
  //     }
  //     if(goalFront.isObjectDetected()){
  //       lastFront = goalFront.objectDistance(mm);
  //     }
  //     goalBack.sleep();
  //     goalFrontVision.takeSnapshot(code);
  //     largestObject = &goalFrontVision.largestObject;
  //     if(!largestObject->exists){
  //       cout << "Nothing found" << endl;
  //       break;
  //     }
  //     if(goalFrontVision.objectCount > 1 && goalFrontVision.objects[1].height > 60){
  //       secondLargest = &goalFrontVision.objects[1];
  //     }
  //     else {
  //       secondLargest = largestObject;
  //     }
  //   }
  //   goalFront.endFiltering();
  //   goalBack.endFiltering();
  //   wc.moveLeft(-30);
  //   wc.moveRight(-30);
  //   s(350);
  //   wc.coastBrake();
  // }
  // FN_WITH_APA_SIG(alignGoalBack, vision::signature&)
  //   vision::signature& code = amnt;
  //   goalDetector.takeSnapshot(code);
  //   auto* largestObject = &goalDetector.largestObject;
  //   if(!largestObject->exists){
  //     cout << "No obj" << endl;;
  //     return;
  //   }
  //   int time = 0;
  //   while(largestObject->exists){
  //     wc.moveLeft(20);
  //     wc.moveRight(20);
  //     if(largestObject->width > 100){
  //       if(largestObject->centerX < 150){
  //         wc.moveRight(20 - (150 - largestObject->centerX) / 4);
  //       }
  //       else if(largestObject->centerX > 150){
  //         wc.moveLeft(20 + (150 - largestObject->centerX) / 4);
  //       }
  //     }
  //     else {
  //       if(largestObject->centerX > 190){
  //         wc.moveLeft(20 + (190 - largestObject->centerX) / 4);
  //       }
  //       else if(largestObject->centerX < 110){
  //         wc.moveRight(20 - (110 - largestObject->centerX) / 4);
  //       }
  //     }
  //     if(backGoalDetect.pressing() == 10){
  //       break;
  //     }
  //     if(time >= 3000){
  //       break;
  //     }
  //     s(10);
  //     time += 10;

  //     goalDetector.takeSnapshot(code);
  //     largestObject = &goalDetector.largestObject;
  //     //cout << largestObject->centerX << ", " << largestObject->originX << ", " << largestObject->width << endl;
  //   }
  //   cout << "Done" << endl;
  //   wc.moveLeft(20);
  //   wc.moveRight(20);
  //   s(500);
  //   wc.coastBrake();
  // }
  //Needed for prog skills (maybe)
  FN_WITH_APA_SIG_NO_ARG(balanceBot)
    PID speedGetter = PID(1.5, 0.001, 0.1);
    bool exit = false;
    bool angleInRange = false;
    int t = 0;
    
    while(!exit){
      double angle = tiltAngle;
      angleInRange = abs(angle) < 3.0;
      if(!angleInRange){
        t = 0;
      } else {
        if(t++ >= 100){
          exit = true;
        }
      }
      double speed = speedGetter.getVal(angle);
      double angleTravel;// = wc.posNeg180(glblBotAngle - angle);
      if(speed < 0){
        angleTravel = posNeg180(-90 - glblBotAngle);
      } else {
        angleTravel = posNeg180(90 - glblBotAngle);
      }
      
      s(1);
      
    }
    

  }

}
