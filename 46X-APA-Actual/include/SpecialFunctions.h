#include "ClassBaseFns.h"
//SpecialFunctions.h -- use this file for any functions that can't be placed directly
//      on the controller and are void(void)
namespace ClassFns {

}

vector<std::function<void(bool)>> specFns = { 

  //sf1, fwdForBall, backward, forwardInTower, halfDrop, fullIntakeOpen, slightBack
};

// FN_WITH_APA_SIG_NO_ARG(sf1)
//   leftWhls.spin(reverse, 100);
//   rghtWhls.spin(reverse, 100);
//   dropBall();
//   intakeOut();
//   counterBottom.reset();
//   int t = 0;
//   while(counterBottom.getCountOut() < 1 && (t++ * 10) < 2000){
//     s(10);
//   }
//   s(1000);
//   stopBall();
//   intakeStop();
//   rghtWhls.stop(coast);
//   leftWhls.stop(coast);
// };
// FN_WITH_APA_SIG_NO_ARG(forwardInTower)
//   wc.forward(100);
//   s(1000);
//   wc.coastBrake();
// }
// FN_WITH_APA_SIG_NO_ARG(halfDrop)
//   dropBall();
//   s(500);
//   stopBall();
// }
// FN_WITH_APA_SIG_NO_ARG(threadHalfDrop)
//   threadFns.push_back([](){
//     halfDrop(false);
//   });

// }
// using namespace ClassFns;
// FN_WITH_APA_SIG_NO_ARG(fullIntakeOpen)
//   threadOpenIntake();

//   threadOpenIntake();

//   //threadOpenIntake();
// }
// FN_WITH_APA_SIG_NO_ARG(slightBack)
//   wc.forward(-100);
//   s(150);
//   wc.hardBrake();
// }



