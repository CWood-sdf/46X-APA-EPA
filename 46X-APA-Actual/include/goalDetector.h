/*vex-vision-config:begin*/
#include "vex.h"
vex::vision::signature NEUTRAL = vex::vision::signature (1, 1089, 1991, 1540, -4811, -3685, -4248, 2.5, 0);
vex::vision::signature BLUE_GOAL = vex::vision::signature (2, -2925, -1747, -2336, 6685, 12689, 9687, 2.5, 0);
vex::vision::signature RED_GOAL = vex::vision::signature (3, 7489, 10795, 9142, -1441, -761, -1102, 2.5, 0);
vex::vision::signature SIG_4 = vex::vision::signature (4, 0, 0, 0, 0, 0, 0, 2.5, 0);
vex::vision::signature SIG_5 = vex::vision::signature (5, 0, 0, 0, 0, 0, 0, 2.5, 0);
vex::vision::signature SIG_6 = vex::vision::signature (6, 0, 0, 0, 0, 0, 0, 2.5, 0);
vex::vision::signature SIG_7 = vex::vision::signature (7, 0, 0, 0, 0, 0, 0, 2.5, 0);
vex::vision goalDetector = vex::vision (vex::PORT15, 50, NEUTRAL, BLUE_GOAL, RED_GOAL, SIG_4, SIG_5, SIG_6, SIG_7);
/*vex-vision-config:end*/