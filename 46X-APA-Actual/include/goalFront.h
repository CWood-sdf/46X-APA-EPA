/*vex-vision-config:begin*/
#include "vex.h"
vex::vision::signature BLUE2 = vex::vision::signature (1, -2321, -1615, -1968, 5179, 7385, 6282, 2.5, 0);
vex::vision::signature RED2 = vex::vision::signature (2, 6057, 9741, 7899, -1001, -87, -544, 2.5, 0);
vex::vision::signature YELLOW2 = vex::vision::signature (3, 443, 1857, 1150, -4849, -3479, -4164, 2.5, 0);
vex::vision::signature SIG_41 = vex::vision::signature (4, 0, 0, 0, 0, 0, 0, 2.5, 0);
vex::vision::signature SIG_51 = vex::vision::signature (5, 0, 0, 0, 0, 0, 0, 2.5, 0);
vex::vision::signature SIG_61 = vex::vision::signature (6, 0, 0, 0, 0, 0, 0, 2.5, 0);
vex::vision::signature SIG_71 = vex::vision::signature (7, 0, 0, 0, 0, 0, 0, 2.5, 0);
vex::vision goalFrontVision = vex::vision (vex::PORT16, 80, BLUE2, RED2, YELLOW2, SIG_41, SIG_51, SIG_61, SIG_71);
/*vex-vision-config:end*/