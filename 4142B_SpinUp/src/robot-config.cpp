#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor fly = motor(PORT2, ratio6_1, true);
motor intake = motor(PORT21, ratio6_1, false);
motor rgtFrt = motor(PORT11, ratio6_1, false);
motor lftFrt = motor(PORT20, ratio6_1, true);
motor rgtBk = motor(PORT12, ratio6_1, false);
motor lftBk = motor(PORT16, ratio6_1, true);
inertial Inertial20 = inertial(PORT18);
rotation FlyEnc = rotation(PORT8, false);
motor rgtMid = motor(PORT1, ratio6_1, true);
motor lftMid = motor(PORT9, ratio6_1, false);
digital_out lftend = digital_out(Brain.ThreeWirePort.F);
digital_out rgtEnd = digital_out(Brain.ThreeWirePort.E);
digital_out botEnd = digital_out(Brain.ThreeWirePort.H);
digital_out blocker = digital_out(Brain.ThreeWirePort.G);
/*vex-vision-config:begin*/
signature eye__BLU_SSIG = signature (1, -1725, -591, -1158, 6697, 9409, 8053, 3, 0);
signature eye__RED_SIG = signature (2, 6977, 10997, 8987, -1593, -203, -898, 2.5, 0);
vision eye = vision (PORT7, 34, eye__BLU_SSIG, eye__RED_SIG);
/*vex-vision-config:end*/

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}