using namespace vex;

extern brain Brain;

using signature = vision::signature;

// VEXcode devices
extern controller Controller1;
extern motor fly;
extern motor intake;
extern motor rgtFrt;
extern motor lftFrt;
extern motor rgtBk;
extern motor lftBk;
extern inertial Inertial20;
extern rotation FlyEnc;
extern motor rgtMid;
extern motor lftMid;
extern digital_out lftend;
extern digital_out rgtEnd;
extern digital_out botEnd;
extern digital_out blocker;
extern signature eye__BLU_SSIG;
extern signature eye__RED_SIG;
extern signature eye__SIG_3;
extern signature eye__SIG_4;
extern signature eye__SIG_5;
extern signature eye__SIG_6;
extern signature eye__SIG_7;
extern vision eye;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );