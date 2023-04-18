/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// fly                  motor         2               
// intake               motor         21              
// rgtFrt               motor         11              
// lftFrt               motor        20              
// rgtBk                motor         12              
// lftBk                motor         16              
// Inertial20           inertial      18              
// FlyEnc               rotation      8               
// rgtMid               motor         1               
// lftMid               motor         9               
// lftend               digital_out   F               
// rgtEnd               digital_out   E               
// botEnd               digital_out   H               
// blocker              digital_out   G               
// eye                  vision        7               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include <cmath>
#include <iostream>
using namespace vex;

// A global instance of competition
competition Competition;

vex::task turnControl;
vex::task driveControl;
vex::task intakeControl;
vex::task visionControl;
// define your global instances of motors and other devices here
int autoOptions = 5;
int autoVal = 0;
int alliance = 0;

int AutonSelected = 0;
int AutonMin = 0;
int AutonMax = 4;
/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/



void drawScreen(){
  Brain.Screen.clearScreen(); //Clear screen for printing
  Brain.Screen.printAt(1, 40, "Select Auton");  //Print string for autonomous selector
  Brain.Screen.printAt(1, 200, "Auton Selected = %d  ", AutonSelected); //Print current auto value picked
  Brain.Screen.setFillColor(purple);  //Turn button purple as its pressed
  Brain.Screen.drawRectangle(20, 50, 100, 100); //Draw rectangle for auto selector button
  Brain.Screen.drawCircle(300, 75, 25); //Draw circle 
  Brain.Screen.printAt(25, 75, "Select"); //More strings 
  Brain.Screen.setFillColor(blue);
  Brain.Screen.drawRectangle(170, 50, 100, 100);  //One more rectangle
  Brain.Screen.printAt(175, 75, "GO");  //Print go once auto is confirmed
  Brain.Screen.setFillColor(black);
}


void selectAuto() {
  bool selectingAuton = true;

  int x = Brain.Screen.xPosition(); // Grab x position 
  int y = Brain.Screen.yPosition(); // Grab y position
  // check to see if buttons were pressed
  if (x >= 20 && x <= 120 && y >= 50 && y <= 150) // select button pressed
  {
    AutonSelected++;
    if (AutonSelected > AutonMax)AutonSelected = AutonMin; // rollover
      
    Brain.Screen.printAt(1, 200, "Auton Selected =  %d   ", AutonSelected);
  }
  if (x >= 170 && x <= 270 && y >= 50 && y <= 150) {
    selectingAuton = false; // GO button pressed
    Brain.Screen.printAt(1, 200, "Auton  =  %d   GO           ", AutonSelected);
  }
  if (!selectingAuton) {
    Brain.Screen.setFillColor(green);
    Brain.Screen.drawCircle(300, 75, 25);
  } else {
    Brain.Screen.setFillColor(red);
    Brain.Screen.drawCircle(300, 75, 25);
  }
  wait(10, msec); //Refresh
  Brain.Screen.setFillColor(black);
}
void pre_auton(void) {

  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  reset();
  Inertial20.resetRotation();
  Brain.Screen.printAt(1, 40, "pre auton running");
  drawScreen();
  Brain.Screen.pressed(selectAuto);
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
  turnControl = task(turnTask); //Initialize our subsystem tasks so they can run during auto
  driveControl = task(driveTask);
  visionControl = task(VisionTask);
  
  if(AutonSelected == 0){
    //Default option of nothing is pressed on the brain
    //Test ur stuff
    move(1, 0, 70);
  }
  if(AutonSelected == 1){
    //Front auto
    //Score one roller and two disks
    intake.spin(fwd, 50, pct);  //Start up intake
    wait(125, msec);
    move(-0.125, 0, 20);  //Reverse into roller
    wait(100, msec);
    intake.spin(fwd, 0, pct); //Stop
    move(0.4, 0, 20);  //Forward
    intake.spin(fwd, -100, pct);
    wait(1.5, sec);
    rotate(45, 80); //Turn parallel to auto line
    wait(0.5, sec);
    fly.spin(fwd, 10.5, voltageUnits::volt);  //Start revving
    intake.spin(fwd, 0, pct);
    wait(0.5, sec);
    move(2, 45, 80);  //Move to mid field
    wait(1, sec);
    rotate(-30, 80);  //Rotate to face goal
    wait(0.75, sec);
    move(0.5, -30, 30);
    wait(0.75, sec);
    intake.spin(fwd, 80, pct); //Shoot one disc
    wait(0.5 , sec);
    intake.spin(fwd, -100, pct);  //Stop reset
    wait(0.5, sec);
    intake.spin(fwd, 100, pct); //Shoot 2nd disc
    wait(0.5, sec);
    intake.spin(fwd, 0, pct); //Stop intake
    fly.spin(fwd, 0, voltageUnits::volt); 


  }
  else if(AutonSelected == 2){
   //Side auto  
   fly.spin(fwd, 10.5, voltageUnits::volt); //Rev fly
   move(0.75, 0, 70); //Forward to face goal
   wait(2, sec);
   rotate(18, 80);  //Turn to goal
   wait(2, sec);
   intake.spin(fwd, 100, pct);  //Shoot one disc
   wait(750, msec);
   intake.spin(fwd, 0, pct);
   fly.spin(fwd, 0, pct); //Stop fly
   wait(0.75, sec);
   rotate(-45, 80); //Parallel to auto line
   wait(0.75, sec); 
   move(-1.45,-25, 80); //Reverse back to roller
   wait(0.5, sec);
   intake.spin(fwd, 60, pct); //Spin roller
   rotate(0, 80);
   wait(750, msec);
   move(-0.45, 0, 30);  //Reverse into roller
   wait(400, msec);
   move(0.25, 0, 20); //Away from roller
   intake.spin(fwd, 0, pct);  //Stop
  }
  else if(AutonSelected == 3){
    //Filler option
  }
  else if(AutonSelected == 4){
    //Skills
    moveRoller(-30);  //Start up intake
    wait(125, msec);
    move(-0.1, 0, 10);  //Reverse into roller
    wait(100, msec);  
    intake.spin(fwd, 0, pct); //Stop roller
    wait(0.75, sec);
    move(0.1, 0, 10);
    wait(0.5, sec);
    rotate(-10, 70);
    wait(0.5, sec);
    move(1.2, -10, 70); //Forward toward center of field
    wait(1, sec);
    rotate(90, 80); //Rotate 90 clockwise to face next roller
    moveRoller(-60); //spin roller 
    wait(2, msec);
    move(-2, 90, 30); //Reverse into roller
    wait(0.125, sec);
    moveRoller(0); //Stop intake
    wait(100, msec);
    move(0.35, 90, 30); //Move forward slightly
    wait(500, msec);
    fly.spin(fwd, 9, voltageUnits::volt);  //Start revving flywheel
    rotate(8, 80);  //Rotate cc to face goal
    wait(1, sec);
    move(2.65, 8, 70); //Move forward towards goal
    wait(1, sec);
    moveRoller(80);  //Index discs into flywheel and score two discs
    wait(1, sec);
    rotate(-45, 80);  //Turn cc to back into field
    wait(0.5, sec);
    move(-2.25, -45, 70); //Reverse to center  field
    wait(1, sec); 
    rotate(25, 80); //Rotate to line up with auto line
    wait(0.75, sec);
    wait(1, sec);
    move(-1, 25, 79);
    wait(2, sec);
    rgtEnd.set(true); //Deploy endgame
    lftend.set(true);
    botEnd.set(true);
  }
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  alliance = 3;
  Brain.Screen.clearScreen(); //Clear brain for teleop data 
  turnControl.stop(); //Stop subsystem tasks for driver
  driveControl.stop();
  intakeControl.stop();
  //visionControl.stop();

  //Initialize teleop variables
  int y;
  int x;
  int rgtF;
  int rgtB;
  int lftF;
  int lftB;
  int Pow[23] = {-100, -90, -80, -70, -60, -50, -40, -30,   //Index tables for controller values
                  -20, -10, -5, 0, 5, 10, 20, 30, 40, 50, 
                  60, 70, 80, 90, 100};
  int Pow1[23] = {-90, -90, -80, -70, -60, -50, -40, -30, 
                  -20, -10, -5, 0, 5, 10, 20, 30, 40, 50, 
                  60, 70, 80, 100, 100};

  int Index;
  int index1;
  FlyEnc.resetPosition();

  while (1) {
    
    Brain.Screen.printAt(30, 30, "Fly Vel: %f", FlyEnc.velocity(rpm));  //Print telemetry
    Brain.Screen.printAt(30, 50, "Alliance: %d", alliance);
    Brain.Screen.printAt(30, 90, "rpm : %f", (fly.velocity(rpm)*6));

    //Print to terminal
    std::cout <<Brain.Timer.value() << ","
    <<fly.velocity(rpm) << "," << fly.torque(Nm) << ","
    <<fly.current() << ", " << fly.voltage(volt)
    << std::endl;

    
    y = Controller1.Axis3.value();  //Set controller axes to variables 
    x = Controller1.Axis1.value();

    Index = y / 12 + 11;  //find average value
    index1 = x / 12 + 11;
    rgtF = (Pow[Index] - Pow1[index1]); //Calculate power for each motor
    rgtB = (Pow[Index] - Pow1[index1]);
    lftF = (Pow[Index] + Pow1[index1]);
    lftB = (Pow[Index] + Pow1[index1]);

    rgtFrt.spin(fwd, rgtF, pct);  //Set drive motors
    rgtMid.spin(fwd, rgtF, pct);
    rgtBk.spin(fwd, rgtF, pct);
    lftFrt.spin(fwd, lftF, pct);
    lftMid.spin(fwd, lftF, pct);
    lftBk.spin(fwd, lftF, pct);

  
    if(Controller1.ButtonR1.pressing()){
      //Spin up to fire
      fly.spin(fwd, 9, voltageUnits::volt); 
    }
    else if(Controller1.ButtonLeft.pressing()){
      fly.spin(fwd, 10, voltageUnits::volt);  //Higher speed for long shots
    }
    else{
      fly.spin(fwd, 5, voltageUnits::volt); //Idle speed
      //fly.stop(coast);
      }
    
    if(Controller1.ButtonL1.pressing()){
      intake.spin(fwd, -100, pct);  //Intake in discs/ move roler
    }
    else if(Controller1.ButtonL2.pressing()){
      intake.spin(fwd, 100, pct); //move roller
    }
    else if(Controller1.ButtonR2.pressing()){
      intake.spin(fwd, 45, pct);  //Feed discs into flywheel at slower power
    }
    else{
      intake.stop(coast); //Cost intake
    }
    
    if(Controller1.ButtonA.pressing()){
      rgtEnd.set(true); //Deploy all endgame on the right side of the robot
      botEnd.set(true);
    }
    if(Controller1.ButtonUp.pressing()){
      lftend.set(true); //Deploy all left side endgame
      botEnd.set(true);
    }
    if(Controller1.ButtonX.pressing()){
      blocker.set(true);  //Enable blocker
    }

    
    if(Controller1.ButtonDown.pressing()){
      visionControl.resume();
      rotateToGoal(true);  //False = red; True = blue
    }
    else{
      visionControl.suspend();
    }
    //if(std::signbit(x))
    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
