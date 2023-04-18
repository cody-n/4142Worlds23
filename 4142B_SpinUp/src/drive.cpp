#include "vex.h"

const int distCon = 1000;   //1 tile equals 1000 ticks
//Tuning constants for Lateral movement + turning pid
const double drivekP = .15;    //Don't adjust any of these
const double drivekI = 0.000; //
const double drivekD = 1.15;

const double angkP = 0.09;
const double angkI = 0.0;
const double angkD = 0.015;

const double turnKp = 0.89;
const double turnKi = 0.0005;
const double turnKd = 1.5;
//-----------------------------------------

double Angle = 0;   //Desired angle for robot
static int maxSpeed = 100;  //Set max robot speed (%)
bool driveEnabled;  
bool turnEnabled;
int Color;  //Set what color we want to track for, 1 = blue, 0 for red

int driveTarget = 0;  //Drive variable for desired encoder position for pid
int turnTarget = 0;   //Drive variable for desired heading for pid
int visionTarget = 0; //
int driveMode = 0;    //1 = fwd/bk movement, -1 = turning


void setBrake(brakeType i){
  //Assign drive motors break type
  //Of either Coast, Brake, or Hold
  rgtFrt.setBrake(i);
  rgtBk.setBrake(i);
  lftFrt.setBrake(i);
  lftBk.setBrake(i);
}

void reset(){
  //Zero drive encoder positions
  rgtFrt.resetPosition();
  lftFrt.resetPosition();
  rgtBk.resetPosition();
  lftBk.resetPosition();
}

double driveVel(){
  //Return the average drive velocity in rotations per minute
  return (rgtFrt.velocity(rpm) + lftFrt.velocity(rpm))/2;
}

int avgDrive(){
  //Return average encoder ticks of robot 
  return (rgtFrt.position(deg) + lftFrt.position(deg)) / 2;
}

double driveHeading(){
  //Return drive Inertial sensor heading
  return Inertial20.rotation();
}

bool isDriving(){
  //Used to check if the robot is still moving
  static int count = 0;
  static int last = 0;
  static int lastTarget = 0; 

  int curr = avgDrive();
  int target = turnTarget;
  
  if(driveMode == 1){
    //Robot is moving forward, we want the rgt/lft encoders
    target = driveTarget; 
  }
  if(abs(last - curr) < 3){
    //If the last reading - current reading is less than 3
    //Robot hasn't moved
    count++;  //Start counting when stationary
  }
  else{
    //Robot is moving
    count = 0;  //Reset Count
  }
  if(target != lastTarget){
    count = 0;
  }

  lastTarget = target;
  last = curr;
  
  if(count > 8){  //Used to be 4
    //Count has looped 4x, we havent moved
    return false; //Drive isn't moving
  }
  else{
    return true; //Robot is moving
  }
}


void waitUntilSettled(){
  //Waits until current drive action is done
  //To move onto next drive command
  while(isDriving()){
    wait(15, msec);
  }
}

void driveAsync(double sp, double ang, int max){
  reset();    //Reset drive sensors
  maxSpeed = max; //Assign value to max speed variable
  Angle = ang;  //Assign value to desired robot angle
  driveTarget = sp * distCon; //Convert our setpoint input encoder ticks
  driveMode = 1;  //Drive will do lateral movment only
  driveEnabled = true;  //Enable drive
}

void move(double sp, double ang, int max){
  driveAsync(sp, ang,  max);  //Run setter function
  wait(400, msec);
  waitUntilSettled(); //Wait until drive movement is finished 
}


void rotate(double sp, int max){
  turnTarget = sp;  //Assign value for desired heading
  driveMode = -1; //Robot will do turning movement only
  turnEnabled = true; //Enable drive
  wait(400, msec);  //
  waitUntilSettled();
}


double getTarget(bool blu){
  //True if want blue
  //False if want red
  if(blu){
    eye.takeSnapshot(eye__BLU_SSIG); //Return Blue snapshot
    double objCenX = eye.largestObject.centerX;
    return objCenX;
  }
  else{
    eye.takeSnapshot(eye__RED_SIG); //Red
    double objCenX = eye.largestObject.centerX;
    return objCenX;
  }
}

void rotateToGoal(bool blue){
  driveMode = 2; //Set drive to use camera data to center goal
  if(blue){
    Color = 1;  //BLUE
  }
  else{
    Color = 0;  //RED
  }
  wait(400, msec);
  waitUntilSettled();
}

int driveTask(){
  int prevErr = 0;  //Initialize task specific variables
  int angPrevErr = 0;
  double integral = 0.0;
  double angIntegral = 0.0;
  int sp;
  int i = 0;
  //PID
  while(1){
    //printf("enc: %d", avgDrive()); //prints for debugging
    wait(20, msec);
    if(Competition.isDriverControl()){  //Disable driver task during teleop
      break;
    }
    //printf("enc: %d", avgDrive()); //prints for debugging
    if(driveMode == 1){
      //Continue calculating if drive is in lateral mode
      //Else continue on
      sp = driveTarget;
    }
    else{
      driveEnabled = false;
      continue;
    }

    //Initializing our sensor input
    int sv = avgDrive();  
    int error = sp - sv;  //Calculate error 
    integral = (integral + error);  //Accumilate error for integral
    if(fabs(integral) > 4000){  //Cap integral 
      integral = 4000;
    }
    else if(error == 0){
      integral = 0; //Reset integral if error is zero
      //If we got to destination, no need to accumilate error
    }

    int delta = error - prevErr;  //Find rate of change for error
    prevErr = error;  

    //Calculate output speed of motor
    //Combine all 3, P, I, D
    int speed = (error * drivekP) + (integral * drivekI) + (delta * drivekD);

    //Limit output 
    if(speed >= maxSpeed){
      speed = maxSpeed;
    }
    if(speed <= -maxSpeed){
      speed = -maxSpeed;
    }

    if(abs(error) <= 20){
      i++;  //Set threshold
            //If error is less than 20, incrment a counter  
      if( i > 4){ //When the counter incremnts past 4, we are stationary
        driveEnabled = false; //
      }
    }

    //----------------------------------
    // Heading PID
    // Maintain a desired angle during lateral movement
    //---------------------------------
    int angErr = Angle - driveHeading();  //Calculate heading error
    angIntegral = (angIntegral + angErr); //Calculate heading accumilated error

    if(angIntegral >= 4000){  //Cap integral
      angIntegral = 4000;
    }
    if(angErr == 0){  //Reset accum error
      angIntegral = 0;
    }
    int angDelta = angErr - angPrevErr; //Calculate rate of change for error
    angPrevErr = angErr;
    //Calculate output
    int sr = (angErr * angkP) + (angIntegral * angkI) + (angDelta * angkD);
    //Combine both lateral movement outputs and straightening
    rgtFrt.spin(fwd, speed - sr, pct);
    rgtBk.spin(fwd, speed - sr, pct);
    lftFrt.spin(fwd, speed + sr, pct);
    lftBk.spin(fwd,speed + sr, pct);
  }
  return 1;
}

int turnTask(){
  int prevErr = 0;  //Initialize task specific variables
  double integral = 0;
  int sp;
  int i = 0;

  while(1){
    wait(20, msec);
    //printf("heading: %f", driveHeading());  //Prints for debugging
    if(driveMode == -1){
      sp = turnTarget;  //Set drive setpoint to be desired heading
    }
    else{
      //Move on with other commands
      continue;
    }
    int sv = driveHeading();  //Initialize sensor input as imu heading
    int error = sp - sv;  //Calculate error (where you want to be - where you are)
    integral = (integral + error);  //Calculate accumilated error

    if(fabs(integral) > 4000){  //Cap integral 
      integral = 0;
    }
    else if(error == 0){  //Reset integral
      integral = 0;
    }
    int delta = error - prevErr;  //Find rate of change for error
    prevErr = error;  

    //Calculate output speed for motors 
    int speed = (error * turnKp) + (integral * turnKi) + (delta * turnKd);

    //Limit output 
    if((speed) > maxSpeed){
      speed = maxSpeed;
    }
    if(speed < -maxSpeed){
      speed = -maxSpeed;
    }

    //Set calculted outputs to drive motors
    rgtFrt.spin(fwd, -speed, pct);
    rgtBk.spin(fwd, -speed, pct);
    lftFrt.spin(fwd, speed, pct);
    lftBk.spin(fwd, speed, pct);
  }
  return 0;
}



int VisionTask(){
  //Initialize task specific variables
  int prevErr = 0;
  double integral = 0;
  int sp;
  int sv;
  int i = 0;
  double kP = 0.2;  //Constants for vision pid
  double kI = 0.0;
  double kD = 0.0;
  while(1){
    wait(20, msec);
    if(driveMode == 2){
      sp = 170; // X coord that registers as centered for our robot
    }
    else{
      continue;
    }

    if(Color == 1){
      sv = getTarget(true); //Grab blue color values from camera
    }
    else{
      sv = getTarget(false);  //Grab red color values
    }
    int error = sp - sv;  //Calculate error
    integral = (integral + error);  //Calculate integral 
    int delta = error - prevErr;  //Calculate rate of change of error

    //Calculate output for motor
    int speed = (error * kP) + (integral * kI) + (delta * kD);


    //Limit output speed 
    if(speed > maxSpeed){
      speed = maxSpeed;
    }
    else if(speed < -maxSpeed){
      speed = -maxSpeed;
    }

    //printf("cameraPos: %f", getTarget(false));  //Print for debugging purposes
    rgtFrt.spin(fwd, speed, pct);
    rgtBk.spin(fwd, speed, pct);
    lftFrt.spin(fwd, -speed, pct);
    lftBk.spin(fwd, -speed, pct);

  }
}
