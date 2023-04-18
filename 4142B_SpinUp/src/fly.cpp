#include "vex.h"


static double maxRPM = 3000;
double tbh = 0.0;
double gain = 0.000001;
double targetRPM = 0.0;
double motorPower = 0.0;

#include "vex.h"
#include "cmath"
const double flyKp = 0.1;


int prevVel = 0;



double getVelocity(){
  return fly.velocity(rpm); //Return flywheel velcity in rpm
}

double getAccel(){
  int vel = getVelocity();
  return vel - prevVel; //Return acceleration as change in velocity
  prevVel =  getVelocity();
}

void moveShooter(double pow){
  fly.spin(fwd, pow, voltageUnits::volt); //Set voltage to flywheel motor
} 

bool isnegative(int n){
  if (n / n != 1){
    return true;    //Number divided by itself isn't 1
  }
  else{
    return false;
  }
}

void setVel(int rpm){
  //Initialize flywheel variables
  //Tried to make p controller for flywheel
  //Need to continue debugging later
  double flykI = 0.01;
  double flykD = 0.0;
  double integral = 0;
  int maxIn = 600;
  int maxOut = 1;
  int prevErr = 0;
  int error = rpm - getVelocity();  //Calculate error
  int output = error * flyKp; 
  int f = 0;  //Feed forward
  integral = (integral + error); //Calculate accumilated error
  if(integral >= 10000){
    integral = 0; //Cap integral
  }
  if(error == 0){
    integral = 0;
  }

  int delta = error - prevErr;  //Calculate change in error
  prevErr = error;
  
  f = (maxOut/maxIn) * rpm; //Feed forward (helps to predict possible changes in output power)

  output = (error * flyKp) + (integral * flykI) + (delta * flykD);
  prevErr = error;
  printf("output: %i", output); //Print output for debugging
  moveShooter(output);  //Set calculated output to shooter
  
}

