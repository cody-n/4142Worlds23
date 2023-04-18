using namespace vex;

#ifndef DRIVE
#define DRIVE

void setBrake(brakeType i);

void reset();

double driveVel();

int avgDrive();

double driveHeading();

bool isDriving();

void waitUntilSettled();

void driveAsync(double sp, double ang, int max);

void move(double sp,double ang, int max);

void rotate(double sp, int max);

double getTarget(bool blu);

void rotateToGoal(bool blue);

int driveTask();

int turnTask();

int VisionTask();

#endif