#include "Joint.h"
#include <cmath>

Joint::Joint(){
    motorGear=1;
    jointGear=1;
    microstepping=microsteps_default;
    offsetAngle=0;
    minAngle=-180;
    maxAngle=180;
}

Joint::Joint(int MotorGear, int JointGear, double MinAngle, double MaxAngle){
    motorGear=MotorGear;
    jointGear=JointGear;
    microstepping=microsteps_default;
    offsetAngle=0;
    minAngle=MinAngle;
    maxAngle=MaxAngle;
}

Joint::Joint(double OffsetAngle, double MinAngle, double MaxAngle){
    motorGear=1;
    jointGear=1;
    microstepping=microsteps_default;
    offsetAngle=OffsetAngle;
    minAngle=MinAngle;
    maxAngle=MaxAngle;
}

Joint::Joint(int MotorGear, int JointGear, double OffsetAngle, double MinAngle, double MaxAngle){
    motorGear=MotorGear;
    jointGear=JointGear;
    microstepping=microsteps_default;
    offsetAngle=OffsetAngle;
    minAngle=MinAngle;
    maxAngle=MaxAngle;
}

double Joint::Ratio(){
    return (double)jointGear/(double)motorGear;
}

double Joint::StepsToAngle(int Steps){
    return (double)((double)Steps*1.8)/((double)microstepping*Ratio());
}

int Joint::AngleToSteps(double Angle){
    return (int)round((Angle*microstepping*Ratio())/1.8);
}

int Joint::StepsOffset(){
    return AngleToSteps(offsetAngle);
}