#ifndef JOINT_H
#define JOINT_H

#include "Joint.h"
#include <cmath>

Joint::Joint(){
    motorGear=1;
    jointGear=1;
    microstepping=microsteps_default;
    offsetAngle=0;
}

Joint::Joint(int MotorGear, int JointGear){
    motorGear=MotorGear;
    jointGear=JointGear;
    microstepping=microsteps_default;
    offsetAngle=0;
}

Joint::Joint(int MotorGear, int JointGear, double OffsetAngle){
    motorGear=MotorGear;
    jointGear=JointGear;
    microstepping=microsteps_default;
    offsetAngle=OffsetAngle;
}

double Joint::Ratio(){
    return (double)jointGear/(double)motorGear;
}

int Joint::AngleToSteps(double Angle){
    return (int)round((Angle*microstepping*Ratio())/1.8);
}

int Joint::StepsOffset(){
    return AngleToSteps(offsetAngle);
}

#endif // JOINT_H