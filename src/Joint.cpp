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

Joint::Joint(int MotorGear, int JointGear, float MinAngle, float MaxAngle){
    motorGear=MotorGear;
    jointGear=JointGear;
    microstepping=microsteps_default;
    offsetAngle=0;
    minAngle=MinAngle;
    maxAngle=MaxAngle;
}

Joint::Joint(float OffsetAngle, float MinAngle, float MaxAngle){
    motorGear=1;
    jointGear=1;
    microstepping=microsteps_default;
    offsetAngle=OffsetAngle;
    minAngle=MinAngle;
    maxAngle=MaxAngle;
}

Joint::Joint(int MotorGear, int JointGear, float OffsetAngle, float MinAngle, float MaxAngle){
    motorGear=MotorGear;
    jointGear=JointGear;
    microstepping=microsteps_default;
    offsetAngle=OffsetAngle;
    minAngle=MinAngle;
    maxAngle=MaxAngle;
}

float Joint::Ratio(){
    return (float)jointGear/(float)motorGear;
}

float Joint::StepsToAngle(int Steps){
    return (float)((float)Steps*1.8)/((float)microstepping*Ratio());
}

int Joint::AngleToSteps(float Angle){
    return (int)round((Angle*microstepping*Ratio())/1.8);
}

int Joint::StepsOffset(){
    return AngleToSteps(offsetAngle);
}

int Joint::getMinSteps(){
    return AngleToSteps(minAngle);
}

int Joint::getMaxSteps(){
    return AngleToSteps(maxAngle);
}