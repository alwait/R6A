#ifndef JOINT_H
#define JOINT_H

#define microsteps_default 8

class Joint{
private:
    int motorGear;
    int jointGear;
    int microstepping;
    double offsetAngle;
    double minAngle=0;
    double maxAngle=0;

public:
    Joint();
    Joint(int, int, double, double);
    Joint(double, double, double);
    Joint(int, int, double, double, double);
    double Ratio();
    int AngleToSteps(double);
    double StepsToAngle(int);
    int StepsOffset();
    
};

#endif 