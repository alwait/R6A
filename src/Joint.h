#ifndef JOINT_H
#define JOINT_H

#define microsteps_default 8

class Joint{
private:
    int motorGear;
    int jointGear;
    int microstepping;
    float offsetAngle;
    float minAngle;
    float maxAngle;

public:
    Joint();
    Joint(int, int, float, float);
    Joint(float, float, float);
    Joint(int, int, float, float, float);
    float Ratio();
    int AngleToSteps(float);
    float StepsToAngle(int);
    int StepsOffset();
    float getMinAngle(){return minAngle;};
    float getMaxAngle(){return maxAngle;};
    int getMinSteps();
    int getMaxSteps();
};

#endif 