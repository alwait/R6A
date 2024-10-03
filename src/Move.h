#ifndef MOVE_H
#define MOVE_H

#include "Options.h"
#include "Joint.h"
#include "vector"
#include "AccelStepper.h"
#include "Arduino.h"

using namespace std;

class Move : public Options{
private:
    vector<AccelStepper*> steppers;
    vector<Joint*> joints;
    vector<float> angles;
    float speed;
public:
    Move(const vector<AccelStepper*>&, const vector<Joint*>&);    
    void change() override; 
    bool move() override; 

    void setSteppers(const vector<AccelStepper*>& Steppers) {steppers = Steppers;};
    void setJoints(const vector<Joint*>& Joints) {joints = Joints;};
    void setAngles(vector<float> Angles) {angles = Angles;};
    vector<float> getAngles(){return angles;}
    bool isAnglesNan();
    void setSpeedScaling(float scale){speed=scale;};
    float getSpeedScaling(){return speed;};
};

#endif // MOVE