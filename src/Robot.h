#ifndef ROBOT_H
#define ROBOT_H

#include "vector"
#include "Offset.h"
#include "GPIO.h"
#include "EmergencyStop.h"
#include <AccelStepper.h>

#define default_speed 2000
#define default_acceleration 1000

using namespace std;

class Robot{
private:
    vector<AccelStepper*> steppers;
    vector<Joint*> joints;
    Offset offset;
    EmergencyStop stop;
    bool running;
    bool enable;
    int stage;
    
public:
    Robot(vector<AccelStepper*>&, vector<Joint*>&);
    AccelStepper* getStepper(size_t) const;
    Offset& getOffset() {return offset;};
    EmergencyStop& getEmergencyStop() {return stop;};
    bool isRunning() {return running;}
    bool isEnable() {return running;}
    bool isStop() {return stop.isStop();}
    void moveOffset();
    void emergencyStop();
    void emergencyRelease();
    int moveTestStages();
};

#endif