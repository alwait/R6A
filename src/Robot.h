#ifndef ROBOT_H
#define ROBOT_H

#include "vector"
#include "Offset.h"
#include "GPIO.h"
#include "EmergencyStop.h"
#include <AccelStepper.h>

#define default_speed 4000
#define default_acceleration 2000

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
    void setMovingSpeed(float);
    void setMovingSpeedDefault();
    void disableRobot();
    void movePositionAbsolut(vector<double>);
    void movePositionHome();
    void movePositionIncremental(vector<double>);
    bool distanceToGoZero();
    int runPositionInput(String);
    bool runManual();
    void run();
    double calculateTime(vector<int>);
    void calculateMaxSpeed(vector<int>);
    vector<int> anglesToSteps(vector<double>);
    vector<double> getOffsets(); 
    void setHome();
    
};

#endif