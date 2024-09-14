#ifndef ROBOT_H
#define ROBOT_H

#include "vector"
#include "Offset.h"
#include "GPIO.h"
#include "EmergencyStop.h"
#include "Kinematics.h"
#include "Move.h"
#include <AccelStepper.h>
#include "Arduino.h"
#include "Steering.h"


#define default_speed 4000
#define default_acceleration 2000

using namespace std;

class Robot{
private:
    vector<AccelStepper*> steppers;
    vector<Joint*> joints;
    Offset offset;
    EmergencyStop stop;
    Kinematics kinematics;
    Move move;
    Steering steering;
    bool running;
    bool enable;
    int stage;
    
public:
    Robot(vector<AccelStepper*>&, vector<Joint*>&, Kinematics);
    AccelStepper* getStepper(size_t) const;
    Offset& getOffset() {return offset;};
    EmergencyStop& getEmergencyStop() {return stop;};
    Kinematics& getKinematics() {return kinematics;};
    Move& getMove() {return move;};
    Steering& getSteering() {return steering;};
    bool isRunning() {return running;}
    bool isEnable() {return running;}
    bool isStop() {return stop.isStop();}
    void moveOffset();
    void emergencyStop();
    void emergencyRelease();
    void moveSteering();
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
    vector<double> stepsToAngle(vector<int>);
    vector<double> getOffsets(); 
    vector<double> currentAngles();
    void setHome();
    void goHome();
    bool isHome();
    
};

#endif