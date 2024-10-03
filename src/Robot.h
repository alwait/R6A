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
#include "Memory.h"


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
    Memory memory;
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
    Memory& getMemory() {return memory;};
    vector<float> getHomePosition() {return kinematics.getHomePosition();};
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
    void movePositionAbsolut(vector<float>);
    void movePositionHome();
    void movePositionIncremental(vector<float>);
    bool distanceToGoZero();
    int runPositionInput(String);
    bool runManual();
    void run();
    float calculateTime(vector<int>);
    void calculateMaxSpeed(vector<int>, float scaling=1);
    vector<int> anglesToSteps(vector<float>);
    vector<float> stepsToAngle(vector<int>);
    vector<float> getOffsets(); 
    vector<float> currentAngles();
    vector<float> currentPosition();
    vector<float> currentPositionWork();
    String getStringCurrentPosition();
    String getStringCurrentAngles();
    String getStringVector(vector<float>);
    void setHome();
    void goHome();
    bool isHome();
    vector<float> setAngles(const std::vector<float>*);
    vector<float> setPositions(const std::vector<float>*);
    vector<float> addAngles(const std::vector<float>*);
    vector<float> addPositions(const std::vector<float>*);
    vector<float> returnPointerValues(const std::vector<float>*);
    bool isAngleLimitOk(vector<int>);
    bool isAngleLimitOk(vector<float>);
    void memoryInit(){memory.mount();};
};

#endif