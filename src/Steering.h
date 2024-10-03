#ifndef STEERING_H
#define STEERING_H

#include "SteeringBase.h"
#include <algorithm> 

#define STEPPERS_NUM 6
#define NUM_OF_AXIS 6

#define MAX_SPEED_SCALING 1.5
#define MIN_SPEED_SCALING 0.1

using namespace std;

enum ControlOptions {
    axisAbsolute,
    axisIncremental,
    positionAbsolute,
    positionIncremental,
};

enum ReadOptions {
    position,
    axis,
};

class SteeringMove{
private:
    SteeringType type;
    vector<float>* angles;
    float speed;
public:
    SteeringMove(); 
    SteeringMove(SteeringType);   
    SteeringMove(SteeringType, vector<float>*, float Speed=1);
    SteeringType& getType() {return type;};
    SteeringOption& getSteeringOption() {return type.getOptionNumber();};
    int& getSteeringSubOption() {return type.getSubOptionNumber();};
    vector<float>* getValues() {return angles;};
    void setSubOptionNumber(int sub) {type.setSubOptionNumber(sub);};
    void updateAngles(const vector<float>& Angles) {*angles = Angles;};
    void resetMove() {type=SteeringType(); angles=nullptr;};
    void setSpeed(float scale){speed=scale;};
    float getSpeed(){return speed;};
};

class Steering{
private:
    vector<SteeringElement> steering;
public:
    Steering();
    SteeringMove handle(String, vector<bool>);
    SteeringType readType(String);
    vector<float>* decodeAngles(String);
    int decodeNumber(String);
    SteeringElement& getElement(size_t i) {return steering.at(i);};
    vector<String>& getElementNames(size_t i) {return steering.at(i).getElementNames();};
    String& getElementName(size_t i, size_t j) {return steering.at(i).getElementName(j);};
    bool isConditionSame(SteeringOption, vector<bool>);
    float decodeSpeed(String);
};


#endif // STEERING