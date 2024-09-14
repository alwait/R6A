#ifndef STEERING_H
#define STEERING_H

#include "SteeringBase.h"
#include <algorithm> 

#define STEPPERS_NUM 6
#define NUM_OF_AXIS 6

using namespace std;

enum ControlOptions {
    axisAbsolute,
    axisIncremental,
    positionAbsolute,
    positionIncremental,
};

class SteeringMove{
private:
    SteeringType type;
    vector<double>* angles;
public:
    SteeringMove(); 
    SteeringMove(SteeringType);   
    SteeringMove(SteeringType, vector<double>*);
    SteeringType& getType() {return type;};
    SteeringOption& getSteeringOption() {return type.getOptionNumber();};
    vector<double>* getAngles() {return angles;};
    void updateAngles(const vector<double>& Angles) {*angles = Angles;};
};

class Steering{
private:
    vector<SteeringElement> steering;
public:
    Steering();
    SteeringMove handle(String, vector<bool>);
    SteeringType readType(String);
    vector<double> decodeAngles(String);
    vector<double> decodePosition(String);
    SteeringElement& getElement(size_t i) {return steering.at(i);};
    vector<String>& getElementNames(size_t i) {return steering.at(i).getElementNames();};
    String& getElementName(size_t i, size_t j) {return steering.at(i).getElementName(j);};
    bool isConditionSame(SteeringOption, vector<bool>);
};


#endif // STEERING