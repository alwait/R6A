#ifndef STEERINGBASE_H
#define STEERINGBASE_H

#include "vector"
#include "Arduino.h"

using namespace std;

enum SteeringOption {
    none,
    start,
    offsetpos,
    estop,
    returnstop,
    home,
    disable,
    test,
    control,
    readpos,
    memread,
    memsave,
    memsetgo,
};

enum Condition {
    no,
    yes,
    off
};

class SteeringElement{
private:
    SteeringOption optionNumber;
    vector<String> optionNames;
    vector<Condition> conditions;
public:
    SteeringElement(SteeringOption, vector<String>, vector<Condition>);
    vector<String>& getElementNames() {return optionNames;};
    String& getElementName(size_t i) {return optionNames.at(i);};
    const SteeringOption& getElementOption() const {return optionNumber;};
    vector<Condition>& getElementConditions() {return conditions;};
};

class SteeringType{
private:
    SteeringOption optionNumber;
    int subOptionNumber;
public:
    SteeringType(); 
    SteeringType(SteeringOption);   
    SteeringType(SteeringOption, int);
    SteeringOption& getOptionNumber() {return optionNumber;};
    int& getSubOptionNumber() {return subOptionNumber;};
    void setSubOptionNumber(int sub) {subOptionNumber=sub;};
    SteeringType& getSteeringType() {return *this;};
    bool isTypeNone();
};

#endif // STEERINGBASE