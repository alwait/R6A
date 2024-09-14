#include "Steering.h"

SteeringMove::SteeringMove(){
    type=SteeringType();
    angles=nullptr;
}

SteeringMove::SteeringMove(SteeringType Type){
    type=Type;
    angles=nullptr;
}

SteeringMove::SteeringMove(SteeringType Type, vector<double>* Angles): type(Type), angles(Angles){
}

// Conditions: {shouldStop, shouldRun, shouldOffset, shouldHome}
Steering::Steering(){
    steering={
        {none,{""},{off, off, off, off}},
        {start,{"start"},{no, no, yes, off}},
        {offsetpos,{"offset"},{no, no, off, yes}},
        {estop,{"stop"},{no, yes, off, off}},
        {returnstop,{"return"},{yes, no, off, off}},
        {home,{"home"},{no, no, yes, no}},
        {disable,{"disable"},{off, off, off, off}},
        {test,{"test"},{no, no, yes, off}},
        {control,{"axisabs","axisinc","posabs","posinc"},{no, no, yes, off}}
    };
}

SteeringMove Steering::handle(String message, vector<bool> conditions){

    SteeringType type=this->readType(message);
    if(type.isTypeNone() || !this->isConditionSame(type.getOptionNumber(), conditions)) return SteeringMove();

    if(type.getOptionNumber()==control){
        vector<double>* angles = nullptr;
        if(type.getSubOptionNumber()==axisAbsolute || type.getSubOptionNumber()==axisIncremental){
            angles = new vector<double>(decodeAngles(message));
        }
        else if(type.getSubOptionNumber()==positionAbsolute || type.getSubOptionNumber()==positionIncremental){
            angles = new vector<double>(decodePosition(message)); 
        }
        else return SteeringMove();
        if(angles->size()!=NUM_OF_AXIS) return SteeringMove();
        else return SteeringMove(type, angles);
    }
    else return SteeringMove(type);
}

SteeringType Steering::readType(String message){
    SteeringOption option=none;
    int subOption=0;
    bool searched=false;

    for(size_t i=0; i<steering.size(); i++){  
        for(size_t j=0; j<this->getElementNames(i).size(); j++){
            if(message.startsWith(this->getElementName(i,j))){
                option=this->getElement(i).getElementOption();
                subOption=j;
                searched=true;
                break;
            };
        }
    }
    //if(option!=none) Serial.println(this->getElementName(option, subOption));
    return SteeringType(option, subOption);
}

bool Steering::isConditionSame(SteeringOption option, vector<bool> boolConditions){

    vector<SteeringElement>::iterator it = std::find_if(steering.begin(), steering.end(), [option](const SteeringElement& element) {
        return element.getElementOption() == option;
    });
    vector<Condition> baseConditions=it->getElementConditions();

    if(boolConditions.size()!=baseConditions.size()) return false;
    
    bool isSame=true;
    for(size_t i=0; i<boolConditions.size(); i++){
        if(baseConditions.at(i)!=off){
            if(baseConditions.at(i)!=boolConditions.at(i)){
                isSame=false;
            }
        }
    }
    return isSame;
}

vector<double> Steering::decodeAngles(String message){
    return vector<double>(NUM_OF_AXIS,0.0);
}

vector<double> Steering::decodePosition(String message){
    return vector<double>(NUM_OF_AXIS,1.0);
}