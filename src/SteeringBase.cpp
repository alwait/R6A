#include "SteeringBase.h"

SteeringElement::SteeringElement(SteeringOption OptionNumber, vector<String> OptionNames, vector<Condition> Conditions){
    optionNumber=OptionNumber;
    optionNames=OptionNames;
    conditions=Conditions;
}

SteeringType::SteeringType(){
    optionNumber=none;
    subOptionNumber=-1;
}

SteeringType::SteeringType(SteeringOption OptionNumber){
    optionNumber=OptionNumber;
    subOptionNumber=-1;
} 

SteeringType::SteeringType(SteeringOption OptionNumber, int SubOptionNumber){
    optionNumber=OptionNumber;
    subOptionNumber=SubOptionNumber;
}

bool SteeringType::isTypeNone(){
    if(optionNumber==none) return true;
    else return false;
}
