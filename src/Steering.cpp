#include "Steering.h"

SteeringMove::SteeringMove(){
    type=SteeringType();
    angles=nullptr;
    speed=1;
}

SteeringMove::SteeringMove(SteeringType Type){
    type=Type;
    angles=nullptr;
    speed=1;
}

SteeringMove::SteeringMove(SteeringType Type, vector<float>* Angles, float Speed): type(Type), angles(Angles){
    speed=Speed;
}

// Conditions: {shouldEStop, shouldRun, shouldOffset, shouldHome}
Steering::Steering(){
    steering={
        {none,{""},{off, off, off, off}},
        {start,{"start"},{no, no, yes, off}},
        {offsetpos,{"offset"},{no, no, off, yes}},
        {estop,{"stop"},{off, yes, off, off}},
        {returnstop,{"return"},{yes, no, off, off}},
        {home,{"home"},{no, no, yes, no}},
        {disable,{"disable"},{off, off, off, off}},
        {test,{"test"},{no, no, yes, off}},
        {control,{"axisabs","axisinc","posabs","posinc"},{no, no, yes, off}},
        {readpos,{"posread","axisread"},{off, no, yes, off}},
        {memread,{"memread"},{{off, off, off, off}}},
        {memsave,{"memsave"},{{off, off, yes, off}}},
        {memsetgo,{"memset"},{{no, no, yes, off}}},
        {memstageinit,{"init"},{{off, off, off, off}}},
        {memstageread,{"stageread"},{{off, off, off, off}}},
        {memstageload,{"load"},{{off, no, off, off}}},
        {memstagereadloaded,{"stagereadloaded"},{{off, off, off, off}}},
        {memstageloop,{"loop"},{{off, off, off, off}}}
    };
}

SteeringMove Steering::handle(String message, vector<bool> conditions){

    SteeringType type=this->readType(message);
    if(type.isTypeNone() || !this->isConditionSame(type.getOptionNumber(), conditions)) return SteeringMove();

    if(type.getOptionNumber()==control){
        vector<float>* angles = nullptr;
        if(type.getSubOptionNumber()==axisAbsolute || type.getSubOptionNumber()==axisIncremental){
            angles = decodeAngles(message);
        }
        else if(type.getSubOptionNumber()==positionAbsolute || type.getSubOptionNumber()==positionIncremental){
            angles = decodeAngles(message); 
        }
        float speed=decodeSpeed(message);
        if(angles==nullptr || angles->size()!=NUM_OF_AXIS){
            delete angles;
            return SteeringMove();
        }
        else return SteeringMove(type, angles, speed);
    }
    else if(type.getOptionNumber()==memread || type.getOptionNumber()==memsave || 
        type.getOptionNumber()==memsetgo || type.getOptionNumber()==memstageinit ||
        type.getOptionNumber()==memstageread || type.getOptionNumber()==memstageload)
        {
        type.setSubOptionNumber(decodeNumber(message));
        if(type.getOptionNumber()==memstageinit){
            vector<float>* code = nullptr;
            code = decodeStagesAdvanced(message);
            return SteeringMove(type, code);
        }
    }
    return SteeringMove(type);
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

vector<float>* Steering::decodeAngles(String message){
    char delimiter=';';
    int startIndex=message.indexOf('(');
    //int endIndex=message.length();
    int endIndex=message.indexOf(')');
    if(endIndex==-1) endIndex=message.length();
    else endIndex++;
    if(startIndex==-1 || startIndex>=endIndex-1) return nullptr;
    if(endIndex==-1) return nullptr;

    vector<float> angles;
    for(int i=startIndex+1; i<endIndex-1; i++){
        if(isDigit(message[i]) || message[i]=='-'){
            int sign=1;
            if(message[i]=='-'){
                sign=-1;
                i++;
                if(i>=endIndex) break;
            }
            int digitStart=i;
            int digitEnd;
            while(isDigit(message[i])){
                digitEnd=i;
                i++;
            }
            float angle=message.substring(digitStart, i).toInt()*sign;
            angles.push_back(angle);
        }
        else if(message[i]==delimiter && !isDigit(message[i-1])){
            angles.push_back(NAN);
        }
    }
    while(angles.size()<NUM_OF_AXIS){
        angles.push_back(NAN);
    }
    
    vector<float>* anglesP = new vector<float>(angles);
    return anglesP;
}

vector<float>* Steering::decodeStagesAdvanced(String message){
    char delimiter=';';
    char speedChar='s';
    char waitChar='w';
    char comma='.';
    int waitAdd=1000;
    int waitDiv=10;
    int startIndex=message.indexOf('(');
    int endIndex=message.indexOf(')');
    if(endIndex==-1) endIndex=message.length();
    else endIndex++;
    if(startIndex==-1 || startIndex>=endIndex-1) return nullptr;
    if(endIndex==-1) return nullptr;

    vector<float> moveCode;
    for(int i=startIndex+1; i<endIndex-1; i++){
        if(isDigit(message[i])){
            int digitStart=i;
            bool isWaiting=false;
            if(message[i]==waitChar){
                i++;
                isWaiting=true;
            }
            while(isDigit(message[i])){
                i++;
            }
            float position=message.substring(digitStart, i).toInt();
            Serial.print("possub:");
            Serial.println(message.substring(digitStart, i));
            if(isWaiting){
                position/=waitDiv;
                position+=waitAdd;
            }
            digitStart=i;
            if(message[i]==speedChar){
                while(isDigit(message[i]) || message[i]==comma){
                    i++;
                }
                float speedcorr =message.substring(digitStart, i).toFloat();
                Serial.print("speedcorr:");
                Serial.println(message.substring(digitStart, i));
                if(speedcorr>9) speedcorr=9;
                else if(speedcorr<0.1) speedcorr=0.1;
                position+=speedcorr/10;
            }
                i++;
                moveCode.push_back(position);
        }
    }
    moveCode.push_back(-1000);

    vector<float>* moveCodeP = new vector<float>(moveCode);
    return moveCodeP;
}

int Steering::decodeNumber(String message){
    int startBackIndex=message.indexOf('(')-1;
    if(startBackIndex==-1) startBackIndex=message.length()-1;
    for (size_t i = message.length()-1; i > 0; i--){
        if(isDigit(message[i])){
         startBackIndex=i;
         break;
        }
        else if(i<=1) return -1;
    }
    int stopBackIndex=startBackIndex;
    for (size_t i = startBackIndex; i > 0; i--){
        if(isDigit(message[i])){
            stopBackIndex=i;
        }
        else break;
    }
    return message.substring(stopBackIndex, startBackIndex+1).toInt();
}


float Steering::decodeSpeed(String message){
    int startIndex=message.lastIndexOf('s');
    int checkIndex=message.indexOf('(');
    int endIndex=message.length();
    if(startIndex==-1 || checkIndex>startIndex) 
        return 1;
    else{
        float speed=abs(message.substring(startIndex+1, endIndex-1).toFloat());
        if(speed>=MAX_SPEED_SCALING)
            return MAX_SPEED_SCALING;
        else if(speed<=MIN_SPEED_SCALING)
            return MIN_SPEED_SCALING;
        else return speed;
    }
}
