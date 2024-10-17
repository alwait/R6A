#include "StageMovement.h"

StageMovement::StageMovement(){
    loadedStages={TERMINATOR};
    loopStages=false;
    delaystart=0;
    delaytrue=false;
}

SteeringDecode StageMovement::decodeStage(float code){
    if(code<1000)
        return movement;
    else
        return delaystage;
}

int StageMovement::decodeMemoryMove(float code){
    return (int)code;
}

float StageMovement::decodeSpeed(float code){
    float speed=(code - (int)code)*10;
    if(speed==0)
        return 1;
    else 
        return speed;
}

float StageMovement::decodeDelayMs(float code){
    return (code-1000)*10;
}
