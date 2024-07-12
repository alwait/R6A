#include "EmergencyStop.h"


EmergencyStop::EmergencyStop(){
    stopSet=false;
    stopChange=false;
    stopChangeBlock=false;
}

void EmergencyStop::emergencyChange(const std::vector<AccelStepper*>& steppers){
    if(stopChange && !stopChangeBlock){
        for (size_t i = 0; i < steppers.size(); ++i) 
        {
            steppers[i]->stop();
        }
        stopSet=true;
        stopChangeBlock=true;
    }
}

RunningState EmergencyStop::emergencyMove(const std::vector<AccelStepper*>& steppers){
    if(stopChange && stopChangeBlock)
    {
        bool running=false;
        for (size_t i = 0; i < steppers.size(); ++i) 
        {
            steppers[i]->run();
            if(steppers[i]->distanceToGo()!=0) running=true;
        }
        if(!running)
        {  
            stopChange=false;
            stopChangeBlock=false;
            return Stop;
        }
        return Running;
    }
    return NotRunning;
}

void EmergencyStop::returnChange(const std::vector<AccelStepper*>& steppers){
    if(unlockChange && !unlockChangeBlock){
        for (size_t i = 0; i < steppers.size(); ++i) 
        {
            steppers[i]->moveTo(0);
        }
        unlockChangeBlock=true;
    }
}

RunningState EmergencyStop::returnMove(const std::vector<AccelStepper*>& steppers){
    if(unlockChange && unlockChangeBlock)
    {
        bool running=false;
        for (size_t i = 0; i < steppers.size(); ++i) 
        {
            steppers[i]->run();
            if(steppers[i]->distanceToGo()!=0) running=true;
        }
        if(!running)
        {  
            unlockChange=false;
            unlockChangeBlock=false;
            stopSet=false;
            return Stop;
        }
        return Running;
    }
    return NotRunning;
}

