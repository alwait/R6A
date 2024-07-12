#include "Offset.h"
#include "GPIO.h"

Offset::Offset(){
    offsetSet=false;
    offsetChange=false; 
    offsetChangeBlock=false;
}

void Offset::OffsetChange(const std::vector<AccelStepper*>& steppers, const std::vector<Joint*>& joints){
    if(offsetChange && !offsetChangeBlock)
    {
      for (size_t i = 0; i < steppers.size(); ++i) {
        if (!offsetSet) {
          steppers[i]->moveTo(joints[i]->StepsOffset()); 
        } 
        else {
          steppers[i]->moveTo(-joints[i]->StepsOffset()); 
        }
      }
      offsetChangeBlock=true;
    }
}

RunningState Offset::OffsetMove(const std::vector<AccelStepper*>& steppers){
    if(offsetChange && offsetChangeBlock)
    {
      bool running=false;
      for (size_t i = 0; i < steppers.size(); ++i) 
      {
        steppers[i]->run();
        if(steppers[i]->distanceToGo()!=0) running=true;
      }

      if(!running)
      {  
        offsetSet=!offsetSet;
        for (size_t i = 0; i < steppers.size(); ++i) 
        {
          steppers[i]->setCurrentPosition(0);
        }

        if(!offsetSet)
        {
          digitalWrite(ENA_PIN,HIGH);
          digitalWrite(LED_PIN,HIGH);
        }
        offsetChange=false;
        offsetChangeBlock=false;
        return Stop;
      }
      return Running;
    }
    return NotRunning;
}