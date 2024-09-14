#include "Move.h"

Move::Move(const std::vector<AccelStepper*>& Steppers, const vector<Joint*>& Joints):
 Options(), steppers(Steppers), joints(Joints), angles(Steppers.size(), 0.0){
};   

void Move::change(){
    for (size_t i = 0; i < steppers.size(); ++i) {
        steppers[i]->moveTo(joints[i]->AngleToSteps(angles[i])); 
    }
}

bool Move::move(){
    running=false;
    for (size_t i = 0; i < steppers.size(); ++i) 
    {
      steppers[i]->run();
      if(steppers[i]->distanceToGo()!=0) running=true;
    }
return running;
};