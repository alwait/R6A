#include "Robot.h"

Robot::Robot(std::vector<AccelStepper*>& Steppers, std::vector<Joint*>& Joints)
    : steppers(Steppers), joints(Joints), offset(), stop(){
      running=false;
      enable=false;
      stage=0;
      for (size_t i = 0; i < steppers.size(); ++i) {
        steppers[i]->setMaxSpeed(default_speed);
        steppers[i]->setAcceleration(default_acceleration);
    }
}

AccelStepper* Robot::getStepper(size_t index) const {
    if (index > 0 && index <= steppers.size()) {
      return steppers[index - 1];
    } else {
      return nullptr;
    }
}

void Robot::moveOffset(){
    offset.OffsetChange(steppers, joints);

    RunningState state=offset.OffsetMove(steppers);

    if(state==Stop) running=false;
    else if(state==Running) running=true;
}

void Robot::emergencyStop(){
    stop.emergencyChange(steppers);

    RunningState state=stop.emergencyMove(steppers);

    if(state==Stop) running=false;
    else if(state==Running)
    {
      running=true;
      offset.setOffsetChange(false);
      offset.setOffsetChangeBlock(false);
    }
}

void Robot::emergencyRelease(){
    stop.returnChange(steppers);

    RunningState state=stop.returnMove(steppers);

    if(state==Stop) running=false;
    else if(state==Running) running=true;

    if(state==Stop && !offset.getOffsetSet())
    {
      digitalWrite(ENA_PIN,HIGH);
      digitalWrite(LED_PIN,HIGH);
    }
    
}

int Robot::moveTestStages(){
    if(stage==0)
    {
      getStepper(1)->moveTo(joints[0]->AngleToSteps(90));
      getStepper(2)->moveTo(400);  
      getStepper(3)->moveTo(1000); 
      stage++;
      running=true;
    }
    else if(stage==1)
    {
      getStepper(1)->run();
      getStepper(2)->run();
      getStepper(3)->run();
      if(getStepper(1)->distanceToGo()==0 && getStepper(2)->distanceToGo()==0 && getStepper(3)->distanceToGo()==0)
      {
        getStepper(1)->moveTo(-1000);
        getStepper(3)->moveTo(2600); 
        getStepper(4)->moveTo(2000);
        stage++;
      }
    }
    else if(stage==2)
    {
      getStepper(1)->run();
      getStepper(3)->run();
      getStepper(4)->run();
      if(getStepper(1)->distanceToGo()==0 && getStepper(3)->distanceToGo()==0 && getStepper(4)->distanceToGo()==0)
      {
        getStepper(1)->moveTo(1000);
        getStepper(3)->moveTo(1400); 
        getStepper(4)->moveTo(0);
        stage++;
      }
    } 
    else if(stage==3)
    {
      getStepper(1)->run();
      getStepper(3)->run();
      getStepper(4)->run();
      if(getStepper(1)->distanceToGo()==0 && getStepper(3)->distanceToGo()==0 && getStepper(4)->distanceToGo()==0)
      {
        getStepper(1)->moveTo(0);
        getStepper(2)->moveTo(0); 
        getStepper(3)->moveTo(0); 
        stage++;
      }
    } 
    else if(stage==4)
    {
      getStepper(1)->run();
      getStepper(2)->run();
      getStepper(3)->run();
      if(getStepper(1)->distanceToGo()==0 && getStepper(2)->distanceToGo()==0 && getStepper(3)->distanceToGo()==0)
      {
        stage++;
      }
    }
    else if(stage==5)
    {
      running=false;
      stage=0;
      return 0;
    }
    return 1;
}