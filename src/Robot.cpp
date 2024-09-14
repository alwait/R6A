#include "Robot.h"

Robot::Robot(std::vector<AccelStepper*>& Steppers, std::vector<Joint*>& Joints, Kinematics Kinematics)
    : steppers(Steppers), joints(Joints), offset(), stop(), move(Steppers,Joints), steering(){
      running=false;
      enable=false;
      stage=0;
      kinematics=Kinematics;
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

void Robot::setHome(){
  for (size_t i = 0; i < steppers.size(); ++i) 
  {
    steppers[i]->setCurrentPosition(0);
    steppers[i]->moveTo(0);
  }
  running=false;
}

void Robot::moveOffset(){

  if(offset.getOffsetChange() && !offset.getOffsetChangeBlock()){ // first run
    this->calculateMaxSpeed(this->anglesToSteps(this->getOffsets()));
  }

  offset.OffsetChange(steppers, joints);

  RunningState state=offset.OffsetMove(steppers);
    
  if(state==Stop) running=false;
  else if(state==Running) running=true;
}

vector<double> Robot::getOffsets(){

  vector<double> steps;
  
  for (size_t i = 0; i < steppers.size(); ++i){
    steps.push_back(joints[i]->StepsOffset());
  }
  return steps;
}

void Robot::emergencyStop(){
  stop.emergencyChange(steppers);

  RunningState state=stop.emergencyMove(steppers);

  if(state==Stop){
    running=false;
    this->setMovingSpeedDefault();
  }
  else if(state==Running)
  {
    running=true;
  }
}

void Robot::moveSteering(){
  if(move.getChange() && !move.getChangeBlock()){ // first run
    this->calculateMaxSpeed(this->anglesToSteps(this->getMove().getAngles()));
  }
  move.changeMain();

  RunningState state=move.moveMain();

  if(state==Stop) running=false;
  else if(state==Running) running=true;
}

void Robot::emergencyRelease(){
  stop.returnChange(steppers);

  RunningState state=stop.returnMove(steppers);

  if(state==Stop){
    offset.setOffsetChange(false);
    offset.setOffsetChangeBlock(false);
    running=false;
    stage=0;
    this->setMovingSpeedDefault();
  }
  else if(state==Running){
    running=true;
  }

  if(state==Stop && !offset.getOffsetSet())
  {
    digitalWrite(ENA_PIN,HIGH);
    digitalWrite(LED_PIN,HIGH);
  }
    
}

void Robot::setMovingSpeed(float multiplier){
  for (size_t i = 0; i < steppers.size(); ++i) {
    steppers[i]->setMaxSpeed((int)((float)default_speed*multiplier));
    steppers[i]->setAcceleration((int)((float)default_acceleration*multiplier));
  }
}

void Robot::setMovingSpeedDefault(){
  for (size_t i = 0; i < steppers.size(); ++i) {
    steppers[i]->setMaxSpeed(default_speed);
    steppers[i]->setAcceleration(default_acceleration);
  }
}

vector<double> Robot::stepsToAngle(vector<int> steps){
  vector<double> angles;

  if(steps.size()<steppers.size()){
    for (size_t i = 0; i < steppers.size(); ++i){
      angles.push_back(0);
    }
    return angles;
  } 
  
  for (size_t i = 0; i < steppers.size(); ++i){
    angles.push_back(joints[i]->StepsToAngle(steps[i]));
  }
  return angles;
}

vector<int> Robot::anglesToSteps(vector<double> angles){

  vector<int> steps;

  if(angles.size()<steppers.size()){
    for (size_t i = 0; i < steppers.size(); ++i){
      steps.push_back(0);
    }
    return steps;
  } 
  
  for (size_t i = 0; i < steppers.size(); ++i){
    steps.push_back(joints[i]->AngleToSteps(angles[i]));
  }
  return steps;

}

void Robot::movePositionHome(){
  this->movePositionAbsolut({0,0,0,0,0,0});
}

void Robot::movePositionAbsolut(vector<double> angles){
  if(angles.size()>=steppers.size()){
    this->calculateMaxSpeed(this->anglesToSteps(angles));
    for (size_t i = 0; i < steppers.size(); ++i){
      steppers[i]->moveTo(joints[i]->AngleToSteps(angles[i]));
    }
  }
}

void Robot::movePositionIncremental(vector<double> angles){
  if(angles.size()>=steppers.size()){
    this->calculateMaxSpeed(this->anglesToSteps(angles));
    for (size_t i = 0; i < steppers.size(); ++i){
      steppers[i]->moveTo(steppers[i]->currentPosition()+joints[i]->AngleToSteps(angles[i]));
    }
  }
}
 
bool Robot::distanceToGoZero(){
  for (size_t i = 0; i < steppers.size(); ++i) {
    if(steppers[i]->distanceToGo()!=0){
      return false;
    }
  }
  return true;
}

void Robot::run(){
   for (size_t i = 0; i < steppers.size(); ++i){
    steppers[i]->run();
  }
}

bool Robot::runManual(){
  if(this->distanceToGoZero()){
    running=false;  
    return false;
  }
  else{
    this->run();
    running=true;
    return true;
  }
}

int Robot::runPositionInput(String message){

  int messageSize = message.length();
  vector<double> angles;
  for (size_t i = 0; i < steppers.size(); ++i){
    angles.push_back(0);
  }
  int i=7;

  while(messageSize>=i+3){
    if(message[i]==';'){
      i++;
      break;
    }

    int sign = 1;
    int digitStartIndex;
    int stepperNum=-1;

    if(isDigit(message[i])){
      stepperNum = ((String)message[i]).toInt();
      if(stepperNum>steppers.size() || stepperNum<=0){
        //Serial.println("return 0 - incorrect stepper number");
        return 0;
      }
      stepperNum--;
      i++;
    }
    else{
      //Serial.println("return 0 - no stepper number");
      return 0;
    }

    char action;
    action = message[i];
    if(action==' ' || isDigit(action)){
      //Serial.println("return 0 - error in action option");
      return 0;
    }

    i++;
    if(message[i]=='m' && messageSize>i){
      sign=-1;
      i++;
    }
    else if(messageSize<=i){
      //Serial.println("return 0 - no info about move angle");
      return 0;
    }

    digitStartIndex=i;

    int angle=0;

    while(messageSize>=i){
      if(isDigit(message[i])){
        i++;
      }
      else{
        angle = message.substring(digitStartIndex, i+1).toInt()*sign;
        break;
      }
    }

    switch (action)
    {
    case 'a':
      angles[stepperNum]=angle;
      break;

    default:
      break;
    }
    i=i+1;
  }

  if(i>7){
    vector<double> currentAngles=this->currentAngles();
    if (angles.size() != currentAngles.size()) return 0;
    for (size_t i = 0; i < steppers.size(); ++i) {
      currentAngles[i] += angles[i];
      Serial.print("angle ");
      Serial.print(i);
      Serial.print(" ");
      Serial.println(currentAngles[i]);
    }
    this->getMove().setAngles(currentAngles);
    
    //this->movePositionIncremental(angles);
    return 2;
  }
  else{
    //Serial.println("return 0 - no data");
    return 0;
  }

}

vector<double> Robot::currentAngles(){

  vector<int> steps;
  for (size_t i = 0; i < steppers.size(); ++i){ 
    steps.push_back(steppers[i]->currentPosition());
  }
  return this->stepsToAngle(steps);
}

void Robot::goHome(){
  vector<double> angles;
  for (size_t i = 0; i < steppers.size(); ++i){
    angles.push_back(0);
  }
  this->movePositionAbsolut(angles);
}

void Robot::calculateMaxSpeed(vector<int> steps){

  if(steps.size()<steppers.size()) return;
  int maxDistance = 0;

  for (size_t i = 0; i < steppers.size(); ++i){
    int distance = abs(steppers[i]->currentPosition()-steps[i]);
    if (distance > maxDistance) {
      maxDistance = distance;
    }
  }

   for (size_t i = 0; i < steppers.size(); ++i){
    steppers[i]->setAcceleration((int)(((double)(abs(steppers[i]->currentPosition()-steps[i]))/(double)maxDistance)*(double)default_acceleration));
    steppers[i]->setMaxSpeed((int)(((double)(abs(steppers[i]->currentPosition()-steps[i]))/(double)maxDistance)*default_speed));
  }

}

bool Robot::isHome(){
  bool home=true;
  for (size_t i = 0; i < steppers.size(); ++i) 
  {
    if(steppers[i]->currentPosition()!=0){
      home=false;
    }
  }
  return home;
}

double Robot::calculateTime(vector<int> steps){

  int maxDistance = 0;
  double maxTime;

  if(steps.size()<steppers.size()) return 0;

  for (size_t i = 0; i < steppers.size(); ++i){
    int distance = abs(steppers[i]->currentPosition()-steps[i]);
    if (distance > maxDistance) {
      maxDistance = distance;
    }
  }

  double maxAccelDistance = (((double)default_speed)*((double)default_speed))/((double)default_acceleration*2.);

  if(2*maxAccelDistance >= maxDistance){ // triangle speed plot
    double timeAccel=sqrt((double)maxDistance/(double)default_acceleration);
    maxTime=timeAccel*2;
  }
  else { // trapeze speed plot
    double timeAccel=sqrt(maxAccelDistance/(double)default_acceleration);
    double timeMaxSpeed=((double)maxDistance-2*maxAccelDistance)/(double)default_speed;
    maxTime=timeAccel*2+timeMaxSpeed;
  }
  return maxTime;
}

int Robot::moveTestStages(){

  switch (stage){
  case 0:
    //movePositionAbsolut({90,30,120,180,90,180});
    movePositionAbsolut(this->getKinematics().inverseKinematics({179.63+100,0,184.88-50,0,0,0}));
    running=true;
    stage++;
    break;
  case 1:
    this->run();
    if(this->distanceToGoZero())
    {
      //movePositionIncremental({45, -45, 60, -135, -135, -360});
      movePositionAbsolut(this->getKinematics().inverseKinematics({179.63+100,100,184.88-50,0,0,0}));
      stage++;
    }
    break;
  case 2:
    this->run();
    if(this->distanceToGoZero())
    {
      //movePositionAbsolut({45, 60, 120, 45, 45, 90});
      movePositionAbsolut(this->getKinematics().inverseKinematics({179.63+100,0,184.88-130,0,0,0}));
      stage++;
    }
    break;  
   case 3:
    this->run();
    if(this->distanceToGoZero())
    {
      //movePositionAbsolut({45, 60, 120, 45, 45, 90});
      movePositionAbsolut(this->getKinematics().inverseKinematics({179.63+100,-100,184.88-60,0,0,0}));
      stage++;
    }
    break;  
   case 4:
    this->run();
    if(this->distanceToGoZero())
    {
      //movePositionAbsolut({45, 60, 120, 45, 45, 90});
      movePositionAbsolut(this->getKinematics().inverseKinematics({179.63-50,0,184.88+100,0,0,0}));
      stage++;
    }
    break;  
   case 5:
    this->run();
    if(this->distanceToGoZero())
    {
      //movePositionAbsolut({45, 60, 120, 45, 45, 90});
      movePositionAbsolut(this->getKinematics().inverseKinematics({179.63-100,-100,184.88+100,0,0,0}));
      stage++;
    }
    break;  
  case 6:
    this->run();
    if(this->distanceToGoZero())
    {
      movePositionHome();
      stage++;
    }
    break;  
  case 7:
    this->run(); 
    if(this->distanceToGoZero())
    {
      stage++;
    }
    break;
  case 8:
    running=false;
    stage=0;
    return 0;
  }
  return 1;

}