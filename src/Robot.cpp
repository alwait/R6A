#include "Robot.h"

Robot::Robot(std::vector<AccelStepper*>& Steppers, std::vector<Joint*>& Joints, Kinematics Kinematics)
    : steppers(Steppers), joints(Joints), offset(), stop(), move(Steppers,Joints), steering(), memory(), stageMovement(){
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

vector<float> Robot::getOffsets(){

  vector<float> steps;
  
  for (size_t i = 0; i < steppers.size(); ++i){
    steps.push_back(joints[i]->StepsOffset());
  }
  return steps;
}


bool Robot::isAngleLimitOk(vector<int> steps){
  for (size_t i = 0; i < STEPPERS_NUM; i++){
    if(steps.at(i)>joints.at(i)->getMaxSteps() || steps.at(i)<joints.at(i)->getMinSteps())
      return false;
  }
  return true;
}

bool Robot::isAngleLimitOk(vector<float> angles){
  for (size_t i = 0; i < STEPPERS_NUM; i++){
    if(angles.at(i)>joints.at(i)->getMaxAngle() || angles.at(i)<joints.at(i)->getMinAngle())
      return false;
  }
    return true;
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
    this->calculateMaxSpeed(this->anglesToSteps(this->getMove().getAngles()), move.getSpeedScaling());
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

String Robot::getStringVector(vector<float> vector){
  String output="{";
  for (size_t i = 0; i < vector.size(); i++){
    output+=vector.at(i);
    if(i<vector.size()-1) output+=";";
  }
  output+="}";
  return output;
}

vector<float> Robot::stepsToAngle(vector<int> steps){
  vector<float> angles;

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

vector<int> Robot::anglesToSteps(vector<float> angles){

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

void Robot::movePositionAbsolut(vector<float> angles){
  if(angles.size()>=steppers.size()){
    this->calculateMaxSpeed(this->anglesToSteps(angles));
    for (size_t i = 0; i < steppers.size(); ++i){
      steppers[i]->moveTo(joints[i]->AngleToSteps(angles[i]));
    }
  }
}

void Robot::movePositionIncremental(vector<float> angles){
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
  vector<float> angles;
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
    vector<float> currentAngles=this->currentAngles();
    if (angles.size() != currentAngles.size()) return 0;
    for (size_t i = 0; i < steppers.size(); ++i) {
      currentAngles[i] += angles[i];
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

int Robot::handleStages(bool looping){
  if(looping)
  return stageMode;
  else 
  return standbyMode;
}

vector<float> Robot::currentAngles(){
  vector<int> steps;
  for (size_t i = 0; i < steppers.size(); ++i){ 
    steps.push_back(steppers[i]->currentPosition());
  }
  return this->stepsToAngle(steps);
}

vector<float> Robot::currentPosition(){
  return this->getKinematics().forwardKinematics(this->currentAngles());
}

vector<float> Robot::currentPositionWork(){
  vector<float> position=this->getKinematics().forwardKinematics(this->currentAngles());
  vector<float> home=this->getHomePosition();
  for (size_t i = 0; i < position.size(); i++){
    position[i]-=home[i];
  }
    
  return position;
}

vector<float> Robot::getAngles(const std::vector<float>* Angles){
  vector<float> result(Angles->size(),-1); 
  if (Angles != nullptr) {
      for (size_t i = 0; i < Angles->size(); ++i) {
          result[i] = (*Angles)[i];
      }
  }
  return result;
}

vector<float> Robot::setAngles(const std::vector<float>* Angles){
  vector<float> result = this->currentAngles(); 
  if (Angles != nullptr) {
    if (result.size() == Angles->size()) {
      for (size_t i = 0; i < result.size(); ++i) {
        if(!isnan((*Angles)[i]))
          result[i] = (*Angles)[i];
      }
    } 
    else {
      return result;
    }
  }
  return result;
}

vector<float> Robot::addAngles(const std::vector<float>* Angles){
  vector<float> result = this->currentAngles(); 
  if (Angles != nullptr) {
    if (result.size() == Angles->size()) {
      for (size_t i = 0; i < result.size(); ++i) {
        if(!isnan((*Angles)[i]))
          result[i] += (*Angles)[i];
      }
    } 
    else {
      return result;
    }
  }
  return result;
}

vector<float> Robot::setPositions(const std::vector<float>* Positions){
  vector<float> result = this->currentPosition(); 
  vector<float> home = this->getHomePosition(); 

  if (Positions != nullptr){
    if (result.size() == Positions->size()){
      for (size_t i = 0; i < result.size(); ++i){
        if(!isnan((*Positions)[i])){
          result[i] = (*Positions)[i];
          result[i] += home[i];
        }
      }
    } 
    else {
      return result;
    }
  }
  return result;
}

vector<float> Robot::addPositions(const std::vector<float>* Positions){
  vector<float> result = this->currentPosition(); 
  if (Positions != nullptr) {
    if (result.size() == Positions->size()) {
      for (size_t i = 0; i < result.size(); ++i) {
        if(!isnan((*Positions)[i]))
          result[i] += (*Positions)[i];
      }
    } 
    else {
      return result;
    }
  }
  return result;
}

vector<float> Robot::returnPointerValues(const std::vector<float>* Values){
  vector<float> result; 
  if (Values != nullptr) {
    if (Values->size() == STEPPERS_NUM) {
      for (size_t i = 0; i < Values->size(); ++i) {
        result.push_back((*Values)[i]);
      }
    } 
    else {
      return vector<float>(STEPPERS_NUM,0.0);
    }
  }
  return vector<float>(STEPPERS_NUM,0.0);
}

void Robot::goHome(){
  vector<float> angles;
  for (size_t i = 0; i < steppers.size(); ++i){
    angles.push_back(0);
  }
  this->movePositionAbsolut(angles);
}

void Robot::calculateMaxSpeed(vector<int> steps, float scaling){

  if(steps.size()<steppers.size()) return;
  int maxDistance = 0;

  for (size_t i = 0; i < steppers.size(); ++i){
    int distance = abs(steppers[i]->currentPosition()-steps[i]);
    if (distance > maxDistance) {
      maxDistance = distance;
    }
  }

   for (size_t i = 0; i < steppers.size(); ++i){
    steppers[i]->setAcceleration((int)(((float)(abs(steppers[i]->currentPosition()-steps[i]))/(float)maxDistance)*(float)default_acceleration*scaling));
    steppers[i]->setMaxSpeed((int)(((float)(abs(steppers[i]->currentPosition()-steps[i]))/(float)maxDistance)*default_speed*scaling));
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

float Robot::calculateTime(vector<int> steps){

  int maxDistance = 0;
  float maxTime;

  if(steps.size()<steppers.size()) return 0;

  for (size_t i = 0; i < steppers.size(); ++i){
    int distance = abs(steppers[i]->currentPosition()-steps[i]);
    if (distance > maxDistance) {
      maxDistance = distance;
    }
  }

  float maxAccelDistance = (((float)default_speed)*((float)default_speed))/((float)default_acceleration*2.);

  if(2*maxAccelDistance >= maxDistance){ // triangle speed plot
    float timeAccel=sqrt((float)maxDistance/(float)default_acceleration);
    maxTime=timeAccel*2;
  }
  else { // trapeze speed plot
    float timeAccel=sqrt(maxAccelDistance/(float)default_acceleration);
    float timeMaxSpeed=((float)maxDistance-2*maxAccelDistance)/(float)default_speed;
    maxTime=timeAccel*2+timeMaxSpeed;
  }
  return maxTime;
}

String Robot::getStringCurrentPosition(){
  vector<float> positionVector=this->currentPositionWork();
  String positionString="Position {";
  for (size_t i = 0; i < positionVector.size(); i++){
    positionString+=positionVector.at(i);
    if(i<positionVector.size()-1) positionString+=";";
  }
  positionString+="}";
  return positionString;
}

String Robot::getStringCurrentAngles(){
  vector<float> anglesVector=this->currentAngles();
  String positionString="Axis {";
  for (size_t i = 0; i < anglesVector.size(); i++){
    positionString+=anglesVector.at(i);
    if(i<anglesVector.size()-1) positionString+=";";
  }
  positionString+="}";
  return positionString;
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