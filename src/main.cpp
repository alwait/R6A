#include <Arduino.h>
#include <AccelStepper.h>
#include <vector>
#include "BluetoothSerial.h"
#include "Robot.h"
#include "GPIO.h"

AccelStepper stepper1(AccelStepper::DRIVER, STEP_PIN_1, DIR_PIN_1);
Joint joint1(20, 120);

AccelStepper stepper2(AccelStepper::DRIVER, STEP_PIN_2, DIR_PIN_2);
Joint joint2(20, 100, 35);

AccelStepper stepper3(AccelStepper::DRIVER, STEP_PIN_3, DIR_PIN_3);
Joint joint3(20, 80, 65);

AccelStepper stepper4(AccelStepper::DRIVER, STEP_PIN_4, DIR_PIN_4);
Joint joint4(20, 60);

AccelStepper stepper5(AccelStepper::DRIVER, STEP_PIN_5, DIR_PIN_5);
Joint joint5(16, 40);

AccelStepper stepper6(AccelStepper::DRIVER, STEP_PIN_6, DIR_PIN_6);
Joint joint6;

std::vector<AccelStepper*> steppers = {&stepper1, &stepper2, &stepper3, &stepper4, &stepper5, &stepper6};
std::vector<Joint*> joints = {&joint1, &joint2, &joint3, &joint4, &joint5, &joint6};

Offset offset;

Robot r6a(steppers, joints);

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;
String message = "";
char incomingChar;

bool test=false;
bool robotMoving=false, stop=false, stopChange=false, returnChange=false;
int stage=-1;

void emergencyStop(){
  stepper1.stop();
  stepper2.stop();
  stepper3.stop();
  stepper4.stop();
  stepper5.stop();
  stepper6.stop();

  stopChange=true;
  stage=-1;
  stop=true;
  returnChange=false;
  robotMoving=false;

  SerialBT.println("EMERGENCY STOP");
};

void emergencyReturn(){
  stepper1.moveTo(0);
  stepper2.moveTo(0);
  stepper3.moveTo(0);
  stepper4.moveTo(0);
  stepper5.moveTo(0);
  stepper6.moveTo(0);

  returnChange=true;
  robotMoving=true;
};

void setup() {

  pinMode(BTN_PIN, INPUT_PULLUP);
  pinMode(ENA_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);

  digitalWrite(ENA_PIN, HIGH);
  digitalWrite(LED_PIN, HIGH);
  
  Serial.begin(115200);

  SerialBT.begin("R6A Test Bluetooth Connection");

  for (size_t i = 0; i < steppers.size(); ++i) {
        steppers[i]->setMaxSpeed(2000);
        steppers[i]->setAcceleration(1000);
    }

}

void loop()
{
  if (SerialBT.available()){
    char incomingChar = SerialBT.read();
    if (incomingChar != '\n'){
      message += String(incomingChar);
    }
    else{
      message = "";
    }
  }

  // Bluetooth control
  if (message =="start" && offset.getOffsetSet() && !robotMoving && !stop){
    stage=0;
  }
  else if (message =="offset" && !stop){
    digitalWrite(ENA_PIN,LOW);
    digitalWrite(LED_PIN,LOW);
    offset.setOffsetChange(true);
    //r6a.getOffset().setOffsetChange(true);
  }
  else if (message =="stop" && robotMoving){
    emergencyStop();
  }
  else if (message =="return" && stop){
    emergencyReturn();
  }

  // Button control
  if (digitalRead(BTN_PIN) == LOW && !offset.getOffsetSet() && !offset.getOffsetChangeBlock() && !robotMoving && !stop)
  {
    digitalWrite(ENA_PIN,LOW);
    digitalWrite(LED_PIN,LOW);
    offset.setOffsetChange(true);
  }
  else if (digitalRead(BTN_PIN) == LOW && offset.getOffsetSet() && !offset.getOffsetChangeBlock() && !robotMoving && !stop)
  {
    stage=0;
  }

  // Emergency stop
  if(digitalRead(ENA_PIN) == LOW && stopChange){
      stepper1.run();
      stepper2.run();
      stepper3.run();
      stepper4.run();
      stepper5.run();
      stepper6.run();
      if(stepper1.distanceToGo()==0 && stepper2.distanceToGo()==0 && stepper3.distanceToGo()==0 && stepper4.distanceToGo()==0 && stepper5.distanceToGo()==0 && stepper6.distanceToGo()==0)
      {
        stopChange=false;
      }
  }
  
  // Return from emergency stop
  if(digitalRead(ENA_PIN) == LOW && returnChange){
      stepper1.run();
      stepper2.run();
      stepper3.run();
      stepper4.run();
      stepper5.run();
      stepper6.run();
      if(stepper1.distanceToGo()==0 && stepper2.distanceToGo()==0 && stepper3.distanceToGo()==0 && stepper4.distanceToGo()==0 && stepper5.distanceToGo()==0 && stepper6.distanceToGo()==0)
      {
        returnChange=false;
        stage=-1;
        offset.setOffsetChange(false);
        offset.setOffsetChangeBlock(false);
        robotMoving=false;
        stop=false;
        if(!offset.getOffsetSet())
        {
          digitalWrite(ENA_PIN,HIGH);
          digitalWrite(LED_PIN,HIGH);
        }
      }
    }

  // Steering movement
  if (digitalRead(ENA_PIN) == LOW && !stop)
  {

    // Offset change if controlled 
    offset.OffsetChange(steppers, joints);
    //r6a.moveOffset();
    // Offset move if controlled 
    RunningState state=offset.OffsetMove(steppers);

    if(state==Stop) robotMoving=false;
    else if(state==Running) robotMoving=true;


    // Moving stages example
    if(stage==0)
    {
      stepper1.moveTo(joint1.AngleToSteps(90));
      stepper2.moveTo(400);  
      stepper3.moveTo(1000); 
      stage++;
      robotMoving=true;
    }
    else if(stage==1)
    {
      stepper1.run();
      stepper2.run();
      stepper3.run();
      if(stepper1.distanceToGo()==0 && stepper2.distanceToGo()==0 && stepper3.distanceToGo()==0)
      {
        stepper1.moveTo(-1000);
        stepper3.moveTo(2600); 
        stepper4.moveTo(2000);
        stage++;
      }
    }
    else if(stage==2)
    {
      stepper1.run();
      stepper3.run();
      stepper4.run();
      if(stepper1.distanceToGo()==0 && stepper3.distanceToGo()==0 && stepper4.distanceToGo()==0)
      {
        stepper1.moveTo(1000);
        stepper3.moveTo(1400); 
        stepper4.moveTo(0);
        stage++;
      }
    } 
    else if(stage==3)
    {
      stepper1.run();
      stepper3.run();
      stepper4.run();
      if(stepper1.distanceToGo()==0 && stepper3.distanceToGo()==0 && stepper4.distanceToGo()==0)
      {
        stepper1.moveTo(0);
        stepper2.moveTo(0); 
        stepper3.moveTo(0); 
        stage++;
      }
    } 
    else if(stage==4)
    {
      stepper1.run();
      stepper2.run();
      stepper3.run();
      if(stepper1.distanceToGo()==0 && stepper2.distanceToGo()==0 && stepper3.distanceToGo()==0)
      {
        stage++;
      }
    }
    else if(stage==5)
    {
      robotMoving=false;
      stage=-1;
      delay(500);
    }
  }
}