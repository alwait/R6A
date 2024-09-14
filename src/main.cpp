#include <Arduino.h>
#include <AccelStepper.h>
#include <vector>
#include "BluetoothSerial.h"
#include "Robot.h"
#include "GPIO.h"

#define default_bt_name "0R6A Test Bluetooth Connection"

AccelStepper stepper1(AccelStepper::DRIVER, STEP_PIN_1, DIR_PIN_1);
Joint joint1(20, 120, -180, 180);

AccelStepper stepper2(AccelStepper::DRIVER, STEP_PIN_2, DIR_PIN_2);
Joint joint2(12, 100, 45, -45, 90);

AccelStepper stepper3(AccelStepper::DRIVER, STEP_PIN_3, DIR_PIN_3);
Joint joint3(20, 80, 65, -65, 180);

AccelStepper stepper4(AccelStepper::DRIVER, STEP_PIN_4, DIR_PIN_4);
Joint joint4(16, 60, -180, 180);

AccelStepper stepper5(AccelStepper::DRIVER, STEP_PIN_5, DIR_PIN_5);
Joint joint5(16, 40, -20, -90, 90);

AccelStepper stepper6(AccelStepper::DRIVER, STEP_PIN_6, DIR_PIN_6);
Joint joint6(0, -180, 180);

std::vector<AccelStepper*> steppers = {&stepper1, &stepper2, &stepper3, &stepper4, &stepper5, &stepper6};
std::vector<Joint*> joints = {&joint1, &joint2, &joint3, &joint4, &joint5, &joint6};

Offset offset;

Kinematics kinematics(30.753,160,150.94,9.486);

Robot r6a(steppers, joints, kinematics);

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

enum Option{standbyMode, testMode, manualMode, homeMode};

SteeringMove SteeringMoveOption = SteeringMove();

BluetoothSerial SerialBT;
SemaphoreHandle_t mutex;
String message = "";
char incomingChar;

bool returnChange=false;
int option=standbyMode;
unsigned long ledStopTimer=0;
int ledStopInterval=300;

void setup() {

  Serial.begin(115200);

  pinMode(BTN_PIN, INPUT_PULLUP);
  pinMode(ENA_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);

  digitalWrite(ENA_PIN, HIGH);
  digitalWrite(LED_PIN, HIGH);

  SerialBT.begin(default_bt_name);
  mutex = xSemaphoreCreateMutex();

  Serial.println("Robot started");
  //Serial.println(r6a.getKinematics().printInverseK({179.63-20,80,184.88-40,0,0,0}));
  Serial.println(r6a.getKinematics().printInverseK({179.63,0,184.88,0,0,0}));
  Serial.println(r6a.getKinematics().printInverseK({179.63,0,184.88,0,0,90}));
  //Serial.println(r6a.getKinematics().printInverseK({179.63,0,184.88,0,0,0}));
}

void loop()
{

  // Bluetooth input check
  if (SerialBT.available()){
    xSemaphoreTake(mutex, portMAX_DELAY);
    char incomingChar = SerialBT.read();
    if (incomingChar != '\n'){
      message += String(incomingChar);
    }
    else{
      message = "";
    }
     xSemaphoreGive(mutex);
  }

  // Reseting by button
  if(digitalRead(BTN_PIN)==LOW){
    digitalWrite(ENA_PIN,HIGH);
    digitalWrite(LED_PIN,HIGH);
    r6a.getOffset().offsetReset();
    r6a.getEmergencyStop().stopReset();
    r6a.setHome();
  }
  
  // Bluetooth control
  if(message!=""){
    xSemaphoreTake(mutex, portMAX_DELAY);
    // Conditions: {isStop, isRun, isOffset, isHome}
    SteeringMoveOption=r6a.getSteering().handle(message,{r6a.isStop(), r6a.isRunning(), r6a.getOffset().getOffsetSet(), r6a.isHome()});
    if(!SteeringMoveOption.getType().isTypeNone()){
      //message="";
      Serial.print("Option: ");
      Serial.println(SteeringMoveOption.getSteeringOption());
      if(SteeringMoveOption.getAngles()!=nullptr){
        Serial.print("Axis: {");
        for(size_t i=0; i<SteeringMoveOption.getAngles()->size(); i++){
          Serial.print(SteeringMoveOption.getAngles()->at(i));
          Serial.print(",");
        }
        Serial.println("}");
      }
    }
    if (message =="start" && r6a.getOffset().getOffsetSet() && !r6a.isRunning() && !r6a.isStop()){
      option=testMode;
      message = "";
    }
    else if (message =="offset" && !r6a.isRunning() && !r6a.isStop() && r6a.isHome()){
      digitalWrite(ENA_PIN,LOW);
      digitalWrite(LED_PIN,LOW);
      r6a.getOffset().setOffsetChange(true);
      message = "";
    }
    else if (message =="stop" && r6a.isRunning()){
      r6a.getEmergencyStop().setStopChange(true);
      option=standbyMode;
      SerialBT.println("EMERGENCY STOP");
      message = "";
    }
    else if (message =="return" && r6a.isStop()){
      digitalWrite(LED_PIN, LOW);
      r6a.getEmergencyStop().setReturnChange(true);
      message = "";
    }
    else if (message =="home" && !r6a.isStop() && !r6a.isRunning()){
      r6a.goHome();
      option=homeMode;
      message = "";
    }
    else if (message =="disable" && r6a.isStop() && !r6a.isRunning()){
      message = "";
    }
    else if (message =="test" && !r6a.isRunning() && !r6a.isStop()){
      r6a.getMove().setAngles({15,40,30,90,20,45});
      r6a.getMove().setChange(true);
      SerialBT.println("TEST");
      message = "";
    }
    else if (message.substring(0, 6)=="manual" && message.endsWith(";") && !r6a.isRunning() && !r6a.isStop()){
      if(r6a.runPositionInput(message)==2){
        r6a.getMove().setChange(true);
      }
      //option=r6a.runPositionInput(message);
      message = "";
    }
    xSemaphoreGive(mutex);
  }

  // LED emergency blinking
  if(r6a.isStop()){
    if(millis()-ledStopTimer>=ledStopInterval){
      bool state=digitalRead(LED_PIN);
      if(state){
        digitalWrite(LED_PIN, LOW);
        if(r6a.getEmergencyStop().getReturnChange()) ledStopInterval=500; 
        else ledStopInterval=300;
      }
      else{
        digitalWrite(LED_PIN, HIGH);
        if(r6a.getEmergencyStop().getReturnChange()) ledStopInterval=50; 
        else ledStopInterval=300;
      }
      ledStopTimer=millis();
    }
  }

  // Motor running loop
  if (digitalRead(ENA_PIN) == LOW) // Steering movement
  {

    // Safety features
    r6a.emergencyStop();  // Stop change if controlled
    r6a.emergencyRelease(); // Return if stopped and controlled
    r6a.moveSteering();
   
    // Moving options
    if(!r6a.isStop()){ // Moves if not in stop mode

      r6a.moveOffset(); // Offset change if controlled 

      // Moving on fully enabled
      if(!r6a.getOffset().getOffsetChange()){
        if(option==testMode)
        {
          option=r6a.moveTestStages(); // Moving stages example
        }
        else if(option==manualMode){
          if(!r6a.runManual()){
            option=standbyMode;
          }
        }
        else if(option==homeMode){
          if(!r6a.runManual()){
            option=standbyMode;
          }
        }
      }

    }

  }
}