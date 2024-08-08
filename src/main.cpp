#include <Arduino.h>
#include <AccelStepper.h>
#include <vector>
#include "BluetoothSerial.h"
#include "Robot.h"
#include "GPIO.h"

#define default_bt_name "R6A Test Bluetooth Connection"

AccelStepper stepper1(AccelStepper::DRIVER, STEP_PIN_1, DIR_PIN_1);
Joint joint1(20, 120, -180, 180);

AccelStepper stepper2(AccelStepper::DRIVER, STEP_PIN_2, DIR_PIN_2);
Joint joint2(20, 100, 35, -35, 90);

AccelStepper stepper3(AccelStepper::DRIVER, STEP_PIN_3, DIR_PIN_3);
Joint joint3(20, 80, 65, -65, 180);

AccelStepper stepper4(AccelStepper::DRIVER, STEP_PIN_4, DIR_PIN_4);
Joint joint4(20, 60, -180, 180);

AccelStepper stepper5(AccelStepper::DRIVER, STEP_PIN_5, DIR_PIN_5);
Joint joint5(16, 40, -30, -90, 90);

AccelStepper stepper6(AccelStepper::DRIVER, STEP_PIN_6, DIR_PIN_6);
Joint joint6(90, -180, 180);

std::vector<AccelStepper*> steppers = {&stepper1, &stepper2, &stepper3, &stepper4, &stepper5, &stepper6};
std::vector<Joint*> joints = {&joint1, &joint2, &joint3, &joint4, &joint5, &joint6};

Offset offset;

Robot r6a(steppers, joints);

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

enum Option{standbyMode, testMode, manualMode};

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
    if (message =="start" && r6a.getOffset().getOffsetSet() && !r6a.isRunning() && !r6a.isStop()){
      option=testMode;
      message = "";
    }
    else if (message =="offset" && !r6a.isRunning() && !r6a.isStop()){
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
    else if (message =="disable" && r6a.isStop()){
      message = "";
    }
    else if (message.substring(0, 6)=="manual" && message.endsWith(";")){
      option=r6a.runPositionInput(message);
      message = "";
    }
    xSemaphoreGive(mutex);
  }

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
      }

    }

  }
}