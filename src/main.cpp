#include <Arduino.h>
#include <AccelStepper.h>
#include <vector>
#include "BluetoothSerial.h"
#include "Robot.h"
#include "GPIO.h"

#define default_bt_name "R6A Test Bluetooth Connection"

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

bool returnChange=false;
int option=0;

void setup() {

  pinMode(BTN_PIN, INPUT_PULLUP);
  pinMode(ENA_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);

  digitalWrite(ENA_PIN, HIGH);
  digitalWrite(LED_PIN, HIGH);

  SerialBT.begin(default_bt_name);

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
  if (message =="start" && r6a.getOffset().getOffsetSet() && !r6a.isRunning() && !r6a.isStop()){
    option=1;
  }
  else if (message =="offset" && !r6a.isRunning() && !r6a.isStop()){
    digitalWrite(ENA_PIN,LOW);
    digitalWrite(LED_PIN,LOW);
    r6a.getOffset().setOffsetChange(true);
  }
  else if (message =="stop" && r6a.isRunning() && !r6a.isStop()){
    option=0;
    r6a.getEmergencyStop().setStopChange(true);
    SerialBT.println("EMERGENCY STOP");
  }
  else if (message =="return" && r6a.isStop()){
    r6a.getEmergencyStop().setReturnChange(true);
  }
  
  if (digitalRead(ENA_PIN) == LOW) // Steering movement
  {
    r6a.emergencyStop();  // Stop change if controlled
    
    r6a.emergencyRelease(); // Return if stopped and controlled
   
    if(!r6a.isStop()){ // Moves if not in stop mode

      r6a.moveOffset(); // Offset change if controlled 
      if(option==1 && !r6a.getOffset().getOffsetChange())
      {
        option=r6a.moveTestStages(); // Moving stages example
      }
    }
  }
}