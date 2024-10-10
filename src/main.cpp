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
Joint joint2(20, 100, 45, -45, 90);

AccelStepper stepper3(AccelStepper::DRIVER, STEP_PIN_3, DIR_PIN_3);
Joint joint3(20, 80, 65, -55, 195);

AccelStepper stepper4(AccelStepper::DRIVER, STEP_PIN_4, DIR_PIN_4);
Joint joint4(16, 60, -180, 180);

AccelStepper stepper5(AccelStepper::DRIVER, STEP_PIN_5, DIR_PIN_5);
Joint joint5(12, 40, -20, -105, 105);

AccelStepper stepper6(AccelStepper::DRIVER, STEP_PIN_6, DIR_PIN_6);
Joint joint6(0, -180, 180);

std::vector<AccelStepper *> steppers = {&stepper1, &stepper2, &stepper3, &stepper4, &stepper5, &stepper6};
std::vector<Joint *> joints = {&joint1, &joint2, &joint3, &joint4, &joint5, &joint6};

Offset offset;

Kinematics kinematics(30.753, 160, 150.94, 9.486); // kinematics joints lenghts

Robot r6a(steppers, joints, kinematics);

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

SteeringMove SteeringMoveOption = SteeringMove();

BluetoothSerial SerialBT;
SemaphoreHandle_t mutex;
String message = "";
bool messageEnd = false;
char incomingChar;

vector<float> movePosition, homePosition={179.63, 0, 184.88, 0, 0, 0}, homePosition2={179.63+20, 0, 184.88, 0, 0, 0} , homePosition3={179.63+100, 0, 184.88, 0, 0, 0};

bool returnChange = false;
int option = standbyMode;
unsigned long ledStopTimer = 0;
int ledStopInterval = 300;

void setup(){

  Serial.begin(115200);

  pinMode(BTN_PIN, INPUT_PULLUP);
  pinMode(ENA_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);

  digitalWrite(ENA_PIN, HIGH);
  digitalWrite(LED_PIN, HIGH);

  SerialBT.begin(default_bt_name);
  mutex = xSemaphoreCreateMutex();
  Serial.println("Serial begin");
  Serial.println("Robot started");
  // Calculations test forward/inverse kinematics
  Serial.println("HOME FORWARD/INVERSE-----");
  Serial.println(r6a.getKinematics().printInverseK(homePosition));
  Serial.println(r6a.getKinematics().printForwardK(r6a.getKinematics().inverseKinematics(homePosition)));
  Serial.println("HOME+(X20) FORWARD/INVERSE-----");
  Serial.println(r6a.getKinematics().printInverseK(homePosition2));
  Serial.println(r6a.getKinematics().printForwardK(r6a.getKinematics().inverseKinematics(homePosition2)));
  Serial.println("HOME+(X100) FORWARD/INVERSE-----");
  Serial.println(r6a.getKinematics().printInverseK(homePosition3));
  Serial.println(r6a.getKinematics().printForwardK(r6a.getKinematics().inverseKinematics(homePosition3)));
  //Serial.println("FORWARD------------------");
  //Serial.println(r6a.getKinematics().printForwardK({0,30,30,0,0,0}));
  //Serial.println(r6a.getKinematics().printForwardK({10,30,30,0,0,0}));
  //Serial.println("INVERSE------------------");
  //Serial.println(r6a.getKinematics().printInverseK({259.63,0.,163.44,0.,0.,0.}));
  //Serial.println(r6a.getKinematics().printInverseK({255.68,45.08,163.44,10.,0.,0.}));

  r6a.memoryInit();
}

void loop(){

  // Bluetooth input check
  while (SerialBT.available()){
    xSemaphoreTake(mutex, portMAX_DELAY);
    char incomingChar = SerialBT.read();
    if (!messageEnd){
      if (incomingChar != '\n'){
        message += String(incomingChar);
      }
      else{
        messageEnd = true;
      }
    }
    xSemaphoreGive(mutex);
  }

  // Serial input check
  while (Serial.available()){
    xSemaphoreTake(mutex, portMAX_DELAY);
    char incomingChar = Serial.read();
    if (!messageEnd){
      if (incomingChar != '\n' && incomingChar != '\r'){
        message += String(incomingChar);
      }
      else{
        messageEnd = true;
      }
    }
    xSemaphoreGive(mutex);
  }

  // Reseting by button
  if (digitalRead(BTN_PIN) == LOW){
    digitalWrite(ENA_PIN, HIGH);
    digitalWrite(LED_PIN, HIGH);
    r6a.getOffset().offsetReset();
    r6a.getEmergencyStop().stopReset();
    r6a.setHome();
  }

  // Bluetooth control
  if (messageEnd){
    xSemaphoreTake(mutex, portMAX_DELAY);   // Conditions: {isStop, isRun, isOffset, isHome}
    SteeringMoveOption = r6a.getSteering().handle(message, {r6a.isStop(), r6a.isRunning(), r6a.getOffset().getOffsetSet(), r6a.isHome()});
    xSemaphoreGive(mutex);
  }

  switch (SteeringMoveOption.getSteeringOption()){
  case start:
    option = stageMode;
    break;

  case offsetpos:
    digitalWrite(ENA_PIN, LOW);
    digitalWrite(LED_PIN, LOW);
    r6a.getOffset().setOffsetChange(true);
    break;

  case estop:
    r6a.getEmergencyStop().setStopChange(true);
    option = standbyMode;
    SerialBT.println("EMERGENCY STOP");
    break;

  case returnstop:
    digitalWrite(LED_PIN, LOW);
    r6a.getEmergencyStop().setReturnChange(true);
    break;

  case home:
    r6a.goHome();
    option = homeMode;
    break;

  case disable:
    break;

  case test:
    option = testMode;
    SerialBT.println("TEST");
    break;

  case control:
    switch (SteeringMoveOption.getSteeringSubOption()){
    case axisAbsolute:
      r6a.getMove().setAngles(r6a.setAngles(SteeringMoveOption.getValues()));
      break;

    case axisIncremental:
      r6a.getMove().setAngles(r6a.addAngles(SteeringMoveOption.getValues()));
      break;

    case positionAbsolute:
      r6a.getMove().setAngles(r6a.getKinematics().inverseKinematics(r6a.setPositions(SteeringMoveOption.getValues())));
      break;

    case positionIncremental:
      r6a.getMove().setAngles(r6a.getKinematics().inverseKinematics(r6a.addPositions(SteeringMoveOption.getValues())));
      break;

    default:
      break;
    }
    if(!r6a.getMove().isAnglesNan() && r6a.isAngleLimitOk(r6a.getMove().getAngles())){
      r6a.getMove().setSpeedScaling(SteeringMoveOption.getSpeed());
      r6a.getMove().setChange(true);
    }
    else{
      SerialBT.println("Position not available");
    }
    break;
  
  case readpos:
  switch (SteeringMoveOption.getSteeringSubOption()){
    case position:
      SerialBT.println(r6a.getStringCurrentPosition());
      break;

    case axis:
      SerialBT.println(r6a.getStringCurrentAngles());
      break;

    default:
      break;
    } 
    break;

  case memread:
    SerialBT.println(r6a.getStringVector(r6a.getMemory().readVector(FILE_NAME, NUM_OF_AXIS ,SteeringMoveOption.getSteeringSubOption())));
    break;

  case memsave:
    r6a.getMemory().writeVector(FILE_NAME, NUM_OF_AXIS ,SteeringMoveOption.getSteeringSubOption(),r6a.currentAngles());
    break;

  case memsetgo:
    r6a.getMove().setAngles(r6a.getMemory().readVector(FILE_NAME, NUM_OF_AXIS ,SteeringMoveOption.getSteeringSubOption()));
    r6a.getMove().setChange(true);
    break;

  case memstageinit:
    SerialBT.println(r6a.getStringVector(r6a.getAngles(SteeringMoveOption.getValues())));
    break;

  case memstageread:
    SerialBT.println(r6a.getStringVector(r6a.getMemory().readVector(FILE_NAME_2, STAGES_SIZE ,SteeringMoveOption.getSteeringSubOption())));
    break;

  case memstageload:
    // TODO loading stages to robot.StageMovement
    break;

  case memstagereadloaded:
    // TODO reading memory number from robot.StageMovement
    break;

  case memstageloop:
    // TODO setting loop stages in robot.StageMovement
    break;

  default:
    break;
  }
  SteeringMoveOption.resetMove();
  message = "";
  messageEnd = false;

  // LED emergency blinking
  if (r6a.isStop()){
    if (millis() - ledStopTimer >= ledStopInterval){
      bool state = digitalRead(LED_PIN);
      if (state){
        digitalWrite(LED_PIN, LOW);
        if (r6a.getEmergencyStop().getReturnChange())
          ledStopInterval = 500;
        else
          ledStopInterval = 300;
      }
      else{
        digitalWrite(LED_PIN, HIGH);
        if (r6a.getEmergencyStop().getReturnChange())
          ledStopInterval = 50;
        else
          ledStopInterval = 300;
      }
      ledStopTimer = millis();
    }
  }

  // Motor running loop
  if (digitalRead(ENA_PIN) == LOW) // Steering movement if motors control enabled
  {
    r6a.emergencyStop();    // Stop change if controlled
    r6a.emergencyRelease(); // Return if stopped and controlled

    if(!r6a.isStop()){
      r6a.moveSteering();     // Serial steering
      r6a.moveOffset();       // Offset change if controlled

      if (option == stageMode){
        option = standbyMode; // TODO stagemode 
      }
      if (option == testMode){
        option = r6a.moveTestStages(); // Moving stages example
      }
      else if(option==homeMode){
        if(!r6a.runManual()){
          option=standbyMode;
        }
      }
    }
  }
}