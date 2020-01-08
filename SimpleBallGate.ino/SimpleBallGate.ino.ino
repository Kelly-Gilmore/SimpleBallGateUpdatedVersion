#pragma ONCE


#include <Arduino.h>
#include "SpeedyStepper.h"
#include "State.h"

#define DEBOUNCE_TIME 300

//Button definition
#define BG1_BUTTON 54
#define BG1_BUTTON_RED 55
#define BG1_BUTTON_GREEN 56
#define BG1_BUTTON_BLUE 57

//Stepper Motor definition
#define BG1_Stepper_Port 1

//Home Sensor definition
#define BG1_CamRotSensor 23
#define BG1_BallInSensor 24


//Potentiometer definition
#define BG1_POT A7

#define INTERMITTENT_MIN 4000
#define INTERMITTENT_MAX 8000
#define POT_CHECK_TIME 1500
#define POT_ACCEPTED_DELTA 20

SpeedyStepper gateStepper;
// byte stepperPort;
States state;  // lastState;
bool buttonEvent = false;
bool moveFinished = true;
bool currentlyRunning = false;
byte ballInSensor = BG1_BallInSensor;
byte buttonPin = BG1_BUTTON;
byte redPin = BG1_BUTTON_RED;
byte greenPin = BG1_BUTTON_GREEN;
byte bluePin = BG1_BUTTON_BLUE;
byte potPin = BG1_POT;
unsigned long potStartTime;
unsigned long buttonStartTime = millis();
long potLastRead;
float readVal;
float rateVal;
long potVal, potHome;
long potMinVal = 0;
long potMaxVal = 0;

bool checkButton();


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(potPin, INPUT);
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(21), buttonPressedEvent, FALLING);
  gateStepper.connectToPort(1);
  gateStepper.setStepsPerRevolution(1036 * 8);
  gateStepper.setSpeedInRevolutionsPerSecond(.2);
  gateStepper.setAccelerationInRevolutionsPerSecondPerSecond(.5);
  gateStepper.enableStepper();
  gateStepper.moveToHomeInRevolutions(-1, .125, 1, 23); //rateVal
  gateStepper.disableStepper();
}

void loop() {


  updateGate();
}
 
void buttonPressedEvent() {
  Serial.println("Entered buttonEvent");
  static unsigned long last_interrupt_time = 0;
  unsigned long interrupt_time = millis();
  if (interrupt_time - last_interrupt_time > DEBOUNCE_TIME) {
    buttonEvent = !buttonEvent;
    last_interrupt_time = millis();
  }
  Serial.print("buttonEventVal: ");
  Serial.println(buttonEvent);
}

bool checkButton() { //this is going to be called in another function
  if (!digitalRead(21)) { //If the button has been pressed
    buttonEvent = true;
  } else {
    buttonEvent = false;
  }
  return buttonEvent;
}

void setState() {
  if (buttonEvent) {
    setStatus();
    if (!currentlyRunning) {
      state = ROTATING;
    }
    if (currentlyRunning) {
      state = RESTING;
    }
  }
}

States getState() {
  Serial.println  (state);
  return state;
}

void setStatus() {
  if (buttonEvent) {
    currentlyRunning = !currentlyRunning;
    buttonEvent = false;
    return;
  }
}


void updateGate() {
  switch (getState()) {

    case (ROTATING) :
      greenLight();
      if (!buttonEvent && checkOpticalSensor()) {   // && checkOpticalSensor() for flashing
        rotateGate();
      }
      else
        flashGreen();
      setState();
      break;

    case (RESTING) :
      redLight();
      delay(500);
      setState();
      break;
  }

}




bool moveToHomeInRevNoBlock(long directionTowardHome, float speedInRevolutionsPerSecond,
                            long maxDistanceToMoveInRevolutions, int homeLimitSwitchPin)
{
  bool limitSwitchFlag;


  pinMode(homeLimitSwitchPin, INPUT_PULLUP);


  //
  // if the home switch is not already set, move toward it
  //
  if (digitalRead(homeLimitSwitchPin) == HIGH)
  {
    //
    // move toward the home switch
    //
    gateStepper.setSpeedInRevolutionsPerSecond(speedInRevolutionsPerSecond);
    gateStepper.setupRelativeMoveInRevolutions(maxDistanceToMoveInRevolutions * directionTowardHome);
    limitSwitchFlag = false;
    while (!gateStepper.processMovement())
    {

      if (digitalRead(homeLimitSwitchPin) == LOW)
      {
        delay(1);
        if (digitalRead(homeLimitSwitchPin) == LOW)
        {
          delay(80);                // allow time for the switch to debounce
          limitSwitchFlag = true;
          break;

        }
      }
    }

    //
    // check if switch never detected
    //
    if (limitSwitchFlag == false)
      return (false);
  }


  //
  // the switch has been detected, now move away from the switch
  //
  gateStepper.setupRelativeMoveInRevolutions(maxDistanceToMoveInRevolutions * directionTowardHome * -1);
  limitSwitchFlag = false;
  while (!gateStepper.processMovement())
  {

    if (digitalRead(homeLimitSwitchPin) == HIGH)
    {
      delay(1);
      if (digitalRead(homeLimitSwitchPin) == HIGH)
      {
        delay(80);                // allow time for the switch to debounce
        limitSwitchFlag = true;
        break;

      }
    }
  }

  //
  // check if switch never detected
  //
  if (limitSwitchFlag == false)
    return (false);


  //
  // have now moved off the switch, move toward it again but slower
  //
  gateStepper.setSpeedInRevolutionsPerSecond(speedInRevolutionsPerSecond / 8);
  gateStepper.setupRelativeMoveInRevolutions(maxDistanceToMoveInRevolutions * directionTowardHome);
  limitSwitchFlag = false;
  while (!gateStepper.processMovement())
  {

    if (digitalRead(homeLimitSwitchPin) == LOW)
    {
      delay(1);
      if (digitalRead(homeLimitSwitchPin) == LOW)
      {
        delay(80);                // allow time for the switch to debounce
        limitSwitchFlag = true;
        break;

      }
    }
  }

  //
  // check if switch never detected
  //
  if (limitSwitchFlag == false)
    return (false);


  //
  // successfully homed, set the current position to 0
  //
  gateStepper.setCurrentPositionInRevolutions(0L);



}



void homeGate() {
  calculateRate();
  gateStepper.enableStepper();  
  moveToHomeInRevNoBlock(-1, rateVal, 1, 23); //rateVal
  gateStepper.disableStepper();
}

void rotateGate() {
  gateStepper.enableStepper();
  gateStepper.setSpeedInRevolutionsPerSecond(5);
  gateStepper.setAccelerationInRevolutionsPerSecondPerSecond(7);
  gateStepper.moveRelativeInRevolutions(-.024); //moves the homing screw off the homing sensor
  homeGate();
  gateStepper.disableStepper();
}


//Potentionmeter Knob Functions Below

void calculateRate() {
  potMinVal = 50;
  potMaxVal = 400;
  if (readPot() < potMinVal) {
    minimizePot();
  }
  if (readPot() > potMaxVal) {
    maximizePot();
  }
  pleaseWork();
}


void minimizePot() { //used to be maximizePot
  readVal = 50; //50 is the minimum value that the gate can rotate at
  return;
}

void maximizePot() {
  readVal = 400; //400 is the maximum value that the gate can rotate at
  return;
}



long readPot() {
  return analogRead(potPin);
}

void pleaseWork() { //convert readPot value into a decimal so that MoveToHomeInReovlutions can actually use the value
  readVal = readPot();
  rateVal = readVal / 1000;
  Serial.print("rateVal: ");
  Serial.println(rateVal);
} 

byte readColorButton() { //Read the button to get its current press state, LOW = pressed
  return digitalRead(buttonPin);
}

void redLight() {
  analogWrite(bluePin, 255);
  analogWrite(greenPin, 255);
  analogWrite(redPin, 0);
}

void greenLight() {
  analogWrite(bluePin, 255);
  analogWrite(greenPin, 0);
  analogWrite(redPin, 255);
}

void blueLight() {
  analogWrite(bluePin, 0);
  analogWrite(greenPin, 255);
  analogWrite(redPin, 255);
}

void turnOff() {
  analogWrite(bluePin, 255);
  analogWrite(greenPin, 255);
  analogWrite(redPin, 255);
}

bool checkOpticalSensor() {
  return digitalRead(24) == HIGH ? true : false;
}

void flashGreen() {
  greenLight();
  delay(500);
  turnOff();
  delay(500);
}
