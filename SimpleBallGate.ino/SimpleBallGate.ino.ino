

//I don't know that the pot knob works to adjust rate so I need to figure out a way to check that; but for now at least it will upload and function sooo 

#include <Arduino.h>
#include "SpeedyStepper.h"
#include "State.h"

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
#define BG1_POT 7

#define INTERMITTENT_MIN 4000
#define INTERMITTENT_MAX 8000
#define POT_CHECK_TIME 1500
#define POT_ACCEPTED_DELTA 20

SpeedyStepper gateStepper;
// byte stepperPort; 
States state;  // lastState;
bool buttonEvent;
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

// potStartTime = millis();

// potMinVal = 0;
// potMaxVal = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  // pinMode(55, OUTPUT);
  // pinMode(56, OUTPUT);
  pinMode(potPin, INPUT);
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
  gateStepper.connectToPort(1);
  gateStepper.setStepsPerRevolution(1036 * 8);
  gateStepper.setSpeedInRevolutionsPerSecond(.2);
  gateStepper.setAccelerationInRevolutionsPerSecondPerSecond(.5);
//  initializePot();
  gateStepper.enableStepper();
  gateStepper.moveToHomeInRevolutions(-1, .125, 1, 23); //rateVal
  gateStepper.disableStepper();
  // Serial.println("Initial Homing Complete");
}

void loop() {
//  checkButton();
//  if(buttonEvent && !stillRunning) { // if there has been a button event and the gate is not rotating then turn the light green and rotate the gate
//    greenLight();
//    rotateGate();
//    checkButton();        
//   }
//  if (buttonEvent && stillRunning) { // If there has been a button event and the gate is running then turn the light red
//    redLight(); 
//  }
//  }


updateGate();

  
/*  checkButton();
  setStatus();
  if(buttonEvent && !stillRunning) {
    greenLight();
    rotateGate();
  }
  if(buttonEvent && stillRunning) {
    redLight();
  }
  
 */
}

void setState() {
  Serial.println("begin setState");
  checkButton();
  if (buttonEvent) {
    setStatus();
    if (!currentlyRunning) {
      state = ROTATING; 
    }
    if (currentlyRunning) {
      state = RESTING;
    }
  }
    Serial.println("end setState");
}


/* if (buttonEvent && !currentlyRunning) {  This is what I initially had for setState();
    state = ROTATING;
//    lastState = ROTATING;
  }
  else if (buttonEvent && currentlyRunning) {
    state = RESTING;
//    lastState = RESTING;
*/

States getState() {
  Serial.println  (state);
  return state;
}

void setStatus() {
  if(buttonEvent){
    currentlyRunning = !currentlyRunning;
    Serial.println(currentlyRunning);
    return;
  }
}


void updateGate() {     
  switch (getState()) {

    case (ROTATING) :
      Serial.println("ROTATING");
      greenLight();
      rotateGate();
      Serial.println("done");
      setState();
      break;

    case (RESTING) : 
      redLight();
      delay(500);
      setState();
      break;
  }
//  checkButton();
//    setState();

  }

void moveToHomeInRevNoBlock(long directionTowardHome, float speedInRevolutionsPerSecond, long maxDistanceToMoveInRevolutions, int homeLimitSwitchPin) { //(-1, rateVal, 1, 23)
  float originalDesiredSpeed_InStepsPerSecond; 
  bool limitSwitchFlag;

// setup the home switch input pin

  pinMode(homeLimitSwitchPin, INPUT_PULLUP);

// setting speed for desired speed in steps per second

//  int desiredSpeed_InStepsPerSecond = 200.0;  ---> don't think I need this, using revolutions not steps

// remember the current speed setting

//  originalDesiredSpeed_InStepsPerSecond = desiredSpeed_InStepsPerSecond;  ---> don't think I need this, using revolutions not steps

// if the home switch is not already set, move toward it

  if(digitalRead(homeLimitSwitchPin) == HIGH) {

// move toward the home switch
    
    gateStepper.setSpeedInRevolutionsPerSecond(speedInRevolutionsPerSecond);  //should be using the value of rateVal
    gateStepper.setupRelativeMoveInRevolutions(maxDistanceToMoveInRevolutions * directionTowardHome);  //should be 1 * -1 = -1
    limitSwitchFlag = false;

    if(digitalRead(homeLimitSwitchPin) == LOW) {
      gateStepper.processMovement();
      delay(1);
      if(digitalRead(homeLimitSwitchPin) == LOW) {
        gateStepper.processMovement();
        delay(80);                                           // allow time for the switch to debounce
        limitSwitchFlag = true;
        return;
      }
    }

// check if switch never detected
  
  if(limitSwitchFlag == false)
    return(false);
 }

// the switch has been detected, now move away from the switch

 gateStepper.setupRelativeMoveInRevolutions(maxDistanceToMoveInRevolutions * directionTowardHome * -1);
 limitSwitchFlag = false;

 if(digitalRead(homeLimitSwitchPin) == HIGH) {
  gateStepper.processMovement();
  delay(1);
  if(digitalRead(homeLimitSwitchPin) == HIGH) {
    gateStepper.processMovement();
    delay(80);                                              // allow time for the switch to debounce
    limitSwitchFlag = true;
    return;
  }
 }

 // check if switch never detected
 
 if(limitSwitchFlag == false)
  return(false);

// have now moved off the switch, move toward it again but slower

 gateStepper.setSpeedInRevolutionsPerSecond(speedInRevolutionsPerSecond/8); 
 gateStepper.setupRelativeMoveInRevolutions(maxDistanceToMoveInRevolutions * directionTowardHome);
 limitSwitchFlag = false;

 if(digitalRead(homeLimitSwitchPin) == LOW) {
  gateStepper.processMovement();
  delay(1);
  if(digitalRead(homeLimitSwitchPin) == LOW) {
    gateStepper.processMovement();
    delay(80);                                              // allow time for the switch to debounce
    limitSwitchFlag = true;
    return;
  }
 }

// check if switch never detected

 if(limitSwitchFlag == false)
  return(false);

// successfully homed, set the current position to 0  

 gateStepper.setCurrentPositionInRevolutions(0L);

// restore original velocity

 gateStepper.setSpeedInRevolutionsPerSecond(originalDesiredSpeed_InStepsPerSecond); 
 return(true);
   
}



/* void setupRelativeMoveInRevolutions(float distanceToMoveInRevolutions) {
  setupRelativeMoveInSteps((long) round(distanceToMoveInRevolutions * stepsPerRevolution));
}

 void setSpeedInRevolutionsPerSecond(float speedInRevolutionsPerSecond) {
  desiredSpeed_InStepsPerSecond = speedInRevolutionsPerSecond * stepsPerRevolution;
}
*/ 

void homeGate() {
  // digitalWrite(55, LOW);
  calculateRate();
  gateStepper.enableStepper();
  moveToHomeInRevNoBlock(-1, rateVal, 1, 23); //rateVal 
  gateStepper.disableStepper();
  // digitalWrite(55, HIGH);
}

void rotateGate() {
  gateStepper.enableStepper();
  gateStepper.moveRelativeInRevolutions(-.04); //moves the homing screw off the homing sensor 
  homeGate();
  gateStepper.disableStepper();
}
 

//Potentionmeter Knob Functions Below

void calculateRate() {
  potMinVal = 50;
  potMaxVal = 400;
  if(readPot() < potMinVal) {
    minimizePot();
  }
  if(readPot() > potMaxVal) {
    maximizePot();
  }
  pleaseWork();
}


void minimizePot() { //used to be maximizePot
/*potHome = readPot();
  potMinVal = potHome;
  potMaxVal = potMinVal + 320;
*/
  readVal = 50; //50 is the minimum value that the gate can rotate at
  return;
}

void maximizePot() {
  readVal = 400; //400 is the maximum value that the gate can rotate at
  return;
}

  


/*void setPotMinVal(byte minVal) {
  potMinVal = minVal;
}

void setPotMaxVal(byte maxVal){
  potMaxVal = maxVal;
}
*/

long readPot() {
  return analogRead(potPin);
}

void pleaseWork() { //convert readPot value into a decimal so that MoveToHomeInReovlutions can actually use the value
  readVal = readPot();
//  Serial.println("readVal: ");
//  Serial.println(readVal);
  rateVal = readVal/1000;
//  Serial.println("rateVal: ");
//  Serial.println(rateVal);
}

//Button Functions Below


void checkButton() { //this is going to be called in another function
//  if (((millis() - buttonStartTime) > 10) && moveFinished) { //If the (curent time) - (last button pressed start time) is greater than 10ms & the ball gate is not in rotation
    if (!readColorButton()) { //If the button has been pressed
      buttonEvent = true; 
  } else {
      buttonEvent = false; 
  }
//  buttonStartTime = millis();
    return;
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
