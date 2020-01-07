#pragma ONCE 
#include <Arduino.h>
bool buttonEvent = false;
bool checkButton() { //this is going to be called in another function
  if (!digitalRead(54)) { //If the button has been pressed
    buttonEvent = true;
  } else {
    buttonEvent = false;
  }
  //  buttonStartTime = millis();
  return buttonEvent;
}
