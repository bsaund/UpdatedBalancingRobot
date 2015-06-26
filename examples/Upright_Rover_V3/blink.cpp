// 
// 
// 

#include "blink.h"

void BlinkClass::init(int pin)
{
  pinMode(pin, OUTPUT);
  PrevTime = millis();
  NumBlinks = 0;
  isOn = false;
  PIN = pin;
  DelayCycles = 0;
}


void BlinkClass::blinkFor(int timeInMillis, int numTimes, int delayCycles){

  if (millis() - PrevTime > timeInMillis){
    PrevTime = millis();

    if (DelayCycles > 0){
      DelayCycles--;
      return;
    }

    if (NumBlinks >= numTimes){
      NumBlinks = 0;
      DelayCycles = delayCycles;
      isOn = false;
      digitalWrite(PIN, false);
      return;
    }
    isOn = !isOn;
    digitalWrite(PIN, isOn);
    
    NumBlinks += !isOn;
  }
}


BlinkClass Blink;

