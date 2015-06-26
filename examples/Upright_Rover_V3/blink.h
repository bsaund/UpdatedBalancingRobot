// blink.h

#ifndef _BLINK_h
#define _BLINK_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

class BlinkClass
{
 protected:
   unsigned long PrevTime;
   int NumBlinks;
   boolean isOn;
   int PIN;
   int DelayCycles;

 public:
	void init(int pin);
  void blinkFor(int timeInMillis, int numTimes, int delayCycles);
};

extern BlinkClass Blink;

#endif

