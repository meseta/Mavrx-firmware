#include "thal.h"


#define ZERO	__COUNTER__
#define ONE		__COUNTER__
#define TWO		__COUNTER__

int main(void) {
	LEDInit(PLED);
	
	if(ONE == 0) {
		LEDOn(PLED);
		Delay(200);
		LEDOff(PLED);
		Delay(200);
	}
	
	if(ZERO == 1) {
		LEDOn(PLED);
		Delay(200);
		LEDOff(PLED);
		Delay(200);
	}
	
	if(TWO == 2) {
		LEDOn(PLED);
		Delay(200);
		LEDOff(PLED);
		Delay(200);
	}
	
	return 0;
}