#include "thal.h"

#define NUM	1024*8 - 1
volatile unsigned char test[NUM];

int main(void) {
	unsigned int i;
	unsigned char r;
	LEDInit(PLED);
	i=0;
	while(1) {
		r = Random();
		test[i] = r;
		if(test[i] == r) LEDOn(PLED);
		else LEDOff(PLED);
		i++;
		if(i >= NUM) i=0;
	}
	
	return 0;
}