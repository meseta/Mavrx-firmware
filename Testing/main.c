#include "thal.h"
#include "MAX3421E.h"

void DebugPin(unsigned char value) {	Port0Write(PIN15, value); }
void ResetPin(unsigned char value) {	Port0Write(PIN13, value); }

unsigned char INTPin(void) { return Port0Read(PIN14); }
unsigned char ResetButton(void) { return Port0Read(PIN16); }

// PIN13: Reset output
// PIN14: Interupt input
// PIN15: Debug output
// PIN16: Reset button input

void setup(void) {
	XBeeReset();
	LEDInit(PLED | VLED);
	Port0Init(PIN13 | PIN14 | PIN15 | PIN16);
	Port0SetOut(PIN13 | PIN15);
	
	ResetPin(1);
	
	
	SSP0Init(1000); // debug
	
	MAXUSBInit();
	
}

void loop(void) {
	MAXUSBProcess();
	Delay(1);
}