#include "thal.h"
#include "MAX3421E.h"
//#include "SSD1306.h"

//void DebugPin(unsigned char value) {	Port0Write(PIN15, value); }
void ResetPin(unsigned char value) {	Port0Write(PIN13, value); }

unsigned char INTPin(void) { return Port0Read(PIN14); }
unsigned char ResetButton(void) { return Port0Read(PIN16); }

// PIN13: Reset output
// PIN14: Interupt input
// PIN15: Debug output
// PIN16: Reset button input

void SSD1306Command(unsigned char command) {
	unsigned char buffer[3];
	buffer[0] = 0x78;
	buffer[1] = 0;
	buffer[2] = command;
	I2CMaster(buffer, 3, 0, 0);
}

void SSD1306Init(void) {
	I2CInit(400);
	
	// set up reset pin
	// issue a hardware reset
	
	SSD1306Command(0xae);	// display off
	SSD1306Command(0xd5);	// display clock div
	SSD1306Command(0x80); 	// (suggested use 0x80)
	SSD1306Command(0xa8);  	// set multiplex
	SSD1306Command(0x1F);	// (0x1f)
	SSD1306Command(0xd3);  	// set display offset
	SSD1306Command(0x00);  	// (zero offset)
	SSD1306Command(0x40);  	// set start line zero
	SSD1306Command(0x8d);  	// charge pump
	SSD1306Command(0x14);	// (use switch cap)
	SSD1306Command(0x20);  	// memory mode
	SSD1306Command(0x00);  	// (ks0108 mode)
	SSD1306Command(0xa1); 	// Segremap 0x1
	SSD1306Command(0xc8);	// comscandec
	SSD1306Command(0xda);	// setcompins
	SSD1306Command(0x02);	// (0x02)
	SSD1306Command(0x81);	// set contrast
	SSD1306Command(0x8F);	// (0x8f)
	SSD1306Command(0xd9);	// setprecharge
	SSD1306Command(0xF1); 	// (0xf1)
	SSD1306Command(0xdb);	// set vcomdetect
	SSD1306Command(0x40);	// (0x40)
	SSD1306Command(0xa4);	// display all on resume
	SSD1306Command(0xa6);	// normal display
	SSD1306Command(0xaf);	// display on
}

void setup(void) {
	XBeeReset();
	LEDInit(PLED | VLED);
	Port0Init(PIN13);
	Port0SetOut(PIN13);
	
	ResetPin(1);

	MAXUSBInit();
	RandomSeed(0);
	SSD1306Init();
	//LEDOn(PLED);
}

void loop(void) {
	MAXUSBProcess();
	Delay(1);
	
	SSD1306Command(0x00);  // low col = 0
	SSD1306Command(0x10);  // hi col = 0
	SSD1306Command(0x22); // line #0
	SSD1306Command(0x00); // line #0
	SSD1306Command(0x07); // line #0
	
	unsigned char buffer[128+2];
	unsigned int i, j;
	buffer[0] = 0x78;
	buffer[1] = 0x40;
	
	for(j=0; j<8; j++) {
		for(i=2; i<128+2; i++) {
			buffer[i] = Random();
		}
		I2CMaster(buffer, 128+2, 0, 0);
	}
/*
	for(j=0; j<32; j++) {
		for(i=2; i<18; i++) {
			buffer[i] = 0;
		}
		I2CMaster(buffer, 18, 0, 0);
	}*/
	
}