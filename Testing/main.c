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


void wreg(unsigned char reg, unsigned char val) {
	SSP1S0SEL();
	SSP1WriteByte(reg | 0x02);
	SSP1WriteByte(val);
	SSP1S0CLR();
}
unsigned char rreg(unsigned char reg) {
	unsigned char retval;
	SSP1S0SEL();
	SSP1WriteByte(reg);
	retval = SSP1Byte(0x00); // dummy 8 bytes
	SSP1S0CLR();
	return retval;
}

void setup(void) {
	XBeeReset();
	LEDInit(PLED | VLED);
	Port0Init(PIN13 | PIN14 | PIN15 | PIN16);
	Port0SetOut(PIN13 | PIN15);
	
	ResetPin(1);
	
	SSP1Init(1000);
	
	wreg(rPINCTL, bmFDUPSPI | bmPOSINT); 	// R17: PINCTL, full duplex mode, interrupt edge, negative edge, GPX is OPERATE
	wreg(rUSBCTL, bmCHIPRES); 				// R15: USBCTL, chip reset
	wreg(rUSBCTL, 0x00); 					// R15: USBCTL, release reset
	while((rreg(rUSBIRQ) & bmOSCOKIRQ) == 0) Delay(1);	// R13: USBIRQ, wait for OSCOKIRC
	
	wreg(rMODE, bmDPPULLDN | bmDMPULLDN | bmHOST); // R27: MODE, enable DP and DM pulldowns and host mode (highspeed mode)
	wreg(rHIRQ, bmCONDETIRQ); // R25: HIRQ, clear the CONDET pin
	
	
}

unsigned char connectedStatus=0;
unsigned char waitFrames = 0;

void loop(void) {
	if(waitFrames > 0) {
		if(rreg(rHIRQ) & bmFRAMEIRQ) { // R25: HRQ, check for the FRAMEIRQ, decrement the waitFrame counter if it is set
			wreg(rHIRQ, bmFRAMEIRQ); // clear the IRQ
			waitFrames--;
		}
	}
	else {
		switch(connectedStatus) {
			default:
			case 0:	// Nothing connected, poll for things plugged in
				wreg(rHCTL, bmSAMPLEBUS); // R29: HCTL, sample state of bus
				unsigned char hrsl = rreg(rHRSL); // R31: HRSL
				if(hrsl & (bmJSTATUS | bmKSTATUS)) {	// something connected
					if(hrsl & bmJSTATUS) {	// check JSTATUS, JStatus means D+ high, therefore FS device
						wreg(rMODE, bmDPPULLDN | bmDMPULLDN | bmHOST | bmSOFKAENAB);	// R27: MODE, enable DP and DM pulldowns, host mode, and auto generation of SOF packets/keep alive
					}
					else if(hrsl & bmKSTATUS) { // KSTATUS, in FS, KStatus means D- high, therefore LS device
						wreg(rMODE, bmDPPULLDN | bmDMPULLDN | bmHOST | bmLOWSPEED | bmSOFKAENAB);	// R27: MODE, enable DP and DM pulldowns, host mode, low-speed, and auto generation of SOF packets/keep alive
					}
					
					waitFrames = 200;			// set to wait for 200 frames
					connectedStatus = 1;
				}
				break;
			case 1:	// Device connecting, collect descriptors
				// ...
				LEDOn(PLED);
				
				
				wreg(rHIRQ, bmCONDETIRQ);	// clear disconnect status
				connectedStatus = 2;
				break;
			case 2: // Device active
				// ...
			
				// check for disconnect
				if(rreg(rHIRQ) & bmCONDETIRQ) {
					connectedStatus = 3;
				}
				break;
			case 3: // Device disconnected, clean up
				wreg(rMODE, bmDPPULLDN | bmDMPULLDN | bmHOST);	// turn off bmSOFKAENAB, go to FS
				connectedStatus = 0;
				LEDOff(PLED);
				break;
		}
	}
	Delay(1);
}