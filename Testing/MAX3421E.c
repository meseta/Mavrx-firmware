#include "thal.h"
#include "MAX3421E.h"
#include "MAX3421EReg.h"

unsigned char connectedStatus=0; /*!< Connection state machine */
unsigned char waitFrames = 0; /*!< Number of frames to delay */

unsigned char maxPacketSize; /*!< Maximum packet size, as per descriptor */
unsigned char deviceDescriptor[8] = {0x80,0x06,0x00,0x01,0x00,0x00,0x00,0x00};
unsigned char configDescriptor[8] = {0x80,0x06,0x00,0x02,0x00,0x00,0x00,0x00};

unsigned int last_transfer_size = 0;

unsigned char MAXUSBData[2000];

void MAXUSBInit(void) {
	SSP1Init(1000);
	
	MAXUSBWriteRegister(rPINCTL, bmFDUPSPI | bmPOSINT); 	// R17: PINCTL, full duplex mode, interrupt edge, negative edge, GPX is OPERATE
	MAXUSBWriteRegister(rUSBCTL, bmCHIPRES); 				// R15: USBCTL, chip reset
	MAXUSBWriteRegister(rUSBCTL, 0x00); 					// R15: USBCTL, release reset
	while((MAXUSBReadRegister(rUSBIRQ) & bmOSCOKIRQ) == 0) Delay(1);	// R13: USBIRQ, wait for OSCOKIRC
	
	MAXUSBWriteRegister(rMODE, bmDPPULLDN | bmDMPULLDN | bmHOST); // R27: MODE, enable DP and DM pulldowns and host mode (highspeed mode)
	MAXUSBWriteRegister(rHIRQ, bmCONDETIRQ); // R25: HIRQ, clear the CONDET pin
}

void MAXUSBWriteRegister(unsigned char reg, unsigned char val) {
	SSP1S0SEL();
	SSP1WriteByte(reg | 0x02);
	SSP1WriteByte(val);
	SSP1S0CLR();
}

void MAXUSBWriteBytes(unsigned char reg, unsigned char * bytes, unsigned char length) {
	SSP1S0SEL();
	SSP1WriteByte(reg | 0x02);
	SSP1Write(bytes, length);
	SSP1S0CLR();
}

unsigned char MAXUSBReadRegister(unsigned char reg) {
	unsigned char retval;
	SSP1S0SEL();
	SSP1WriteByte(reg);
	retval = SSP1Byte(0x00); // dummy 8 bytes
	SSP1S0CLR();
	return retval;
}

unsigned char MAXUSBSendPacket(unsigned char token, unsigned char endpoint) {
	unsigned char retval;
	unsigned char retry_count = 0;
	unsigned char nak_count = 0;

	while(1) { // loop until complete or timeout                                    
		MAXUSBWriteRegister(rHXFR, token | endpoint);         	// R31: HRSL, launch host transfer
		while((MAXUSBReadRegister(rHIRQ) & bmHXFRDNIRQ) == 0);  	// R25: HIRQ, wait for the completion
		MAXUSBWriteRegister(rHIRQ,bmHXFRDNIRQ);              	// R25: HIRQ, clear interrupt flag
		retval = MAXUSBReadRegister(rHRSL) & 0x0F;    					// R31: HRSL, get result
		if(retval == hrNAK) {
			if(++nak_count==300) break;		// maximum 200 NAKs
		}
		else if(retval == hrTIMEOUT) {
			if (++retry_count==3) break;	// maximum 3 retries
		}
		else break;
	}
	return retval;
}

unsigned char MAXUSBReadCTL(unsigned char * pSUD) {
	unsigned char retval;

	// SETUP packet
	MAXUSBWriteBytes(rSUDFIFO, pSUD, 8);   // send SETUP packet to FIFO
	retval=MAXUSBSendPacket(tokSETUP,0);   	// send FIFO to EP0
	if(retval) return retval;	// return if error
	
	// One or more IN packets (may be a multi-packet transfer)
	MAXUSBWriteRegister(rHCTL, bmRCVTOG1);	// R29: HCTL, set receive toggle 1
	retval = MAXUSBINTransfer(0, (pSUD[6] | (pSUD[7] << 8)));
	if(retval) return retval;	// return if error

	retval = MAXUSBSendPacket(tokOUTHS,0);	// OUT status stage
	if (retval) return retval; // return if error
	return 0;
}

unsigned char MAXUSBINTransfer(unsigned char endpoint, unsigned int length) {
	unsigned char retval;
	unsigned char packetSize;
	unsigned int count=0, i;

	do {	// loop until return
		retval = MAXUSBSendPacket(tokIN, endpoint);     	// Send IN packet to endpoint
		if(retval) return retval;  							// return if error
		
		packetSize = MAXUSBReadRegister(rRCVBC);            // R6: receive byte count
		for(i=0; i<packetSize; i++) {                      	// add this packet's data to XfrData array
			MAXUSBData[i+count] = MAXUSBReadRegister(rRCVFIFO);	// R1: Read FIFO
		}
		
		MAXUSBWriteRegister(rHIRQ, bmRCVDAVIRQ);            // R25: HIRQ, clear IRQ to free buffer
		count += packetSize;                            // add this packet's byte count to total transfer length
	} while (packetSize >= maxPacketSize && count < length); // transfer complet if device sent a short packet or number of bytes reached
	
	last_transfer_size = count;
	return retval;
}

void MAXUSBProcess(void) {
	if(waitFrames > 0) {
		if(MAXUSBReadRegister(rHIRQ) & bmFRAMEIRQ) { // R25: HRQ, check for the FRAMEIRQ, decrement the waitFrame counter if it is set
			MAXUSBWriteRegister(rHIRQ, bmFRAMEIRQ); // clear the IRQ
			waitFrames--;
		}
	}
	else {
		switch(connectedStatus) {
			default:
			case 0:	// Nothing connected, poll for things plugged in
				MAXUSBWriteRegister(rHCTL, bmSAMPLEBUS); // R29: HCTL, sample state of bus
				unsigned char hrsl = MAXUSBReadRegister(rHRSL); // R31: HRSL
				if(hrsl & (bmJSTATUS | bmKSTATUS)) {	// something connected
					if(hrsl & bmJSTATUS) {	// check JSTATUS, JStatus means D+ high, therefore FS device
						MAXUSBWriteRegister(rMODE, bmDPPULLDN | bmDMPULLDN | bmHOST | bmSOFKAENAB);	// R27: MODE, enable DP and DM pulldowns, host mode, and auto generation of SOF packets/keep alive
					}
					else if(hrsl & bmKSTATUS) { // KSTATUS, in FS, KStatus means D- high, therefore LS device
						MAXUSBWriteRegister(rMODE, bmDPPULLDN | bmDMPULLDN | bmHOST | bmLOWSPEED | bmSOFKAENAB);	// R27: MODE, enable DP and DM pulldowns, host mode, low-speed, and auto generation of SOF packets/keep alive
					}
					
					waitFrames = 200;			// set to wait for 200 frames
					connectedStatus = 1;
				}
				break;
			case 1:	// Device connecting, perform initial reset
				MAXUSBWriteRegister(rHCTL, bmBUSRST);			// R29: perform bus reset
				connectedStatus = 2;
				// initialise stuff
				maxPacketSize = 8;
				deviceDescriptor[6]=8;		// wLengthL
				deviceDescriptor[7]=0;		// wLengthH
				break;
			case 2: // Wait for bus to reset
				/*! \todo: bus reset timeout */
				if((MAXUSBReadRegister(rHCTL) & bmBUSRST) == 0) {  // wait for bus reset
					waitFrames = 200;			// set to wait for 200 frames after bus reset
					connectedStatus = 3;
				}
				break;
			case 3: // Get device descriptor
				MAXUSBWriteRegister(rPERADDR, 0x00); // first packet to address 0
				MAXUSBReadCTL(deviceDescriptor);
				connectedStatus = 10;
				break;
			
			
			case 10:
				MAXUSBWriteRegister(rHIRQ, bmCONDETIRQ);	// clear disconnect status
				connectedStatus = 11;
				break;
			case 11: // Device active
				// ...
			
				// check for disconnect
				if(MAXUSBReadRegister(rHIRQ) & bmCONDETIRQ) {
					connectedStatus = 12;
				}
				break;
			case 12: // Device disconnected, clean up
				MAXUSBWriteRegister(rMODE, bmDPPULLDN | bmDMPULLDN | bmHOST);	// turn off bmSOFKAENAB, go to FS
				connectedStatus = 0;
				break;
		}
	}
}