#include "thal.h"
#include "MAX3421E.h"
#include "MAX3421EReg.h"

void DebugPin(unsigned char value) {	Port0Write(PIN15, value); }

unsigned char connectedStatus=0; /*!< Connection state machine */
unsigned char waitFrames = 0; /*!< Number of frames to delay */

unsigned char maxPacketSize; /*!< Maximum packet size, as per descriptor */
unsigned char deviceDescriptor[8] = {0x80,0x06,0x00,0x01,0x00,0x00,0x00,0x00};
unsigned char configDescriptor[8] = {0x80,0x06,0x00,0x02,0x00,0x00,0x00,0x00};

unsigned int last_transfer_size = 0;

unsigned char MAXUSBData[2000]; /*!< Data buffer \todo move to USBRAM */

unsigned short MAXUSB_VID;
unsigned short MAXUSB_PID;
unsigned char MAXUSB_MFG;
unsigned char MAXUSB_PRO;
unsigned char MAXUSB_SER;
unsigned char MAXUSB_CFG;
unsigned char MAXUSB_MFGStr[128];
unsigned char MAXUSB_PROStr[128];
unsigned char MAXUSB_SERStr[128];

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
	MAXUSBWriteBytes(rSUDFIFO, pSUD, 8);   	// send SETUP packet to FIFO
	retval = MAXUSBSendPacket(tokSETUP,0);	// send FIFO to EP0
	if(retval) return retval;				// return if error
	
	// One or more IN packets (may be a multi-packet transfer)
	MAXUSBWriteRegister(rHCTL, bmRCVTOG1);	// R29: HCTL, set receive toggle 1
	retval = MAXUSBINTransfer(0, (pSUD[6] | (pSUD[7] << 8)));
	if(retval) return retval;				// return if error

	retval = MAXUSBSendPacket(tokOUTHS, 0);	// OUT status stage
	if (retval) return retval; 				// return if error
	
	return 0;
}

unsigned char MAXUSBWriteCTL(unsigned char *pSUD) {
	unsigned char retval;
	
	// SETUP packet
	MAXUSBWriteBytes(rSUDFIFO, pSUD, 8);   	// send SETUP packet to FIFO
	retval = MAXUSBSendPacket(tokSETUP,0);  // send FIFO to EP0
	if(retval) return retval;				// return if error

	retval = MAXUSBSendPacket(tokINHS, 0);  // This function takes care of NAK retries.
	if(retval) return retval;  				// return if error
  
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
		for(i=0; i<packetSize; i++) {                      	// add this packet's data to MAXUSBData array
			MAXUSBData[i+count] = MAXUSBReadRegister(rRCVFIFO);	// R1: Read FIFO
		}
		
		MAXUSBWriteRegister(rHIRQ, bmRCVDAVIRQ);            // R25: HIRQ, clear IRQ to free buffer
		count += packetSize;                            // add this packet's byte count to total transfer length
	} while (packetSize >= maxPacketSize && count < length); // transfer complet if device sent a short packet or number of bytes reached
	
	last_transfer_size = count;
	return retval;
}

void MAXUSBProcess(void) {
	static unsigned char SetAddressto7[8]      		= {0x00,0x05,0x07,0x00,0x00,0x00,0x00,0x00};
	static unsigned char deviceDescriptor[8]  		= {0x80,0x06,0x00,0x01,0x00,0x00,0x00,0x00}; // code fills in length field
	static unsigned char configDescriptor[8]  		= {0x80,0x06,0x00,0x02,0x00,0x00,0x00,0x00};
	static unsigned char str[8] = {0x80,0x06,0x00,0x03,0x00,0x00,0x40,0x00};	// Get_Descriptor-String template. Code fills in idx at str[2].
	unsigned int i, j;
	
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
				if(MAXUSBReadCTL(deviceDescriptor)) connectedStatus = 8; // if error, go to wait for disconnect
				else {
					maxPacketSize = MAXUSBData[7];
					MAXUSBWriteRegister(rHCTL, bmBUSRST);			// R29: perform anothrbus reset
					connectedStatus = 4;
				}
				break;
			case 4: // Wait for bus to reset another time
				/*! \todo: bus reset timeout */
				if((MAXUSBReadRegister(rHCTL) & bmBUSRST) == 0) {  // wait for bus reset
					waitFrames = 200;			// set to wait for 200 frames after bus reset
					connectedStatus = 5;
				}
				break;
			case 5: // Set address to 7
				if(MAXUSBWriteCTL(SetAddressto7)) connectedStatus = 8; // if error, wait for disconnect
				else {
					waitFrames = 30;
					connectedStatus = 6;
				}
				break;
			case 6: // Read device descriptors
				MAXUSBWriteRegister(rPERADDR, 7); // all transfers to addres 7
				deviceDescriptor[6] = 0x12; // device descriptor length
				if(MAXUSBReadCTL(deviceDescriptor)) connectedStatus = 8; // if error, wait for disconnect
				else {
					// IDs
					MAXUSB_VID = MAXUSBData[8] | (MAXUSBData[9] << 8);
					MAXUSB_PID = MAXUSBData[10] | (MAXUSBData[11] << 8);
					MAXUSB_MFG = MAXUSBData[14];
					MAXUSB_PRO = MAXUSBData[15];
					MAXUSB_SER = MAXUSBData[16];
					MAXUSB_CFG = MAXUSBData[17];
					
					// Lang
					str[2]=0;	// index 0 is language ID string
					str[4]=0;	// lang ID is 0
					str[5]=0;
					str[6]=4;	// wLengthL
					str[7]=0;	// wLengthH
					if(MAXUSBReadCTL(str) == 0) { // update language if success
						str[4] = MAXUSBData[2];
						str[5] = MAXUSBData[3];
						str[6] = 255;
					}
					
					// Manufacturer string
					if(MAXUSB_MFG > 0) { //
						str[2] = MAXUSB_MFG;
						MAXUSBReadCTL(str);
						j=0;
						for(i=2; i<last_transfer_size; i+=2) {
							MAXUSB_MFGStr[j++] = MAXUSBData[i];
						}
					}
					else MAXUSB_MFGStr[0] = 0;
					
					// Product string
					if(MAXUSB_PRO > 0) {
						str[2] = MAXUSB_PRO;
						MAXUSBReadCTL(str);
						j=0;
						for(i=2; i<last_transfer_size; i+=2) {
							MAXUSB_PROStr[j++] = MAXUSBData[i];
						}
					}
					else MAXUSB_PROStr[0] = 0;
					
					// Serial
					if(MAXUSB_SER > 0) {
						str[2] = MAXUSB_SER;
						MAXUSBReadCTL(str);
						j=0;
						for(i=2; i<last_transfer_size; i+=2) {
							MAXUSB_SERStr[j++] = MAXUSBData[i];
						}
					}
					else MAXUSB_SERStr[0] = 0;
					
					// 9-byte Config descriptor
					configDescriptor[6] = 9;
					configDescriptor[7] = 0;
					if(MAXUSBReadCTL(configDescriptor)) connectedStatus = 8; // if error, wait for disconnect
					else {
						
						// Full config descriptor
						configDescriptor[6] = MAXUSBData[2];
						configDescriptor[7] = MAXUSBData[2];
						MAXUSBReadCTL(configDescriptor);
						
						// config MAXUSBData[5]
						// interfaces MAXUSBData[4]
						// if MAXUSBData[7] & 0x40 - self-powered
						// else MAXUSBData[8]*2 milliamps
						
						// Parse config
						unsigned int TotalLen;
						unsigned char len, type, adr, pktsize, icfg;
						icfg = MAXUSBData[6]; // optional configuration string
						
						TotalLen=last_transfer_size;
						i=0;
						do {
							len = MAXUSBData[i];		// length of first descriptor (the CONFIG descriptor)
							type = MAXUSBData[i+1];
							adr = MAXUSBData[i+2];
							pktsize = MAXUSBData[i+4];

							if(type == 0x04) {			// Interface descriptor?
								// interface MAXUSBData[i+2], alternate setting MAXUSBData[i+3]
							}
							else if(type == 0x05) {		// check for endpoint descriptor type
								// endpoint adr&0x0f
								// if MAXUSBData[i+2] & 0x80 in endpoint
								// else out endpoint
								// printf("(%02u) is type ",(BYTE)pktsize);

								switch(MAXUSBData[i+3] & 0x03) {
									case 0x00:
										// control endpoint
										break;
									case 0x01:
										// isochronous endpoint
										break;
									case 0x02:
										// bulk endpoint
										break;
									case 0x03:
										// Interrupt with polling interval of MAXUSBData[i+6]
										break;
									}
								}
							i += len; // next descriptor
						} while (i<TotalLen);
						
						// parse alternative config
						if(icfg) { // alternative configuration
							str[2] = icfg;
							MAXUSBReadCTL(str);
							j=0;
							for (i=2; i<last_transfer_size;i+=2) {
								// somebuffer[j++} = MAXUSBData[i];
							}
						}
					
						MAXUSBWriteRegister(rHIRQ, bmCONDETIRQ);	// clear disconnect status
						connectedStatus = 7;
						
						DebugPin(0);
						SSP1WriteByte(MAXUSB_VID >> 8);
						SSP1WriteByte(MAXUSB_VID & 0xff);
						SSP1WriteByte(MAXUSB_PID >> 8);
						SSP1WriteByte(MAXUSB_PID & 0xff);
						SSP1Wait();
						DebugPin(1);
						
						SSP1WriteByte(00);
						
						DebugPin(0);
						i=0;
						while(MAXUSB_MFGStr[i] != 0 && i<128) SSP1WriteByte(MAXUSB_MFGStr[i++]);
						SSP1Wait();
						DebugPin(1);
						
						SSP1WriteByte(00);
						
						DebugPin(0);
						i=0;
						while(MAXUSB_PROStr[i] != 0 && i<128) SSP1WriteByte(MAXUSB_PROStr[i++]);
						SSP1Wait();
						DebugPin(1);
						
						LEDOn(PLED);
					}
				}
				break;
			case 7: // Connected, device active
				// do stuff related to device being active
				
				// fallthrough! to check for disconnect
			case 8: // Check/Wait for disconnect
				if(MAXUSBReadRegister(rHIRQ) & bmCONDETIRQ) {
					connectedStatus = 9;
				}
				break;
			case 9: // Device disconnected, clean up
				LEDOff(PLED);
				MAXUSBWriteRegister(rMODE, bmDPPULLDN | bmDMPULLDN | bmHOST);	// turn off bmSOFKAENAB, go to FS
				MAXUSBWriteRegister(rHIRQ, bmCONDETIRQ);	// clear disconnect status
				connectedStatus = 0;
				break;
		}
	}
}