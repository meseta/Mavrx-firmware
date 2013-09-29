#include "thal.h"
#include "MAX3421E.h"
#include "MAX3421EReg.h"

void DebugPin(unsigned char value) {	Port0Write(PIN15, value); }

unsigned char USBState=0; /*!< Connection state machine */
unsigned char waitFrames = 0; /*!< Number of frames to delay */

unsigned char maxPacketSize; /*!< Maximum packet size, as per descriptor */

unsigned int last_transfer_size = 0;
unsigned short maxNak = 300;
unsigned char usbConnected = 0;
unsigned char usbActivity = 0;

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

unsigned char inCount, outCount;
unsigned char inEndpoint[4], outEndpoint[4];
unsigned short inEndpointSize[4], outEndpointSize[4];

#define MAX_AXIS	6

unsigned int buttons;
unsigned int axis[MAX_AXIS];
unsigned char buttonCount = 0;
unsigned char buttonStart, buttonLength;
unsigned char axisStart[MAX_AXIS], axisLength[MAX_AXIS];

void process_xbox360(void) {
	if(MAXUSBINTransfer(inEndpoint[0], inEndpointSize[0]) == 0) {
		if(MAXUSBData[0] == 0 && MAXUSBData[1] == 0x14) {
			buttons = MAXUSBData[2] | (MAXUSBData[3] << 8);
			axis[0] = MAXUSBData[6] | (MAXUSBData[7] << 8);
			axis[1] = MAXUSBData[8] | (MAXUSBData[9] << 8);
			axis[2] = MAXUSBData[10] | (MAXUSBData[11] << 8);
			axis[3] = MAXUSBData[12] | (MAXUSBData[13] << 8);
			axis[4] = MAXUSBData[4];
			axis[5] = MAXUSBData[5];
			
			if(buttons) LEDOn(PLED);
			else LEDOff(PLED);
		}
	}
}

unsigned char setup_xbox360(void) {
    unsigned char setConfigto1[8]     = {0x00,0x09,0x01,0x00,0x00,0x00,0x00,0x00}; 
	
	LEDOn(VLED);
	
	inEndpoint[0] = 0x01;
	inEndpointSize[0] = 32;
	inCount = 1;
	outEndpoint[0] = 0x02;
	outEndpointSize[0] = 32;
	outCount = 1;
	
	MAXUSBWriteCTL(setConfigto1); // set config to 1
	return 1;
}

void process_genericMouse(void) { // Generic mouse with the HID Boot protocol - no HID descriptors to mess with, X-Y axis and scroll only, no buttons
	if(MAXUSBINTransfer(inEndpoint[0], inEndpointSize[0]) == 0) {
		LEDToggle(VLED);
        buttons = getBits(MAXUSBData, buttonStart, buttonLength);
		axis[0] = getBits(MAXUSBData, axisStart[0], axisLength[0]);	// X-axis
		axis[1] = getBits(MAXUSBData, axisStart[1], axisLength[1]);	// Y-axis
		axis[2] = getBits(MAXUSBData, axisStart[2], axisLength[2]);	// wheel
		axis[3] = getBits(MAXUSBData, axisStart[3], axisLength[3]);	// wheel tilt
		
		if(buttons) LEDOn(PLED);
		else LEDOff(PLED);
	}
}

unsigned char setup_genericMouse(void) { // extract information form HID descriptor, certain assumptions are made about the structure of this descriptor
	unsigned int bitCount = 0, i, j;
	unsigned char usageCount=0;
	unsigned short usageList[5];
	unsigned char usagePage=0;
	unsigned char reportCount=1;
	unsigned char reportSize=0;
	
	LEDOn(VLED);
	
    unsigned char setConfigto1[8]     = {0x00,0x09,0x01,0x00,0x00,0x00,0x00,0x00}; 
	unsigned char getHIDDescriptor[8] = {0x81,0x06,0x00,0x22,0x00,0x00,0x80,0x00};
					
	if(MAXUSBReadCTL(getHIDDescriptor)) return 0;
	
	buttonLength = 0;
	for(i=0; i< MAX_AXIS; i++) axisLength[i] = 0;
	buttonCount = 0;
	
	for(i=0; i< last_transfer_size; ) { // don't increment i, i must be incremented by 1, 2, or 3 based on value
		switch(MAXUSBData[i]) {
			case 0x05: // usage page (1 byte)
				usagePage = MAXUSBData[i+1];
				break;
			case 0x09: // usage (1 byte)
				usageList[usageCount++] = MAXUSBData[i+1];
				break;
			case 0x0a: // usage (2 byte)
				usageList[usageCount++] = MAXUSBData[i+1] | (MAXUSBData[i+2] << 8);
				break;
			case 0x95: // report count
				reportCount = MAXUSBData[i+1];
				break;
			case 0x75: // report size
				reportSize = MAXUSBData[i+1];
				break;
			// case 0x25: // Logical maximum
				// logicalMaximum = MAXUSBData[i+1];
				// break;
			// case 0x15: // Logical minimum
				// logicalMinimum = MAXUSBData[i+1];
				// break;
			case 0x81: // Input
				switch(usagePage) {
					case 0x09: // buttons
						buttonStart = bitCount;
						buttonLength = reportCount * reportSize;
						buttonCount = buttonLength;
						bitCount += buttonLength;
						break;
					case 0x01: // Generic Desktop
						if(reportCount < usageCount) usageCount = reportCount;
					
						for(j=0; j<usageCount; j++) {
							switch(usageList[j]) {
								case 0x30: // X
									axisStart[0] = bitCount;
									axisLength[0] = reportSize;
									break;
								case 0x31: // Y
									axisStart[1] = bitCount;
									axisLength[1] = reportSize;
									break;
								case 0x38: // Wheel
									axisStart[2] = bitCount;
									axisLength[2] = reportSize;
									break;
							}
							bitCount += reportSize;
						}
						
						break;
					case 0x0c: // Consumer
						if(reportCount < usageCount) usageCount = reportCount;
						for(j=0; j<usageCount; j++) {
							switch(usageList[j]) {
								case 0x238: // AC Pan (Wheel tilt)
									axisStart[3] = bitCount;
									axisLength[3] = reportSize;
									break;
							}
							bitCount += reportSize;
						}
						break;
					default: // everything else, ignore, but still increment the bit counter
						bitCount += reportCount * reportSize;
				}
				usagePage = 0;
				usageCount = 0;
				// fallthrough!
			case 0x91: // output, ignores everything
				usageCount = 0;
				break;
		}
		i += (MAXUSBData[i] & 0x3) + 1;
	}
	
		DebugPin(0);
SSP1WriteByte(buttonStart);
SSP1WriteByte(buttonLength);
SSP1WriteByte(axisStart[0]);
SSP1WriteByte(axisLength[0]);
SSP1WriteByte(axisStart[1]);
SSP1WriteByte(axisLength[1]);
SSP1WriteByte(axisStart[2]);
SSP1WriteByte(axisLength[2]);
SSP1WriteByte(axisStart[3]);
SSP1WriteByte(axisLength[3]);
SSP1Wait();
DebugPin(1);
	
	MAXUSBWriteCTL(setConfigto1); // set config to 1
	return 1;
}

unsigned int getBits(unsigned char * buffer, unsigned char start, unsigned char length) { // get bits from buffer
	unsigned int result = 0;
	unsigned int resultMask = 0xffffffff;
	unsigned char byteNumber = (start >> 3);
	unsigned char bitNumber = start & 0x7;
	unsigned char gotBits = 0;
	
	while(gotBits < length) {
		result |= ((buffer[byteNumber] >> bitNumber) & 0xff) << gotBits;
		gotBits += 8-bitNumber;
		byteNumber++;
		bitNumber = 0;
	}
	
	if(length > 32 && length > 1) resultMask = (0x1 << length) - 1;
	return result & resultMask;
}

void (*processFunction)(void);
unsigned char (*setupFunction)(void);

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
			if(++nak_count>=maxNak) break;		// maximum 200 NAKs
		}
		else if(retval == hrTIMEOUT) {
			if (++retry_count>=3) break;	// maximum 3 retries
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
	static unsigned char setAddressto7[8]    = {0x00,0x05,0x07,0x00,0x00,0x00,0x00,0x00};
	static unsigned char deviceDescriptor[8] = {0x80,0x06,0x00,0x01,0x00,0x00,0x00,0x00}; // code fills in length field
	static unsigned char configDescriptor[8] = {0x80,0x06,0x00,0x02,0x00,0x00,0x00,0x00};
	static unsigned char descriptorString[8] = {0x80,0x06,0x00,0x03,0x00,0x00,0x40,0x00};
    
	unsigned int i, j;
	static unsigned char pollCounter, dcCheckCounter;

	static unsigned char lastState;
	if(USBState != lastState) {
			DebugPin(0);
			SSP1WriteByte(USBState);
			SSP1Wait();
			DebugPin(1);
			lastState = USBState;
	}
	
	if(waitFrames > 0) {
		if(MAXUSBReadRegister(rHIRQ) & bmFRAMEIRQ) { // R25: HRQ, check for the FRAMEIRQ, decrement the waitFrame counter if it is set
			MAXUSBWriteRegister(rHIRQ, bmFRAMEIRQ); // clear the IRQ
			waitFrames--;
		}
	}
	else {
		switch(USBState) {
			default:
			case 0:	// Nothing connected, poll for things plugged in
                maxNak = 300;
                
                if(++dcCheckCounter >= 100) {
                    dcCheckCounter = 0;
                
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
                        USBState = 1;
                    }
                }
				break;
			case 1:	// Device connecting, perform initial reset
				MAXUSBWriteRegister(rHCTL, bmBUSRST);			// R29: perform bus reset
				USBState = 2;
				// initialise stuff
				maxPacketSize = 8;
				deviceDescriptor[6]=8;		// wLengthL
				deviceDescriptor[7]=0;		// wLengthH
				break;
			case 2: // Wait for bus to reset
				/*! \todo: bus reset timeout */
				if((MAXUSBReadRegister(rHCTL) & bmBUSRST) == 0) {  // wait for bus reset
					waitFrames = 200;			// set to wait for 200 frames after bus reset
					USBState = 3;
				}
				break;
			case 3: // Get device descriptor
				MAXUSBWriteRegister(rPERADDR, 0x00); // first packet to address 0
				if(MAXUSBReadCTL(deviceDescriptor)) USBState = 8; // if error, go to wait for disconnect
				else {
					maxPacketSize = MAXUSBData[7];
					MAXUSBWriteRegister(rHCTL, bmBUSRST);			// R29: perform anothrbus reset
					USBState = 4;
				}
				break;
			case 4: // Wait for bus to reset another time
				/*! \todo: bus reset timeout */
				if((MAXUSBReadRegister(rHCTL) & bmBUSRST) == 0) {  // wait for bus reset
					waitFrames = 200;			// set to wait for 200 frames after bus reset
					USBState = 5;
				}
				break;
			case 5: // Set address to 7
				if(MAXUSBWriteCTL(setAddressto7)) USBState = 8; // if error, wait for disconnect
				else {
					waitFrames = 200; // was 20, increase to 300
					USBState = 6;
				}
				break;
			case 6: // Read device descriptors
				MAXUSBWriteRegister(rPERADDR, 7); // all transfers to addres 7
				deviceDescriptor[6] = 0x12; // device descriptor length
				if(MAXUSBReadCTL(deviceDescriptor)) USBState = 8; // if error, wait for disconnect
				else {
					// IDs
					MAXUSB_VID = MAXUSBData[8] | (MAXUSBData[9] << 8);
					MAXUSB_PID = MAXUSBData[10] | (MAXUSBData[11] << 8);
					MAXUSB_MFG = MAXUSBData[14];
					MAXUSB_PRO = MAXUSBData[15];
					MAXUSB_SER = MAXUSBData[16];
					MAXUSB_CFG = MAXUSBData[17];

					// Lang
					descriptorString[2]=0;	// index 0 is language ID string
					descriptorString[4]=0;	// lang ID is 0
					descriptorString[5]=0;
					descriptorString[6]=4;	// wLengthL
					descriptorString[7]=0;	// wLengthH
					if(MAXUSBReadCTL(descriptorString) == 0) { // update language if success
						descriptorString[4] = MAXUSBData[2];
						descriptorString[5] = MAXUSBData[3];
						descriptorString[6] = 255;
					}
					
					// Manufacturer string
					if(MAXUSB_MFG > 0) { //
						descriptorString[2] = MAXUSB_MFG;
						MAXUSBReadCTL(descriptorString);
						j=0;
						for(i=2; i<last_transfer_size; i+=2) {
							MAXUSB_MFGStr[j++] = MAXUSBData[i];
						}
					}
					else MAXUSB_MFGStr[0] = 0;
					
					// Product string
					if(MAXUSB_PRO > 0) {
						descriptorString[2] = MAXUSB_PRO;
						MAXUSBReadCTL(descriptorString);
						j=0;
						for(i=2; i<last_transfer_size; i+=2) {
							MAXUSB_PROStr[j++] = MAXUSBData[i];
						}
					}
					else MAXUSB_PROStr[0] = 0;
					
					// Serial
					/*if(MAXUSB_SER > 0) {
						descriptorString[2] = MAXUSB_SER;
						MAXUSBReadCTL(descriptorString);
						j=0;
						for(i=2; i<last_transfer_size; i+=2) {
							MAXUSB_SERStr[j++] = MAXUSBData[i];
						}
					}
					else MAXUSB_SERStr[0] = 0;*/
					
										
					// Detect supported devices
					processFunction = 0;
					setupFunction = 0;
					switch(MAXUSB_VID) {
						case VID_MICROSOFT:
							if(MAXUSB_PID == PID_XBOX360) {
								processFunction = process_xbox360;
								setupFunction = setup_xbox360;
							}
							break;
					}
					
					
					// If no supported devices found, try to find generic
					if(processFunction == 0) {
						// 9-byte Config descriptor
						configDescriptor[6] = 9;
						configDescriptor[7] = 0;
						if(MAXUSBReadCTL(configDescriptor)) USBState = 8; // if error, wait for disconnect
						else {
							
							// Full config descriptor
							// 0 bLength
							// 1 bDescriptorType
							// 2 wTotalLength
							// 3 wTotalLength
							// 4 bNumInterfaces
							// 5 bConfigurationValue
							// 6 iConfiguration
							// 7 bmAttributes
							// 8 bMaxPower
							
							configDescriptor[6] = MAXUSBData[2];
							configDescriptor[7] = MAXUSBData[3];
							MAXUSBReadCTL(configDescriptor);
							
							// config MAXUSBData[5]
							// interfaces MAXUSBData[4]
							// if MAXUSBData[7] & 0x40 - self-powered
							// else MAXUSBData[8]*2 milliamps
							
							// Parse config
							unsigned int TotalLen;
							unsigned char len, type;
							//unsigned char icfg = MAXUSBData[6]; // optional configuration string
							
							// Get Interface descriptors
							TotalLen=last_transfer_size;
							i=0;
							inCount = 0;
							outCount = 0;
							
							do {
								// All descriptors start with
								// 0 bLength
								// 1 bDescriptorType
								len = MAXUSBData[i];		// length of first descriptor (the CONFIG descriptor)
								type = MAXUSBData[i+1];
								
								switch(type) {
									case 0x04: // Interface descriptor
										// 2 bInterfaceNumber
										// 3 bAlternateSetting
										// 4 bNumEndpoints
										// 5 bInterfaceClass
										// 6 bInterfaceSubClass
										// 7 bInterfaceProtocol
										// 8 iInterface
										if(MAXUSBData[i+5] == 0x03 && MAXUSBData[i+6] == 0x01 && MAXUSBData[i+7] == 0x02) { // HID device && HID boot protocol && ID mouse
											processFunction = process_genericMouse;
											setupFunction = setup_genericMouse;
											inEndpoint[0] = 0x01;	// safe values to use in case endpoints not detected
											inEndpointSize[0] = 0x3;
										}
										break;
									case 0x05: // Endpoint descriptor
										// 2 bEndpointAddress
										// 3 bmAttributes
										// 4 wMaxPacketSize
										// 5 wMaxPacketSize
										// 6 bInterval
										if((MAXUSBData[i+3] & 0x03) == 0x03) { // Interrupt type endpoint
											if((MAXUSBData[i+2] & 0x80) == 0x80) { // input
												inEndpoint[inCount] = MAXUSBData[i+2] & 0xF;
												inEndpointSize[inCount] = MAXUSBData[i+4] | MAXUSBData[i+5] << 8;
												inCount++;
											}
											else {	// output
												outEndpoint[outCount] = MAXUSBData[i+2] & 0xF;
												outEndpointSize[outCount] = MAXUSBData[i+4] | MAXUSBData[i+5] << 8;
												outCount++;
											}
										}
									case 0x21: // HID Class Descriptor
										// 2 bcdHID
										// 3 bcdHID
										// 4 bCountryCode
										// 5 bNumDescriptors
										// 6 bDescriptorType
										// 7 wDescriptorLength
										// 8 wDescriptorLength
										break;
								}
								i += len; // next descriptor
							} while (i<TotalLen);
							
							// parse alternative config
							/*if(icfg) { // alternative configuration
								descriptorString[2] = icfg;
								MAXUSBReadCTL(descriptorString);
								j=0;
								for (i=2; i<last_transfer_size;i+=2) {
									// somebuffer[j++} = MAXUSBData[i];
								}
							}*/
						
						}
					}
                    
					// if generic found, get HID descriptors
					
					MAXUSBWriteRegister(rHIRQ, bmCONDETIRQ);	// clear disconnect status
					if(setupFunction) {
						if((*setupFunction)()) {
							usbConnected = 1;
							USBState = 7;
						}
						else {
							USBState = 8; // if error, go to wait for disconnect
						}
					}
					else {
						USBState = 8;
					}
				}
				break;
			case 7: // Connected, device active
				if(processFunction) { // supported device
					// poll device
					if(++pollCounter >= 11) {
						pollCounter = 0;
						maxNak = 0;
						(*processFunction)();
						maxNak = 300;
						usbActivity ^= 1;
					}
				}
				else {
					// unsupported device, go to disconnect
					USBState = 8;
				}
				
				// fallthrough! to check for disconnect
			case 8: // Check/Wait for disconnect
                if(++dcCheckCounter >= 100) {
                    dcCheckCounter = 0;
                    if(MAXUSBReadRegister(rHIRQ) & bmCONDETIRQ) {
                        USBState = 9;
                    }
                }
				break;
			case 9: // Device disconnected, clean up
				LEDOff(PLED | VLED);
				usbConnected = 0;
				processFunction = 0;
				setupFunction = 0;
        		MAXUSBWriteRegister(rMODE, bmDPPULLDN | bmDMPULLDN | bmHOST); // R27: MODE, enable DP and DM pulldowns and host mode (highspeed mode)
				MAXUSBWriteRegister(rHIRQ, bmCONDETIRQ); // R25: HIRQ, clear the CONDET pin
				USBState = 0;
				break;
		}
	}
}