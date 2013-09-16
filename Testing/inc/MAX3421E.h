#ifndef __MAX3421E_H__
#define __MAX3421E_H__



extern unsigned char connectedStatus;
extern unsigned char waitFrames;

extern unsigned char maxPacketSize;
extern unsigned char deviceDescriptor[8];
extern unsigned char configDescriptor[8];

extern unsigned int last_transfer_size;

void MAXUSBInit(void);
void MAXUSBWriteRegister(unsigned char reg, unsigned char val);
void MAXUSBWriteBytes(unsigned char reg, unsigned char * bytes, unsigned char length);
unsigned char MAXUSBReadRegister(unsigned char reg);

unsigned char MAXUSBSendPacket(unsigned char token, unsigned char endpoint);
unsigned char MAXUSBReadCTL(unsigned char * pSUD);
unsigned char MAXUSBINTransfer(unsigned char endpoint, unsigned int length);

void MAXUSBProcess(void);
#endif


