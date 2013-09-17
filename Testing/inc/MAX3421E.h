#ifndef __MAX3421E_H__
#define __MAX3421E_H__



extern void DebugPin(unsigned char value);

extern unsigned char connectedStatus;
extern unsigned char waitFrames;

extern unsigned char maxPacketSize;

extern unsigned int last_transfer_size;

extern unsigned char MAXUSBData[2000];
extern unsigned short MAXUSB_VID;
extern unsigned short MAXUSB_PID;
extern unsigned char MAXUSB_MFG;
extern unsigned char MAXUSB_PRO;
extern unsigned char MAXUSB_SER;
extern unsigned char MAXUSB_CFG;
extern unsigned char MAXUSB_MFGStr[128];
extern unsigned char MAXUSB_PROStr[128];
extern unsigned char MAXUSB_SERStr[128];

void MAXUSBInit(void);
void MAXUSBWriteRegister(unsigned char reg, unsigned char val);
void MAXUSBWriteBytes(unsigned char reg, unsigned char * bytes, unsigned char length);
unsigned char MAXUSBReadRegister(unsigned char reg);

unsigned char MAXUSBSendPacket(unsigned char token, unsigned char endpoint);
unsigned char MAXUSBReadCTL(unsigned char * pSUD);
unsigned char MAXUSBWriteCTL(unsigned char * pSUD);
unsigned char MAXUSBINTransfer(unsigned char endpoint, unsigned int length);

void MAXUSBProcess(void);
#endif


