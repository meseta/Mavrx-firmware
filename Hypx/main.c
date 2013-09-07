#include "thal.h"

// Buttons and LED stuff
volatile unsigned int PRGTimer;
volatile unsigned int PRGLastState;
volatile unsigned int PRGPushTime;
volatile unsigned int PRGBlankTimer;
unsigned int PRGMode;

unsigned int sysMS;
unsigned int flashVLED;

// Xbee stuff
xbee_modem_status_t xbee_modem_status;
xbee_at_command_t xbee_at_command;
xbee_at_response_t xbee_at_response;
xbee_transmit_status_t xbee_transmit_status;
xbee_receive_packet_t xbee_receive_packet;
xbee_transmit_request_t xbee_transmit_request;
xbee_node_identification_indicator_t xbee_node_identification_indicator;

#define ADDRESSLIST_SIZE    20
#define STRIKE_COUNT_MAX    100
unsigned short networkAddressList[ADDRESSLIST_SIZE];
unsigned long long sourceAddressList[ADDRESSLIST_SIZE];
unsigned char strikeAddressList[ADDRESSLIST_SIZE];
unsigned int addressListCount;

void StrikeNetworkAddress(unsigned short networkAddress);
void AddNetworkAddress(unsigned short networkAddress, unsigned long long sourceAddress);
void SendToList(unsigned char * buffer, unsigned int length);

void AddNetworkAddress(unsigned short networkAddress, unsigned long long sourceAddress) {
    unsigned int i, found;
    found = 0;
    for(i=0; i<addressListCount; i++) {
        if(sourceAddressList[i] == sourceAddress) {
            networkAddressList[i] = networkAddress;
            strikeAddressList[i] = 0;
            found = 1;
            break;
        }
    }
    
    if(found == 0) {
        if(addressListCount < ADDRESSLIST_SIZE) {
            networkAddressList[addressListCount] = networkAddress;
            sourceAddressList[addressListCount] = sourceAddress;
            strikeAddressList[addressListCount] = 0;
            addressListCount++;
        }
    }
}

void StrikeNetworkAddress(unsigned short networkAddress) {
    unsigned int i;
    for(i=0; i<addressListCount; i++) {
        if(networkAddressList[i] == networkAddress) {
            strikeAddressList[i]++;
            
            // check to see if maximum strikes reached
            if(strikeAddressList[i] > STRIKE_COUNT_MAX) {
                // drop the address and move the last address into place
                if(addressListCount > 1) {
                    networkAddressList[i] = networkAddressList[addressListCount-1];
                    sourceAddressList[i] = sourceAddressList[addressListCount-1];
                    strikeAddressList[i] = strikeAddressList[addressListCount-1];
                }
                
                if(addressListCount > 0) addressListCount--;
            }
            break;
        }
    }
}

void SendToList(unsigned char * buffer, unsigned int length) {
    unsigned int i;
    for(i=0; i<length; i++) {
        xbee_transmit_request.RFData[i] = buffer[i];
    }
    xbee_transmit_request.varLen = length;
   
    for(i=0; i<addressListCount; i++) {
        xbee_transmit_request.networkAddress = networkAddressList[i];
        xbee_transmit_request.destinationAddress = sourceAddressList[i]; //broadcast (big-endian)
        XBeeSendPacket();
    }
}


// CDC stuff
#define CDCBUFSIZE 255
unsigned char CDCBuf[CDCBUFSIZE];
volatile unsigned int CDCCount;
volatile unsigned int CDCFlag;
volatile unsigned int CDCTimeout;

// Mode/EEPROM stuff
#define EEPROM_SETTING_ADDR 0x261
#define EEPROM_XBEE_MODE    0xff
#define EEPROM_BYPASS_MODE  0xfe

unsigned int XBeeBypassMode;

void setup () {
    LEDInit(PLED);
    LEDOn(PLED);
    flashVLED = 0;
    
    // Initialise virtual serial port
	
	Delay(100);
    CDCInit(VIRTUAL);
    CDCCount = 0;
	
	Delay(200);
	
    XBeeInit();
	
	
    addressListCount = 0;
    
    // Check to see if we should go into bypass mode
    if(EEPROMReadByte(EEPROM_SETTING_ADDR) == EEPROM_BYPASS_MODE) {
        XBeeStartBypass();
        XBeeBypassMode = 1;
    }
    else {
        XBeeBypassMode = 0;
    }
    
    
    LEDOff(PLED);
    LEDInit(VLED);
    LEDWrite(VLED, XBeeBypassMode);
    PRGBlankTimer = 100;
    PRGPushTime = 0;
    PRGTimer = 0;
    PRGMode = 0;
}

void loop() {
    if(PRGBlankTimer == 0) {
        // provide some visual feedback when using the buttons, this LED comes on after 3 seconds, and then off after 8 (or vice versa in bypass mode)
        if(PRGTimer > 8000) {
            LEDWrite(VLED, XBeeBypassMode);
        }
        else if(PRGTimer > 3000) {
            LEDWrite(VLED, 1-XBeeBypassMode);
        }
        
        // begin working out button presses
        if(PRGPushTime > 15000) {   // Factory reset the XBee
            flashVLED = 5;
            EEPROMWriteByte(EEPROM_SETTING_ADDR, EEPROM_XBEE_MODE);
            XBeeStopBypass();
            XBeeBypassMode = 0;
            XBeeFactoryReset();
            
            PRGPushTime = 0;
            PRGTimer = 0;
            PRGBlankTimer = 100;
        }
        else if(PRGPushTime > 8000) { // Toggle bypass mode
            flashVLED = 5;
            if(XBeeBypassMode) {
                if(EEPROMReadByte(EEPROM_SETTING_ADDR) != EEPROM_XBEE_MODE) EEPROMWriteByte(EEPROM_SETTING_ADDR, EEPROM_XBEE_MODE);
                XBeeStopBypass();
                XBeeBypassMode = 0;
            }
            else {
                if(EEPROMReadByte(EEPROM_SETTING_ADDR) != EEPROM_BYPASS_MODE) EEPROMWriteByte(EEPROM_SETTING_ADDR, EEPROM_BYPASS_MODE);
                XBeeStartBypass();
                XBeeBypassMode = 1;
            }
            
            PRGPushTime = 0;
            PRGTimer = 0;
            PRGBlankTimer = 100;
        }
        else if(PRGPushTime > 3000) { // Create new network
            flashVLED = XBEE_JOINPERIOD*10;
            XBeeCoordinatorJoin();
            PRGMode = 1;
            
            PRGPushTime = 0;
            PRGTimer = 0;
            PRGBlankTimer = 100;
        }
        else if(PRGPushTime > 50) {
            if(PRGMode == 0) {
                flashVLED = XBEE_JOINPERIOD*10;
                XBeeAllowJoin();
                PRGMode = 1;
            }
            else {
                flashVLED = 1;
                XBeeStopJoin();
                PRGMode = 0;
            }
            
            PRGPushTime = 0;
            PRGTimer = 0;
            PRGBlankTimer = 100;
        }
    }
    
    if(XBeeBypassMode == 0 && CDCTimeout == 0 && CDCCount > 0) {
        SendToList(CDCBuf, CDCCount);
        CDCCount = 0;
    }
}

void SysTickInterrupt() {
    if(PRGBlankTimer) {
        PRGBlankTimer--;
    }
    else {
        if(PRGPoll() == 0) {
            PRGTimer++;
            PRGLastState = 0;
        }
        else {
            if(PRGLastState == 0) {
                PRGPushTime = PRGTimer;
            }
            PRGTimer = 0;
            PRGLastState = 1;
        }
    }
    
    sysMS++;
    if(sysMS % 100 == 0) {
        if(flashVLED > 0) {
            flashVLED--;
            LEDToggle(VLED);
            if(flashVLED == 0) LEDWrite(VLED, XBeeBypassMode);
        }
    }
    
    if(CDCTimeout) {
        CDCTimeout--;
    }
}

void XBeeMessage(unsigned char id, unsigned char * buffer, unsigned short length) {
    if(XBeeBypassMode) {
        CDCWrite(buffer, length);
    }
    else {
        unsigned char * ptr = 0;
        unsigned int j;
        unsigned int structsize;
        
        // Select decode
        switch(id) {
            case ID_XBEE_MODEMSTATUS:
                ptr = (unsigned char *) &xbee_modem_status;
                structsize = sizeof(xbee_modem_status);
                xbee_modem_status.isNew = 1;
                break;
            case ID_XBEE_ATRESPONSE:
                ptr = (unsigned char *) &xbee_at_response;
                structsize = sizeof(xbee_at_response);
                xbee_at_response.isNew = 1;
                xbee_at_response.varLen = length - 4;
                break;
            case ID_XBEE_TRANSMITSTATUS:
                ptr = (unsigned char *) &xbee_transmit_status;
                structsize = sizeof(xbee_transmit_status);
                xbee_at_response.isNew = 1;
                break;
            case ID_XBEE_RECEIVEPACKET:
                ptr = (unsigned char *) &xbee_receive_packet;
                structsize = sizeof(xbee_receive_packet);
                xbee_receive_packet.isNew = 1;
                xbee_receive_packet.varLen = length - 11;
                CDCWrite(buffer + 11,length - 11);  // send direct to CDCWrite
                break;
            case IX_XBEE_NODEIDENTIFICATIONINDICATOR:
                ptr = (unsigned char *) &xbee_node_identification_indicator;
                structsize = sizeof(xbee_node_identification_indicator);
                xbee_node_identification_indicator.isNew = 1;
                break;
        }
        
        if(ptr) {
            if(length > structsize-3) length = structsize-3;
            for(j=0; j<length; j++) {
                ptr[j] = buffer[j];
            }
        }
        
        // take action
        switch(id) {
            case ID_XBEE_TRANSMITSTATUS:
                if(xbee_transmit_status.deliveryStatus == 0x21) {
                    StrikeNetworkAddress(xbee_transmit_status.networkAddress);
                }
                break;
            case ID_XBEE_RECEIVEPACKET:
                AddNetworkAddress(xbee_receive_packet.networkAddress, xbee_receive_packet.sourceAddress);
                break;
            case IX_XBEE_NODEIDENTIFICATIONINDICATOR:
                if(xbee_node_identification_indicator.sourceEvent == 0x02) { // if a join event
                    XBeeStopJoin(); // disallow joining
                    flashVLED = 3;
                    PRGMode = 0;
                }
                AddNetworkAddress(xbee_node_identification_indicator.remoteNetworkAddress, xbee_node_identification_indicator.remoteSourceAddress);
                break;
        }
    }
}

void CDCReadByte(unsigned char byte) {
    if(XBeeBypassMode) {
        UARTWriteByte(byte);
    }
    else {
        CDCFlag = 1;
        if(CDCCount < 255) {
            CDCBuf[CDCCount++] = byte;
        }
        else {
            SendToList(CDCBuf, CDCCount);
            CDCCount = 0;
        }
        CDCFlag = 0;
        CDCTimeout = 10;
    }
}