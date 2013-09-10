#include "all.h"

// *** Status stuff
volatile unsigned int allowTransmit=1;

// *** LEDs and buttons stuff
volatile unsigned int flashVLED=0;
volatile unsigned int PRGTimer;
volatile unsigned int PRGLastState;
volatile unsigned int PRGPushTime;
volatile unsigned int PRGBlankTimer;

unsigned int counter = 0;

// *** Timers and counters
unsigned int sysMS=0;
unsigned long long sysUS=0;
unsigned int statusCounter;
unsigned int idleCount=0;
unsigned short heartbeatWatchdog=0;
unsigned short gpsWatchdog=0;
unsigned short thalWatchdog=0;
unsigned short thalAvailable=0;


// Timers
unsigned char dataRate[MAV_DATA_STREAM_ENUM_END];
unsigned char heartbeatCounter=0;
unsigned short extra3ChannelCounter;
unsigned short extra2ChannelCounter;
unsigned short extra1ChannelCounter;
unsigned short rcChannelCounter;
unsigned short rawControllerCounter;
unsigned short positionStreamCounter;
unsigned short extStatusStreamCounter;
unsigned short rawSensorStreamCounter;
