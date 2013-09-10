#ifndef __GLOBALS_H__
#define __GLOBALS_H__

// *** Config stuff
#define FIRMWARE_VERSION    1           // Firmware version
#define MESSAGE_LOOP_HZ     50          // Max frequency of message loop in Hz (keep this number low, like around 25)
#define XBEE_PANIC          3           // Number of seconds after missing heartbeat from GCS before considering XBee fail
#define GPS_PANIC           3           // Number of seconds after no message from GPS before considering GPS fail
#define THAL_PANIC          3           // Number of seconds after no message from Thalamus before considering Thalamus fail
#define IDLE_SPRF           0.9         // Some filtering on the CPU load
#define IDLE_MAX            0x1ff480    // Set this to the number that the idleCounter counts up to every second.  This is used to measure the CPU load - when there are no interrupts (i.e. processor not "busy"), the idleCounter is being incremented

#define INTMODE_OFF         0 // horizontal is off (maximum), veritcal is off (maximum)
#define INTMODE_VERTICAL    1 // horizontal is off (maximum), vertical interpolates up to the target
#define INTMODE_HORIZONTAL  2 // horizontal interpolates, vertical is off (maximum)
#define INTMODE_3D          3 // horizontal interpolates, vertical interpolates
#define INTMODE_DOWN        4 // horizontal is off (maximum), vertical decrements indefinately
#define INTMODE_UP_AND_GO   5 // same as UP, but auto-switches to waypoint on completion
#define INTMODE_SEQUENCE_UP 6 // same as UP, but auto-switches to SEQUENCE_NORMAL on completion
#define INTMODE_SEQUENCE_3D 7 // same as 3D, but auto-switches to SEQUENCE_DOWN on completion
#define INTMODE_SEQUENCE_DOWN   8 // same as DOWN, but with allow_land explicitely set


// *** Status stuff
extern volatile unsigned int allowTransmit;

// *** LEDs and buttons stuff
extern volatile unsigned int flashVLED;
extern volatile unsigned int PRGTimer;
extern volatile unsigned int PRGLastState;
extern volatile unsigned int PRGPushTime;
extern volatile unsigned int PRGBlankTimer;

extern unsigned int counter;

// *** Timers and counters
extern unsigned int sysMS;
extern unsigned long long sysUS;
extern unsigned int idleCount;
extern unsigned short heartbeatWatchdog;
extern unsigned short gpsWatchdog;
extern unsigned short thalWatchdog;
extern unsigned short thalAvailable;


// Timers
extern unsigned char dataRate[MAV_DATA_STREAM_ENUM_END];
extern unsigned char heartbeatCounter;
extern unsigned short extra3ChannelCounter;
extern unsigned short extra2ChannelCounter;
extern unsigned short extra1ChannelCounter;
extern unsigned short rcChannelCounter;
extern unsigned short rawControllerCounter;
extern unsigned short positionStreamCounter;
extern unsigned short extStatusStreamCounter;
extern unsigned short rawSensorStreamCounter;

#endif