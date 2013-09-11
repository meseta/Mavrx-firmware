/*!
\file Hypo/inc/globals.h
\brief Global values

\author Yuan Gao
*/

#ifndef __GLOBALS_H__
#define __GLOBALS_H__

#define FIRMWARE_VERSION    1           /*!< Firmware version */
#define MESSAGE_LOOP_HZ     25          /*!< Max frequency of message loop in Hz (keep this number low, like around 25) */
#define XBEE_PANIC          3           /*!< Number of seconds after missing heartbeat from GCS before considering XBee fail */
#define GPS_PANIC           3           /*!< Number of seconds after no message from GPS before considering GPS fail */
#define THAL_PANIC          3           /*!< Number of seconds after no message from Thalamus before considering Thalamus fail */
#define IDLE_SPRF           0.9         /*!< Some filtering on the CPU load */
#define IDLE_MAX            0x1ff480    /*!< Set this to the number that the idleCounter counts up to every second.  This is used to measure the CPU load - when there are no interrupts (i.e. processor not "busy"), the idleCounter is being incremented */

extern volatile unsigned char allowTransmit;

extern volatile unsigned int flashVLED;
extern volatile unsigned int PRGTimer;
extern volatile unsigned char PRGLastState;
extern volatile unsigned int PRGPushTime;
extern volatile unsigned int PRGBlankTimer;

extern unsigned int sysMS;
extern unsigned long long sysUS;
extern unsigned int idleCount;
extern unsigned short heartbeatWatchdog;
extern unsigned short gpsWatchdog;
extern unsigned short thalWatchdog;
extern unsigned char thalAvailable;

extern unsigned char dataRate[MAV_DATA_STREAM_ENUM_END];

#endif