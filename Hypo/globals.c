/*!
\file Hypo/globals.c
\brief Functions for zeroing and calibrating sensors

\author Yuan Gao
*/

#include "all.h"

volatile unsigned char allowTransmit=1; /*!< Boolean to allow telemetry transmission, used to avoid transmittig stuff when nothing is connected */

// *** LEDs and buttons stuff
volatile unsigned int flashVLED=0;		/*!< Boolean for flashing LED */
volatile unsigned int PRGTimer;			/*!< Timer for counting button push */
volatile unsigned char PRGLastState;	/*!< Boolean for last button state */
volatile unsigned int PRGPushTime;		/*!< Timer for counting button push */
volatile unsigned int PRGBlankTimer;	/*!< Timer for blanking button push (i.e. debounce) */

// *** Timers and counters
unsigned int sysMS=0;					/*!< System milliseconds since boot */
unsigned long long sysUS=0;				/*!< System microseconds since boot */
unsigned int idleCount=0;				/*!< Idle loop counter */
unsigned short heartbeatWatchdog=0;		/*!< Watchdog counter to indicate when base station is lost */
unsigned short gpsWatchdog=0;			/*!< Watchdog counter to indicate when GPS is lost */
unsigned short thalWatchdog=0;			/*!< Watchdog counter to indicate when thalamus is lost */
unsigned char thalAvailable=0;			/*!< Boolean to indicate if Thalamus is available */

// Timers
unsigned char dataRate[MAV_DATA_STREAM_ENUM_END];	/*!< Data stream rates */