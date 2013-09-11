/*!
\file Thalamus/inc/globals.h
\brief Contains all the global values
*/

#ifndef __GLOBALS_H__
#define __GLOBALS_H__

#define FIRMWARE_VERSION	1			/*!< Firmware version */
#define MESSAGE_LOOP_HZ	 	25		  	/*!< Max frequency of ilink param messages in Hz (keep this number low, like around 20) */
#define RX_PANIC_SECONDS	1		   	/*!< Number of seconds after missing RX before craft considered "disconnected" */

#define FAST_RATE		   	400			/*!< Main loop rate in Hz*/
#define SLOW_RATE		   	75			/*!< Slow rate in Hz */
#define SLOW_DIVIDER		FAST_RATE/SLOW_RATE		/*!< The divider value (number of loop iterations) that is used to obtain the slow rate from fast rate */
#define RX_PANIC			RX_PANIC_SECONDS*SLOW_RATE	/*!< Number of loop iterations that is used to obtain the correct RX_PANIC period */

extern unsigned int PRGBlankTimer;
extern unsigned int PRGTimer;
extern unsigned int PRGPushTime;
extern unsigned char flashPLED, flashVLED, flashRLED;

#endif