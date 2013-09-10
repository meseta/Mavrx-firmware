#ifndef __GLOBALS_H__
#define __GLOBALS_H__

#define FIRMWARE_VERSION	1			/*!< Firmware version */
#define MESSAGE_LOOP_HZ	 	10		  	/*!< Max frequency of ilink param messages in Hz (keep this number low, like around 15) */
#define RX_PANIC_SECONDS	1		   	/*!< Number of seconds after missing RX before craft considered "disconnected" */

#define FAST_RATE		   	400			/*!< Main loop rate in Hz*/
#define SLOW_RATE		   	75			/*!< Slow rate in Hz */
#define SLOW_DIVIDER		FAST_RATE/SLOW_RATE		/*!< The divider value that is used to obtain the slow rate from fast rate */
#define RX_PANIC			RX_PANIC_SECONDS*SLOW_RATE

extern unsigned int PRGBlankTimer; // Blanking time for button pushes
extern unsigned int PRGTimer; // Timer for button pushes, continuously increments as the button is held
extern unsigned int PRGPushTime; // Contains the time that a button was pushed for, populated after button is released

extern unsigned char flashPLED, flashVLED, flashRLED;

#endif