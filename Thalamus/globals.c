/*!
\file Thalamus/globals.c
\brief Contains all the global values
*/

#include "all.h"

unsigned int PRGBlankTimer;				/*!< Blanking time for button pushes */
unsigned int PRGTimer;					/*!< Timer for button pushes, continuously increments as the button is held */
unsigned int PRGPushTime;				/*!< Contains the time that a button was pushed for, populated after button is released */

unsigned char flashPLED=0;				/*!< Boolean to enable PLED flashing */
unsigned char flashVLED=0;				/*!< Boolean to enable VLED flashing */
unsigned char flashRLED=0;				/*!< Boolean to enable RLED flashing */