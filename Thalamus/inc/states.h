#ifndef __STATES_H__
#define __STATES_H__

// State
#define STATE_UNINIT 		0		/*!< Uninitialised State */
#define STATE_DISARMED 		1		/*!< Disarmed State */
#define STATE_MANUAL 		2		/*!< Manual State */
#define STATE_MANUAL_GPS 	3		/*!< Manual+GPS State */
#define STATE_SIMPLICITY 	4		/*!< Simplicity Mode */
#define STATE_AUTO 			5		/*!< Auto State */

extern unsigned char armed;
extern unsigned char state;
extern unsigned char airborne; //boolean

void state_machine(void);
void arm(void);
void disarm(void);

#endif