#ifndef __CONTROL_H__
#define __CONTROL_H__

// Control/PID
extern unsigned char got_setpoint;

#define THROTTLEOFFSET	900				/*!< Corresponds to less than zero output PWM. Nominally 1000=1ms, but lower just in case */
#define IDLETHROTTLE		175			/*!< Minimum PWM output to ESC when in-flight to avoid motors turning off */
#define MAXTHROTTLE	 	1200			/*!< Maximum PWM output to ESC (PWM = THROTTLEOFFSET + MAXTHROTTLE) */
#define MAXTHROTTLEPERCENT	0.9	 		/*!< Maximum percentage throttle should be (reserves some extra output for stabilisation at high throttle). */

extern unsigned char thal_throt_cont;
extern unsigned char thal_motor_off;

extern float GPS_KerrI;
extern float ULT_KerrI;
extern float targetZ_ult;
extern float alt_tkoff;

extern float ROLL_SPL_set;
extern float PITCH_SPL_set;
extern float YAW_SPL_set;



void control_throttle(void);
void control_motors(void);

#endif