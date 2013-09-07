#ifndef __CONTROL_H__
#define __CONTROL_H__

// void control_gimbal() {

// }

// void control_led() {

// }


#define THROTTLEOFFSET	900		// Corresponds to zero output PWM. Nominally 1000=1ms, but 800 works better
#define IDLETHROTTLE		175		// Minimum PWM output to ESC when in-flight to avoid motors turning off
#define MAXTHROTTLE	 	1200	// Maximum PWM output to ESC (PWM = THROTTLEOFFSET + MAXTHROTTLE)
#define MAXTHROTTLEPERCENT	0.9	 // Maximum percentage throttle should be (reserves some extra output for stabilisation at high throttle).


extern unsigned char got_setpoint; //bool
extern float pitchcorrectionav, rollcorrectionav, yawcorrectionav;
extern float motorN, motorE, motorS, motorW;
extern float motorNav, motorEav, motorSav, motorWav;
extern float tempN;
extern float tempE;
extern float tempS;
extern float tempW;

void control_throttle(void);
void control_motors(void);

#endif