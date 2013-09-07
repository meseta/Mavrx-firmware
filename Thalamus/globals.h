#ifndef __GLOBALS_H__
#define __GLOBALS_H__


#define FIRMWARE_VERSION	1		   // Firmware version
#define MESSAGE_LOOP_HZ	 	15		  // Max frequency of messages in Hz (keep this number low, like around 15)
#define RX_PANIC			2		   // Number of seconds after missing RX before craft considered "disconnected"

#define FAST_RATE		   400
#define SLOW_RATE		   75

#define ZEROTHROTMAX		1*FAST_RATE

#define SLOW_DIVIDER		FAST_RATE/SLOW_RATE

//This is used with mode
#define STATE_DISARMED 1
#define STATE_MANUAL 2
#define STATE_MANUAL_GPS 3
#define STATE_AUTO 5


// LEDs
extern unsigned char flashPLED, flashVLED, flashRLED;

extern float thetaAngle, phiAngle, psiAngle, psiAngleinit;



///////////////////////////////////////// GLOBAL VARIABLE STRUCTURES /////////////////////

typedef struct {
	float roll;
	float pitch;
	float yaw;
	float throttle;
} userStruct;
extern userStruct user;

typedef struct {
	float roll;
	float pitch;
	float yaw;
} attitude_demand_body_struct;
extern attitude_demand_body_struct attitude_demand_body;


extern unsigned char armed;
extern unsigned char state;
/////////////////////////////////// GLOBAL VARIABLES /////////////////////////////////
// TODO: Why don't we just set all the variable here? Some of them are being set here, and some in the setup function.
// Timers and counters
extern unsigned int sysMS;
extern unsigned long long sysUS;
extern unsigned short RxWatchdog;
extern unsigned short UltraWatchdog;
extern unsigned short slowSoftscale;

// LEDs

// Quaternion and Rotation Matrix
extern float q1, q2, q3, q4;
extern float M1, M2, M3, M4, M5, M6, M7, M8, M9;
extern float RM1, RM2, RM3, RM4, RM5, RM6, RM7, RM8, RM9;



// Button
extern unsigned int PRGBlankTimer; // Blanking time for button pushes
extern unsigned int PRGTimer; // Timer for button pushes, continuously increments as the button is held
extern unsigned int PRGPushTime; // Contains the time that a button was pushed for, populated after button is released

extern unsigned char airborne; //boolean


extern unsigned char thal_throt_cont;

extern unsigned char thal_motor_off;


//Altitude PID states
extern float GPS_KerrI;
extern float ULT_KerrI;
extern float targetZ_ult;
extern float alt_tkoff;
extern float oldUltra;

extern float ROLL_SPL_set;
extern float PITCH_SPL_set;
extern float YAW_SPL_set;


#endif