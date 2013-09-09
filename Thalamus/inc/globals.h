#ifndef __GLOBALS_H__
#define __GLOBALS_H__


#define FIRMWARE_VERSION	1		   // Firmware version
#define MESSAGE_LOOP_HZ	 	10		  // Max frequency of messages in Hz (keep this number low, like around 15)
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





// Outputs/control
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



// Inputs

//  IMU sensors
#define GAV_LEN 8
#define AAV_LEN 30
#define MAV_LEN 30

typedef struct{
	volatile signed short raw;
	volatile float av;
	volatile float value;
	volatile float offset;
	volatile signed int total;
	float error;
	signed short history[GAV_LEN];
} sensorStructGyro;

typedef struct{
	sensorStructGyro X;
	sensorStructGyro Y;
	sensorStructGyro Z;
	unsigned int count;
} threeAxisSensorStructGyro;
extern threeAxisSensorStructGyro Gyro;

typedef struct{
	volatile signed short raw;
	volatile float av;
	volatile float value;
	volatile signed int total;
	signed short history[AAV_LEN];
} sensorStructAccel;

typedef struct{
	sensorStructAccel X;
	sensorStructAccel Y;
	sensorStructAccel Z;
	unsigned int count;
} threeAxisSensorStructAccel;
extern threeAxisSensorStructAccel Accel;


typedef struct{
	volatile signed short raw;
	volatile float av;
	volatile float value;
	volatile signed int total;
	signed short history[AAV_LEN];
} sensorStructMag;

typedef struct{
	sensorStructMag X;
	sensorStructMag Y;
	sensorStructMag Z;
	unsigned int count;
} threeAxisSensorStructMag;
extern threeAxisSensorStructMag Mag;


typedef struct {
	float pressure;
    float baro;
	float gps;
	float filtered;
	float ultra;
	float vel;
	float ult_conf;
	float ult;
	float barobias;
} altStruct;
extern altStruct alt;
extern float ultra;
extern unsigned int ultraLoss;
extern float batteryVoltage;


extern unsigned char gps_valid;



#define OFFSTICK			50
#define MIDSTICK			512		// Corresponds to input when stick is in middle (approx value).
#define MAXSTICK			850		// Corresponds to input when stick is at the top

#define MAXTHRESH		   750 //(MAXSTICK+MIDSTICK)/2 - 50
#define MINTHRESH		   250 //(MIDSTICK+OFFSTICK)/2 + 50

extern unsigned short rcInput[7];
extern unsigned int rxLoss;
extern unsigned int rxFirst;
extern signed short yawtrim;
extern signed short throttletrim;
extern float throttle;
extern float throttle_angle;
extern int hold_thro_off;
extern unsigned char auxState, flapState, rateState, throState, aileState, elevState, ruddState;
extern unsigned int flapswitch;
extern float pitchDemandSpin;
extern float rollDemandSpin;
extern float pitchDemandSpinold;
extern float rollDemandSpinold;
extern float flpswitch;
#endif