// GENERAL TODO list in order of priority
// Specific TODO's are listed next to the relevant bit of code.

//TODO: (HENRY DOES THIS, NEEDS LOTS OF SPACE AND SPARE PARTS!)
//Diagnose loss of control experienced at full throttle
//I ascended at full throttle while probably applying some pitch and roll demands and suddenly the craft just seemed to loose stability.
//It rolled/ pitched upside down a few times, luckily it recovered and I was able to bring it  back safely. 
//However, have heard that a backer had a similar problem. 
//Possible causes are Yuan's Max throttle stuff, my roll angle prioritisation (less likely as wasn't in the backers code), ROLL/ PITCH SPL (spin limit). Or other!
// Have done some test flights and witnessed the loss of control at high throttles, it only does it when applying a high throttle and a large attitude demand, it inverts then enters a continual spin at high throttle
// Have added code on line 51 of control.h to try and solve this problem
//Rescues craft if error gets too large at high throttles by detecting roll and pitch error getting too large and reducing the throttle.
	// TODO: Test to see if this code solves the problem
	// Tried this: if (((pitcherror > 0.08) || (pitcherror < -0.08) || (rollerror > 0.08) || (rollerror < -0.08)) && (throttle > 600)) throttle -= 200;
// Should really be able to solve with better PID tuning.

//TODO: check all integrators for decoupling, and make sure they are locked in this case

// TODO: Assess and Improve leveling on take off and leveling in flight
// Key areas to investigate are integral gain on takeoff
// Accelerometer feedback gain
// Accelerometer feedback method - additional filtering needed?
// Landing Gear separation seems to play a large part, it causes undesired integral wind-up before the craft is in the air
// Landing gear would need switches or be sprung and damped with longish travel

// TODO: Measure loop rates instead of just assuming it
// The control needs to know how fast they're going, right now we assume the loops are going at their specified rate
// however, it would be better to just time instead.  Use one of the hardware timers to get sub-ms resolution.

// TODO: Check for some discontinuities in flight
// Throttle and attitude blips when flying under GPS
// Throttle blips when flying normally

/////////////////////// Libraries to include /////////////
#include "thal.h"
#include "mavlink.h"
#include <math.h>

#include "eeprom.h"


// ****************************************************************************
// ****************************************************************************
// *** DECLARATIONS
// ****************************************************************************
// ****************************************************************************

////////////////////////////////////////// CONFIGURATION PARAMETERS ////////////////////////////////

#define FIRMWARE_VERSION	1		   // Firmware version
#define MESSAGE_LOOP_HZ	 	15		  // Max frequency of messages in Hz (keep this number low, like around 15)
#define RX_PANIC			2		   // Number of seconds after missing RX before craft considered "disconnected"

#define FAST_RATE		   400
#define SLOW_RATE		   75


#define ZEROTHROTMAX		1*FAST_RATE

#define SLOW_DIVIDER		FAST_RATE/SLOW_RATE

// TODO: check ESC response at THROTTLEOFFSET, consider raising THROTTLEOFFSET to 1000
#define THROTTLEOFFSET	900		// Corresponds to zero output PWM. Nominally 1000=1ms, but 800 works better
#define IDLETHROTTLE		175		// Minimum PWM output to ESC when in-flight to avoid motors turning off
#define MAXTHROTTLE	 	1200	// Maximum PWM output to ESC (PWM = THROTTLEOFFSET + MAXTHROTTLE)
#define MAXTHROTTLEPERCENT	0.9	 // Maximum percentage throttle should be (reserves some extra output for stabilisation at high throttle).

#define OFFSTICK			50
#define MIDSTICK			512		// Corresponds to input when stick is in middle (approx value).
#define MAXSTICK			850		// Corresponds to input when stick is at the top

#define MAXTHRESH		   750 //(MAXSTICK+MIDSTICK)/2 - 50
#define MINTHRESH		   250 //(MIDSTICK+OFFSTICK)/2 + 50
//  Running Average Lengths
#define GAV_LEN 8
#define AAV_LEN 30
#define MAV_LEN 30

//This is used with mode
#define STATE_DISARMED 1
#define STATE_MANUAL 2
#define STATE_MANUAL_GPS 3
#define STATE_AUTO 5

#define STILL_THRESH	8000 // this value is used to detect whether the craft is still or not for the purpose of calibration, it's the square of the euclidian distance of the maximum expected gyro noise in raw values


/////////////////////////////// FUNCTIONS //////////////////////////////////////
void sensor_zero(void);
void calibrate_gyr(void);
void calibrate_gyr_temporary(unsigned int seconds);
void calibrate_mag(void);
void arm(void);
void disarm(void);
void read_gyr_sensors(void);
void read_acc_sensors(void);
void read_mag_sensors(void);
void read_ultrasound(void);
void read_batt_voltage(void);
void read_rx_input(void);
void eeprom_load_all(void);
void eeprom_save_all(void);
void control_throttle(void);
void control_motors(void);
void filter_gps_baro(void);


///////////////////////////////////////////// ILINK ///////////////////////////////////////
ilink_identify_t ilink_identify;
ilink_thalstat_t ilink_thalstat;
ilink_thalctrl_t ilink_thalctrl_rx;
ilink_thalctrl_t ilink_thalctrl_tx;
ilink_imu_t ilink_rawimu;
ilink_imu_t ilink_scaledimu;
ilink_altitude_t ilink_altitude;
ilink_attitude_t ilink_attitude;
ilink_attitude_t ilink_attitude_demand;
ilink_thalparam_t ilink_thalparam_tx;
ilink_thalparam_t ilink_thalparam_rx;
ilink_thalpareq_t ilink_thalpareq;
ilink_iochan_t ilink_inputs0;
ilink_iochan_t ilink_outputs0;
ilink_gpsfly_t ilink_gpsfly;
ilink_gpsreq_t ilink_gpsreq;
ilink_debug_t ilink_debug;


///////////////////////////////////////// GLOBAL VARIABLE STRUCTURES /////////////////////

typedef struct {
	float roll;
	float pitch;
	float yaw;
	float throttle;
} userStruct;
userStruct user;

typedef struct {
	float roll;
	float pitch;
	float yaw;
} attitude_demand_body_struct;
attitude_demand_body_struct attitude_demand_body = {0};


typedef struct 
{
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
altStruct alt = {0};

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
threeAxisSensorStructGyro Gyro;

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
threeAxisSensorStructAccel Accel;


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
threeAxisSensorStructMag Mag;



/////////////////////////////////// GLOBAL VARIABLES /////////////////////////////////
// TODO: Why don't we just set all the variable here? Some of them are being set here, and some in the setup function.
// Timers and counters
unsigned int sysMS;
unsigned long long sysUS;
unsigned short RxWatchdog;
unsigned short UltraWatchdog;
unsigned short slowSoftscale;

// LEDs
unsigned char flashPLED, flashVLED, flashRLED;

// Quaternion and Rotation Matrix
float q1, q2, q3, q4;
float M1, M2, M3, M4, M5, M6, M7, M8, M9;
float thetaAngle, phiAngle, psiAngle, psiAngleinit;
float RM1, RM2, RM3, RM4, RM5, RM6, RM7, RM8, RM9;


// Inputs
unsigned short rcInput[7];
unsigned int rxLoss;
unsigned int rxFirst;
signed short yawtrim;
signed short throttletrim;
float throttle;
float throttle_angle;
int hold_thro_off;
unsigned char auxState, flapState, rateState, throState, aileState, elevState, ruddState;
unsigned int flapswitch = 0;
float pitchDemandSpin = 0;
float rollDemandSpin = 0;
float pitchDemandSpinold = 0;
float rollDemandSpinold = 0;
float flpswitch = 0;

// Ultrasound
float ultra;
unsigned int ultraLoss;

// Outputs
float pitchcorrectionav, rollcorrectionav, yawcorrectionav;
float motorN, motorE, motorS, motorW;
float motorNav, motorEav, motorSav, motorWav;
float tempN;
float tempE;
float tempS;
float tempW;

// Arm
unsigned int armed, calib, zeroThrotCounter;

// Battery
float batteryVoltage;

// Button
unsigned int PRGBlankTimer; // Blanking time for button pushes
unsigned int PRGTimer; // Timer for button pushes, continuously increments as the button is held
unsigned int PRGPushTime; // Contains the time that a button was pushed for, populated after button is released

unsigned char airborne = 0; //boolean

unsigned char gps_valid = 0;

unsigned char thal_throt_cont = 0;

unsigned char thal_motor_off = 0;

unsigned char got_setpoint = 0; //bool

//Altitude PID states
float GPS_KerrI = 0;
float ULT_KerrI = 0;
float targetZ_ult = 0;
float alt_tkoff = 0;
float oldUltra = 0;

float ROLL_SPL_set;
float PITCH_SPL_set;
float YAW_SPL_set;



/////////////////////////////////////////// TUNABLE PARAMETERS ////////////////////////////////////


#include "setup.h"
#include "filter.h"
#include "calibrate.h"
#include "states.h"
#include "control.h"
#include "inputs.h"
#include "communication.h"


//Main loop, nothing much happens in here.
void loop() {
	//if(idleCount < IDLE_MAX) idleCount++; // this is the counter for CPU idle time
	//Deal with button push for entering bind mode for RX
		if(PRGBlankTimer == 0) {
			if(PRGTimer > 3000) {
				RXBind();
				PRGPushTime = 0;
				PRGTimer = 0;
				PRGBlankTimer = 200;
			}
		}
	
	__WFI();
}

// SysTick timer: deals with general timing
void SysTickInterrupt(void) {
	sysMS += 1;
	sysUS += 1000;
	
	//Deal with flashing LEDs
		if(sysMS % 25 == 0) {
			if(sysMS % 100 == 0) {
				if(flashPLED) LEDToggle(PLED);
				if(flashVLED) LEDToggle(VLED);
				if(flashRLED) LEDToggle(RLED);
			}
			else {
				if(flashPLED == 2) LEDToggle(PLED);
				if(flashVLED == 2) LEDToggle(VLED);
				if(flashRLED == 2) LEDToggle(RLED);
			}
		}
	
	// Time the button pushes
		if(PRGPoll() == 0) PRGTimer++;
		else {
			PRGPushTime = PRGTimer;
			PRGTimer = 0;
			if(PRGBlankTimer) PRGBlankTimer--;
		}
}

// RIT interrupt, deal with timed iLink messages.
void RITInterrupt(void) {
	
	// Deal with iLink parameter transmission
		unsigned int i;
		if(paramSendCount < paramCount) {
			unsigned short thisParam = paramSendCount; // store this to avoid race hazard since paramSendCount can change outside this interrupt
			ilink_thalparam_tx.paramID = thisParam;
			ilink_thalparam_tx.paramValue = paramStorage[thisParam].value;
			ilink_thalparam_tx.paramCount = paramCount;
			for(i=0; i<16; i++) {
				ilink_thalparam_tx.paramName[i] = paramStorage[thisParam].name[i];
				if(paramStorage[thisParam].name[i] == '\0') break;
			}
			if(ILinkSendMessage(ID_ILINK_THALPARAM, (unsigned short *) & ilink_thalparam_tx, sizeof(ilink_thalparam_tx)/2 -1)) {
				if(paramSendSingle) {
					paramSendSingle = 0;
					paramSendCount = paramCount;
				}
				else {
					paramSendCount = thisParam+1;
				}
			}
		}
}



//Main functional periodic loop
void Timer0Interrupt0() {

	
	if(++slowSoftscale >= SLOW_DIVIDER) {
		slowSoftscale = 0;

		// These run at SLOW_RATE - The order they run in matters
        trig_batt_voltage();
		read_mag_sensors();
		read_rx_input();		
		read_ultrasound();
		read_barometer();
		read_batt_voltage();
		filter_gps_baro();
		gps_status();
		state_machine();
		// We have to reread everytime transmitted data is used
		read_rx_input();
			
	}

	// These run at FAST_RATE  - The order they run in matters
	read_acc_sensors();
	read_gyr_sensors();
	
	a_h_r_s();
	
	control_throttle();
	control_motors();


}
