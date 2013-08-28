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

//TODO: check all integrators for decoupling, and make sure they are locked in this case

// TODO: Assess and Improve leveling on take off and leveling in flight
// Key areas to investigate are integral gain on takeoff
// Accelerometer feedback gain
// Accelerometer feedback method - additional filtering needed?

// TODO: Reinsert Modes, build the State machines out to deal with changes in throttle functionality 
// Add barometer, GPS and Ultrasound merging.

// TODO: Write code for Orientation calibration 
// When disarmed, right stick all the way left should enter this mode.  
// The craft should be position flat and level for at least 3 seconds. (45 degree tolerance)
// The craft should be tilted in the forward direction greater than 45 degrees and for at least 3 seconds. (snap to nearest 45 degree angle)

// TODO: Measure loop rates instead of just assuming it
// The control needs to know how fast they're going, right now we assume the loops are going at their specified rate
// however, it would be better to just time instead.  Use one of the hardware timers to get sub-ms resolution.

// TODO: Add GPS_Confidence Code, use ilink_gpsfly.isNew to check for new data, use a timer, add autoland code.





/////////////////////// Libraries to include /////////////
#include "thal.h"
#include "mavlink.h"
#include <math.h>


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

#define EEPROM_MAX_PARAMS   100 // this should be greater than or equal to the above number of parameters
#define EEPROM_OFFSET   0 // EEPROM Offset used for moving the EEPROM values around storage (wear levelling I guess)
#define EEPROM_VERSION	22 // version of variables in EEPROM, change this value to invalidate EEPROM contents and restore defaults

//  Running Average Lengths
#define GAV_LEN 8
#define AAV_LEN 30
#define MAV_LEN 30

//This is used with mode
#define STATE_DISARMED 1
#define STATE_MANUAL 2
#define STATE_MANUAL_GPS 3
#define STATE_AUTO 5


/////////////////////////////// FUNCTIONS //////////////////////////////////////
void sensor_zero(void);
void calibrate_gyr(void);
void calibrate_gyr_temperature(unsigned int seconds);
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
	float baro;
	float gps;
	float filtered;
	float ultra;
	float vel;
	float ult_conf;
	float ult;
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

typedef struct paramStorage_struct {
	char name[16];
	float value;
} paramStorage_t;


/////////////////////////////////// GLOBAL VARIABLES /////////////////////////////////
// TODO: Why don't we just set all the variable here? Some of them are being set here, and some in the setup function.
// Timers and counters
unsigned int sysMS;
unsigned long long sysUS;
unsigned short RxWatchdog;
unsigned short UltraWatchdog;
unsigned short slowSoftscale;

unsigned int paramSendCount;
unsigned int paramCount;
unsigned char paramSendSingle;

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
unsigned int auxState, flapState;
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

struct paramStorage_struct paramStorage[] = {
	{"DRIFT_AKp",		   0.2f},
	{"DRIFT_MKp",	  		0.2f},   
	#define DRIFT_AccelKp   paramStorage[0].value	
	#define DRIFT_MagKp	 paramStorage[1].value  
  
	{"LPF_ULTRA",	   0.95f},  
	#define LPF_ULTRA 		paramStorage[2].value

	{"YAW_SEN",	 0.0001f},	 
	{"PITCH_SEN",	0.0022f},	
	{"ROLL_SEN",	 0.0022f},  
	{"YAW_DZN",	  0.001f},	
	#define YAW_SENS		paramStorage[3].value
	#define PITCH_SENS	  paramStorage[4].value
	#define ROLL_SENS	   paramStorage[5].value
	#define YAW_DEADZONE	paramStorage[6].value 
	
	{"PITCH_Kp",	  400.0f},	 
	{"PITCH_Ki",		2.0f},	
	{"PITCH_Kd",	  100.0f},	 
	{"PITCH_Kdd",	1500.0f},
	{"PITCH_Bst",	 0.0f},	  
	{"PITCH_De",	  0.999f},	
	#define PITCH_Kp		paramStorage[7].value 
	#define PITCH_Ki		paramStorage[8].value
	#define PITCH_Kd		paramStorage[9].value 
	#define PITCH_Kdd	   paramStorage[10].value 
	#define PITCH_Boost	 paramStorage[11].value 
	#define PITCH_De		paramStorage[12].value 

	{"ROLL_Kp",	   400.0f},	   
	{"ROLL_Ki",		 2.0f},	  
	{"ROLL_Kd",	   100.0f},		
	{"ROLL_Kdd",	 1500.0f},	 		
	{"ROLL_Bst",	   0.00f},					
	{"ROLL_De",	   0.999f},
	#define ROLL_Kp		 paramStorage[13].value  
	#define ROLL_Ki		 paramStorage[14].value 
	#define ROLL_Kd		 paramStorage[15].value 
	#define ROLL_Kdd		paramStorage[16].value 	
	#define ROLL_Boost	  paramStorage[17].value 
	#define ROLL_De		 paramStorage[18].value

	{"YAW_Kp",		1000.0f},	 
	{"YAW_Kd",		250.0f},  
	{"YAW_Bst",		0.00f},   
	#define YAW_Kp		  paramStorage[19].value 
	#define YAW_Kd		  paramStorage[20].value 
	#define YAW_Boost	   paramStorage[21].value 	

	// Mode
	{"state",	   STATE_DISARMED},  //
	#define state 		paramStorage[22].value

	//Limits
	{"LIM_ANGLE",	   0.35f},  // Roll and Pitch Angle Limit in Radians
	{"LIM_ALT",		 1000.0f},  // Altitude Limit in mm when in Ultrasound Mode
	#define LIM_ANGLE 		paramStorage[23].value
	#define LIM_ALT 		paramStorage[24].value

	// Magneto Correction
	{"CAL_MAGN1",	 0.001756f},	  
	{"CAL_MAGN2",   0.00008370f},		
	{"CAL_MAGN3",   0.00005155f},	 	
	{"CAL_MAGN5",	 0.001964f},			
	{"CAL_MAGN6",   0.00002218f},	 
	{"CAL_MAGN9",	 0.001768f},	 
	{"CAL_MAGM1",	   0.0f},	 
	{"CAL_MAGM2",		0.0f},	 	
	{"CAL_MAGM3",		0.0f},  
	#define MAGCOR_N1	   paramStorage[25].value 
	#define MAGCOR_N2	   paramStorage[26].value 		
	#define MAGCOR_N3	   paramStorage[27].value 		
	#define MAGCOR_N5	   paramStorage[28].value 
	#define MAGCOR_N6	   paramStorage[29].value 
	#define MAGCOR_N9	   paramStorage[30].value 
	#define MAGCOR_M1	   paramStorage[31].value 		
	#define MAGCOR_M2	   paramStorage[32].value 
	#define MAGCOR_M3	   paramStorage[33].value

	// Ultrasound
	{"ULTRA_Kp",		0.05f},
	{"ULTRA_Kd",		0.1f},
	{"ULTRA_Ki",		0.00001f},
	{"ULTRA_De",	  	0.9999f},
	{"ULTRA_TKOFF",   	150.0f}, 
	{"ULTRA_LND",   	100.0f}, 
	#define ULTRA_Kp		paramStorage[34].value 
	#define ULTRA_Kd		paramStorage[35].value 
	#define ULTRA_Ki		paramStorage[36].value 
	#define ULTRA_De		paramStorage[37].value
	#define ULTRA_TKOFF	 paramStorage[38].value 
	#define ULTRA_LND	   paramStorage[39].value

	// TODO: I don't think these should be tunable parameters should they? Remember that the gyros are calibrated on every Arm
	{"CAL_GYROX",   0.0f},
	{"CAL_GYROY",   0.0f},
	{"CAL_GYROZ",   0.0f},
	#define CAL_GYROX	   paramStorage[40].value 
	#define CAL_GYROY	   paramStorage[41].value 
	#define CAL_GYROZ	   paramStorage[42].value 
	
	{"DETUNE",			0.2f},
	#define DETUNE		paramStorage[43].value
	
	{"LIM_RATE",			100.0f},
	#define LIM_RATE		paramStorage[44].value   
	{"LIM_ULTRA",			4.0f},
	#define LIM_ULTRA		paramStorage[45].value
	
	{"ULTRA_DRMP",	 3.0f}, 
	{"ULTRA_DTCT",	 6.0f},
	#define ULTRA_DRMP	  paramStorage[46].value
	#define ULTRA_DTCT	  paramStorage[47].value
	
	{"LIM_THROT", 		0.3f},
	#define LIM_THROT		paramStorage[48].value
	
	{"ULTRA_OVDEC",		0.01f},
	#define ULTRA_OVDEC		paramStorage[49].value
	
	{"ULTRA_DEAD",		100},
	#define ULTRA_DEAD		paramStorage[50].value
	
	{"ULTRA_OVTH",		40},
	#define ULTRA_OVTH		paramStorage[51].value
  
	
	{"CAL_AUTO", 1.0f},
	#define CAL_AUTO		paramStorage[52].value	
	
	
	{"LPF_OUT",	   0.6f},  
	#define LPF_OUT 		paramStorage[53].value
	
	
	{"BAT_LOW",		 11000.0f},
	{"BAT_CRIT",		10000.0f},
	#define BATT_LOW		paramStorage[54].value
	#define BATT_CRIT	   paramStorage[55].value
	
	{"ULTRA_OFFSET",		 350},
	#define ULTRA_OFFSET		paramStorage[56].value
	
	{"ROLL_SPL",		 0.04},
	#define ROLL_SPL		paramStorage[57].value
	{"PITCH_SPL",		 0.04},
	#define PITCH_SPL		paramStorage[58].value
	
	// TODO: Tune Yaw integral
	{"YAW_Ki",		 0.0},
	#define YAW_Ki		paramStorage[59].value
	{"YAW_De",		 1.0},
	#define YAW_De		paramStorage[60].value

	{"Filt_GPS_K",		 1.0},
	#define Filt_GPS_K		paramStorage[61].value

	{"LPF_BARO",   1.0},
	 #define LPF_BARO  paramStorage[62].value

	{"GPS_ALTKp", 10.0f},
    {"GPS_ALTKi", 0.0001f},
    {"GPS_ALTDe", 1.0f},
    {"GPS_ALTKd", 20.0f},
    #define GPS_ALTKp paramStorage[63].value
    #define GPS_ALTKi paramStorage[64].value
    #define GPS_ALTDe paramStorage[65].value
    #define GPS_ALTKd paramStorage[66].value

    {"Filt_baroK",		 0.0},
	#define Filt_baroK		paramStorage[67].value
	
	{"YAW_SPL",		 0.04},
	#define YAW_SPL		paramStorage[68].value
    
	{"ORI",		 0.00},
	#define ORI		paramStorage[69].value

	};



#include "setup.h"
#include "filter.h"
#include "calibrate.h"
#include "states.h"
#include "control.h"
#include "inputs.h"
#include "eeprom.h"
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

		// These run at SLOW_RATE
		read_mag_sensors();
		read_rx_input();		
		read_ultrasound();
		read_barometer();
		read_batt_voltage();
		filter_gps_baro();
		gps_status();
		state_machine();
			
	}

	// These run at FAST_RATE
	read_acc_sensors();
	read_gyr_sensors();
	
	a_h_r_s();
	
	
	control_throttle();
	control_motors();


}
