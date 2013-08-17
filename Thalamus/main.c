#include "thal.h"
#include "mavlink.h"
#include <math.h>

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

// TODO: Assess and Improve leveling on take off and leveling in flight
// Key areas to investigate are integral gain on takeoff
// Accelerometer feedback gain
// Accelerometer feedback method - additional filtering needed?

// Switch to using vectors rather than angles for motor control. 
// Angles will only be used for telemetry readout, could be calculated on Hypo.

// TODO: Reinsert Modes, build the State machines out to deal with changes in throttle functionality 
// Add barometer, GPS and Ultrasound merging.

// TODO: Write code for Orientation calibration 
// When disarmed, right stick all the way left should enter this mode.  
// The craft should be position flat and level for at least 3 seconds. (45 degree tolerance)
// The craft should be tilted in the forward direction greater than 45 degrees and for at least 3 seconds. (snap to nearest 45 degree angle)


// ****************************************************************************
// ****************************************************************************
// *** DECLARATIONS
// ****************************************************************************
// ****************************************************************************
// TODO: Needs sorting, there are unused parameters and a lack of categorisation throughout


////////////////////////////////////////// CONFIGURATION PARAMETERS ////////////////////////////////

#define FIRMWARE_VERSION	1		   // Firmware version
#define MESSAGE_LOOP_HZ	 15		  // Max frequency of messages in Hz (keep this number low, like around 15)
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

#define MAXTHRESH		   (MAXSTICK+MIDSTICK)/2 - 50
#define MINTHRESH		   (MIDSTICK+OFFSTICK)/2 + 50

#define EEPROM_MAX_PARAMS   100 // this should be greater than or equal to the above number of parameters
#define EEPROM_OFFSET   0 // EEPROM Offset used for moving the EEPROM values around storage (wear levelling I guess)
#define EEPROM_VERSION	32 // version of variables in EEPROM, change this value to invalidate EEPROM contents and restore defaults

//  Running Average Lengths
#define GAV_LEN 8
#define AAV_LEN 30 // With FIFO on, accelerometers effectively run at three times the rate
#define MAV_LEN 30


/////////////////////////////// FUNCTIONS //////////////////////////////////////
void SensorZero(void);
void CalibrateGyro(void);
void CalibrateGyroTemp(unsigned int seconds);
void CalibrateMagneto(void);
void Arm(void);
void Disarm(void);
void ReadGyroSensors(void);
void ReadAccelSensors(void);
void ReadMagSensors(void);
void ReadUltrasound(void);
void ReadBattVoltage(void);
void ReadRXInput(void);
void LinkInit(void);
void EEPROMLoadAll(void);
void EEPROMSaveAll(void);
void control_throttle(void);
void filter_GPS_baro();


///////////////////////////////////////////// ILINK ///////////////////////////////////////
ilink_identify_t ilink_identify;
ilink_thalstat_t ilink_thalstat;
ilink_thalctrl_t ilink_thalctrl;
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
ilink_position_t ilink_position;
ilink_payldctrl_t ilink_payldctrl;


///////////////////////////////////////// GLOBAL VARIABLE STRUCTURES /////////////////////
typedef struct{
	float demand;
	float demandOld;
	float valueOld;
	float derivative;
	float integral;
} directionStruct;

directionStruct pitch;
directionStruct roll;
directionStruct yaw;

typedef struct 
{
	float baro;
	float gps;
	float filtered;
	float GPS_baro_loopgain;
	float ultra;
} altStruct;
altStruct alt;

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
int throttleHoldOff;
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




/////////////////////////////////////////// TUNABLE PARAMETERS ////////////////////////////////////

struct paramStorage_struct paramStorage[] = {
	{"DRIFT_AKp",		   0.4f},
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
	{"MODE_ST",	   1.0f},  //0 is Acrobatic,  1 is Regular, 2 is Simplicity, 3 is Easy, 4 is GPS Hold, 5 is Auto 1, 6 is Auto 2
	#define MODE_ST 		paramStorage[22].value

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
	{"ULTRA_Kd",		5.0f},
	{"ULTRA_Ki",		0.00001f},
	{"ULTRA_De",	  	0.9999f},
	{"ULTRA_TKOFF",   	200.0f}, 
	{"ULTRA_LND",   	150.0f}, 
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
	};



// System functionality crudly split into files
// TODO: make it more standard

#include "setup.h"
#include "AHRS.h"
#include "userinput.h"
#include "control.h"
#include "sensors.h"
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
void Timer0Interrupt0() { // Runs at about 400Hz

	// We collect some data at a slower rate
	if(++slowSoftscale >= SLOW_DIVIDER) {
		slowSoftscale = 0;

		ReadMagSensors();
		ReadRXInput();
		read_sticks();
		ReadUltrasound();
		ReadBattVoltage();
			
	}

	ReadAccelSensors();
	ReadGyroSensors();
	AHRS();

	control_throttle();
	control_attitude();
	control_motors();

}


void Arm(void) {

	if(CAL_AUTO > 0) {
		CalibrateGyroTemp(1);
	}
	
	PWMInit(PWM_NESW);
	PWMSetNESW(THROTTLEOFFSET, THROTTLEOFFSET, THROTTLEOFFSET, THROTTLEOFFSET);
	//TODO: inline Delays cause system hang.
	if(armed == 0) {
		Delay(500);
		PWMSetNESW(THROTTLEOFFSET + IDLETHROTTLE, THROTTLEOFFSET + IDLETHROTTLE, THROTTLEOFFSET + IDLETHROTTLE, THROTTLEOFFSET + IDLETHROTTLE);
		Delay(100);
		PWMSetNESW(THROTTLEOFFSET, THROTTLEOFFSET, THROTTLEOFFSET, THROTTLEOFFSET);
		Delay(300);
		PWMSetNESW(THROTTLEOFFSET + IDLETHROTTLE, THROTTLEOFFSET + IDLETHROTTLE, THROTTLEOFFSET + IDLETHROTTLE, THROTTLEOFFSET + IDLETHROTTLE);
		Delay(100);
		PWMSetNESW(THROTTLEOFFSET, THROTTLEOFFSET, THROTTLEOFFSET, THROTTLEOFFSET);
	}
	
	psiAngleinit = psiAngle; 
	yawtrim = rcInput[RX_RUDD];
	
	armed = 1;
	
	ilink_thalstat.sensorStatus &= ~(0x7); // mask status
	ilink_thalstat.sensorStatus |= 4; // active/armed
}



void Disarm(void) {
	if(armed) {
		PWMSetNESW(THROTTLEOFFSET, THROTTLEOFFSET, THROTTLEOFFSET, THROTTLEOFFSET);
		//TODO: inline Delays cause system hang.
		Delay(100);
		
		PWMSetN(THROTTLEOFFSET + IDLETHROTTLE);
		Delay(100);
		PWMSetN(THROTTLEOFFSET);
		Delay(33);
		PWMSetE(THROTTLEOFFSET + IDLETHROTTLE);
		Delay(100);
		PWMSetE(THROTTLEOFFSET);
		Delay(33);
		PWMSetS(THROTTLEOFFSET + IDLETHROTTLE);
		Delay(100);
		PWMSetS(THROTTLEOFFSET);
		Delay(33);
		PWMSetW(THROTTLEOFFSET + IDLETHROTTLE);
		Delay(100);
		PWMSetW(THROTTLEOFFSET);

		Delay(100);
	}
	PWMSetNESW(0, 0, 0, 0);
	armed = 0;
	ilink_thalstat.sensorStatus &= ~(0x7); // mask status
	ilink_thalstat.sensorStatus |= 3; // standby
}


