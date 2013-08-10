#include "thal.h"
#include "mavlink.h"
#include <math.h>


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
#define EEPROM_VERSION	33 // version of variables in EEPROM, change this value to invalidate EEPROM contents and restore defaults

//  Running Average Lengths
#define GAV_LEN 8
#define AAV_LEN 30
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

typedef struct {
	float ultra;
	float value;
	float valueOld;
	float error;
	float demand;
	float derivative;
	float integral;
	float demandold;
} altStruct;
altStruct alt;

typedef struct{
	volatile signed short raw;
	volatile float av;
	volatile float value;
	volatile float offset;
	volatile signed int total;
	float bias;
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
	{"DRIFT_AKp",		   0.2f},
	{"DRIFT_MKp",	  		0.2f},   
	#define DRIFT_AccelKp   paramStorage[0].value	
	#define DRIFT_MagKp	 paramStorage[1].value  
  
	{"SPR_ULTRA",	   0.95f},  
	#define SPR_ULTRA 		paramStorage[2].value

	{"YAW_SEN",	 0.00002f},	 
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
	
	
	{"SPR_OUT",	   0.6f},  
	#define SPR_OUT 		paramStorage[53].value
	
	
	{"BAT_LOW",		 11000.0f},
	{"BAT_CRIT",		10000.0f},
	#define BATT_LOW		paramStorage[54].value
	#define BATT_CRIT	   paramStorage[55].value
	
	{"ULTRA_OFFSET",		 350},
	#define ULTRA_OFFSET		paramStorage[56].value
	
	{"ROLL_SPL",		 0.002},
	#define ROLL_SPL		paramStorage[57].value
	{"PITCH_SPL",		 0.002},
	#define PITCH_SPL		paramStorage[58].value
	
	// TODO: Tune Yaw integral
	{"YAW_Ki",		 0.0},
	#define YAW_Ki		paramStorage[59].value
	{"YAW_De",		 1.0},
	#define YAW_De		paramStorage[60].value
	};


// ****************************************************************************
// ****************************************************************************
// *** INITIALISATION
// ****************************************************************************
// ****************************************************************************

// Initialiser function: sets everything up.
void setup() {	
	
	throttle_angle = 0;
	
	// *** LED setup
		LEDInit(PLED | VLED);
		LEDOff(PLED | VLED);
		
		flashPLED = 0;
		flashVLED = 0;
		flashRLED = 0;
	
		armed = 0;
		calib = 0;
		zeroThrotCounter = 0;
	
	// *** Timers and couters6
		rxLoss = 50;
		ultraLoss = ULTRA_OVTH + 1;
		sysMS = 0;
		sysUS = 0;
		SysTickInit();  // SysTick enable (default 1ms)
		

	// *** Parameters
		paramCount = sizeof(paramStorage)/20;
		EEPROMLoadAll();
		paramSendCount = paramCount;
		paramSendSingle = 0;
	
	// *** Establish ILink
		ilink_thalstat.sensorStatus = 1; // set ilink status to boot
		ilink_thalstat.flightMode = (0x1 << 0); // attitude control
		ilink_identify.deviceID = WHO_AM_I;
		ilink_identify.firmVersion = FIRMWARE_VERSION;
		ILinkInit(SLAVE);
	
	// *** Initialise input
		throttle = 0;
		rxFirst = 0;
		auxState = 0;
		RXInit();
		
		
	// *** Initialise Ultrasound
		UltraInit();


		
	// *** Battery sensor
		ADCInit(CHN7);
		ADCTrigger(CHN7);
		Delay(1);
		ilink_thalstat.battVoltage = (ADCGet() * 6325) >> 10; // Because the factor is 6325/1024, we can do this in integer maths by right-shifting 10 bits instead of dividing by 1024.
		batteryVoltage = 0;
		
	// *** Calibrate Sensors
		Delay(500);
		SensorInit();
		SensorZero();
		

	// *** Ultrasound
		UltraInit();
		UltraFast();
	
	// *** quaternion AHRS init
		q1 = 1;
		q2 = 0;
		q3 = 0;
		q4 = 0;
		
	// *** Rotation matrix Initialisation
		M1 = 1;
		M2 = 0;
		M3 = 0;
		M4 = 0;
		M5 = 1;
		M6 = 0;
		M7 = 0;
		M8 = 0;
		M9 = 1;
		
	// *** Rotation matrix Initialisation
		RM1 = 1;
		RM2 = 0;
		RM3 = 0;
		RM4 = 0;
		RM5 = 1;
		RM6 = 0;
		RM7 = 0;
		RM8 = 0;
		RM9 = 1;
		
		
	// *** Timer for AHRS
		// Set high confidence in accelerometer/magneto to rotate AHRS to initial heading
		float tempAccelKp = DRIFT_AccelKp;
		float tempMagKp = DRIFT_MagKp;
		DRIFT_MagKp *= 100;
		DRIFT_AccelKp *= 100;
		
		Timer0Init(59);
		Timer0Match0(1200000/FAST_RATE, INTERRUPT | RESET);
		Delay(1000);
	   
	
		DRIFT_MagKp = tempMagKp;
		DRIFT_AccelKp = tempAccelKp;
	
		slowSoftscale = 0;
	
	// *** Initialise PWM outputs
		motorN = 0;
		motorE = 0;
		motorS = 0;
		motorW = 0;
		
		motorNav = 0;
		motorEav = 0;
		motorSav = 0;
		motorWav = 0;
		
		Gyro.Y.bias = 0;
		Gyro.X.bias = 0;
		Gyro.Z.bias = 0;
		
		throttleHoldOff = 1;
		
	// *** Initialise timers and loops
		//PWMInit(PWM_NESW);
		PWMInit(PWM_X | PWM_Y);
		RITInitms(1000/MESSAGE_LOOP_HZ);
		flashPLED = 0;
		LEDOff(PLED);
		
}





// ****************************************************************************
// ****************************************************************************
// LOOPS
// ****************************************************************************
// ****************************************************************************

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



void Timer0Interrupt0() { // Runs at about 400Hz

// ****************************************************************************
// *** COLLECT INPUT DATA
// ****************************************************************************

	// We collect some data at a slower rate
	if(++slowSoftscale >= SLOW_DIVIDER) {
		slowSoftscale = 0;

		ReadMagSensors();
		ReadRXInput();
		ReadUltrasound();
		ReadBattVoltage();
			
	}

	ReadAccelSensors();
	ReadGyroSensors();
	
	
// ****************************************************************************
// *** ATTITUDE HEADING REFERENCE SYSTEM
// ****************************************************************************
	
	// CREATE THE MEASURED ROTATION MATRIX //
	
	//Both mag and acc are normalized in their Read functions
	
	// Compose the Measured Rotation Matrix from Accelerometer and Magnetometer Vectors
	//  Global Z axis in Local Frame/ RM Third Row - Accelerometer (already normalised)
	
	RM7 = Accel.X.value;
	RM8 = Accel.Y.value;
	RM9 = Accel.Z.value;
	
	// Global Y axis in local frame/ RM Second Row - Acclerometer cross Magnetometer
	RM4 = Accel.Y.value*(Mag.Z.value) - (Accel.Z.value)*Mag.Y.value;
	RM5 = (Accel.Z.value)*Mag.X.value - Accel.X.value*(Mag.Z.value);
	RM6 = Accel.X.value*Mag.Y.value - Accel.Y.value*Mag.X.value;
	// Normalise
	float temp = sqrt(RM4*RM4 + RM5*RM5 + RM6*RM6);
	RM4 = RM4/temp;
	RM5 = RM5/temp;
	RM6 = RM6/temp;
	
	//  Global X axis in Local Frame/ RM First Row - Y axis cross Z axis
	RM1 = RM5*RM9 - RM6*RM8;
	RM2 = RM6*RM7 - RM4*RM9;
	RM3 = RM4*RM8 - RM5*RM7;
	// Normalise
	temp = sqrt(RM1*RM1 + RM2*RM2 + RM3*RM3);
	RM1 = RM1/temp;
	RM2= RM2/temp;
	RM3 = RM3/temp;
	

	// CREATE THE ESTIMATED ROTATION MATRIX //
	// The measured quaternion updates the estimated quaternion via the Gyro.*.bias terms applied to the gyros
	float g1 = (Gyro.X.value - Gyro.X.bias*DRIFT_AccelKp)/(float)FAST_RATE;
	float g2 = (Gyro.Y.value - Gyro.Y.bias*DRIFT_AccelKp)/(float)FAST_RATE;
	float g3 = (Gyro.Z.value - Gyro.Z.bias*DRIFT_MagKp)/(float)FAST_RATE;
	
	// Increment the Estimated Rotation Matrix by the Gyro Rate
	//First row is M1, M2, M3 - World X axis in local frame
	M1 = M1 + g3*M2 - g2*M3;
	M2 = M2 - g3*M1 + g1*M3;
	M3 = M3 + g2*M1 - g1*M2;
	
	//Second row is M4, M5, M6 - World Y axis in local frame
	M4 = M4 + g3*M5 - g2*M6;
	M5 = M5 - g3*M4 + g1*M6;
	M6 = M6 + g2*M4 - g1*M5;
	
	// We use the dot product and both X and Y axes to adjust any lack of orthogonality between the two.
	float MX_dot_MY = M1*M4 + M2*M5 + M3*M6;
	M1 = M1 - M4*(MX_dot_MY/2);
	M2 = M2 - M5*(MX_dot_MY/2);
	M3 = M3 - M6*(MX_dot_MY/2);
	
	sumsqu = finvSqrt(M1*M1 + M2*M2 + M3*M3);
	M1 = M1*sumsqu;
	M2 = M2*sumsqu;
	M3 = M3*sumsqu;
	
	M4 = M4 - M1*(MX_dot_MY/2);
	M5 = M5 - M2*(MX_dot_MY/2);
	M6 = M6 - M3*(MX_dot_MY/2);
	
	sumsqu = finvSqrt(M4*M4 + M5*M5 + M6*M6);
	M4 = M4*sumsqu;
	M5 = M5*sumsqu;
	M6 = M6*sumsqu;
	
	//We find the Z axis by calculating the cross product of X and Y
	M7 = M2*M6 - M3*M5;
	M8 = M3*M4 - M1*M6;
	M9 = M1*M5 - M2*M4;
	
	sumsqu = finvSqrt(M7*M7 + M8*M8 + M9*M9);
	M7 = M7*sumsqu;
	M8 = M8*sumsqu;
	M9 = M9*sumsqu;
	
	// CALCULATE GYRO BIAS //
	// The gyro biases adjust the estimated matrix towards the measured rotation matrix.
	// Use x and y components of the Cross Product between the z vector from each Rotation Matrix for Gyro Biases
	Gyro.X.bias = RM9*M8 - RM8*M9;
	Gyro.Y.bias = RM7*M9 - RM9*M7;
	
	// Use z component of the cross product between the x vector from each rotation matrix to create the Z gyro bias
	Gyro.Z.bias = RM2*M1 - RM1*M2;


	// CALCULATE THE ESTIMATED QUATERNION //
	float trace = M1 + M5 + M9;
	if( trace > 0 ) {
		float s = 0.5f / sqrt(trace + 1.0f);
		q1 = 0.25f / s;
		q2 = ( M6 - M8 ) * s;
		q3 = ( M7 - M3 ) * s;
		q4 = ( M2 - M4 ) * s;
	} 
	else {
		if ( M1 > M5 && M1 > M9 ) {
			float s = 2.0f * sqrt( 1.0f + M1 - M5 - M9);
			q1 = (M6 - M8 ) / s;
			q2 = 0.25f * s;
			q3 = (M4 + M2 ) / s;
			q4 = (M7 + M3 ) / s;
		} 
		else if (M5 > M9) {
			float s = 2.0f * sqrt( 1.0f + M5 - M1 - M9);
			q1 = (M7 - M3 ) / s;
			q2 = (M4 + M2 ) / s;
			q3 = 0.25f * s;
			q4 = (M8 + M6 ) / s;
		} 
		else {
			 float s = 2.0f * sqrt( 1.0f + M9 - M1 - M5 );
			q1 = (M2 - M4 ) / s;
			q2 = (M7 + M3 ) / s;
			q3 = (M8 + M6 ) / s;
			q4 = 0.25f * s;
		}
	}	
	q1 = q1;
	q2 = -q2;
	q3 = -q3;
	q4 = -q4;

	// renormalise using fast inverse sq12re root
	sumsqu = finvSqrt(q1*q1 + q2*q2 + q3*q3 + q4*q4);
	q1 *= sumsqu;
	q2 *= sumsqu;
	q3 *= sumsqu;
	q4 *= sumsqu;
		
	
	// CALCULATE THE EULER ANGLES //
	// precalculated values for optimisation
	float q12 = q1 * q2;
	float q13 = q1 * q3;
	float q22 = q2 * q2;
	float q23 = q2 * q4;
	float q33 = q3 * q3;
	float q34 = q3 * q4;
	float q44 = q4 * q4;
	float q22Pq33 = q22 + q33;
	float Tq13Mq23 = 2 * (q13 - q23);
	float Tq34Pq12 = 2 * (q34 + q12);
	// avoid gimbal lock at singularity points
	if (Tq13Mq23 == 1) {
		psiAngle = 2 * fatan2(q2, q1);
		thetaAngle = M_PI_2;
		phiAngle = 0;
	}
	else if (Tq13Mq23 == -1) {
		psiAngle = -2 * fatan2(q2, q1);
		thetaAngle = - M_PI_2;
		phiAngle = 0;
	}
	else {
		thetaAngle = fasin(Tq13Mq23);	
		phiAngle = fatan2(Tq34Pq12, (1 - 2*q22Pq33));  
		psiAngle = fatan2((2*(q1 * q4 + q2 * q3)), (1 - 2*(q33 + q44)));  
	}
	// Output angles over telemetry
	ilink_attitude.roll = phiAngle;
	ilink_attitude.pitch = thetaAngle;
	ilink_attitude.yaw = psiAngle;
	
	
	
	
// ****************************************************************************
// *** STATE MACHINE
// ****************************************************************************
		
	///////////////////////// REGULAR MODE ///////////////////////////////////
	if (MODE_ST == 1) { 
	
	
		// Arm, Disarm and Calibrate
		if(rcInput[RX_THRO] - throttletrim <  OFFSTICK && rxFirst != 0) {
			if(rxLoss < 50) {
				if(rcInput[RX_AILE] < MAXTHRESH && rcInput[RX_AILE] > MINTHRESH) {
					if(rcInput[RX_ELEV] > MAXTHRESH) {
						// Arm
						if(zeroThrotCounter++ > ZEROTHROTMAX) {
							zeroThrotCounter = 0;

							Arm();
						}
					}
					else if(rcInput[RX_ELEV] < MINTHRESH) {
						// Disarm
						if(zeroThrotCounter++ > ZEROTHROTMAX) {
							zeroThrotCounter = 0;
							Disarm();
						}
					}
					else {
						zeroThrotCounter = 0;
					}
				}
				else if(rcInput[RX_ELEV] < MAXTHRESH && rcInput[RX_ELEV] > MINTHRESH) {
					if(rcInput[RX_AILE] > MAXTHRESH) {
						// Request gyro calibration
						// TODO: This needs to be swapped for orientation calibration
						if(zeroThrotCounter++ > ZEROTHROTMAX) {
							zeroThrotCounter = 0;
							CalibrateGyro();
						}
					}
					else if(rcInput[RX_AILE] < MINTHRESH) {
						// Request magneto calibration
						if(zeroThrotCounter++ > ZEROTHROTMAX) {
							zeroThrotCounter = 0;
							CalibrateMagneto();
						}
					}
				}
				
			}
			else {
				zeroThrotCounter = 0;
			}
  
		}
		
		// General Flight Demands
		else {	
			// In manual mode, set pitch and roll demands based on the user commands collected from the rx unit
			pitch.demand = -((float)MIDSTICK - (float)rcInput[RX_ELEV])*PITCH_SENS; 
			roll.demand = ((float)MIDSTICK - (float)rcInput[RX_AILE])*ROLL_SENS;
			float tempf = -(float)(yawtrim - rcInput[RX_RUDD])*YAW_SENS; 						
			throttle = rcInput[RX_THRO] - throttletrim;
			if (throttle < 0) throttle = 0;
			
			// Pitch and roll demands can be remapped to different variables if the orientation of the 
			// board in the craft is non standard or if the pitch and roll demands need to vary with position
			pitchDemandSpin = pitch.demand;
			rollDemandSpin = roll.demand;
			
			// A yaw rate is demanded by the rudder input, not an absolute angle.
			// This code increments the demanded angle at a rate proportional to the rudder input
			if(fabsf(tempf) > YAW_DEADZONE) {
				yaw.demand += tempf;
				if(yaw.demand > M_PI) {
					yaw.demand -= M_TWOPI;
					yaw.demandOld -= M_TWOPI;
				}
				else if(yaw.demand < -M_PI) {
					yaw.demand += M_TWOPI;
					yaw.demandOld += M_TWOPI;
				}
			} 
		}
		
	
	}
	

	
 
	
// ****************************************************************************
// *** Attitude PID Control
// ****************************************************************************	

		float pitcherror, rollerror, yawerror;
		float pitchcorrection, rollcorrection, yawcorrection;

		// This section of code applies some throttle increase with high tilt angles
		//It doesn't seem hugely effective and maybe completely redundant when Barometer control is implemented
		// TODO: Reassess whether it is useful or not
		float M9temp;
		if (M9 > 0) M9temp = M9;
		else M9temp = -M9;
		throttle_angle = ((throttle / M9temp) - throttle); 
		if (throttle_angle < 0) throttle_angle = 0;
		
		// This section of code limits the rate at which the craft is allowed to track angle demand changes
		if ((pitchDemandSpin - pitchDemandSpinold) > PITCH_SPL) pitchDemandSpin = pitchDemandSpinold + PITCH_SPL;
		if ((pitchDemandSpin - pitchDemandSpinold) < -PITCH_SPL) pitchDemandSpin = pitchDemandSpinold - PITCH_SPL;
		pitchDemandSpinold = pitchDemandSpin;		
		if ((rollDemandSpin - rollDemandSpinold) > ROLL_SPL) rollDemandSpin = rollDemandSpinold + ROLL_SPL;
		if ((rollDemandSpin - rollDemandSpinold) < -ROLL_SPL) rollDemandSpin = rollDemandSpinold - ROLL_SPL;
		rollDemandSpinold = rollDemandSpin;
				
				
		// This part of the code sets the maximum angle the quadrotor can go to in the pitch and roll axes
		if(pitchDemandSpin > LIM_ANGLE) pitchDemandSpin = LIM_ANGLE;	
		if(pitchDemandSpin < -LIM_ANGLE) pitchDemandSpin = -LIM_ANGLE;
		if(rollDemandSpin > LIM_ANGLE) rollDemandSpin = LIM_ANGLE;
		if(rollDemandSpin < -LIM_ANGLE) rollDemandSpin = -LIM_ANGLE;
 
		// Create the demand derivative (demand and external rotations are split) term for the attitude motor control
		pitch.derivative = (pitchDemandSpin - pitch.demandOld);	
		roll.derivative = (rollDemandSpin - roll.demandOld);
		yaw.derivative = (yaw.demand - yaw.demandOld);	   
		pitch.demandOld = pitchDemandSpin;
		roll.demandOld = rollDemandSpin;
		yaw.demandOld = yaw.demand;
		
		// Use the Current and demanded angles to create the error for the proportional part of the PID loop
		// TODO: it might be more mathematically elegant (but harder to understand) 
		//to express the demand as a quaternion, but this is not a high priority
		pitcherror = pitchDemandSpin + thetaAngle;
		rollerror = rollDemandSpin - phiAngle;
		yawerror = yaw.demand + psiAngle; 
		
		// This section ensures that on swapping between -PI and PI, 
		// the craft always takes the shortest route to the desired angle
		if(pitcherror > M_PI) pitcherror -= M_TWOPI;
		else if(pitcherror < -M_PI) pitcherror += M_TWOPI;
		if(rollerror > M_PI) rollerror -= M_TWOPI;
		else if(rollerror < -M_PI) rollerror += M_TWOPI;  
		if(yawerror > M_PI) yawerror -= M_TWOPI;
		else if(yawerror < -M_PI) yawerror += M_TWOPI;
		
		// Creating the integral for the motor PID
		// TODO: Is this the cause of poor leveling on takeoff? (took out wierd throttle dependent rates)
		//TODO: Check to see if added Yaw integral solves yaw offsets in autonomous flight
		pitch.integral += pitcherror;
		roll.integral += rollerror;
		yaw.integral += yawerror;
		pitch.integral *= PITCH_De;
		roll.integral *= ROLL_De;
		yaw.integral *= YAW_De;
		
		// Detune at high throttle - We turn the tunings down at high throttle to prevent oscillations
		// happening on account of the higher energy input to the system
		float throttlefactor = throttle/MAXSTICK;
		if(throttlefactor > 1) throttlefactor = 1;	   
		float detunefactor = 1-(throttlefactor * DETUNE);
		float thisPITCH_Kd = PITCH_Kd;
		float thisPITCH_Kdd = PITCH_Kdd * detunefactor;
		float thisROLL_Kd = ROLL_Kd;
		float thisROLL_Kdd = ROLL_Kdd * detunefactor;
		float thisPITCH_Ki = PITCH_Ki;
		float thisROLL_Ki = ROLL_Ki;
		
				
		// Attitude control PID Assembly - We use a proportional, derivative, and integral on pitch roll and yaw
		// we add a double derivative term for pitch and roll.
		pitchcorrection = -((float)Gyro.Y.value - PITCH_Boost*pitch.derivative) * thisPITCH_Kd;
		pitchcorrection += -thisPITCH_Kdd*((float)Gyro.Y.value - pitch.valueOld);
		pitchcorrection += -PITCH_Kp*pitcherror;
		pitchcorrection += -thisPITCH_Ki*pitch.integral;

		rollcorrection = -((float)Gyro.X.value - ROLL_Boost*roll.derivative) * thisROLL_Kd;
		rollcorrection += -thisROLL_Kdd*((float)Gyro.X.value - roll.valueOld);
		rollcorrection += ROLL_Kp*rollerror;
		rollcorrection += thisROLL_Ki*roll.integral;

		yawcorrection = -((float)Gyro.Z.value + YAW_Boost*yaw.derivative) * YAW_Kd; 
		yawcorrection += -YAW_Kp*yawerror;
		// TODO: Check direction of yaw integral
		yawcorrection += -YAW_Ki*yaw.integral;
		
		// If the craft is upsidedown, turn off pitch and yaw control until roll control brings it back upright.
		// TODO: Test this code
		if ((phiAngle > M_PI_2) || (phiAngle < -M_PI_2)) {
			yawcorrection = 0;
			pitchcorrection = 0;
		}
		
		pitch.valueOld = (float)Gyro.Y.value;
		roll.valueOld = (float)Gyro.X.value;
	   
		//Assigning the PID results to the correct motors
		// TODO: add support for multiple orientations here
		motorN = pitchcorrection + rollcorrection;
		motorE = pitchcorrection - rollcorrection;
		motorS = -pitchcorrection - rollcorrection;
		motorW = -pitchcorrection + rollcorrection;
		motorN -= yawcorrection;
		motorE += yawcorrection;
		motorS -= yawcorrection;
		motorW += yawcorrection;
		
		// We run an SPR filter on the outputs to ensure they aren't too noisey and don't demand changes too quickly
		// This seems to reduce power consumption a little and ESC heating a little also
		motorNav *= SPR_OUT;
		motorNav += (1-SPR_OUT) * motorN;		
		motorEav *= SPR_OUT;
		motorEav += (1-SPR_OUT) * motorE;		
		motorSav *= SPR_OUT;
		motorSav += (1-SPR_OUT) * motorS;		
		motorWav *= SPR_OUT;
		motorWav += (1-SPR_OUT) * motorW;
			   
			   
// ****************************************************************************
// *** THROTTLE STATE MACHINE
// ****************************************************************************	
		
		// Combine attitude stabilisation demands from PID loop with throttle demands
		tempN = (signed short)motorNav + (signed short)throttle + THROTTLEOFFSET + (signed short)throttle_angle;
		tempE = (signed short)motorEav + (signed short)throttle + THROTTLEOFFSET + (signed short)throttle_angle;
		tempS = (signed short)motorSav + (signed short)throttle + THROTTLEOFFSET + (signed short)throttle_angle;
		tempW = (signed short)motorWav + (signed short)throttle + THROTTLEOFFSET + (signed short)throttle_angle;

		// TODO: Add Auto Land on rxLoss!
		if (rcInput[RX_THRO] - throttletrim <  OFFSTICK || throttleHoldOff > 0 || rxLoss > 25) {
			
			// Set throttle off
			throttle = 0;
						
			// Reset Important variables
			pitch.integral=0;
			roll.integral=0;
			alt.integral = 0;   
			motorN = 0;
			motorE = 0;
			motorS = 0;
			motorW = 0;
			motorNav = 0;
			motorEav = 0;
			motorSav = 0;
			motorWav = 0;
			// Reseting the yaw demand to the actual yaw angle continuously helps stop yawing happening on takeoff
			yaw.demand = -psiAngle;
			yaw.demandOld = -psiAngle;


			
			// Reset the throttle hold variable, this prevents reactivation of the throttle until 
			// the input is dropped and re-applied
			if(rcInput[RX_THRO] - throttletrim <  OFFSTICK) throttleHoldOff = 0;
			
			// If the craft is armed, set the PWM channels to the PWM value corresponding to off!
			if(armed) PWMSetNESW(THROTTLEOFFSET, THROTTLEOFFSET, THROTTLEOFFSET, THROTTLEOFFSET);
			
			// Output the motor PWM demand on the telemetry link
			ilink_outputs0.channel[0] = THROTTLEOFFSET;
			ilink_outputs0.channel[1] = THROTTLEOFFSET;
			ilink_outputs0.channel[2] = THROTTLEOFFSET;
			ilink_outputs0.channel[3] = THROTTLEOFFSET;
			

		}
		else if(armed) {
			
			float temp;
			
			// Limit the maximum throttle output to a percentage of the highest throttle available 
			// to allow additional throttle for manouevering
			if(throttle > MAXTHROTTLE*MAXTHROTTLEPERCENT) throttle = MAXTHROTTLE*MAXTHROTTLEPERCENT;
			
			// Set the PWM channels. Maximum of MAXTHROTTLE + THROTTLEOFFSET, MINMUM OF IDLETHROTTLE + THROTTLE OFFSET
			// Throttle offset offsets the throttle readings (which start at 0) to the PWM values (in ms?) which need to start at around 1000
			temp = tempN;
			if(temp > (MAXTHROTTLE + THROTTLEOFFSET)) temp = (MAXTHROTTLE + THROTTLEOFFSET);
			else if(temp < (IDLETHROTTLE + THROTTLEOFFSET)) temp = (IDLETHROTTLE + THROTTLEOFFSET);
			PWMSetN(temp);
			ilink_outputs0.channel[0] = temp;
			
			temp = tempE;
			if(temp > (MAXTHROTTLE + THROTTLEOFFSET)) temp = (MAXTHROTTLE + THROTTLEOFFSET);
			else if(temp < (IDLETHROTTLE + THROTTLEOFFSET)) temp = (IDLETHROTTLE + THROTTLEOFFSET);
			PWMSetE(temp);
			ilink_outputs0.channel[1] = temp;
			
			temp = tempS;
			if(temp > (MAXTHROTTLE + THROTTLEOFFSET)) temp = (MAXTHROTTLE + THROTTLEOFFSET);
			else if(temp < (IDLETHROTTLE + THROTTLEOFFSET)) temp = (IDLETHROTTLE + THROTTLEOFFSET);
			PWMSetS(temp);
			ilink_outputs0.channel[2] = temp;
			
			temp = tempW;
			if(temp > (MAXTHROTTLE + THROTTLEOFFSET)) temp = (MAXTHROTTLE + THROTTLEOFFSET);
			else if(temp < (IDLETHROTTLE + THROTTLEOFFSET)) temp = (IDLETHROTTLE + THROTTLEOFFSET);
			PWMSetW(temp);
			ilink_outputs0.channel[3] = temp;
			ilink_outputs0.isNew = 1;

		}
			   
}


// ****************************************************************************
// ****************************************************************************
// *** FUNCTIONS
// ****************************************************************************
// ****************************************************************************
// TODO: Simplify the calibration routines, there seems to be a lot of overlap and unnecessary/ unused code

void ReadRXInput(void) {			

	if(RXGetData(rcInput)) {
		if(rxLoss > 10) rxLoss -= 10;
		ilink_inputs0.channel[0] = rcInput[RX_THRO];
		ilink_inputs0.channel[1] = rcInput[RX_AILE];
		ilink_inputs0.channel[2] = rcInput[RX_ELEV];
		ilink_inputs0.channel[3] = rcInput[RX_RUDD];
		ilink_inputs0.channel[4] = rcInput[RX_AUX1];
		ilink_inputs0.channel[5] = rcInput[RX_FLAP];
		ilink_inputs0.isNew = 1;
		
		if(rxFirst < 25) {
			throttletrim = rcInput[RX_THRO];
			rxFirst++;
		}
		
		// Controller's aux or gear switch (Can be switched permanently either way)
		if(rcInput[RX_AUX1] > MIDSTICK) {
			auxState = 0;
		}
		else {								
			auxState = 1;
		}
		
		
		// Controller's flap switch/ button (mechanically sprung return)
		if(rcInput[RX_FLAP] > MIDSTICK) {
			if((flapState == 0) && (flpswitch == 0)) {
				flapState = 1;
				flpswitch = 1;
			}
			if((flapState == 1) && (flpswitch == 0)) {
				flapState = 0;
				flpswitch = 1;
			}
			
		}
		else {
			flpswitch = 0;
		}
			
			
		flashVLED = 0;
		LEDOff(VLED);
	}
	else {
		rxLoss ++;
		if(rxLoss > 50) {
			rxLoss = 50;
			// RC signal lost
			// TODO: Perform RC signal loss state setting
			if(armed) PWMSetNESW(THROTTLEOFFSET, THROTTLEOFFSET, THROTTLEOFFSET, THROTTLEOFFSET);
			ilink_outputs0.channel[0] = THROTTLEOFFSET;
			ilink_outputs0.channel[1] = THROTTLEOFFSET;
			ilink_outputs0.channel[2] = THROTTLEOFFSET;
			ilink_outputs0.channel[3] = THROTTLEOFFSET;
			flashVLED = 2;
		}
	}			
			
}

			
void ReadUltrasound(void) {			
	// Get ultrasound data and scale to mm??
	// TODO: Scale to m (everywhere should use SI units)
	ultra = (UltraGetNewRawData()) * 0.17;
	
	
	if(ultra > 0)  {
		ultraLoss = 0;
		
		// We run an SPR filter on the ultrasound readings
		alt.ultra *= SPR_ULTRA;
		alt.ultra += (1-SPR_ULTRA) * ultra;
		
		// Output the ultrasound altitude
		ilink_altitude.relAlt = alt.ultra;
	
		
	}
	// if ultra = 0 then there isn't valid data
	// TODO: Improve ultrasound confidence estimator
	else {
		ultraLoss++;
		if(ultraLoss > ULTRA_OVTH) ultraLoss = ULTRA_OVTH;
	}
			
}			
		
void ReadBattVoltage(void) {
	// Because the factor is 6325/1024, we can do this in integer maths by right-shifting 10 bits instead of dividing by 1024.
	unsigned short battV = (ADCGet() * 6325) >> 10; 
	// battVoltage in milivolts
	batteryVoltage *= 0.99f;
	batteryVoltage += 0.01f * (float)battV;
	ilink_thalstat.battVoltage = battV;
	ADCTrigger(CHN7);

}
			
			
			
void ReadGyroSensors(void) {
	signed short data[4];
	if(GetGyro(data)) {
		// Read raw Gyro data
		Gyro.X.raw = data[0];
		Gyro.Y.raw = data[2];
		Gyro.Z.raw = -data[1];
		// Output raw data over telemetry
		ilink_rawimu.xGyro = Gyro.X.raw;
		ilink_rawimu.yGyro = Gyro.Y.raw;
		ilink_rawimu.zGyro = Gyro.Z.raw;
		// Perform running average
		Gyro.X.total -= Gyro.X.history[Gyro.count];
		Gyro.Y.total -= Gyro.Y.history[Gyro.count];
		Gyro.Z.total -= Gyro.Z.history[Gyro.count];
		Gyro.X.total += Gyro.X.raw;
		Gyro.Y.total += Gyro.Y.raw;
		Gyro.Z.total += Gyro.Z.raw;
		Gyro.X.history[Gyro.count] = Gyro.X.raw;
		Gyro.Y.history[Gyro.count] = Gyro.Y.raw;
		Gyro.Z.history[Gyro.count] = Gyro.Z.raw;
		Gyro.X.av = (float)Gyro.X.total/(float)GAV_LEN;
		Gyro.Y.av = (float)Gyro.Y.total/(float)GAV_LEN;
		Gyro.Z.av = (float)Gyro.Z.total/(float)GAV_LEN;
		if(++Gyro.count >= GAV_LEN) Gyro.count = 0;
		// Add the offset calculated on calibration (to set no rotational movement to 0 corresponding output)
		// and scale to radians/s
		Gyro.X.value = (Gyro.X.av - Gyro.X.offset)/818.51113590117601252569f;
		Gyro.Y.value = (Gyro.Y.av - Gyro.Y.offset)/818.51113590117601252569f;
		Gyro.Z.value = (Gyro.Z.av - Gyro.Z.offset)/818.51113590117601252569f;
		// Send processed values over telemetry
		ilink_scaledimu.xGyro = Gyro.X.value * 1000;
		ilink_scaledimu.yGyro = Gyro.Y.value * 1000;
		ilink_scaledimu.zGyro = Gyro.Z.value * 1000;
	}
}

void ReadAccelSensors(void) {
	float sumsqu;
	signed short data[4];
	if(GetAccel(data)) {
		// Get raw Accelerometer data
		Accel.X.raw = data[0];
		Accel.Y.raw = data[2];
		Accel.Z.raw = -data[1];
		// Output raw data over telemetry
		ilink_rawimu.xAcc = Accel.X.raw;
		ilink_rawimu.yAcc = Accel.Y.raw;
		ilink_rawimu.zAcc = Accel.Z.raw;
		// Perform Running Average
		Accel.X.total -= Accel.X.history[Accel.count];
		Accel.Y.total -= Accel.Y.history[Accel.count];
		Accel.Z.total -= Accel.Z.history[Accel.count];
		Accel.X.total += Accel.X.raw;
		Accel.Y.total += Accel.Y.raw;
		Accel.Z.total += Accel.Z.raw;
		Accel.X.history[Accel.count] = Accel.X.raw;
		Accel.Y.history[Accel.count] = Accel.Y.raw;
		Accel.Z.history[Accel.count] = Accel.Z.raw;
		Accel.X.av = (float)Accel.X.total/(float)AAV_LEN;
		Accel.Y.av = (float)Accel.Y.total/(float)AAV_LEN;
		Accel.Z.av = (float)Accel.Z.total/(float)AAV_LEN;
		if(++Accel.count >= AAV_LEN) Accel.count = 0;
		// Normalise accelerometer so it is a unit vector
		sumsqu = finvSqrt((float)Accel.X.av*(float)Accel.X.av + (float)Accel.Y.av*(float)Accel.Y.av + (float)Accel.Z.av*(float)Accel.Z.av); // Accelerometr data is normalised so no need to convert units.
		Accel.X.value = (float)Accel.X.av * sumsqu;
		Accel.Y.value = (float)Accel.Y.av * sumsqu;
		Accel.Z.value = (float)Accel.Z.av * sumsqu;
		//Output processed values over telemetry
		ilink_scaledimu.xAcc = Accel.X.value * 1000;
		ilink_scaledimu.yAcc = Accel.Y.value * 1000;
		ilink_scaledimu.zAcc = Accel.Z.value * 1000;
	}
}

void ReadMagSensors(void) {
	float sumsqu, temp1, temp2, temp3;
	signed short data[4];
	if(GetMagneto(data)) {
		// Get raw magnetometer data
		Mag.X.raw = data[0];
		Mag.Y.raw = data[2];
		Mag.Z.raw = -data[1];
		// Output raw data over telemetry
		ilink_rawimu.xMag = Mag.X.raw;
		ilink_rawimu.yMag = Mag.Y.raw;
		ilink_rawimu.zMag = Mag.Z.raw;
		// Perform running average
		Mag.X.total -= Mag.X.history[Mag.count];
		Mag.Y.total -= Mag.Y.history[Mag.count];
		Mag.Z.total -= Mag.Z.history[Mag.count];
		Mag.X.total += Mag.X.raw;
		Mag.Y.total += Mag.Y.raw;
		Mag.Z.total += Mag.Z.raw;
		Mag.X.history[Mag.count] = Mag.X.raw;
		Mag.Y.history[Mag.count] = Mag.Y.raw;
		Mag.Z.history[Mag.count] = Mag.Z.raw;
		Mag.X.av = (float)Mag.X.total/(float)MAV_LEN;
		Mag.Y.av = (float)Mag.Y.total/(float)MAV_LEN;
		Mag.Z.av = (float)Mag.Z.total/(float)MAV_LEN;
		if(++Mag.count >= MAV_LEN) Mag.count = 0;
		
		// Correcting Elipsoid Centre Point (These values are found during Magneto Calibration)
		temp1 = Mag.X.av - MAGCOR_M1;
		temp2 = Mag.Y.av - MAGCOR_M2;
		temp3 = Mag.Z.av - MAGCOR_M3;

		// Reshaping Elipsoid to Sphere (These values are set the same for all Thalamus Units)
		temp1 = MAGCOR_N1 * temp1 + MAGCOR_N2 * temp2 + MAGCOR_N3 * temp3;
		temp2 = MAGCOR_N5 * temp2 + MAGCOR_N6 * temp3;
		temp3 = MAGCOR_N9 * temp3;				

		// Normalize magneto into unit vector
		sumsqu = finvSqrt((float)temp1*(float)temp1 + (float)temp2*(float)temp2 + (float)temp3*(float)temp3); // Magnetoerometr data is normalised so no need to convert units.
		Mag.X.value = (float)temp1 * sumsqu;
		Mag.Y.value = (float)temp2 * sumsqu;
		Mag.Z.value = (float)temp3 * sumsqu;
		
		// Output processed data over telemetry
		ilink_scaledimu.xMag = Mag.X.value * 1000;
		ilink_scaledimu.yMag = Mag.Y.value * 1000;
		ilink_scaledimu.zMag = Mag.Z.value * 1000;
	
	}	
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



void CalibrateMagneto(void) {
		unsigned int i;
		unsigned int  good;
		float Xav, Yav, Zav;
		float distance;
		
		signed short Xmax, Xmin, Ymax, Ymin, Zmax, Zmin;
		
		unsigned int started;
	
		if(armed == 0) {
			ilink_thalstat.sensorStatus = 2; // calibrating
		
			started = 0;
			
			Xav = 0;
			Yav = 0;
			Zav = 0;

			flashVLED = 1;
			good = 0;
			
			ReadMagSensors();
			Xmax = Mag.X.raw;
			Xmin = Xmax;
			Ymax = Mag.Y.raw;
			Ymin = Ymax;
			Zmax = Mag.Z.raw;
			Zmin = Zmax;
			
			// Calibrate for 70 seconds or until still for 2.8 seconds.
			//TODO: filter out spikes
			for(i=0; i<5000; i++) {
				ReadGyroSensors();
				
				// calculate distance of data from running average
				distance  = (Xav - (float)Gyro.X.raw)*(Xav - (float)Gyro.X.raw);
				distance += (Yav - (float)Gyro.Y.raw)*(Yav - (float)Gyro.Y.raw);
				distance += (Zav - (float)Gyro.Z.raw)*(Zav - (float)Gyro.Z.raw);
				
				if(started == 0) {
					// before starting, wait for gyro to move around
					if(distance > 2000) {
						// high-movement, increment good counter and add average value.
						good++;
						if(good >= 10) {
							started = 1; // if enough movement readings, escape loop
							good = 0;
							flashVLED = 2;
						}
					}
					else {
						good = 0;
					}
				}
				else {
					ReadMagSensors();
					if(Mag.X.raw > Xmax) Xmax = Mag.X.raw;
					else if(Mag.X.raw < Xmin) Xmin = Mag.X.raw;
					if(Mag.Y.raw > Ymax) Ymax = Mag.Y.raw;
					else if(Mag.Y.raw < Ymin) Ymin = Mag.Y.raw;
					if(Mag.Z.raw > Zmax) Zmax = Mag.Z.raw;
					else if(Mag.Z.raw < Zmin) Zmin = Mag.Z.raw;
					
					if(distance < 2000) {
						// high-movement, increment good counter and add average value.
						good++;
						if(good >= 200) break; // if enough movement readings, escape loop
					}
					else {
						good = 0;
					}
				}
				
				Xav *= 0.95f;
				Xav += 0.05f * (float)Gyro.X.raw;
				Yav *= 0.95f;
				Yav += 0.05f * (float)Gyro.Y.raw;
				Zav *= 0.95f;
				Zav += 0.05f * (float)Gyro.Z.raw; 
				
				//TODO: Inline delay
				Delay(14);
			}
			
			MAGCOR_M1 = (Xmax + Xmin)/2;
			MAGCOR_M2 = (Ymax + Ymin)/2;
			MAGCOR_M3 = (Zmax + Zmin)/2;
			EEPROMSaveAll();
			
			flashPLED = 0;
			LEDOff(PLED);
		
			ilink_thalstat.sensorStatus &= ~(0x7); // mask status
			ilink_thalstat.sensorStatus |= 3; // standby
	}
}



void SensorZero(void) {
	unsigned int i;
	signed short data[4];
	
	if(!GetGyro(data) || !GetMagneto(data) || !GetAccel(data)/* || GetBaro() == 0*/) {
		LEDInit(PLED | VLED);
		LEDOn(PLED);
		LEDOff(VLED);
		flashPLED = 2;
		flashVLED = 2;
		while(1);
	}
	
	// *** Zero totals
		Gyro.X.total = 0;
		Gyro.Y.total = 0;
		Gyro.Z.total = 0;
		Accel.X.total = 0;
		Accel.Y.total = 0;
		Accel.Z.total = 0;
		Mag.X.total = 0;
		Mag.Y.total = 0;
		Mag.Z.total = 0;
		


		for(i=0; (i<GAV_LEN); i++) {
			Gyro.X.history[i] = 0;
			Gyro.Y.history[i] = 0;
			Gyro.Z.history[i] = 0;
		}
		
		for(i=0; (i<AAV_LEN); i++) {
			Accel.X.history[i] = 0;
			Accel.Y.history[i] = 0;
			Accel.Z.history[i] = 0;
		}
		
		for(i=0; (i<MAV_LEN); i++) {
			Mag.X.history[i] = 0;
			Mag.Y.history[i] = 0;
			Mag.Z.history[i] = 0;
		}
		
		Gyro.X.offset = 0;
		Gyro.Y.offset = 0;
		Gyro.Z.offset = 0;
		
		// pre-seed averages
		for(i=0; (i<GAV_LEN); i++) {
			ReadGyroSensors();
		}
		
		for(i=0; (i<AAV_LEN); i++) {
			ReadAccelSensors();
		}
		
		for(i=0; (i<MAV_LEN); i++) {
			ReadMagSensors();
		}
	
		ilink_thalstat.sensorStatus |= (0xf << 3);
}



void CalibrateGyro(void) {
	CalibrateGyroTemp(6);
	CAL_GYROX = Gyro.X.offset;
	CAL_GYROY = Gyro.Y.offset;
	CAL_GYROZ = Gyro.Z.offset;
	EEPROMSaveAll();
}



void CalibrateGyroTemp(unsigned int seconds) {
	unsigned int i;
	// *** Calibrate Gyro
	unsigned int  good;
	float Xav, Yav, Zav;
	signed int Xtotal, Ytotal, Ztotal;
	float distance;
	
	if(armed == 0) {
		
		ReadGyroSensors();
		Xtotal = 0;
		Ytotal = 0;
		Ztotal = 0;
		Xav = Gyro.X.raw;
		Yav = Gyro.Y.raw;
		Zav = Gyro.Z.raw;

		flashPLED = 1;
		good = 0;
		
		// Wait for Gyro to be steady or 20 seconds
		for(i=0; i<5000; i++) {
			ReadGyroSensors();
			
			// calculate distance of data from running average
			distance  = (Xav - (float)Gyro.X.raw)*(Xav - (float)Gyro.X.raw);
			distance += (Yav - (float)Gyro.Y.raw)*(Yav - (float)Gyro.Y.raw);
			distance += (Zav - (float)Gyro.Z.raw)*(Zav - (float)Gyro.Z.raw);

			// LPF for above
			Xav *= 0.95f;
			Xav += 0.05f * (float)Gyro.X.raw;
			Yav *= 0.95f;
			Yav += 0.05f * (float)Gyro.Y.raw;
			Zav *= 0.95f;
			Zav += 0.05f * (float)Gyro.Z.raw; 

			//TODO: This distnace was never small enough on the ICRS Quad. Double check
			if(distance < 2000) {
				// low-movement, increment good counter and add average value.
				good++;
				if(good >= 333) break; // if enough good readings, escape loop
			}
			else {
				// high movement, zero good counter, and average values.
				good = 0;
			}
		
			Delay(3);
		}
		
		ilink_thalstat.sensorStatus &= ~(0x7); // mask status
		ilink_thalstat.sensorStatus |= 2; // calibrating

		flashPLED=2;
		LEDOn(PLED);
		
		// at this point should have at least 200 good Gyro readings, take some more
			
		for(i=0; i<seconds*333; i++) {
			ReadGyroSensors();
			
			Xtotal += Gyro.X.raw;
			Ytotal += Gyro.Y.raw;
			Ztotal += Gyro.Z.raw;
			
			Delay(3);
		}

		Gyro.X.offset = (float)Xtotal/(float)(seconds * 333);
		Gyro.Y.offset = (float)Ytotal/(float)(seconds * 333);
		Gyro.Z.offset = (float)Ztotal/(float)(seconds * 333);
		
		flashPLED = 0;

		LEDOff(PLED);
	
		ilink_thalstat.sensorStatus &= ~(0x7); // mask status
		ilink_thalstat.sensorStatus |= 3; // standby
		
	}
}



// ****************************************************************************
// *** EEPROM Functions
// ****************************************************************************

// *** This function loads all parameters from EEPROM.  First it loads the
// parameters into temporary storage to verify the checksum.  Only if the
// checksums are correct will the function update the parameters in RAM.
void EEPROMLoadAll(void) {
	unsigned char chkA, chkB;
	unsigned int i;
	unsigned char * ptr;
	float tempStorage[EEPROM_MAX_PARAMS+1];
	
	// Read EEPROM into some temporarily allocated space
	EEPROMRead(EEPROM_OFFSET, (unsigned char *)&tempStorage, paramCount * 4 + 2);
	
	// Calculate the Fletcher-16 checksum
	chkA=EEPROM_VERSION;
	chkB=EEPROM_VERSION;
	ptr = (unsigned char *)&tempStorage;
	for(i=0; i<paramCount*4; i++) {
		chkA += ptr[i];
		chkB += chkA;
	}
	
	// Verify the checksum is valid (at this point i points to the correct elements for chkA and chkB)
	if(chkA == ptr[i] && chkB == ptr[i+1]) {
		// For valid data, load into parameters in RAM
		for(i=0; i<paramCount; i++) {
			paramStorage[i].value = tempStorage[i];
		}
	}
}

// *** This function saves all parameters to EEPROM.
void EEPROMSaveAll(void) {
	unsigned char chkA, chkB;
	unsigned int i;
	unsigned char * ptr;
	float tempStorage[EEPROM_MAX_PARAMS+1];
	
	// Save parameters into temporary space
	
	chkA=EEPROM_VERSION;
	chkB=EEPROM_VERSION;
	for(i=0; i<paramCount && i<EEPROM_MAX_PARAMS; i++) {
		tempStorage[i] = paramStorage[i].value;
		ptr = (unsigned char *)&(tempStorage[i]);
		chkA += ptr[0];
		chkB += chkA;
		chkA += ptr[1];
		chkB += chkA;
		chkA += ptr[2];
		chkB += chkA;
		chkA += ptr[3];
		chkB += chkA;
	}
	
	ptr = (unsigned char *)&(tempStorage[i]);
	ptr[0] = chkA;
	ptr[1] = chkB;
	
	EEPROMWrite(EEPROM_OFFSET, (unsigned char *)&tempStorage, paramCount * 4 + 2);
}


// ****************************************************************************
// *** Communications Functions
// ****************************************************************************

// *** Deal with ilink requets
void ILinkMessageRequest(unsigned short id) {
	unsigned short * ptr = 0;
	unsigned short maxlength = 0;
	
	switch(id) {
		case ID_ILINK_IDENTIFY:	  ptr = (unsigned short *) &ilink_identify;   maxlength = sizeof(ilink_identify)/2 - 1;   break;
		case ID_ILINK_THALSTAT:	 ptr = (unsigned short *) &ilink_thalstat;   maxlength = sizeof(ilink_thalstat)/2 - 1;   break;
		case ID_ILINK_RAWIMU:	   ptr = (unsigned short *) &ilink_rawimu;	 maxlength = sizeof(ilink_rawimu)/2 - 1;	 break;
		case ID_ILINK_SCALEDIMU:	ptr = (unsigned short *) &ilink_scaledimu;  maxlength = sizeof(ilink_scaledimu)/2 - 1;  break;
		case ID_ILINK_ALTITUDE:	 ptr = (unsigned short *) &ilink_altitude;   maxlength = sizeof(ilink_altitude)/2 - 1;   break;
		case ID_ILINK_ATTITUDE:	 ptr = (unsigned short *) &ilink_attitude;   maxlength = sizeof(ilink_attitude)/2 - 1;   break;
		case ID_ILINK_INPUTS0:
			if(ilink_inputs0.isNew) {
				ilink_inputs0.isNew = 0;
				ptr = (unsigned short *) &ilink_inputs0;
				maxlength = sizeof(ilink_inputs0)/2 - 1;
			}
			break;
		case ID_ILINK_OUTPUTS0:	 ptr = (unsigned short *) &ilink_outputs0;   maxlength = sizeof(ilink_outputs0)/2 - 1;   break;
		case ID_ILINK_CLEARBUF:
			FUNCILinkTxBufferPushPtr = 0;
			FUNCILinkTxBufferPopPtr = 0;
			break;
	}

	if(ptr) {
		ILinkSendMessage(id, ptr, maxlength);
	}
}

// *** iLink interrupt handler
void ILinkMessage(unsigned short id, unsigned short * buffer, unsigned short length) {
	unsigned short * ptr = 0;
	unsigned int i, j;
	
	switch(id) {
		case ID_ILINK_THALPAREQ: ptr = (unsigned short *) &ilink_thalpareq; break;
		case ID_ILINK_THALPARAM: ptr = (unsigned short *) &ilink_thalparam_rx; break;
		case ID_ILINK_THALCTRL: ptr = (unsigned short *) &ilink_thalctrl; break;
		case ID_ILINK_POSITION: ptr = (unsigned short *) &ilink_position; break;
		case ID_ILINK_PAYLDCTRL: ptr = (unsigned short *) &ilink_payldctrl; break;
	}
	
	if(ptr) {
		for(i=0; i<length; i++) {
			ptr[i] = buffer[i];
		}
		ptr[i] = 1; // this should be the isNew byte
	}
	
	switch(id) {
		case ID_ILINK_THALPAREQ:
			ilink_thalpareq.isNew = 0;
			switch(ilink_thalpareq.reqType) {
				case 1: // get one
					if(ilink_thalpareq.paramID == 0xffff) {

						for (i=0; i<paramCount; i++){
							unsigned char match = 1;
							for (j=0; j<16; j++) {
								if (paramStorage[i].name[j] !=  ilink_thalparam_rx.paramName[j]) {

									match = 0;
									break;
								}
								if (paramStorage[i].name[j] == '\0') break;
							}
							
							if(match == 1) {
								// when a match is found get the iD
								paramSendCount = i;
								paramSendSingle = 1;
								break;
							}
						}
					}


					else {
						paramSendCount = ilink_thalpareq.paramID;
						paramSendSingle = 1;
					}
					break;
				case 2: // save all
					EEPROMSaveAll();
					ilink_thalpareq.isNew = 1;
					ilink_thalctrl.command = MAVLINK_MSG_ID_COMMAND_LONG;
					ilink_thalctrl.data = MAV_CMD_PREFLIGHT_STORAGE;
					ILinkSendMessage(ID_ILINK_THALCTRL, (unsigned short *) &ilink_thalctrl, sizeof(ilink_thalctrl)/2 - 1);
					break;
				case 3: // reload all
					EEPROMLoadAll();
					ilink_thalpareq.isNew = 1;
					ilink_thalctrl.command = MAVLINK_MSG_ID_COMMAND_LONG;
					ilink_thalctrl.data = MAV_CMD_PREFLIGHT_STORAGE;
					ILinkSendMessage(ID_ILINK_THALCTRL, (unsigned short *) &ilink_thalctrl, sizeof(ilink_thalctrl)/2 - 1);
					// fall through to get all
				default:
				case 0: // get all
					paramSendCount = 0;
					paramSendSingle = 0;
					break;
				}
			break;
		
		
			
		case ID_ILINK_THALPARAM:
			// match up received parameter with stored parameter.
			for (i=0; i<paramCount; i++){
				unsigned char match = 1;
				for (j=0; j<16; j++) {
					if (paramStorage[i].name[j] !=  ilink_thalparam_rx.paramName[j]) {
						match = 0;
						break;
					}
					if (paramStorage[i].name[j] == '\0') break;
				}
				
				if(match == 1) {
					// when a match is found, save it to paramStorage
					paramStorage[i].value = ilink_thalparam_rx.paramValue;
					
					// then order the value to be sent out again using the param send engine
					// but deal with cases where it's already in the process of sending out data
					if(paramSendCount < paramCount) {
						// parameter engine currently sending out data
						if(paramSendCount >= i) {
							// if parameter engine already sent out this now-changed data, redo this one, otherwise no action needed
							paramSendCount = i;
						}
					}
					else {
						// parameter engine not currently sending out data, so send single parameter
						paramSendCount = i;
						paramSendSingle = 1;
					}
					break;
				}
			}
			break;
	}
}