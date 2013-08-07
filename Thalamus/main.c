#include "thal.h"
#include "mavlink.h"
#include <math.h>


// ****************************************************************************
// ****************************************************************************
// *** DECLARATIONS
// ****************************************************************************
// ****************************************************************************


// *** Config stuff
#define FIRMWARE_VERSION    1           // Firmware version
#define MESSAGE_LOOP_HZ     15          // Max frequency of messages in Hz (keep this number low, like around 15)
#define RX_PANIC            2           // Number of seconds after missing RX before craft considered "disconnected"

// *** LED stuff
unsigned char flashPLED, flashVLED, flashRLED;

float gim_ptemp;

// *** ILink stuff
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

void LinkInit(void);

// *** Timers and counters
unsigned int sysMS;
unsigned long long sysUS;
unsigned short RxWatchdog;
unsigned short UltraWatchdog;

// *** Functions
void SensorZero(void);
void CalibrateGyro(void);
void CalibrateGyroTemp(unsigned int seconds);
void CalibrateMagneto(void);

// *** Configs
#define FAST_RATE           400
#define SLOW_RATE           75

#define ZEROTHROTMAX        1*FAST_RATE

#define SLOW_DIVIDER        FAST_RATE/SLOW_RATE

unsigned short slowSoftscale;//, inputSoftscale, baroSoftscale;

// TODO: check ESC response at THROTTLEOFFSET, consider raising THROTTLEOFFSET to 1000
#define THROTTLEOFFSET    900		// Corresponds to zero output PWM. Nominally 1000=1ms, but 800 works better
#define IDLETHROTTLE    	175		// Minimum PWM output to ESC when in-flight to avoid motors turning off
#define MAXTHROTTLE     	1200	// Maximum PWM output to ESC (PWM = THROTTLEOFFSET + MAXTHROTTLE)
#define MAXTHROTTLEPERCENT	0.9     // Maximum percentage throttle should be (reserves some extra output for stabilisation at high throttle).

#define OFFSTICK        	50
#define MIDSTICK        	512		// Corresponds to input when stick is in middle (approx value).
#define MAXSTICK        	850		// Corresponds to input when stick is at the top

#define MAXTHRESH           (MAXSTICK+MIDSTICK)/2 - 50
#define MINTHRESH           (MIDSTICK+OFFSTICK)/2 + 50


// *** Parameters
typedef struct paramStorage_struct {
    char name[16];
    float value;
} paramStorage_t;

struct paramStorage_struct paramStorage[] = {
    {"DRIFT_AKp",           0.2f},
    {"DRIFT_MKp",      		0.2f},   
    #define DRIFT_AccelKp   paramStorage[0].value    
    #define DRIFT_MagKp     paramStorage[1].value  
  
    {"SPR_ULTRA",       0.95f},  
    #define SPR_ULTRA 		paramStorage[2].value

    {"YAW_SEN",     0.00002f},     
    {"PITCH_SEN",    0.0022f},    
    {"ROLL_SEN",     0.0022f},   
    #define YAW_SENS        paramStorage[3].value
    #define PITCH_SENS      paramStorage[4].value
    #define ROLL_SENS       paramStorage[5].value

    {"YAW_DZN",      0.001f},
    #define YAW_DEADZONE    paramStorage[6].value 

    {"PITCH_Kp",      400.0f},     
    {"PITCH_Ki",        2.0f},    
    {"PITCH_Kd",      100.0f},     
    {"PITCH_Kdd",    1500.0f},
    {"PITCH_Bst",     0.0f},      
    {"PITCH_De",      0.999f},    
    #define PITCH_Kp        paramStorage[7].value 
    #define PITCH_Ki        paramStorage[8].value
    #define PITCH_Kd        paramStorage[9].value 
    #define PITCH_Kdd       paramStorage[10].value 
    #define PITCH_Boost     paramStorage[11].value 
    #define PITCH_De        paramStorage[12].value 

    {"ROLL_Kp",       400.0f},       
    {"ROLL_Ki",         2.0f},      
    {"ROLL_Kd",       100.0f},        
    {"ROLL_Kdd",     1500.0f},     		
    {"ROLL_Bst",       0.00f},    				
    {"ROLL_De",       0.999f},
    #define ROLL_Kp         paramStorage[13].value  
    #define ROLL_Ki         paramStorage[14].value 
    #define ROLL_Kd         paramStorage[15].value 
    #define ROLL_Kdd        paramStorage[16].value 	
    #define ROLL_Boost      paramStorage[17].value 
    #define ROLL_De         paramStorage[18].value

    {"YAW_Kp",        1000.0f},     
    {"YAW_Kd",        250.0f},  
    {"YAW_Bst",        0.00f},   
    #define YAW_Kp          paramStorage[19].value 
    #define YAW_Kd          paramStorage[20].value 
    #define YAW_Boost       paramStorage[21].value 	

    // Mode
    {"MODE_ST",       5.0f},  //0 is Acrobatic,  1 is Regular, 2 is Simplicity, 3 is Easy, 4 is GPS Hold, 5 is Auto 1, 6 is Auto 2
    #define MODE_ST 		paramStorage[22].value

    //Limits
    {"LIM_ANGLE",       0.35f},  // Roll and Pitch Angle Limit in Radians
    {"LIM_ALT",         1000.0f},  // Altitude Limit in mm when in Ultrasound Mode
    #define LIM_ANGLE 		paramStorage[23].value
    #define LIM_ALT 		paramStorage[24].value

    // Magneto Correction
    {"CAL_MAGN1",     0.001756f},      
    {"CAL_MAGN2",   0.00008370f},        
    {"CAL_MAGN3",   0.00005155f},     	
    {"CAL_MAGN5",     0.001964f},    		
    {"CAL_MAGN6",   0.00002218f},     
    {"CAL_MAGN9",     0.001768f},     
    {"CAL_MAGM1",       0.0f},     
    {"CAL_MAGM2",        0.0f},     	
    {"CAL_MAGM3",        0.0f},  

    #define MAGCOR_N1       paramStorage[25].value 
    #define MAGCOR_N2       paramStorage[26].value 		
    #define MAGCOR_N3       paramStorage[27].value 		
    #define MAGCOR_N5       paramStorage[28].value 
    #define MAGCOR_N6       paramStorage[29].value 
    #define MAGCOR_N9       paramStorage[30].value 
    #define MAGCOR_M1       paramStorage[31].value 		
    #define MAGCOR_M2       paramStorage[32].value 
    #define MAGCOR_M3       paramStorage[33].value

    // Ultrasound
    {"ULTRA_Kp",        0.05f},
    {"ULTRA_Kd",        5.0f},
    {"ULTRA_Ki",        0.00001f},
    {"ULTRA_De",      	0.9999f},
    {"ULTRA_TKOFF",   	200.0f}, 
    {"ULTRA_LND",   	150.0f}, 
    #define ULTRA_Kp        paramStorage[34].value 
    #define ULTRA_Kd        paramStorage[35].value 
    #define ULTRA_Ki        paramStorage[36].value 
    #define ULTRA_De        paramStorage[37].value
    #define ULTRA_TKOFF     paramStorage[38].value 
    #define ULTRA_LND       paramStorage[39].value 

    {"CAL_GYROX",   0.0f},
    {"CAL_GYROY",   0.0f},
    {"CAL_GYROZ",   0.0f},
    #define CAL_GYROX       paramStorage[40].value 
    #define CAL_GYROY       paramStorage[41].value 
    #define CAL_GYROZ       paramStorage[42].value 
    
	{"DETUNE",			0.2f},
	#define DETUNE		paramStorage[43].value
	
	{"LIM_RATE",			100.0f},
	#define LIM_RATE		paramStorage[44].value
    
	{"LIM_ULTRA",			4.0f},
	#define LIM_ULTRA		paramStorage[45].value
    
	{"ULTRA_DRMP",     3.0f}, 
	{"ULTRA_DTCT",     6.0f},
    #define ULTRA_DRMP      paramStorage[46].value
    #define ULTRA_DTCT      paramStorage[47].value
	
	{"LIM_THROT", 		0.3f},
	#define LIM_THROT		paramStorage[48].value
	
	{"ULTRA_OVDEC",		0.01f},
	#define ULTRA_OVDEC		paramStorage[49].value
	
	{"ULTRA_DEAD",		100},
	#define ULTRA_DEAD		paramStorage[50].value
	
	{"ULTRA_OVTH",		40},
	#define ULTRA_OVTH		paramStorage[51].value
	
	
	{"GPS_Kp",      0.03f}, 	
	{"GPS_De",      1.0f}, 	
	{"GPS_Ki",     	0.00005f}, 	
	{"GPS_Kd",     	0.5f}, 	
    #define GPS_Kp		paramStorage[52].value
    #define GPS_De		paramStorage[53].value
    #define GPS_Ki		paramStorage[54].value
    #define GPS_Kd		paramStorage[55].value
    
    {"GPS_ALTKp", 10.0f},
    {"GPS_ALTKi", 0.0001f},
    {"GPS_ALTDe", 1.0f},
    {"GPS_ALTKd", 20.0f},
    #define GPS_ALTKp		paramStorage[56].value
    #define GPS_ALTKi		paramStorage[57].value
    #define GPS_ALTDe		paramStorage[58].value
    #define GPS_ALTKd		paramStorage[59].value
   
    {"GPS_ALTHo", 20.0f},
    #define GPS_ALTHo       paramStorage[60].value
    
    {"CAL_AUTO", 1.0f},
    #define CAL_AUTO        paramStorage[61].value    
    
    
     //Gimbal in the Pitch Axis	
    {"GIM_PMID",     1500.0f},		//Mid Point 
    {"GIM_PSCAL",    -1650.0f},		//Scaling
    {"GIM_PLIMH",    2300.0f}, 		//Limit High
    {"GIM_PLIML",     700.0f},		//Limit Low
    {"GIM_PPLUS",     100.0f},		//Limit Low
    #define GIM_PMID        paramStorage[62].value 		
    #define GIM_PSCAL       paramStorage[63].value 
    #define GIM_PLIMH       paramStorage[64].value 		
    #define GIM_PLIML       paramStorage[65].value 
    #define GIM_PPLUS       paramStorage[66].value 

    //Gimbal in the Roll Axis
    {"GIM_RMID",     1500.0f},       //Mid Point    	
    {"GIM_RSCAL",    1650.0f},      //Scaling
    {"GIM_RLIMH",    2300.0f},     	//Limit High
    {"GIM_RLIML",     700.0f},		//Limit Low
    #define GIM_RMID        paramStorage[67].value 
    #define GIM_RSCAL       paramStorage[68].value
    #define GIM_RLIMH       paramStorage[69].value 
    #define GIM_RLIML       paramStorage[70].value
	
	{"SPR_OUT",       0.6f},  
    #define SPR_OUT 		paramStorage[71].value
    
    {"SPR_GPSYAW",     0.66f},
    #define SPR_GPSYAW 		paramStorage[72].value
    
    {"SPR_BARO",        0.999f},
    #define SPR_BARO        paramStorage[73].value
    
    {"BAT_LOW",         11000.0f},
    {"BAT_CRIT",        10000.0f},
    #define BATT_LOW        paramStorage[74].value
    #define BATT_CRIT       paramStorage[75].value
    
    {"GPS_LNDRT",         0.1},
    {"LIM_GPSALV",         0.1},
    #define GPS_LNDRT        paramStorage[76].value
    #define LIM_GPSALV        paramStorage[77].value
	
	{"ULTRA_OFFSET",         350},
    #define ULTRA_OFFSET        paramStorage[78].value
	
	
	{"GPS_THRLM",         0.1},
    #define GPS_THRLM        paramStorage[79].value
	
	{"ROLL_SPL",         0.002},
	#define ROLL_SPL        paramStorage[80].value
	{"PITCH_SPL",         0.002},
	#define PITCH_SPL        paramStorage[81].value

	
	
	};

unsigned int paramSendCount;
unsigned int paramCount;
unsigned char paramSendSingle;

void EEPROMLoadAll(void);
void EEPROMSaveAll(void);

#define EEPROM_MAX_PARAMS   100 // this should be greater than or equal to the above number of parameters
#define EEPROM_OFFSET   0 // EEPROM Offset used for moving the EEPROM values around storage (wear levelling I guess)
#define EEPROM_VERSION	43 // version of variables in EEPROM, change this value to invalidate EEPROM contents and restore defaults

// *** quaternion storage
float q1, q2, q3, q4;
float thetaAngle, phiAngle, psiAngle;
float M1, M2, M3, M4, M5, M6, M7, M8, M9;

double lat_diff;
double lon_diff;
double lat_diff_i;
double lon_diff_i;
double lat_diff_d;
double lon_diff_d;
double lon_diff_old;
double lat_diff_old;
double alt_diff_old;
double alt_diff;
double alt_diff_i;
double alt_diff_d;
double alt_throttle;
double gpsThrottle;
double targetY_tko;
double targetX_tko;
unsigned int gpsTakeoffCount;
double Xout;
double Yout;

unsigned int gpsHoldSet;
double gpsHoldX;
double gpsHoldY;
double gpsHoldZ;
double oldZtarget;

float CFDC_mag_x_zero;
float CFDC_mag_y_zero;
float CFDC_mag_z_zero;
float CFDC_mag_x_error;
float CFDC_mag_y_error;
float CFDC_mag_z_error;

float switchover_count = 0;

unsigned int gpsHomeSet;
double gpsHomeX;
double gpsHomeY;
double gpsHomeZ;

typedef struct{
	float demandav;
    float demand;
	float demandtemp;
	float demandtempold;
	float derivativetemp;
	float secondderivativetemp;
	float derivativetempold;
	float demandOld;
	float valueOld;
	float derivative;
	float integral;
} directionStruct;

directionStruct pitch;
directionStruct roll;
directionStruct yaw;

typedef struct {
    float value;
    float valueOld;
    float error;
    float demand;
    float demandav;
    float derivative;
    float integral;
	float demandincr;
	float demandold;
    float baroOffset;
} altStruct;
altStruct alt;

float thetaAngle, phiAngle, psiAngle, psiAngleinit;

// *** Sensor Storage
#define GAV_LEN 8
#define AAV_LEN 30
#define MAV_LEN 30

typedef struct{
    volatile signed short raw;
    volatile float av;
    volatile float value;
    volatile float offset;
    volatile float error;
    volatile float verterror;
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

threeAxisSensorStructGyro Gyro;
threeAxisSensorStructAccel Accel;
threeAxisSensorStructMag Mag;

void ReadGyroSensors(void);
void ReadAccelSensors(void);
void ReadMagSensors(void);

// *** Input stuff
unsigned short rcInput[7];
unsigned int rxLoss;
unsigned int rxFirst;
signed short yawtrim;
signed short throttletrim;
float throttle;
float throttle_angle;
unsigned int ultraerror = 0;
float nonLinearThrottle(float input);
unsigned int auxState, flapState, buttonState;

float pitchcorrectionav, rollcorrectionav, yawcorrectionav;

// *** Ultrasound
float ultra;
float ultraav = 0;
float baro;
float baroav;
unsigned int ultraLoss;
float ultrathrottle;
float ultraTkOffThrottle;
unsigned int ultraTkOffInput;

unsigned int airborne;
unsigned int landing;
unsigned int throttleHoldOff;
// *** Output stuff
float motorN, motorE, motorS, motorW;
float motorNav, motorEav, motorSav, motorWav;
float tempN;
float tempE;
float tempS;
float tempW;

float pitch_dd;
float roll_dd;

float throttleold;

float RM1;
float RM2;
float RM3;
float RM4;
float RM5;
float RM6;
float RM7;
float RM8;
float RM9;

float DRIFT_MagKp_temp;

unsigned int CFDC_count;

float flpswitch = 0;
int counter;


// *** ARM
unsigned int armed, calib, zeroThrotCounter;
void Arm(void);
void Disarm(void);

// Battery
float batteryVoltage;

float pitchDemandSpin = 0;
float rollDemandSpin = 0;
float pitchDemandSpinold = 0;
float rollDemandSpinold = 0;

// *** Button
unsigned int PRGBlankTimer; // Blanking time for button pushes
unsigned int PRGTimer; // Timer for button pushes, continuously increments as the button is held
unsigned int PRGPushTime; // Contains the time that a button was pushed for, populated after button is released

unsigned int GPS_HOLD = 0;
double targetX, targetY, targetZ;


// ****************************************************************************
// ****************************************************************************
// *** INITIALISATION
// ****************************************************************************
// ****************************************************************************

// *** Initialiser function: sets everything up.
void setup() {

	CFDC_mag_x_zero = 0;
	CFDC_mag_y_zero = 0;
	CFDC_mag_z_zero = 0;
	CFDC_mag_x_error = 0;
	CFDC_mag_y_error = 0;
	CFDC_mag_z_error = 0;
	
	
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
		
		CFDC_count = 0;
    
    // *** Timers and couters6
        rxLoss = 50;
        ultraLoss = ULTRA_OVTH + 1;
        sysMS = 0;
        sysUS = 0;
        SysTickInit();  // SysTick enable (default 1ms)
		
		DRIFT_MagKp_temp = 0;

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
        //UltraFast();
		alt.demandincr =  0;
        
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
	
		pitch_dd = 0;
		roll_dd = 0;
        
        
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
		
		throttleold = 0;
	
        
    // *** Initialise timers and loops
        //PWMInit(PWM_NESW);
		PWMInit(PWM_X | PWM_Y);
		gim_ptemp = 0;
        RITInitms(1000/MESSAGE_LOOP_HZ);
        flashPLED = 0;
        LEDOff(PLED);
		
		lat_diff = 0;
		lon_diff = 0;
		lat_diff_i = 0;
		lon_diff_i = 0;
		lat_diff_d = 0;
		lon_diff_d = 0;
        
        gpsHoldSet = 0;
        gpsHoldX = 0;
        gpsHoldY = 0;
        gpsHoldZ = 0;
		
		airborne = 0;
}

void Arm(void) {

    if(CAL_AUTO > 0) {
        CalibrateGyroTemp(1);
    }
    
    PWMInit(PWM_NESW);
    PWMSetNESW(THROTTLEOFFSET, THROTTLEOFFSET, THROTTLEOFFSET, THROTTLEOFFSET);
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
	
	Xout = fcos(psiAngleinit)*0.000800;
	Yout = fsin(psiAngleinit)*0.000800;	
		
	armed = 1;
    
    ilink_thalstat.sensorStatus &= ~(0x7); // mask status
    ilink_thalstat.sensorStatus |= 4; // active/armed
}

void Disarm(void) {
    if(armed) {
        PWMSetNESW(THROTTLEOFFSET, THROTTLEOFFSET, THROTTLEOFFSET, THROTTLEOFFSET);
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
            
            // Wait for Gyro to be steady or 20 seconds
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

// *** Calibrates all the sensors
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
        
        Gyro.X.error = 0;
        Gyro.Y.error = 0;
        Gyro.Z.error = 0;

        Gyro.X.verterror = 0;
        Gyro.Y.verterror = 0;
        Gyro.Z.verterror = 0;

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
                
                if(distance < 2000) {
                    // low-movement, increment good counter and add average value.
                    good++;
                    Xtotal += Gyro.X.raw;
                    Ytotal += Gyro.Y.raw;
                    Ztotal += Gyro.Z.raw;
                    if(good >= 333) break; // if enough good readings, escape loop
                }
                else {
                    // high movement, zero good counter, and average values.
                    good = 0;
                    
                    Xtotal = 0;
                    Ytotal = 0;
                    Ztotal = 0;
                }
                
                Xav *= 0.95f;
                Xav += 0.05f * (float)Gyro.X.raw;
                Yav *= 0.95f;
                Yav += 0.05f * (float)Gyro.Y.raw;
                Zav *= 0.95f;
                Zav += 0.05f * (float)Gyro.Z.raw; 
                
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

            Gyro.X.offset = (float)Xtotal/(float)((seconds+1) * 333);
            Gyro.Y.offset = (float)Ytotal/(float)((seconds+1) * 333);
            Gyro.Z.offset = (float)Ztotal/(float)((seconds+1) * 333);
            
            flashPLED = 0;

            LEDOff(PLED);
        
            ilink_thalstat.sensorStatus &= ~(0x7); // mask status
            ilink_thalstat.sensorStatus |= 3; // standby
			
			
	
	
        }
}



// ****************************************************************************
// ****************************************************************************
// LOOPS
// ****************************************************************************
// ****************************************************************************
// *** Main loop, nothing much happens in here.
void loop() {
    //if(idleCount < IDLE_MAX) idleCount++; // this is the counter for CPU idle time
    // *** Deal with button push
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

// *** SysTick timer: deals with general timing
void SysTickInterrupt(void) {
    sysMS += 1;
    sysUS += 1000;
    
    // *** deal with flashing LEDs
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
    
    // *** time the button pushes
        if(PRGPoll() == 0) PRGTimer++;
        else {
            PRGPushTime = PRGTimer;
            PRGTimer = 0;
            if(PRGBlankTimer) PRGBlankTimer--;
        }
}

// *** RIT interrupt, deal with timed iLink messages.
void RITInterrupt(void) {
    
    // *** deal with iLink parameter transmission
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



void Timer0Interrupt0() {
    if(++slowSoftscale >= SLOW_DIVIDER) {
        slowSoftscale = 0;
        

// ****************************************************************************
// *** COLLECT INPUT DATA
// ****************************************************************************

        ReadMagSensors();


        // READ RC RX INPUT //
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
			
			
			// Controller's flap switch (mechanically sprung return)
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
				// *** TODO: Perform RC signal loss state setting
				if(armed) PWMSetNESW(THROTTLEOFFSET, THROTTLEOFFSET, THROTTLEOFFSET, THROTTLEOFFSET);
				ilink_outputs0.channel[0] = THROTTLEOFFSET;
				ilink_outputs0.channel[1] = THROTTLEOFFSET;
				ilink_outputs0.channel[2] = THROTTLEOFFSET;
				ilink_outputs0.channel[3] = THROTTLEOFFSET;
				flashVLED = 2;
			}
		}
        
		
		
            
        // READ ULTRASOUND //
            ultra = (UltraGetNewRawData()) * 0.17;
			
            if(ultra > 0)  {
                ultraLoss = 0;
				
				//if(ultra - ultraav > LIM_ULTRA/SLOW_DIVIDER) ultra = ultraav + LIM_ULTRA/SLOW_DIVIDER;
				//if(ultraav - ultra > LIM_ULTRA/SLOW_DIVIDER) ultra = ultraav - LIM_ULTRA/SLOW_DIVIDER;
				
                ultraav *= SPR_ULTRA;
                ultraav += (1-SPR_ULTRA) * ultra;
                
                ilink_altitude.relAlt = ultraav;
                
                alt.valueOld = alt.value;
                alt.value = ultraav;
                
                alt.baroOffset = baroav - ultraav;
			}
            else {
                ultraLoss++;
				if(ultraLoss > ULTRA_OVTH) ultraLoss = ULTRA_OVTH;
            }
			
			
			
			
			
        // READ BATTERY VOLTAGE //
            unsigned short battV = (ADCGet() * 6325) >> 10; // Because the factor is 6325/1024, we can do this in integer maths by right-shifting 10 bits instead of dividing by 1024.
            // battVoltage in milivolts
            batteryVoltage *= 0.99f;
            batteryVoltage += 0.01f * (float)battV;
            ilink_thalstat.battVoltage = battV;
            ADCTrigger(CHN7);
            
    }

	
	ReadAccelSensors();
	ReadGyroSensors();
    
    
// ****************************************************************************
// *** ATTITUDE HEADING REFERENCE SYSTEM
// ****************************************************************************
	
	// CREATE THE MEASURED ROTATION MATRIX //
	//Normalise
	float sumsqu = finvSqrt(Mag.X.value*Mag.X.value + Mag.Y.value*Mag.Y.value+ Mag.Z.value*Mag.Z.value); // Magnetoerometr data is normalised so no need to convert units.
        Mag.X.value = Mag.X.value * sumsqu;
        Mag.Y.value = Mag.Y.value * sumsqu;
        Mag.Z.value = Mag.Z.value * sumsqu;
		
	sumsqu = finvSqrt((float)Accel.X.value*(float)Accel.X.value + (float)Accel.Y.value*(float)Accel.Y.value + (float)Accel.Z.value*(float)Accel.Z.value); // Accelerometr data is normalised so no need to convert units.
        Accel.X.value = (float)Accel.X.value * sumsqu;
        Accel.Y.value = (float)Accel.Y.value * sumsqu;
        Accel.Z.value = (float)Accel.Z.value * sumsqu;
	
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
	float g1 = (Gyro.X.value - Gyro.X.bias*DRIFT_AccelKp)/(float)FAST_RATE;
    float g2 = (Gyro.Y.value - Gyro.Y.bias*DRIFT_AccelKp)/(float)FAST_RATE;
    float g3 = (Gyro.Z.value - Gyro.Z.bias*DRIFT_MagKp_temp)/(float)FAST_RATE;
	
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
    // ilink_attitude.roll = phiAngle;  // temporary
    // ilink_attitude.pitch = thetaAngle;
    // ilink_attitude.yaw = psiAngle;
	
	
	
	
// ****************************************************************************
// *** STATE MACHINE
// ****************************************************************************
	
	float pitcherror, rollerror, yawerror;
	float pitchcorrection, rollcorrection, yawcorrection;
	
	///////////////////// ACROBATIC MODE ///////////////////////////////////
	if (MODE_ST == 0) {

	}
		
	///////////////////////// REGULAR MODE ///////////////////////////////////
	if (MODE_ST == 1) { 
	
		DRIFT_MagKp_temp = 0; // We don't use Magnetometer feedback in regular mode

	
		// Arm, Disarm and Calibrate
		if(rcInput[RX_THRO] - throttletrim <  OFFSTICK && rxFirst != 0) {
            if(rxLoss < 50) {
                if(rcInput[RX_AILE] < MAXTHRESH && rcInput[RX_AILE] > MINTHRESH) {
                    if(rcInput[RX_ELEV] > MAXTHRESH) {
                        // arm
                        if(zeroThrotCounter++ > ZEROTHROTMAX) {
                            zeroThrotCounter = 0;

                            Arm();
                        }
                    }
                    else if(rcInput[RX_ELEV] < MINTHRESH) {
                        // disarm
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
                        // calib gyro    
                        if(zeroThrotCounter++ > ZEROTHROTMAX) {
                            zeroThrotCounter = 0;
                            CalibrateGyro();
                        }
                    }
                    else if(rcInput[RX_AILE] < MINTHRESH) {
                        // calib magneto
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
			pitch.demandtemp = -((float)MIDSTICK - (float)rcInput[RX_ELEV])*PITCH_SENS; 
            roll.demandtemp = ((float)MIDSTICK - (float)rcInput[RX_AILE])*ROLL_SENS;
			float tempf = -(float)(yawtrim - rcInput[RX_RUDD])*YAW_SENS; 
			
			float limiter;
			
			limiter = LIM_RATE/(float)FAST_RATE;
            
            
            if(pitch.demandtemp > pitch.demand) {
                if(pitch.demandtemp - pitch.demand > limiter && pitch.demand > 0) pitch.demand = pitch.demand + limiter;
                else pitch.demand = pitch.demandtemp;
            }
            else {
                if(pitch.demand - pitch.demandtemp > limiter && pitch.demand < 0) pitch.demand = pitch.demand - limiter;
                else pitch.demand = pitch.demandtemp;
            }
            
            if(roll.demandtemp > roll.demand) {
                if(roll.demandtemp - roll.demand > limiter && roll.demand > 0) roll.demand = roll.demand + limiter;
                else roll.demand = roll.demandtemp;
            }
            else {
                if(roll.demand - roll.demandtemp > limiter && roll.demand < 0) roll.demand = roll.demand - limiter;
                else roll.demand = roll.demandtemp;
            }
			
			throttle = rcInput[RX_THRO] - throttletrim;
			if (throttle < 0) throttle = 0;
			
			pitchDemandSpin = fsin(0.78539816339744830962+M_PI_2)*pitch.demand - fsin(0.78539816339744830962)*roll.demand;
			rollDemandSpin =  fsin(0.78539816339744830962)*pitch.demand + fsin(0.78539816339744830962+M_PI_2)*roll.demand;
			  
			
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
	
	
	//////////////////////////////// SIMPLICITY MODE //////////////////////////////////////////
	if (MODE_ST == 2) { 
	
		DRIFT_MagKp_temp = DRIFT_MagKp;
			
			
		// Arm, Disarm and Calibrate
		if(rcInput[RX_THRO] - throttletrim <  OFFSTICK && rxFirst != 0) {
			if(rxLoss < 50) {
				if(rcInput[RX_AILE] < MAXTHRESH && rcInput[RX_AILE] > MINTHRESH) {
					if(rcInput[RX_ELEV] > MAXTHRESH) {
						// arm
						if(zeroThrotCounter++ > ZEROTHROTMAX) {
							zeroThrotCounter = 0;

							Arm();
						}
					}
					else if(rcInput[RX_ELEV] < MINTHRESH) {
						// disarm
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
						// calib gyro    
						if(zeroThrotCounter++ > ZEROTHROTMAX) {
							zeroThrotCounter = 0;
							CalibrateGyro();
						}
					}
					else if(rcInput[RX_AILE] < MINTHRESH) {
						// calib magneto
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

			// This section of code applies some throttle increase with high tilt angles
			float M9temp;
			if (M9 > 0) M9temp = M9;
			else M9temp = -M9;
			throttle_angle = ((throttle / M9temp) - throttle); 
			if (throttle_angle < 0) throttle_angle = 0;
			
			pitch.demandtemp = -((float)MIDSTICK - (float)rcInput[RX_ELEV])*PITCH_SENS; 
			roll.demandtemp = ((float)MIDSTICK - (float)rcInput[RX_AILE])*ROLL_SENS;
			float tempf = -(float)(yawtrim - rcInput[RX_RUDD])*YAW_SENS; 
			
			if (CFDC_count == 0) {
				pitch.demandtemp = 0; 
				roll.demandtemp = 0;
				float tempf = 0.0; 
			}
			
			float limiter;
			
			limiter = LIM_RATE/(float)FAST_RATE;
			
			
			if(pitch.demandtemp > pitch.demand) {
				if(pitch.demandtemp - pitch.demand > limiter && pitch.demand > 0) pitch.demand = pitch.demand + limiter;
				else pitch.demand = pitch.demandtemp;
			}
			else {
				if(pitch.demand - pitch.demandtemp > limiter && pitch.demand < 0) pitch.demand = pitch.demand - limiter;
				else pitch.demand = pitch.demandtemp;
			}
			
			if(roll.demandtemp > roll.demand) {
				if(roll.demandtemp - roll.demand > limiter && roll.demand > 0) roll.demand = roll.demand + limiter;
				else roll.demand = roll.demandtemp;
			}
			else {
				if(roll.demand - roll.demandtemp > limiter && roll.demand < 0) roll.demand = roll.demand - limiter;
				else roll.demand = roll.demandtemp;
			}
			
			throttle = rcInput[RX_THRO] - throttletrim;
			if (throttle < 0) throttle = 0;
					
			// SIMPLICITY MODE //	
			pitchDemandSpin = fsin(-psiAngle+psiAngleinit+M_PI_2)*pitch.demand - fsin(-psiAngle+psiAngleinit)*roll.demand;
			rollDemandSpin = fsin(-psiAngle+psiAngleinit)*pitch.demand + fsin(-psiAngle+psiAngleinit+M_PI_2)*roll.demand;
			
			


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
		
	

	
 
 
 
	//////////////////////////////// EASY MODE //////////////////////////////////////////
	if (MODE_ST == 3) { 
	

		DRIFT_MagKp_temp = DRIFT_MagKp;
										
		// Arm, Disarm and Calibrate
		if(rcInput[RX_THRO] - throttletrim <  OFFSTICK && rxFirst != 0) {
            if(rxLoss < 50) {
                if(rcInput[RX_AILE] < MAXTHRESH && rcInput[RX_AILE] > MINTHRESH) {
                    if(rcInput[RX_ELEV] > MAXTHRESH) {
                        // arm
                        if(zeroThrotCounter++ > ZEROTHROTMAX) {
                            zeroThrotCounter = 0;

                            Arm();
                        }
                    }
                    else if(rcInput[RX_ELEV] < MINTHRESH) {
                        // disarm
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
                        // calib gyro    
                        if(zeroThrotCounter++ > ZEROTHROTMAX) {
                            zeroThrotCounter = 0;
                            CalibrateGyro();
                        }
                    }
                    else if(rcInput[RX_AILE] < MINTHRESH) {
                        // calib magneto
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
		
			// This section of code applies some throttle increase with high tilt angles
			float M9temp;
			if (M9 > 0) M9temp = M9;
			else M9temp = -M9;
			throttle_angle = ((throttle / M9temp) - throttle); 
			if (throttle_angle < 0) throttle_angle = 0;
			
			
			pitch.demandtemp = -((float)MIDSTICK - (float)rcInput[RX_ELEV])*PITCH_SENS; 
            roll.demandtemp = ((float)MIDSTICK - (float)rcInput[RX_AILE])*ROLL_SENS;
			float tempf = -(float)(yawtrim - rcInput[RX_RUDD])*YAW_SENS;
			
			if (CFDC_count == 0) {
					pitch.demandtemp = 0; 
					roll.demandtemp = 0;
					float tempf = 0.0; 
			}
			
			float limiter;
			
			limiter = LIM_RATE/(float)FAST_RATE;
            
            
            if(pitch.demandtemp > pitch.demand) {
                if(pitch.demandtemp - pitch.demand > limiter && pitch.demand > 0) pitch.demand = pitch.demand + limiter;
                else pitch.demand = pitch.demandtemp;
            }
            else {
                if(pitch.demand - pitch.demandtemp > limiter && pitch.demand < 0) pitch.demand = pitch.demand - limiter;
                else pitch.demand = pitch.demandtemp;
            }
            
            if(roll.demandtemp > roll.demand) {
                if(roll.demandtemp - roll.demand > limiter && roll.demand > 0) roll.demand = roll.demand + limiter;
                else roll.demand = roll.demandtemp;
            }
            else {
                if(roll.demand - roll.demandtemp > limiter && roll.demand < 0) roll.demand = roll.demand - limiter;
                else roll.demand = roll.demandtemp;
            }
			
			// SIMPLICITY MODE //	
			pitchDemandSpin = fsin(-psiAngle+psiAngleinit+M_PI_2)*pitch.demand - fsin(-psiAngle+psiAngleinit)*roll.demand;
			rollDemandSpin = fsin(-psiAngle+psiAngleinit)*pitch.demand + fsin(-psiAngle+psiAngleinit+M_PI_2)*roll.demand;
			
			
			// ULTRASOUND ALTITUDE HOLD//
			if(airborne == 0) { // If craft is not airbourne
		
				//if (((rcInput[RX_THRO] - throttletrim - MIDSTICK) < 50) && ((rcInput[RX_THRO] - throttletrim - MIDSTICK) > -50)) { // and the throttle stick is in the middle
				if (((rcInput[RX_THRO] - throttletrim) > 320) && ((rcInput[RX_THRO] - throttletrim) < 420)) {
					if (flapState == 1) { // Auto take off when the button is pressed
						if(throttle < 380) {
							throttle += 0.9;
						}
						throttle += 0.1;
					}

					alt.demandincr =  0;	
				}
										
			}
			else {
				if((flapState == 0) && (airborne == 1)) { // If the button is pressed while airborne
					alt.demandincr -= 1; // land
				}
			}
			
			if((alt.value > ULTRA_TKOFF) && (airborne == 0)) { 
				// just taken off, set airborne to 1 and remember takeoff throttle
				airborne = 1;
				alt.demandincr = 0;				
				ultraTkOffThrottle = throttle;
				ultraTkOffInput = ((float)rcInput[RX_THRO] - (float) throttletrim);
			}
		
			if ((ultra > 0) && (ultra < ULTRA_LND) && (airborne == 1)) {
				// If valid ultrasound reading and reading is less than landing threshold and we are airborne
				// then increase landing counter
				landing++;
				
				//If the above is satisfied consecutively more than ULTRA_DTCT times then shut down the motors
				if(landing > ULTRA_DTCT) { 
					airborne = 0;
					throttleHoldOff = 1;
					throttle = 0;
					alt.demandincr =  0;
					ultraerror = 0;
					
				}
				
			}	
			// If consecutive run of readings is broken, reset the landing counter.
			else {
				landing = 0;
			}			
			
			// Create a deadzone around throttle at lift off, if new stick position is higher than this deadzone then increase altitude demand and vice versa		
			if (((((float)rcInput[RX_THRO] - (float) throttletrim) - ultraTkOffInput) > ULTRA_DEAD) || ((((float)rcInput[RX_THRO] - (float) throttletrim) - ultraTkOffInput) < -ULTRA_DEAD)) {
				// if  ((((float)rcInput[RX_THRO] - (float) throttletrim) - ultraTkOffInput)*(ULTRA_DRMP/FAST_RATE) > LIM_ULTRA ) alt.demandincr =  alt.demandincr + LIM_ULTRA;
				// else if  ((((float)rcInput[RX_THRO] - (float) throttletrim) - ultraTkOffInput)*(ULTRA_DRMP/FAST_RATE) < (-LIM_ULTRA/2) ) alt.demandincr =  alt.demandincr - LIM_ULTRA/2;
				//else alt.demandincr =  alt.demandincr + (((float)rcInput[RX_THRO] - (float) throttletrim) - ultraTkOffInput)*(ULTRA_DRMP/FAST_RATE);
			 	alt.demandincr =  alt.demandincr + (((float)rcInput[RX_THRO] - (float) throttletrim) - ultraTkOffInput)*(ULTRA_DRMP/FAST_RATE);
			}
		
			
			if(alt.demandincr < -LIM_ALT) alt.demandincr = -LIM_ALT;
			if(alt.demandincr > LIM_ALT) alt.demandincr = LIM_ALT; 
			
			alt.demand = ULTRA_TKOFF + ULTRA_OFFSET + alt.demandincr;    
			alt.error = alt.demand - alt.value;
			alt.derivative = alt.value - alt.valueOld;
			
			
			
			alt.integral *= ULTRA_De;
			if(airborne == 1) {
					alt.integral += alt.error;
			}
			
			if (((ultraLoss >= ULTRA_OVTH)) && (ultraerror == 0)) {   // when consecutive ultrasound altitude data losses rise above ULTRA_OVTH, throttle level is reduced

				
				ultrathrottle -= 10;
				ultraerror = 1;			
			}
			
			else if (ultraerror == 1) {
				if (throttle > 380)  ultrathrottle -= ULTRA_OVDEC;  //Decays throttle to ensure craft returns to ground level	


			}
			else {
			ultrathrottle = alt.derivative * -ULTRA_Kd;
			ultrathrottle += ULTRA_Kp*alt.error;
			ultrathrottle += ULTRA_Ki*alt.integral;
			}
			
			if (airborne == 1){
				throttle = ultraTkOffThrottle + ultrathrottle;
			}
			
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
	
	
	
	
	//////////////////////////////// GPS HOLD MODE //////////////////////////////////////////
	if (MODE_ST == 4) { 
	
		
		
		DRIFT_MagKp_temp = DRIFT_MagKp;
		
		
					
			
		// Arm, Disarm and Calibrate
		if(rcInput[RX_THRO] - throttletrim <  OFFSTICK && rxFirst != 0) {
			if(rxLoss < 50) {
				if(rcInput[RX_AILE] < MAXTHRESH && rcInput[RX_AILE] > MINTHRESH) {
					if(rcInput[RX_ELEV] > MAXTHRESH) {
						// arm
						if(zeroThrotCounter++ > ZEROTHROTMAX) {
							zeroThrotCounter = 0;

							Arm();
						}
					}
					else if(rcInput[RX_ELEV] < MINTHRESH) {
						// disarm
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
						// calib gyro    
						if(zeroThrotCounter++ > ZEROTHROTMAX) {
							zeroThrotCounter = 0;
							CalibrateGyro();
						}
					}
					else if(rcInput[RX_AILE] < MINTHRESH) {
						// calib magneto
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
			
			alt_diff = 0;
			alt_diff_i = 0;	
			alt_diff_d = 0;
			
			lat_diff = 0;
			lat_diff_i = 0;	
			lat_diff_d = 0;
			
			lon_diff = 0;
			lon_diff_i = 0;	
			lon_diff_d = 0;
			
		}
		
		
		// Flight Demands
		else {						
			
			if (auxState == 0) {
			
				// This section of code applies some throttle increase with high tilt angles
				float M9temp;
				if (M9 > 0) M9temp = M9;
				else M9temp = -M9;
				throttle_angle = ((throttle / M9temp) - throttle); 
				if (throttle_angle < 0) throttle_angle = 0;
				
				pitch.demandtemp = -((float)MIDSTICK - (float)rcInput[RX_ELEV])*PITCH_SENS; 
				roll.demandtemp = ((float)MIDSTICK - (float)rcInput[RX_AILE])*ROLL_SENS;
				float tempf = -(float)(yawtrim - rcInput[RX_RUDD])*YAW_SENS; 
				
				if (CFDC_count == 0) {
					pitch.demandtemp = 0; 
					roll.demandtemp = 0;
					float tempf = 0.0; 
				}
				
				float limiter;
				
				limiter = LIM_RATE/(float)FAST_RATE;
				
				
				if(pitch.demandtemp > pitch.demand) {
					if(pitch.demandtemp - pitch.demand > limiter && pitch.demand > 0) pitch.demand = pitch.demand + limiter;
					else pitch.demand = pitch.demandtemp;
				}
				else {
					if(pitch.demand - pitch.demandtemp > limiter && pitch.demand < 0) pitch.demand = pitch.demand - limiter;
					else pitch.demand = pitch.demandtemp;
				}
				
				if(roll.demandtemp > roll.demand) {
					if(roll.demandtemp - roll.demand > limiter && roll.demand > 0) roll.demand = roll.demand + limiter;
					else roll.demand = roll.demandtemp;
				}
				else {
					if(roll.demand - roll.demandtemp > limiter && roll.demand < 0) roll.demand = roll.demand - limiter;
					else roll.demand = roll.demandtemp;
				}
				
				throttle = rcInput[RX_THRO] - throttletrim;
				if (throttle < 0) throttle = 0;
						
				// SIMPLICITY MODE //	
				pitchDemandSpin = fsin(-psiAngle+psiAngleinit+M_PI_2)*pitch.demand - fsin(-psiAngle+psiAngleinit)*roll.demand;
				rollDemandSpin = fsin(-psiAngle+psiAngleinit)*pitch.demand + fsin(-psiAngle+psiAngleinit+M_PI_2)*roll.demand;
			

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

						
				GPS_HOLD = 0;
				
			}
			
			if (auxState == 1) {
			
					// This section of code applies some throttle increase with high tilt angles
					float M9temp;
					if (M9 > 0) M9temp = M9;
					else M9temp = -M9;
					throttle_angle = ((throttle / M9temp) - throttle); 
					if (throttle_angle < 0) throttle_angle = 0;
				
				
					if(ilink_position.isNew) {
                    ilink_position.isNew = 0;
					
					if (GPS_HOLD == 0) {
						alt_throttle = throttle;
						targetX = ((ilink_position.craftX / 10000000.0) + 51.0);
						targetY = ilink_position.craftY;
						targetZ = ilink_position.craftZ;
						GPS_HOLD = 1;
					}
					
			
				
					
					
					
		  
					lat_diff = (double)(targetX - ((ilink_position.craftX / 10000000.0) + 51.0)) * (double)111194.92664455873734580834; // 111194.92664455873734580834f is radius of earth in metres and deg-rad conversion: 6371000*PI()/180
					lon_diff = (double)(targetY - ilink_position.craftY) * (double)111194.92664455873734580834 * fcos((((ilink_position.craftX / 10000000.0) + 51.0) * (double)0.01745329251994329577)); // 0.01745329251994329577f is deg-rad conversion PI()/180

					
					
					lat_diff_d = lat_diff - lat_diff_old;
					lon_diff_d = lon_diff - lon_diff_old;
					
					
					
					lat_diff_old = lat_diff;
					lon_diff_old = lon_diff;
					
				
				
				
					// if(targetZ - oldZtarget > LIM_GPSALV) oldZtarget += LIM_GPSALV;
					// else if(targetZ - oldZtarget < -LIM_GPSALV) oldZtarget -= LIM_GPSALV;
					// else oldZtarget = targetZ;
					
					alt_diff = (double)(targetZ - ilink_position.craftZ);
				
				
					alt_diff_d = alt_diff - alt_diff_old;
					alt_diff_old = alt_diff;
							
							
					lat_diff_i += lat_diff;
					lon_diff_i += lon_diff;
					lat_diff_i *= GPS_De;
					lon_diff_i *= GPS_De;
						
				}

				
				
				
			
				
				
				pitch.demand = GPS_Kp*lat_diff + GPS_Ki*lat_diff_i + GPS_Kd*lat_diff_d;
				roll.demand = GPS_Kp*lon_diff + GPS_Ki*lon_diff_i + GPS_Kd*lon_diff_d;
					
				pitchDemandSpin = fsin(-psiAngle+M_PI_2)*pitch.demand - fsin(-psiAngle)*roll.demand;
				rollDemandSpin = fsin(-psiAngle)*pitch.demand + fsin(-psiAngle+M_PI_2)*roll.demand;
					
				alt_diff_i += alt_diff;
				alt_diff_i *= GPS_ALTDe;                    

				gpsThrottle = GPS_ALTKp * alt_diff + GPS_ALTKi * alt_diff_i + GPS_ALTKd * alt_diff_d + alt_throttle;
				throttle = gpsThrottle;		
				
				if ((throttle - throttleold) > GPS_THRLM) throttle = throttle + GPS_THRLM;
				if ((throttle - throttleold) < -GPS_THRLM) throttle = throttle - GPS_THRLM;
				
				if (throttle < 380) throttle = 380;
			
				throttleold = throttle;
				
				if ((pitchDemandSpin - pitchDemandSpinold) > PITCH_SPL) pitchDemandSpin = pitchDemandSpinold + PITCH_SPL;
				if ((pitchDemandSpin - pitchDemandSpinold) < -PITCH_SPL) pitchDemandSpin = pitchDemandSpinold - PITCH_SPL;
				
				pitchDemandSpinold = pitchDemandSpin;
				
				if ((rollDemandSpin - rollDemandSpinold) > ROLL_SPL) rollDemandSpin = rollDemandSpinold + ROLL_SPL;
				if ((rollDemandSpin - rollDemandSpinold) < -ROLL_SPL) rollDemandSpin = rollDemandSpinold - ROLL_SPL;
				
				rollDemandSpinold = rollDemandSpin;
				
			}
		 
		}

	}	

	
	/////////////////////////////// AUTONOMOUS MODE 1 (Up and Down) /////////////////////////////////////
	if (MODE_ST == 5) { 
	

		DRIFT_MagKp_temp = DRIFT_MagKp;

			
		// Arm, Disarm and Calibrate
		if(rcInput[RX_THRO] - throttletrim <  OFFSTICK && rxFirst != 0) {
			if(rxLoss < 50) {
				if(rcInput[RX_AILE] < MAXTHRESH && rcInput[RX_AILE] > MINTHRESH) {
					if(rcInput[RX_ELEV] > MAXTHRESH) {
						// arm
						if(zeroThrotCounter++ > ZEROTHROTMAX) {
							zeroThrotCounter = 0;

							Arm();
						}
					}
					else if(rcInput[RX_ELEV] < MINTHRESH) {
						// disarm
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
						// calib gyro    
						if(zeroThrotCounter++ > ZEROTHROTMAX) {
							zeroThrotCounter = 0;
							CalibrateGyro();
						}
					}
					else if(rcInput[RX_AILE] < MINTHRESH) {
						// calib magneto
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
			
			alt_diff = 0;
			alt_diff_i = 0;	
			alt_diff_d = 0;
			
			lat_diff = 0;
			lat_diff_i = 0;	
			lat_diff_d = 0;
			
			lon_diff = 0;
			lon_diff_i = 0;	
			lon_diff_d = 0;
			
			flapState = 0;
			targetX = 0;
			targetY = 0;
			targetZ = 0;
			switchover_count = 0;


		}
		
		
		// Flight Demands
		else {						
			
			if (auxState == 0) {
			
				// This section of code applies some throttle increase with high tilt angles
				float M9temp;
				if (M9 > 0) M9temp = M9;
				else M9temp = -M9;
				throttle_angle = ((throttle / M9temp) - throttle); 
				if (throttle_angle < 0) throttle_angle = 0;
			
				pitch.demandtemp = -((float)MIDSTICK - (float)rcInput[RX_ELEV])*PITCH_SENS; 
				roll.demandtemp = ((float)MIDSTICK - (float)rcInput[RX_AILE])*ROLL_SENS;
				float tempf = -(float)(yawtrim - rcInput[RX_RUDD])*YAW_SENS; 
				
				if (CFDC_count == 0) {
					pitch.demandtemp = 0; 
					roll.demandtemp = 0;
					float tempf = 0.0; 
				}
				
				float limiter;
				
				limiter = LIM_RATE/(float)FAST_RATE;
				
				
				if(pitch.demandtemp > pitch.demand) {
					if(pitch.demandtemp - pitch.demand > limiter && pitch.demand > 0) pitch.demand = pitch.demand + limiter;
					else pitch.demand = pitch.demandtemp;
				}
				else {
					if(pitch.demand - pitch.demandtemp > limiter && pitch.demand < 0) pitch.demand = pitch.demand - limiter;
					else pitch.demand = pitch.demandtemp;
				}
				
				if(roll.demandtemp > roll.demand) {
					if(roll.demandtemp - roll.demand > limiter && roll.demand > 0) roll.demand = roll.demand + limiter;
					else roll.demand = roll.demandtemp;
				}
				else {
					if(roll.demand - roll.demandtemp > limiter && roll.demand < 0) roll.demand = roll.demand - limiter;
					else roll.demand = roll.demandtemp;
				}
				
				throttle = rcInput[RX_THRO] - throttletrim;
				if (throttle < 0) throttle = 0;
						
				// SIMPLICITY MODE //	
				pitchDemandSpin = fsin(-psiAngle+psiAngleinit+M_PI_2)*pitch.demand - fsin(-psiAngle+psiAngleinit)*roll.demand;
				rollDemandSpin = fsin(-psiAngle+psiAngleinit)*pitch.demand + fsin(-psiAngle+psiAngleinit+M_PI_2)*roll.demand;
			

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

				flapState = 0;
				targetX = 0;
				targetY = 0;
				targetZ = 0;
				switchover_count = 0;
				
			}
			
			if (auxState == 1) {
			
				// This section of code applies some throttle increase with high angles
				float M9temp;
				if (M9 > 0) M9temp = M9;
				else M9temp = -M9;
				throttle_angle = ((throttle / M9temp) - throttle); 
				if (throttle_angle < 0) throttle_angle = 0;
				

				// ULTRASOUND ALTITUDE HOLD//
				if(airborne == 0) { // If craft is not airbourne
			
					switchover_count = 0;
			
					//if (((rcInput[RX_THRO] - throttletrim - MIDSTICK) < 50) && ((rcInput[RX_THRO] - throttletrim - MIDSTICK) > -50)) { // and the throttle stick is in the middle
					if (((rcInput[RX_THRO] - throttletrim) > 320) && ((rcInput[RX_THRO] - throttletrim) < 420)) {
						ilink_attitude.roll = flapState;  // temporary
						
						if (flapState == 1) { // Auto take off when the button is pressed
							if(throttle < 380) {
								throttle += 0.9;
							}
							throttle += 0.1;
						}

						alt.demandincr =  0;	
					}
				}						
			
			
			
				if((alt.value > ULTRA_TKOFF) && (airborne == 0)) { 
					// just taken off, set airborne to 1 and remember takeoff throttle
					airborne = 1;
					alt.demandincr = 0;
					ultraTkOffThrottle = throttle;
					ultraTkOffInput = ((float)rcInput[RX_THRO] - (float) throttletrim);				
					targetX = ((ilink_position.craftX / 10000000.0) + 51.0);
					targetY = ilink_position.craftY;
				}
			
				if ((ultra > 0) && (ultra < ULTRA_LND) && (airborne == 1)) {
					// If valid ultrasound reading and reading is less than landing threshold and we are airborne
					// then increase landing counter
					landing++;
					
					//If the above is satisfied consecutively more than ULTRA_DTCT times then shut down the motors
					if(landing > ULTRA_DTCT) { 
						airborne = 0;
						throttleHoldOff = 1;
						throttle = 0;
						alt.demandincr =  0;
						ultraerror = 0;
						
					}
					
				}	
				// If consecutive run of readings is broken, reset the landing counter.
				else {
					landing = 0;
				}			
				
				if (switchover_count < 40) {
					alt.demand = ULTRA_TKOFF + ULTRA_OFFSET + alt.demandincr;    
					alt.error = alt.demand - alt.value;
					alt.derivative = alt.value - alt.valueOld;
					
					
					
					alt.integral *= ULTRA_De;
					if(airborne == 1) {
							alt.integral += alt.error;
					}
					
					if (((ultraLoss >= ULTRA_OVTH)) && (ultraerror == 0)) {   // when consecutive ultrasound altitude data losses rise above ULTRA_OVTH, throttle level is reduced	
						ultrathrottle -= 10;
						ultraerror = 1;			
					}
					
					else if (ultraerror == 1) {
						if (throttle > 380)  ultrathrottle -= ULTRA_OVDEC;  //Decays throttle to ensure craft returns to ground level	
					}
					else {
						ultrathrottle = alt.derivative * -ULTRA_Kd;
						ultrathrottle += ULTRA_Kp*alt.error;
						ultrathrottle += ULTRA_Ki*alt.integral;
					}
					
					if (airborne == 1) {
						throttle = ultraTkOffThrottle + ultrathrottle;
						switchover_count++;
					}
				}
					
				if (switchover_count == 39) {
					alt_throttle = throttle;
					targetZ = ilink_position.craftZ;
					gpsHoldZ = ilink_position.craftZ;
					switchover_count = 40;
						
				}
				
				if (airborne == 1) {
				
					if(ilink_position.isNew) {
						ilink_position.isNew = 0;
						
						lat_diff = (double)(targetX - ((ilink_position.craftX / 10000000.0) + 51.0)) * (double)111194.92664455873734580834; // 111194.92664455873734580834f is radius of earth in metres and deg-rad conversion: 6371000*PI()/180
						lon_diff = (double)(targetY - ilink_position.craftY) * (double)111194.92664455873734580834 * fcos(((ilink_position.craftX / 10000000.0) + 51.0) *(double)0.01745329251994329577); // 0.01745329251994329577f is deg-rad conversion PI()/180

						
						
						ilink_attitude.pitch = targetY; //temporary
						ilink_attitude.yaw =  ilink_position.craftY;
						
						if (switchover_count >= 40) {
							alt_diff = (double)(targetZ - ilink_position.craftZ);
						}
						
						if (switchover_count == 40) targetZ += 0.2;
							
							
						
						if ( (switchover_count == 40 ) && (((gpsHoldZ + 20) - ilink_position.craftZ) > -3) && (((gpsHoldZ + 20) - ilink_position.craftZ) < 3)) {
							switchover_count = 43;
							
						}
						
						
						if (switchover_count == 43) targetZ -= 0.05;

						
							
						
						lat_diff_d = lat_diff - lat_diff_old;
						lon_diff_d = lon_diff - lon_diff_old;
											
						
						lat_diff_old = lat_diff;
						lon_diff_old = lon_diff;
						
						
						// if(targetZ - oldZtarget > LIM_GPSALV) oldZtarget += LIM_GPSALV;
						// else if(targetZ - oldZtarget < -LIM_GPSALV) oldZtarget -= LIM_GPSALV;
						// else oldZtarget = targetZ;

						
						alt_diff_d = alt_diff - alt_diff_old;
						alt_diff_old = alt_diff;
								
								
						lat_diff_i += lat_diff;
						lon_diff_i += lon_diff;
						lat_diff_i *= GPS_De;
						lon_diff_i *= GPS_De;
							
					}

					if ((switchover_count == 43) && (ultraLoss == 0) && (alt.value < 1000)) {
							switchover_count = 44;
							alt.integral = 0;
							ultraTkOffThrottle = throttle;
							alt.demand = alt.value;
					}
					
					if (switchover_count == 44) {
						alt.demand -= 0.2;    
						alt.error = alt.demand - alt.value;
						alt.derivative = alt.value - alt.valueOld;
						
						
						
						alt.integral *= ULTRA_De;
						if(airborne == 1) {
								alt.integral += alt.error;
						}
						
						if (((ultraLoss >= ULTRA_OVTH)) && (ultraerror == 0)) {   // when consecutive ultrasound altitude data losses rise above ULTRA_OVTH, throttle level is reduced	
							ultrathrottle -= 1;
							ultraerror = 1;			
						}
						
						else if (ultraerror == 1) {
							if (throttle > 380)  ultrathrottle -= ULTRA_OVDEC;  //Decays throttle to ensure craft returns to ground level	
						}
						else {
							ultrathrottle = alt.derivative * -ULTRA_Kd;
							ultrathrottle += ULTRA_Kp*alt.error;
							ultrathrottle += ULTRA_Ki*alt.integral;
						}
						
						
						throttle = ultraTkOffThrottle + ultrathrottle;
						
					}
					
					// if ((lat_diff_d * GPS_Kd) > 0.2) lat_diff_d = (0.2 / GPS_Kd);
					// if ((lat_diff_d * GPS_Kd) < -0.2) lat_diff_d = (-0.2 / GPS_Kd);
					// if ((lon_diff_d * GPS_Kd) > 0.2) lon_diff_d = (0.2 / GPS_Kd);
					// if ((lon_diff_d * GPS_Kd) < -0.2) lon_diff_d = (-0.2 / GPS_Kd);
					
					pitch.demand = GPS_Kp*lat_diff + GPS_Ki*lat_diff_i + GPS_Kd*lat_diff_d;
					roll.demand = GPS_Kp*lon_diff + GPS_Ki*lon_diff_i + GPS_Kd*lon_diff_d;
						
					pitchDemandSpin = fsin(-psiAngle+M_PI_2)*pitch.demand - fsin(-psiAngle)*roll.demand;
					rollDemandSpin = fsin(-psiAngle)*pitch.demand + fsin(-psiAngle+M_PI_2)*roll.demand;
						
					alt_diff_i += alt_diff;
					alt_diff_i *= GPS_ALTDe;                    
					
					if ((switchover_count >= 40) && (switchover_count < 44)) {
					gpsThrottle = GPS_ALTKp * alt_diff + GPS_ALTKi * alt_diff_i + GPS_ALTKd * alt_diff_d + alt_throttle;
					throttle = gpsThrottle;	
					}				
					
					if ((throttle - throttleold) > GPS_THRLM) throttle = throttle + GPS_THRLM;
					if ((throttle - throttleold) < -GPS_THRLM) throttle = throttle - GPS_THRLM;
					
					if (throttle < 380) throttle = 380;
				
					throttleold = throttle;
													
					
					if ((pitchDemandSpin - pitchDemandSpinold) > PITCH_SPL) pitchDemandSpin = pitchDemandSpinold + PITCH_SPL;
					if ((pitchDemandSpin - pitchDemandSpinold) < -PITCH_SPL) pitchDemandSpin = pitchDemandSpinold - PITCH_SPL;
					
					pitchDemandSpinold = pitchDemandSpin;
					
					if ((rollDemandSpin - rollDemandSpinold) > ROLL_SPL) rollDemandSpin = rollDemandSpinold + ROLL_SPL;
					if ((rollDemandSpin - rollDemandSpinold) < -ROLL_SPL) rollDemandSpin = rollDemandSpinold - ROLL_SPL;
					
					rollDemandSpinold = rollDemandSpin;
					
				}
			 
			}

			

			
		}
		

	} 
	
	
	
/////////////////////////////// AUTONOMOUS MODE 2 (Up and Out to One Waypoint, back and land.) /////////////////////////////////////
	if (MODE_ST == 6) { 
	
		
		DRIFT_MagKp_temp = DRIFT_MagKp;
		throttle_angle = 0;

		lat_diff = (double)(51.592882 - ((ilink_position.craftX/10000000.0)+51.0)) * (double)111194.92664455873734580834; // 111194.92664455873734580834f is radius of earth in metres and deg-rad conversion: 6371000*PI()/180
		lon_diff = (double)(-1.118982 - ilink_position.craftY) * (double)111194.92664455873734580834 * fcos(((ilink_position.craftX/10000000.0)+51.0) *(double)0.01745329251994329577); // 0.01745329251994329577f is deg-rad conversion PI()/180

		ilink_attitude.roll = lat_diff; // temporary
		ilink_attitude.pitch = lon_diff; // temporary
		ilink_attitude.yaw = psiAngle; // temporary
		
			
		// Arm, Disarm and Calibrate
		if(rcInput[RX_THRO] - throttletrim <  OFFSTICK && rxFirst != 0) {
			if(rxLoss < 50) {
				if(rcInput[RX_AILE] < MAXTHRESH && rcInput[RX_AILE] > MINTHRESH) {
					if(rcInput[RX_ELEV] > MAXTHRESH) {
						// arm
						if(zeroThrotCounter++ > ZEROTHROTMAX) {
							zeroThrotCounter = 0;

							Arm();
						}
					}
					else if(rcInput[RX_ELEV] < MINTHRESH) {
						// disarm
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
						// calib gyro    
						if(zeroThrotCounter++ > ZEROTHROTMAX) {
							zeroThrotCounter = 0;
							CalibrateGyro();
						}
					}
					else if(rcInput[RX_AILE] < MINTHRESH) {
						// calib magneto
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
			
			alt_diff = 0;
			alt_diff_i = 0;	
			alt_diff_d = 0;
			
			lat_diff = 0;
			lat_diff_i = 0;	
			lat_diff_d = 0;
			
			lon_diff = 0;
			lon_diff_i = 0;	
			lon_diff_d = 0;
			
			flapState = 0;
			targetX = 0;
			targetY = 0;
			targetZ = 0;
			targetX_tko = 0;
			targetY_tko = 0;
			switchover_count = 0;
		}
		
		
		// Flight Demands
		else {						
			
			if (auxState == 0) {
			
				// This section of code applies some throttle increase with high angles
				float M9temp;
				if (M9 > 0) M9temp = M9;
				else M9temp = -M9;
				throttle_angle = ((throttle / M9temp) - throttle); 
				if (throttle_angle < 0) throttle_angle = 0;
				
			
				pitch.demandtemp = -((float)MIDSTICK - (float)rcInput[RX_ELEV])*PITCH_SENS; 
				roll.demandtemp = ((float)MIDSTICK - (float)rcInput[RX_AILE])*ROLL_SENS;
				float tempf = -(float)(yawtrim - rcInput[RX_RUDD])*YAW_SENS; 
				
				if (CFDC_count == 0) {
					pitch.demandtemp = 0; 
					roll.demandtemp = 0;
					float tempf = 0.0; 
				}
				
				float limiter;
				
				limiter = LIM_RATE/(float)FAST_RATE;
				
				
				if(pitch.demandtemp > pitch.demand) {
					if(pitch.demandtemp - pitch.demand > limiter && pitch.demand > 0) pitch.demand = pitch.demand + limiter;
					else pitch.demand = pitch.demandtemp;
				}
				else {
					if(pitch.demand - pitch.demandtemp > limiter && pitch.demand < 0) pitch.demand = pitch.demand - limiter;
					else pitch.demand = pitch.demandtemp;
				}
				
				if(roll.demandtemp > roll.demand) {
					if(roll.demandtemp - roll.demand > limiter && roll.demand > 0) roll.demand = roll.demand + limiter;
					else roll.demand = roll.demandtemp;
				}
				else {
					if(roll.demand - roll.demandtemp > limiter && roll.demand < 0) roll.demand = roll.demand - limiter;
					else roll.demand = roll.demandtemp;
				}
				
				throttle = rcInput[RX_THRO] - throttletrim;
				if (throttle < 0) throttle = 0;
						
				// SIMPLICITY MODE //	
				pitchDemandSpin = fsin(-psiAngle+psiAngleinit+M_PI_2)*pitch.demand - fsin(-psiAngle+psiAngleinit)*roll.demand;
				rollDemandSpin = fsin(-psiAngle+psiAngleinit)*pitch.demand + fsin(-psiAngle+psiAngleinit+M_PI_2)*roll.demand;
			

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

					
				flapState = 0;
				targetX = 0;
				targetY = 0;
				targetZ = 0;
				switchover_count = 0;
				targetX_tko = 0;
				targetY_tko = 0;
			
			}
			
			if (auxState == 1) {
				
				// This section of code applies some throttle increase with high tilt angles
				float M9temp;
				if (M9 > 0) M9temp = M9;
				else M9temp = -M9;
				throttle_angle = ((throttle / M9temp) - throttle); 
				if (throttle_angle < 0) throttle_angle = 0; 
				
				// ULTRASOUND ALTITUDE HOLD//
				if(airborne == 0) { // If craft is not airbourne
			
					switchover_count = 0;
					pitch.demand = 0;
					roll.demand = 0;
			
					//if (((rcInput[RX_THRO] - throttletrim - MIDSTICK) < 50) && ((rcInput[RX_THRO] - throttletrim - MIDSTICK) > -50)) { // and the throttle stick is in the middle
					if (((rcInput[RX_THRO] - throttletrim) > 320) && ((rcInput[RX_THRO] - throttletrim) < 420)) {
						if (flapState == 1) { // Auto take off when the button is pressed
							if(throttle < 380) {
								throttle += 1.2;
							}
							throttle += 0.2;
						}

						alt.demandincr =  0;	
					}
				}						
			
			
			
				if((alt.value > ULTRA_TKOFF) && (airborne == 0)) { 
					// just taken off, set airborne to 1 and remember takeoff throttle
					airborne = 1;
					alt.demandincr = 0;
					lat_diff_i = 0;
					lon_diff_i = 0;
					ultraTkOffThrottle = throttle;
					ultraTkOffInput = ((float)rcInput[RX_THRO] - (float) throttletrim);										
					targetX_tko = ((ilink_position.craftX / 10000000.0) + 51.0);
					targetY_tko = ilink_position.craftY;
					targetX = targetX_tko;
					targetY = targetY_tko;	
				}
			
				if ((ultra > 0) && (ultra < ULTRA_LND) && (airborne == 1)) {
					// If valid ultrasound reading and reading is less than landing threshold and we are airborne
					// then increase landing counter
					landing++;
					
					//If the above is satisfied consecutively more than ULTRA_DTCT times then shut down the motors
					if(landing > ULTRA_DTCT) { 
						airborne = 0;
						throttleHoldOff = 1;
						throttle = 0;
						alt.demandincr =  0;
						ultraerror = 0;
						
					}
					
				}	
				// If consecutive run of readings is broken, reset the landing counter.
				else {
					landing = 0;
				}			
				
				if (switchover_count < 40) {
					alt.demand = ULTRA_TKOFF + ULTRA_OFFSET + alt.demandincr;    
					alt.error = alt.demand - alt.value;
					alt.derivative = alt.value - alt.valueOld;
					
					
					
					alt.integral *= ULTRA_De;
					if(airborne == 1) {
							alt.integral += alt.error;
					}
					
					if (((ultraLoss >= ULTRA_OVTH)) && (ultraerror == 0)) {   // when consecutive ultrasound altitude data losses rise above ULTRA_OVTH, throttle level is reduced	
						ultrathrottle -= 10;
						ultraerror = 1;			
					}
					
					else if (ultraerror == 1) {
						if (throttle > 380)  ultrathrottle -= ULTRA_OVDEC;  //Decays throttle to ensure craft returns to ground level	
					}
					else {
						ultrathrottle = alt.derivative * -ULTRA_Kd;
						ultrathrottle += ULTRA_Kp*alt.error;
						ultrathrottle += ULTRA_Ki*alt.integral;
					}
					
					if (airborne == 1) {
						throttle = ultraTkOffThrottle + ultrathrottle;
						switchover_count++;
					}
				}
					
				if (switchover_count == 39) {
					alt_throttle = throttle;
					targetZ = ilink_position.craftZ;
					gpsHoldZ = ilink_position.craftZ;
					switchover_count = 40;
						
				}
				
				if (airborne == 1) {
				
					if(ilink_position.isNew) {
						ilink_position.isNew = 0;
						
						lat_diff = (double)(targetX - ((ilink_position.craftX / 10000000.0) + 51.0)) * (double)111194.92664455873734580834; // 111194.92664455873734580834f is radius of earth in metres and deg-rad conversion: 6371000*PI()/180
						lon_diff = (double)(targetY - ilink_position.craftY) * (double)111194.92664455873734580834 * fcos(((ilink_position.craftX / 10000000.0) + 51.0) *(double)0.01745329251994329577); // 0.01745329251994329577f is deg-rad conversion PI()/180

						
						
						if (switchover_count >= 40) {
							alt_diff = (double)(targetZ - ilink_position.craftZ);
						}
						
						if (switchover_count == 40) targetZ += 0.4;
							
							
						
						if ( (switchover_count == 40 ) && (((gpsHoldZ + 30) - ilink_position.craftZ) > -3) && (((gpsHoldZ + 30) - ilink_position.craftZ) < 3)) {
							switchover_count = 41;
							
							targetX = targetX_tko + Xout;
							targetY = targetY_tko + Yout;	
						}
						
						if ( (switchover_count == 41 ) && ((targetX - ((ilink_position.craftX / 10000000.0) + 51.0)) > - 0.00005) && ((targetX - ((ilink_position.craftX / 10000000.0) + 51.0)) < 0.00005)  &&  ((targetY - ilink_position.craftY) > - 0.00005) && ((targetY - ilink_position.craftY) < 0.00005)) {
							switchover_count = 42;
						
							targetX += 0.000200;
							targetY += 0.000200;
								
						}
						
						if ( (switchover_count == 42 ) && ((targetX - ((ilink_position.craftX / 10000000.0) + 51.0)) > - 0.00005) && ((targetX - ((ilink_position.craftX / 10000000.0) + 51.0)) < 0.00005)  &&  ((targetY - ilink_position.craftY) > - 0.00005) && ((targetY - ilink_position.craftY) < 0.00005)) {
							switchover_count = 43;
							
							targetX -= 0.000400;
						}	
								
						if ( (switchover_count == 43 ) && ((targetX - ((ilink_position.craftX / 10000000.0) + 51.0)) > - 0.00005) && ((targetX - ((ilink_position.craftX / 10000000.0) + 51.0)) < 0.00005)  &&  ((targetY - ilink_position.craftY) > - 0.00005) && ((targetY - ilink_position.craftY) < 0.00005)) {
							switchover_count = 44;
							
							targetY -= 0.0001;
						}		
								
						if ( (switchover_count == 44 ) && ((targetX - ((ilink_position.craftX / 10000000.0) + 51.0)) > - 0.00005) && ((targetX - ((ilink_position.craftX / 10000000.0) + 51.0)) < 0.00005)  &&  ((targetY - ilink_position.craftY) > - 0.00005) && ((targetY - ilink_position.craftY) < 0.00005)) {
							switchover_count = 45;
							
							targetX += 0.000400;
						}	
								
						if ( (switchover_count == 45 ) && ((targetX - ((ilink_position.craftX / 10000000.0) + 51.0)) > - 0.00005) && ((targetX - ((ilink_position.craftX / 10000000.0) + 51.0)) < 0.00005)  &&  ((targetY - ilink_position.craftY) > - 0.00005) && ((targetY - ilink_position.craftY) < 0.00005)) {
							switchover_count = 46;
							
							targetY -= 0.0001;
						}		
						
						if ( (switchover_count == 46 ) && ((targetX - ((ilink_position.craftX / 10000000.0) + 51.0)) > - 0.00005) && ((targetX - ((ilink_position.craftX / 10000000.0) + 51.0)) < 0.00005)  &&  ((targetY - ilink_position.craftY) > - 0.00005) && ((targetY - ilink_position.craftY) < 0.00005)) {
							switchover_count = 47;
							
							targetX -= 0.000400;
						}	
								
						if ( (switchover_count == 47 ) && ((targetX - ((ilink_position.craftX / 10000000.0) + 51.0)) > - 0.00005) && ((targetX - ((ilink_position.craftX / 10000000.0) + 51.0)) < 0.00005)  &&  ((targetY - ilink_position.craftY) > - 0.00005) && ((targetY - ilink_position.craftY) < 0.00005)) {
							switchover_count = 48;
							
							targetY -= 0.0001;
						}		
								
						if ( (switchover_count == 48 ) && ((targetX - ((ilink_position.craftX / 10000000.0) + 51.0)) > - 0.00005) && ((targetX - ((ilink_position.craftX / 10000000.0) + 51.0)) < 0.00005)  &&  ((targetY - ilink_position.craftY) > - 0.00005) && ((targetY - ilink_position.craftY) < 0.00005)) {
							switchover_count = 49;
							
							targetX += 0.000400;
						}	
								
						if ( (switchover_count == 49 ) && ((targetX - ((ilink_position.craftX / 10000000.0) + 51.0)) > - 0.00005) && ((targetX - ((ilink_position.craftX / 10000000.0) + 51.0)) < 0.00005)  &&  ((targetY - ilink_position.craftY) > - 0.00005) && ((targetY - ilink_position.craftY) < 0.00005)) {
							switchover_count = 50;
							
							targetY -= 0.0001;
						}
						
						if ( (switchover_count == 50 ) && ((targetX - ((ilink_position.craftX / 10000000.0) + 51.0)) > - 0.00005) && ((targetX - ((ilink_position.craftX / 10000000.0) + 51.0)) < 0.00005)  &&  ((targetY - ilink_position.craftY) > - 0.00005) && ((targetY - ilink_position.craftY) < 0.00005)) {
							switchover_count = 51;
							
							targetX -= 0.000400;
						}	
						
						if ( (switchover_count == 51 ) && ((targetX - ((ilink_position.craftX / 10000000.0) + 51.0)) > - 0.00005) && ((targetX - ((ilink_position.craftX / 10000000.0) + 51.0)) < 0.00005)  &&  ((targetY - ilink_position.craftY) > - 0.00005) && ((targetY - ilink_position.craftY) < 0.00005)) {
							switchover_count = 52;
							
							targetX = targetX_tko;
							targetY = targetY_tko;
						}
						
							
							
						if ( (switchover_count == 52 ) && ((targetX - ((ilink_position.craftX / 10000000.0) + 51.0)) > - 0.00005) && ((targetX - ((ilink_position.craftX / 10000000.0) + 51.0)) < 0.00005) && ((targetY - ilink_position.craftY) > - 0.00005) && ((targetY - ilink_position.craftY) < 0.00005)) {
							switchover_count = 53;
							
						}
						
						if (switchover_count == 53) targetZ -= 0.08;

						
							
						
						lat_diff_d = lat_diff - lat_diff_old;
						lon_diff_d = lon_diff - lon_diff_old;
											
						
						lat_diff_old = lat_diff;
						lon_diff_old = lon_diff;
						
						
						// if(targetZ - oldZtarget > LIM_GPSALV) oldZtarget += LIM_GPSALV;
						// else if(targetZ - oldZtarget < -LIM_GPSALV) oldZtarget -= LIM_GPSALV;
						// else oldZtarget = targetZ;

						
						alt_diff_d = alt_diff - alt_diff_old;
						alt_diff_old = alt_diff;
								
								
						lat_diff_i += lat_diff;
						lon_diff_i += lon_diff;
						lat_diff_i *= GPS_De;
						lon_diff_i *= GPS_De;
							
					}

					if ((switchover_count == 53) && (ultraLoss == 0) && (alt.value < 1000)) {
							switchover_count = 54;
							alt.integral = 0;
							ultraTkOffThrottle = throttle;
							alt.demand = alt.value ;
					}
					
					if (switchover_count == 54) {
						alt.demand -= 0.2;    
						alt.error = alt.demand - alt.value;
						alt.derivative = alt.value - alt.valueOld;
						
						
						
						alt.integral *= ULTRA_De;
						if(airborne == 1) {
								alt.integral += alt.error;
						}
						
						if (((ultraLoss >= ULTRA_OVTH)) && (ultraerror == 0)) {   // when consecutive ultrasound altitude data losses rise above ULTRA_OVTH, throttle level is reduced	
							ultrathrottle -= 10;
							ultraerror = 1;			
						}
						
						else if (ultraerror == 1) {
							if (throttle > 380)  ultrathrottle -= ULTRA_OVDEC;  //Decays throttle to ensure craft returns to ground level	
						}
						else {
							ultrathrottle = alt.derivative * -ULTRA_Kd;
							ultrathrottle += ULTRA_Kp*alt.error;
							ultrathrottle += ULTRA_Ki*alt.integral;
						}
						
						
						throttle = ultraTkOffThrottle + ultrathrottle;
						
					}
					
					// if ((lat_diff_d * GPS_Kd) > 0.2) lat_diff_d = (0.2 / GPS_Kd);
					// if ((lat_diff_d * GPS_Kd) < -0.2) lat_diff_d = (-0.2 / GPS_Kd);
					// if ((lon_diff_d * GPS_Kd) > 0.2) lon_diff_d = (0.2 / GPS_Kd);
					// if ((lon_diff_d * GPS_Kd) < -0.2) lon_diff_d = (-0.2 / GPS_Kd);
					
					pitch.demand = GPS_Kp*lat_diff + GPS_Ki*lat_diff_i + GPS_Kd*lat_diff_d;
					roll.demand = GPS_Kp*lon_diff + GPS_Ki*lon_diff_i + GPS_Kd*lon_diff_d;
						
					pitchDemandSpin = fsin(-psiAngle+M_PI_2)*pitch.demand - fsin(-psiAngle)*roll.demand;
					rollDemandSpin = fsin(-psiAngle)*pitch.demand + fsin(-psiAngle+M_PI_2)*roll.demand;
						
					alt_diff_i += alt_diff;
					alt_diff_i *= GPS_ALTDe;                    
					
					if ((switchover_count >= 40) && (switchover_count < 54)) {
					gpsThrottle = GPS_ALTKp * alt_diff + GPS_ALTKi * alt_diff_i + GPS_ALTKd * alt_diff_d + alt_throttle;
					throttle = gpsThrottle;	
					}				
					
					if ((throttle - throttleold) > GPS_THRLM) throttle = throttle + GPS_THRLM;
					if ((throttle - throttleold) < -GPS_THRLM) throttle = throttle - GPS_THRLM;
					
					if (throttle < 380) throttle = 380;
				
					throttleold = throttle;
							
					
					if ((pitchDemandSpin - pitchDemandSpinold) > PITCH_SPL) pitchDemandSpin = pitchDemandSpinold + PITCH_SPL;
					if ((pitchDemandSpin - pitchDemandSpinold) < -PITCH_SPL) pitchDemandSpin = pitchDemandSpinold - PITCH_SPL;
					
					pitchDemandSpinold = pitchDemandSpin;
					
					if ((rollDemandSpin - rollDemandSpinold) > ROLL_SPL) rollDemandSpin = rollDemandSpinold + ROLL_SPL;
					if ((rollDemandSpin - rollDemandSpinold) < -ROLL_SPL) rollDemandSpin = rollDemandSpinold - ROLL_SPL;
					
					rollDemandSpinold = rollDemandSpin;
					
				}
			 
			}

			

			
		}
		

	} 
	
 
	
// ****************************************************************************
// *** Motor PID Control
// ****************************************************************************		
		
		if(pitchDemandSpin > LIM_ANGLE) pitchDemandSpin = LIM_ANGLE;	
		if(pitchDemandSpin < -LIM_ANGLE) pitchDemandSpin = -LIM_ANGLE;
		if(rollDemandSpin > LIM_ANGLE) rollDemandSpin = LIM_ANGLE;
		if(rollDemandSpin < -LIM_ANGLE) rollDemandSpin = -LIM_ANGLE;
 
        pitch.derivative = (pitchDemandSpin - pitch.demandOld);    
        roll.derivative = (rollDemandSpin - roll.demandOld);
        yaw.derivative = (yaw.demand - yaw.demandOld);
        
        pitch.demandOld = pitchDemandSpin;
        roll.demandOld = rollDemandSpin;
        yaw.demandOld = yaw.demand;
        
        pitcherror = pitchDemandSpin + thetaAngle;
        rollerror = rollDemandSpin - phiAngle;
        yawerror = yaw.demand + psiAngle; 
        
        if(pitcherror > M_PI) pitcherror -= M_TWOPI;
        else if(pitcherror < -M_PI) pitcherror += M_TWOPI;
        
        if(rollerror > M_PI) rollerror -= M_TWOPI;
        else if(rollerror < -M_PI) rollerror += M_TWOPI;
        
        if(yawerror > M_PI) yawerror -= M_TWOPI;
        else if(yawerror < -M_PI) yawerror += M_TWOPI;
        
        
        if(throttle > 380) {
            pitch.integral += pitcherror;
            roll.integral += rollerror;
        }
        else {
            pitch.integral += pitcherror/5.0f;
            roll.integral += rollerror/5.0f;
        }
        
        pitch.integral *= PITCH_De;
        roll.integral *= ROLL_De;
        
        
        // Detune at high throttle
        float throttlefactor = throttle/MAXSTICK;
        if(throttlefactor > 1) throttlefactor = 1;
        
        float detunefactor = 1-(throttlefactor * DETUNE);
        float thisPITCH_Kd = PITCH_Kd;
        float thisPITCH_Kdd = PITCH_Kdd * detunefactor;
        float thisROLL_Kd = ROLL_Kd;
        float thisROLL_Kdd = ROLL_Kdd * detunefactor;
        float thisPITCH_Ki = PITCH_Ki;
        float thisROLL_Ki = ROLL_Ki;
                
        // Attitude control PID loops
        
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
        
        pitch.valueOld = (float)Gyro.Y.value;
        roll.valueOld = (float)Gyro.X.value;
        yaw.valueOld = (float)Gyro.Z.value;
        
        motorN = pitchcorrection + rollcorrection;
        motorE = pitchcorrection - rollcorrection;
        motorS = -pitchcorrection - rollcorrection;
        motorW = -pitchcorrection + rollcorrection;
        
        motorN -= yawcorrection;
        motorE += yawcorrection;
        motorS -= yawcorrection;
        motorW += yawcorrection;
        
        motorNav *= SPR_OUT;
        motorNav += (1-SPR_OUT) * motorN;
		
		motorEav *= SPR_OUT;
        motorEav += (1-SPR_OUT) * motorE;
		
		motorSav *= SPR_OUT;
        motorSav += (1-SPR_OUT) * motorS;
		
		motorWav *= SPR_OUT;
        motorWav += (1-SPR_OUT) * motorW;
		       
        tempN = (signed short)motorNav + (signed short)throttle + THROTTLEOFFSET + (signed short)throttle_angle;
        tempE = (signed short)motorEav + (signed short)throttle + THROTTLEOFFSET + (signed short)throttle_angle;
        tempS = (signed short)motorSav + (signed short)throttle + THROTTLEOFFSET + (signed short)throttle_angle;
        tempW = (signed short)motorWav + (signed short)throttle + THROTTLEOFFSET + (signed short)throttle_angle;
        
        if (rcInput[RX_THRO] - throttletrim <  OFFSTICK || throttleHoldOff > 0 || rxLoss > 25) {
            pitch.integral=0;
            roll.integral=0;
            alt.integral = 0;
            
            throttle = 0;
            airborne = 0;
            alt.demandincr =  0;
			ultraerror = 0;
            
            
            if(rcInput[RX_THRO] - throttletrim <  OFFSTICK) throttleHoldOff = 0;
            
            yaw.demand = -psiAngle;
            yaw.demandOld = -psiAngle;
            
            if(armed) {
                PWMSetNESW(THROTTLEOFFSET, THROTTLEOFFSET, THROTTLEOFFSET, THROTTLEOFFSET);
            }
            ilink_outputs0.channel[0] = THROTTLEOFFSET;
            ilink_outputs0.channel[1] = THROTTLEOFFSET;
            ilink_outputs0.channel[2] = THROTTLEOFFSET;
            ilink_outputs0.channel[3] = THROTTLEOFFSET;
            
            motorN = 0;
            motorE = 0;
            motorS = 0;
            motorW = 0;
			
			motorNav = 0;
            motorEav = 0;
            motorSav = 0;
            motorWav = 0;
        }
        else if(armed) {
            float temp;
            
            if(throttle > MAXTHROTTLE*MAXTHROTTLEPERCENT) throttle = MAXTHROTTLE*MAXTHROTTLEPERCENT;
            
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


void ReadGyroSensors(void) { // 400Hz -ish
    signed short data[4];
    if(GetGyro(data)) {
        Gyro.X.raw = data[0];
        Gyro.Y.raw = data[2];
        Gyro.Z.raw = -data[1];
        ilink_rawimu.xGyro = Gyro.X.raw;
        ilink_rawimu.yGyro = Gyro.Y.raw;
        ilink_rawimu.zGyro = Gyro.Z.raw;
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
        

        // Gyroscale factors: 2000dps: 818.51113590117601252569
        // 500dps: 3274.04454360470405010275
        // 250dps: 6548.0890872094081002055
        Gyro.X.value = (Gyro.X.av - Gyro.X.offset)/818.51113590117601252569f;
        Gyro.Y.value = (Gyro.Y.av - Gyro.Y.offset)/818.51113590117601252569f;
        Gyro.Z.value = (Gyro.Z.av - Gyro.Z.offset)/818.51113590117601252569f;
        ilink_scaledimu.xGyro = Gyro.X.value * 1000;
        ilink_scaledimu.yGyro = Gyro.Y.value * 1000;
        ilink_scaledimu.zGyro = Gyro.Z.value * 1000;
    }
}

void ReadAccelSensors(void) {
    float sumsqu;
    signed short data[4];
    if(GetAccel(data)) {
        Accel.X.raw = data[0];
        Accel.Y.raw = data[2];
        Accel.Z.raw = -data[1];
        ilink_rawimu.xAcc = Accel.X.raw;
        ilink_rawimu.yAcc = Accel.Y.raw;
        ilink_rawimu.zAcc = Accel.Z.raw;
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
        
        // Normalise accelerometer
        sumsqu = finvSqrt((float)Accel.X.av*(float)Accel.X.av + (float)Accel.Y.av*(float)Accel.Y.av + (float)Accel.Z.av*(float)Accel.Z.av); // Accelerometr data is normalised so no need to convert units.
        Accel.X.value = (float)Accel.X.av * sumsqu;
        Accel.Y.value = (float)Accel.Y.av * sumsqu;
        Accel.Z.value = (float)Accel.Z.av * sumsqu;
        ilink_scaledimu.xAcc = Accel.X.value * 1000;
        ilink_scaledimu.yAcc = Accel.Y.value * 1000;
        ilink_scaledimu.zAcc = Accel.Z.value * 1000;
    }
}

void ReadMagSensors(void) { // 50Hz-ish
    float sumsqu, temp1, temp2, temp3;
    signed short data[4];
    if(GetMagneto(data)) {
        Mag.X.raw = data[0];
        Mag.Y.raw = data[2];
        Mag.Z.raw = -data[1];
		ilink_rawimu.xMag = Mag.X.raw;
        ilink_rawimu.yMag = Mag.Y.raw;
        ilink_rawimu.zMag = Mag.Z.raw;
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
	
		if (CFDC_count == 1) {
			Mag.X.av += CFDC_mag_x_error;
			Mag.Y.av += CFDC_mag_y_error;
			Mag.Z.av += CFDC_mag_z_error;
		}
		
        // Correcting Elipsoid Centre Point
        temp1 = Mag.X.av - MAGCOR_M1;
        temp2 = Mag.Y.av - MAGCOR_M2;
        temp3 = Mag.Z.av - MAGCOR_M3;

        // Reshaping Elipsoid to Sphere
        temp1 = MAGCOR_N1 * temp1 + MAGCOR_N2 * temp2 + MAGCOR_N3 * temp3;
        temp2 = MAGCOR_N5 * temp2 + MAGCOR_N6 * temp3;
        temp3 = MAGCOR_N9 * temp3;
		/////////////Current Field Distortion Compensation (CFDC)
		
		if (throttle == 0) {
			CFDC_mag_x_zero = Mag.X.av;
			CFDC_mag_y_zero = Mag.Y.av;
			CFDC_mag_z_zero = Mag.Z.av;
			CFDC_count = 0;			
		}
		
		
		if ((throttle > 380) && (CFDC_count == 0)) {
			CFDC_mag_x_error = CFDC_mag_x_zero - Mag.X.av;
			CFDC_mag_y_error = CFDC_mag_y_zero - Mag.Y.av;
			CFDC_mag_z_error = CFDC_mag_z_zero - Mag.Z.av;
			CFDC_count = 1;	
			
		}
		
		

        // Normalize magneto
        sumsqu = finvSqrt((float)temp1*(float)temp1 + (float)temp2*(float)temp2 + (float)temp3*(float)temp3); // Magnetoerometr data is normalised so no need to convert units.
        Mag.X.value = (float)temp1 * sumsqu;
        Mag.Y.value = (float)temp2 * sumsqu;
        Mag.Z.value = (float)temp3 * sumsqu;
        
        ilink_scaledimu.xMag = Mag.X.value * 1000;
        ilink_scaledimu.yMag = Mag.Y.value * 1000;
        ilink_scaledimu.zMag = Mag.Z.value * 1000;
	
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
        case ID_ILINK_IDENTIFY:     ptr = (unsigned short *) &ilink_identify;   maxlength = sizeof(ilink_identify)/2 - 1;   break;
        case ID_ILINK_THALSTAT:     ptr = (unsigned short *) &ilink_thalstat;   maxlength = sizeof(ilink_thalstat)/2 - 1;   break;
        case ID_ILINK_RAWIMU:       ptr = (unsigned short *) &ilink_rawimu;     maxlength = sizeof(ilink_rawimu)/2 - 1;     break;
        case ID_ILINK_SCALEDIMU:    ptr = (unsigned short *) &ilink_scaledimu;  maxlength = sizeof(ilink_scaledimu)/2 - 1;  break;
        case ID_ILINK_ALTITUDE:     ptr = (unsigned short *) &ilink_altitude;   maxlength = sizeof(ilink_altitude)/2 - 1;   break;
        case ID_ILINK_ATTITUDE:     ptr = (unsigned short *) &ilink_attitude;   maxlength = sizeof(ilink_attitude)/2 - 1;   break;
        case ID_ILINK_INPUTS0:
            if(ilink_inputs0.isNew) {
                ilink_inputs0.isNew = 0;
                ptr = (unsigned short *) &ilink_inputs0;
                maxlength = sizeof(ilink_inputs0)/2 - 1;
            }
            break;
        case ID_ILINK_OUTPUTS0:     ptr = (unsigned short *) &ilink_outputs0;   maxlength = sizeof(ilink_outputs0)/2 - 1;   break;
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
        
		/*
        case ID_ILINK_THALCTRL:
            switch(ilink_thalctrl.command) {
                case MAVLINK_MSG_ID_COMMAND_LONG:
                    switch(ilink_thalctrl.data) {
                        case MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
                            // KILL UAS
                            Disarm();
                            ILinkSendMessage(ID_ILINK_THALCTRL, (unsigned short *) &ilink_thalctrl, sizeof(ilink_thalctrl)/2 - 1);
                            break;
                        
                        case MAV_CMD_NAV_LAND:
                            if(MODE_GPS >= 2) {
                                MODE_GPS = 4;
                            }
                            break;
                        
                        case MAV_CMD_NAV_TAKEOFF:
                            if(MODE_GPS == 1) { // 1 is idle
                                MODE_GPS = 2; // 2 is GPS takeoff
                            }
                    }
                    break;
                case MAVLINK_MSG_ID_SET_MODE:
                    if(ilink_thalctrl.data & MAV_MODE_FLAG_SAFETY_ARMED) {
                        Arm();
                    }
                    else {
                        Disarm();
                    }
                    
                    if(ilink_thalctrl.data & MAV_MODE_FLAG_GUIDED_ENABLED) {
                        if(MODE_GPS == 0) {
                            MODE_GPS = 1;
                        }
                        ilink_thalstat.flightMode |= (0x1 << 4);
                    }
                    else {
                        if(MODE_GPS == 1) { // 1 is GPS idle       
                            MODE_GPS = 0;
                        }
                        else {
                            MODE_GPS = 5; // 5 is GPS land and switch off GPS;
                        }
                        ilink_thalstat.flightMode &= ~(0x1 << 4);
                    }
                    
                    break;
            }
            break;
		*/	
			
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