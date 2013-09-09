#ifndef __GLOBALS_H__
#define __GLOBALS_H__

#define FIRMWARE_VERSION	1			/*!< Firmware version */
#define MESSAGE_LOOP_HZ	 	10		  	/*!< Max frequency of ilink param messages in Hz (keep this number low, like around 15) */
#define RX_PANIC			2		   	/*!< Number of seconds after missing RX before craft considered "disconnected" */

#define FAST_RATE		   	400			/*!< Main loop rate in Hz*/
#define SLOW_RATE		   	75			/*!< Slow rate in Hz */
#define SLOW_DIVIDER		FAST_RATE/SLOW_RATE		/*!< The divider value that is used to obtain the slow rate from fast rate */

// Control/PID
extern unsigned char got_setpoint;

#define THROTTLEOFFSET	900				/*!< Corresponds to less than zero output PWM. Nominally 1000=1ms, but lower just in case */
#define IDLETHROTTLE		175			/*!< Minimum PWM output to ESC when in-flight to avoid motors turning off */
#define MAXTHROTTLE	 	1200			/*!< Maximum PWM output to ESC (PWM = THROTTLEOFFSET + MAXTHROTTLE) */
#define MAXTHROTTLEPERCENT	0.9	 		/*!< Maximum percentage throttle should be (reserves some extra output for stabilisation at high throttle). */

extern unsigned char flashPLED, flashVLED, flashRLED;
extern unsigned char thal_throt_cont;
extern unsigned char thal_motor_off;

extern float GPS_KerrI;
extern float ULT_KerrI;
extern float targetZ_ult;
extern float alt_tkoff;

extern float ROLL_SPL_set;
extern float PITCH_SPL_set;
extern float YAW_SPL_set;

// Filters/AHRS
extern float thetaAngle, phiAngle, psiAngle, psiAngleinit;

/*! \brief User demand struct */
typedef struct {
	float roll; 	/*!< user roll demand */
	float pitch;	/*!< user pitch demand */
	float yaw;		/*!< user yaw demand */
	float throttle;	/*!< user throttle demand */
} userStruct;
extern userStruct user;

/*! \brief Body demand demand struct */
typedef struct {
	float roll;		/*!< body roll demand */
	float pitch;	/*!< pitch roll demand */
	float yaw;		/*!< yaw roll demand */
} attitude_demand_body_struct;
extern attitude_demand_body_struct attitude_demand_body;

extern float q1, q2, q3, q4;
extern float M1, M2, M3, M4, M5, M6, M7, M8, M9;

// Inputs
extern unsigned int PRGBlankTimer; // Blanking time for button pushes
extern unsigned int PRGTimer; // Timer for button pushes, continuously increments as the button is held
extern unsigned int PRGPushTime; // Contains the time that a button was pushed for, populated after button is released

#define OFFSTICK			50		/*!< Corresponds to input when stick is at bottom (approx) */
#define MIDSTICK			512		/*!< Corresponds to input when stick is in middle (approx) */
#define MAXSTICK			850		/*!< Corresponds to input when stick is at top (approx) */

#define MAXTHRESH		   	750 	/*!<(MAXSTICK+MIDSTICK)/2 - 50 */
#define MINTHRESH		   	250 	/*!< MIDSTICK+OFFSTICK)/2 + 50 */

extern unsigned short rcInput[7];
extern unsigned int rxLoss;
extern unsigned int rxFirst;
extern signed short yawtrim;
extern signed short throttletrim;
extern float throttle;
extern float throttle_angle;
extern unsigned char hold_thro_off;
extern unsigned char auxState, flapState, rateState, throState, aileState, elevState, ruddState;
extern unsigned int flapswitch;
extern float pitchDemandSpin;
extern float rollDemandSpin;
extern float pitchDemandSpinold;
extern float rollDemandSpinold;

extern unsigned char gps_valid;

extern float batteryVoltage;
extern float ultra;
extern float oldUltra;
extern unsigned int ultraLoss;

#define GAV_LEN 8		/*!< Running average length for Gyro averaging */
#define AAV_LEN 30		/*!< Running average length for Accel averaging */
#define MAV_LEN 30		/*!< Running average length for Magneto averaging */
#define ALTDAV_LEN 60	/*!< Running average length for Altitude derivative averaging */
#define ALTAV_LEN 30	/*!< Running average length for Altitude averaging */

/*! \brief Altitude data struct */
typedef struct {
	float pressure;	/*!< Atmospheric pressure */
    float baro;		/*!< Barometer altitude */
	float gps;		/*!< GPS altitude */
	float filtered;	/*!< Filtered altitude */
	float ultra;	/*!< Ultrasound height over ground */
	float vel;		/*!< Vertical velocity */
	float ult_conf;	/*!< Ultrasound confidence */
	float barobias;	/*!< Barometer/GPS bias */
	unsigned int count;				/*!< Altitude running average counter */
	signed int history[ALTAV_LEN];	/*!< Altitude running average history */
	volatile unsigned int total;	/*!< Altitude running average total */
	unsigned int dcount;			/*!< Altitude derivative running average counter */
	signed int dhistory[ALTDAV_LEN];/*!< Altitude derivative running average history */
	volatile signed int dtotal;		/*!< Altitude derivative running average total */
} altStruct;
extern altStruct alt;

/*! \brief Gyro data struct (single axis) */
typedef struct{
	volatile signed short raw;		/*!< Raw gyro data */
	volatile float av;				/*!< Average gyro data */
	volatile float value;			/*!< Processed gyro data */
	volatile float offset;			/*!< Gyro offset */
	volatile signed int total;		/*!< Gyro running average total */
	float error;					/*!< Gyro error */
	signed short history[GAV_LEN];	/*!< Gyro running average history */
} sensorStructGyro;

/*! \brief Gyro data struct (three axis) */
typedef struct{
	sensorStructGyro X;	/*!< Gyro X data */
	sensorStructGyro Y;	/*!< Gyro Y data */
	sensorStructGyro Z;	/*!< Gyro Z data */
	unsigned int count;	/*!< Gyro running average counter */
} threeAxisSensorStructGyro;
extern threeAxisSensorStructGyro Gyro;

/*! \brief Accel data struct (single axis) */
typedef struct{
	volatile signed short raw;		/*!< Raw accel data */
	volatile float av;				/*!< Average accel data */
	volatile float value;			/*!< Processed accel data */
	volatile signed int total;		/*!< Accel running average total */
	signed short history[AAV_LEN];	/*!< Accel running average history */
} sensorStructAccel;

/*! \brief Accel data struct (three axis) */
typedef struct{
	sensorStructAccel X;	/*!< Accel X data */
	sensorStructAccel Y;	/*!< Accel Y data */
	sensorStructAccel Z;	/*!< Accel Z data */
	unsigned int count;		/*!< Accel running average counter */
} threeAxisSensorStructAccel;
extern threeAxisSensorStructAccel Accel;

/*! \brief Magneto data struct (single axis) */
typedef struct{
	volatile signed short raw;		/*!< Raw magneto data */
	volatile float av;				/*!< Average magneto data */
	volatile float value;			/*!< Processed magneto data */
	volatile signed int total;		/*!< Magneto running average total */
	signed short history[AAV_LEN];	/*!< Magneto running average history */
} sensorStructMag;

/*! \brief Magneto data struct (three axis) */
typedef struct{
	sensorStructMag X;	/*!< Magneto X data */
	sensorStructMag Y;	/*!< Magneto Y data */
	sensorStructMag Z;	/*!< Magneto Z data */
	unsigned int count;	/*!< Magneto running average counter */
} threeAxisSensorStructMag;
extern threeAxisSensorStructMag Mag;


// State
#define STATE_DISARMED 1		/*!< Disarmed State */
#define STATE_MANUAL 2			/*!< Manual State */
#define STATE_MANUAL_GPS 3		/*!< Manual+GPS State */
#define STATE_AUTO 5			/*!< Auto State */

extern unsigned char armed;
extern unsigned char state;
extern unsigned char airborne; //boolean

#endif