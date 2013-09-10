#ifndef __INPUTS_H__
#define __INPUTS_H__

#define OFFSTICK			50		/*!< Corresponds to input when stick is at bottom (approx) */
#define MIDSTICK			512		/*!< Corresponds to input when stick is in middle (approx) */
#define MAXSTICK			850		/*!< Corresponds to input when stick is at top (approx) */

#define MAXTHRESH		   	750 	/*!< ~(MAXSTICK+MIDSTICK)/2 - 50 */
#define MINTHRESH		   	250 	/*!< ~(MIDSTICK+OFFSTICK)/2 + 50 */

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


// grabs GPS status

void gps_status(void);


// Ultrasound

void read_barometer(void);
void read_ultrasound(void);

// battery sensor
void trig_batt_voltage(void);
void read_batt_voltage(void);

void read_gyr_sensors(void);
void read_acc_sensors(void);
void read_mag_sensors(void);

// sensor orientation
void convert_ori(volatile signed short * X, volatile signed short * Y, volatile signed short * Z, signed short * data);


void read_rx_input(void);

#endif