#ifndef __INPUTS_H__
#define __INPUTS_H__

//  IMU sensors
#define GAV_LEN 8
#define AAV_LEN 30
#define MAV_LEN 30
#define ALTAV_LEN 30
#define ALTDAV_LEN 60

// Inputs

#define OFFSTICK			50
#define MIDSTICK			512		// Corresponds to input when stick is in middle (approx value).
#define MAXSTICK			850		// Corresponds to input when stick is at the top

#define MAXTHRESH		   750 //(MAXSTICK+MIDSTICK)/2 - 50
#define MINTHRESH		   250 //(MIDSTICK+OFFSTICK)/2 + 50

// grabs GPS status

extern unsigned char gps_valid;
void gps_status(void);

// Altitude sensors


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
	unsigned int count;
	signed int history[ALTAV_LEN];
	volatile unsigned int total;
	unsigned int dcount;
	signed int dhistory[ALTDAV_LEN];
	volatile signed int dtotal;
} altStruct;
extern altStruct alt;

// Ultrasound
extern float ultra;
extern unsigned int ultraLoss;

void read_barometer(void);
void read_ultrasound(void);

// battery sensor
extern float batteryVoltage;
void trig_batt_voltage(void);
void read_batt_voltage(void);

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

void read_gyr_sensors(void);
void read_acc_sensors(void);
void read_mag_sensors(void);

// sensor orientation
void convert_ori(volatile signed short * X, volatile signed short * Y, volatile signed short * Z, signed short * data);

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

void read_rx_input(void);

#endif