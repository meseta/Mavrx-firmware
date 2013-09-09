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