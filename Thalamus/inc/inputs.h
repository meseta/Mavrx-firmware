#ifndef __INPUTS_H__
#define __INPUTS_H__

// grabs GPS status

void gps_status(void);

// Altitude sensors


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

// user inputs
// Inputs


void read_rx_input(void);

#endif