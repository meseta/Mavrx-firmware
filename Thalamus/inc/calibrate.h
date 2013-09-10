#ifndef __CALIBRATE_H__
#define __CALIBRATE_H__

#define STILL_THRESH	8000 // this value is used to detect whether the craft is still or not for the purpose of calibration, it's the square of the euclidian distance of the maximum expected gyro noise in raw 

unsigned char detect_ori(void);

void calibrate_ori(void);

void calibrate_mag(void);

void calibrate_gyr(void);
void calibrate_gyr_temporary(unsigned int seconds);

#endif