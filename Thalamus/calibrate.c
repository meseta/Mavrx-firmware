/*!
\file Thalamus/calibrate.c
\brief Functions for zeroing and calibrating sensors

\author Yuan Gao
\author Henry Fletcher
\author Oskar Weigl

*/

#include "all.h"

/*!
\brief Detects orientation of craft, used for selecting the right axis mixings

The same code can be used on several different craft, some of which mount the
flight controller in a different orientation.  This function detects which
orientation Thalamus is in so that the correct axes can be used.  This
function simply calculates this from which axis is down.
*/
unsigned char detect_ori(void) {
    signed short data[3];
    signed short x_axis, y_axis, z_axis;
    unsigned int x_mag, y_mag, z_mag;
    
    if(GetAccel(data)) {
        x_axis = data[0];
        y_axis = data[1];
        z_axis = data[2];
        x_mag = (signed int)x_axis * (signed int)x_axis;
        y_mag = (signed int)y_axis * (signed int)y_axis;
        z_mag = (signed int)z_axis * (signed int)z_axis;
        
        if(x_mag > y_mag && x_mag > z_mag) {
            if(x_axis > 0)  return 1;    // x is down
            else            return 2;    // x is up
        }
        else if(y_mag > x_mag && y_mag > z_mag) {
            if(y_axis > 0)  return 3;   // y is down: NAVY
            else            return 4;   // y is up: R10+ or NAVY upside down
        }
        else if(z_mag > y_mag && z_mag > x_mag) {
            if(z_axis > 0)  return 5;   // z is down: R10
            else            return 6;   // z is up
        }
    }

    return 7; // orientation not detected - this will happen if two of the highest axes are equal
}

/*!
\brief Detects and stores the orientation of the craft.

This function detects and then stores the orientation of the craft in EEPROM.
(and then plays an animation on the props to notify of this).  This function
only needs to be run once (or after a firmware update with a  different EEPROM version)
*/
void calibrate_ori(void) {
    ORI = detect_ori();
    eeprom_save_all();
	
	// play disarm animation
	PWMSetNESW(1500, 1500, 1500, 1500);
	Delay(100);
	
	PWMSetN(1600);
	Delay(100);
	PWMSetN(1500);
	Delay(33);
	PWMSetE(1600);
	Delay(100);
	PWMSetE(1500);
	Delay(33);
	PWMSetS(1600);
	Delay(100);
	PWMSetS(1500);
	Delay(33);
	PWMSetW(1600);
	Delay(100);
	PWMSetW(1500);

	Delay(100);
	PWMSetNESW(1500, 1500, 1500, 1500);
}


/*!
\brief Calibrates magnetometers (hard iron calibration)

This function does a hard iron calibration of the magnetometers, this is
achieved simply by swinging the magnetometers around and measuring the maximum
and minimum axis readings.

Soft iron calibrations are hard-coded.

\todo soft iron calibration on the navy craft
*/
void calibrate_mag(void) {
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
			
			read_mag_sensors();
			Xmax = Mag.X.raw;
			Xmin = Xmax;
			Ymax = Mag.Y.raw;
			Ymin = Ymax;
			Zmax = Mag.Z.raw;
			Zmin = Zmax;
			
			// Calibrate for 70 seconds or until still for 2.8 seconds.
			/*! \todo filter out spikes */
			for(i=0; i<5000; i++) {
				read_gyr_sensors();
				
				// calculate distance of data from running average
				distance  = (Xav - (float)Gyro.X.raw)*(Xav - (float)Gyro.X.raw);
				distance += (Yav - (float)Gyro.Y.raw)*(Yav - (float)Gyro.Y.raw);
				distance += (Zav - (float)Gyro.Z.raw)*(Zav - (float)Gyro.Z.raw);
				
				if(started == 0) {
					// before starting, wait for gyro to move around
					if(distance > STILL_THRESH) {
						// high-movement, increment good counter (which is used to detect the craft moving) and add average value.
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
					read_mag_sensors();
					if(Mag.X.raw > Xmax) Xmax = Mag.X.raw;
					else if(Mag.X.raw < Xmin) Xmin = Mag.X.raw;
					if(Mag.Y.raw > Ymax) Ymax = Mag.Y.raw;
					else if(Mag.Y.raw < Ymin) Ymin = Mag.Y.raw;
					if(Mag.Z.raw > Zmax) Zmax = Mag.Z.raw;
					else if(Mag.Z.raw < Zmin) Zmin = Mag.Z.raw;
					
					if(distance < STILL_THRESH) {
						// low-movement, increment good counter (which is used to detect the craft being still) and add average value.
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
				
				/*! \todo Inline delay */
				Delay(14);
			}
			
			MAGCOR_M1 = (Xmax + Xmin)/2;
			MAGCOR_M2 = (Ymax + Ymin)/2;
			MAGCOR_M3 = (Zmax + Zmin)/2;
			eeprom_save_all();
			
			flashPLED = 0;
			LEDOff(PLED);
		
			ilink_thalstat.sensorStatus &= ~(0x7); // mask status
			ilink_thalstat.sensorStatus |= 3; // standby
	}
}

/*!
\brief Calibrates gyro offsets and saves results

Performs a calibrate_gyr_temporary() and then saves to EEPROM
*/
void calibrate_gyr(void) {
	calibrate_gyr_temporary(6);
	CAL_GYROX = Gyro.X.offset;
	CAL_GYROY = Gyro.Y.offset;
	CAL_GYROZ = Gyro.Z.offset;
	eeprom_save_all();
}

/*!
\brief Calibrates gyro offsets but don't save it

Performs gyro offset calibration, but don't save it to EEPROM.

\param seconds number of seconds to take calibration for
*/
void calibrate_gyr_temporary(unsigned int seconds) {
	unsigned int i;
	// *** Calibrate Gyro
	unsigned int  good;
	float Xav, Yav, Zav;
	signed int Xtotal, Ytotal, Ztotal;
	float distance;
	
	if(armed == 0) {
		
		read_gyr_sensors();
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
			read_gyr_sensors();
			
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

			if(distance < STILL_THRESH) {
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
			read_gyr_sensors();
			
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