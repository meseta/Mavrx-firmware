void arm(void) {

	if(CAL_AUTO > 0) {
		calibrate_gyr_temperature(1);
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
	
	// Request that Hypo stores Return to Arm location
	ilink_gpsreq.request = 1;
	ilink_gpsreq.sequence++;
	
	armed = 1;
	
	ilink_thalstat.sensorStatus &= ~(0x7); // mask status
	ilink_thalstat.sensorStatus |= 4; // active/armed
}



void disarm(void) {
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

unsigned char detect_ori(void) {
    signed short data[3];
    signed short x_axis, y_axis, z_axis;
    
    if(GetAccel(data)) {
        x_axis = data[0];
        y_axis = data[1];
        z_axis = data[2];
        
        if     (x_axis > y_axis && x_axis > z_axis) return 1; // x is down
        else if(x_axis < y_axis && x_axis < z_axis) return 2; // x is up
        else if(y_axis > x_axis && y_axis > z_axis) return 3; // y is down; NAVY EDITION
        else if(y_axis < x_axis && y_axis < z_axis) return 4; // y is up
        else if(z_axis > y_axis && z_axis > x_axis) return 5; // z is down
        else if(z_axis < y_axis && z_axis < x_axis) return 6; // z is up
    }

    return 7; // orientation not detected - this will happen if two of the highest axes are equal
}

void calibrate_ori(void) {
    ORI = detect_ori();
    eeprom_save_all();
}

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
			//TODO: filter out spikes
			for(i=0; i<5000; i++) {
				read_gyr_sensors();
				
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
					read_mag_sensors();
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
			eeprom_save_all();
			
			flashPLED = 0;
			LEDOff(PLED);
		
			ilink_thalstat.sensorStatus &= ~(0x7); // mask status
			ilink_thalstat.sensorStatus |= 3; // standby
	}
}



void sensor_zero(void) {
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
		read_gyr_sensors();
	}
	
	for(i=0; (i<AAV_LEN); i++) {
		read_acc_sensors();
	}
	
	for(i=0; (i<MAV_LEN); i++) {
		read_mag_sensors();
	}

	ilink_thalstat.sensorStatus |= (0xf << 3);
}



void calibrate_gyr(void) {
	calibrate_gyr_temperature(6);
	CAL_GYROX = Gyro.X.offset;
	CAL_GYROY = Gyro.Y.offset;
	CAL_GYROZ = Gyro.Z.offset;
	eeprom_save_all();
}



void calibrate_gyr_temperature(unsigned int seconds) {
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