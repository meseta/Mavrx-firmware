
void ReadBaroSensors(void) {
	// Get raw barometric pressure reading in Pascals, when it reads 0
	float baro = GetBaroPressure();	
	// There is an I2C or sensor error if 0 is returned, so only update altitude when pressure is greater than 0
	if(baro > 0) {	
		// Run an LPF filter on the barometer data
		alt.baro *= LPF_BARO;
		alt.baro += (1-LPF_BARO) * baro;
		// Linearise around Sea Level
		
		//Scale to Metres
		alt.baro = alt.baro/1.0;
		// Output processed data over telemetry
		ilink_altitude.baro = alt.baro;
	}
}

void ReadUltrasound(void) {			
	// Get ultrasound data and scale to mm??
	// TODO: Scale to m (everywhere should use SI units)
	ultra = (UltraGetNewRawData()) * 0.17;
	
	
	if(ultra > 0)  {
		ultraLoss = 0;
		
		// We run an LPF filter on the ultrasound readings
		alt.ultra *= LPF_ULTRA;
		alt.ultra += (1-LPF_ULTRA) * ultra;
		
		// Output the ultrasound altitude
		ilink_altitude.ultra = alt.ultra;
		
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
	
        // Output raw data over telemetry (this is just the last value used)
        ilink_rawimu.xAcc = Accel.X.raw;
        ilink_rawimu.yAcc = Accel.Y.raw;
        ilink_rawimu.zAcc = Accel.Z.raw;
        // Get average
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



// TODO: Simplify the calibration routines, there seems to be a lot of overlap and unnecessary/ unused code

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