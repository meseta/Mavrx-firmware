#include "all.h"



void gps_status(void) {	
	static unsigned int loss_counter = 0;
	

	// check if GPS data is present and valid
	if((ilink_gpsfly.isNew) && (ilink_gpsfly.flags & 0x1)) {
		ilink_gpsfly.isNew = 0;
		loss_counter = 0;
		gps_valid = 1;

	}
	else {
		// if GPS data is invalid or missing increment loss counter
		if(loss_counter < 10000) loss_counter++;
		if(loss_counter > SLOW_RATE*2) { // if no GPS for 2 seconds GPS is considered lost
			gps_valid = 0;
		}
	}
}


void read_rx_input(void) {			

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
			// Make sure that the current yaw stick location is trimmed to zero.
			yawtrim = rcInput[RX_RUDD];
		}
		
		// Controller's aux or gear switch (Can be switched permanently either way)
        if(rcInput[RX_AUX1] > MIDSTICK) {
            auxState = 0;
        }
        else {
            auxState = 1;

        }

		// If the button is pressed, start incrementing a counter
		if(rcInput[RX_FLAP] > MIDSTICK) {
			flapswitch++;
			if (flapswitch > 4000) flapswitch = 4000;
		}
		// If the button is released and the counter is greater than zero but less than 3 seconds
		if ((rcInput[RX_FLAP] < MIDSTICK) && (flapswitch > 0)  && (flapswitch <= (SLOW_RATE*3)) ) {
			// Then reset the counter
			flapswitch = 0;
			// There are three states
			// If the button is pressed in states 2 or 0 then go to state 1
			if ((flapState == 0) || (flapState == 2)) {
				flapState = 1;
				// and request Position Hold/ pause on from Hypo
				ilink_gpsreq.request = 3;
				ilink_gpsreq.sequence++;               
			}
			// If the button is pressed in state 1 then go to state 0
			else {
				flapState = 0;
				if (state == STATE_AUTO) {
					// and request resume/ go from Hypo if we are in Auto
					ilink_gpsreq.request = 4;
					ilink_gpsreq.sequence++; 
				}
				else {
					// else set idle mode on Hypo if we are in Manual MODE
					ilink_gpsreq.request = 7;
					ilink_gpsreq.sequence++;
				}
			}
		}
		// We set the state to 2 if the button has been released after being held for over three seconds.
		if ((rcInput[RX_FLAP] < MIDSTICK) && (flapswitch > (SLOW_RATE*3))) {
			flapswitch = 0;
			flapState = 2;
			// and request go home and land from Hypo
			ilink_gpsreq.request = 6;
			ilink_gpsreq.sequence++; 
			
		}
			
            
                
        // MODIFIED DX4e ONLY: Controller's rate switch (can be switched permanently either way
        if((rcInput[RX_AUX1] & 0x00F) == 0 && (rcInput[RX_FLAP] & 0x00f) == 0) { // Detect if this is a modified controller
            rateState = (rcInput[RX_AUX1] >> 4) & 0x01;
            throState = (rcInput[RX_FLAP] >> 7) & 0x01;
            aileState = (rcInput[RX_FLAP] >> 6) & 0x01;
            elevState = (rcInput[RX_FLAP] >> 5) & 0x01;
            ruddState = (rcInput[RX_FLAP] >> 4) & 0x01;
        }
        
            
		flashVLED = 0;
		LEDOff(VLED);
	}
	else {
		rxLoss ++;
		if(rxLoss > 50) {
			rxLoss = 50;
			// RC signal lost
			// TODO: Perform RC signal loss state setting
			if(armed) PWMSetNESW(THROTTLEOFFSET, THROTTLEOFFSET, THROTTLEOFFSET, THROTTLEOFFSET);
			ilink_outputs0.channel[0] = THROTTLEOFFSET;
			ilink_outputs0.channel[1] = THROTTLEOFFSET;
			ilink_outputs0.channel[2] = THROTTLEOFFSET;
			ilink_outputs0.channel[3] = THROTTLEOFFSET;
			flashVLED = 2;
		}
	}			
			
}


void read_barometer(void) {
    // There are two versions of Thalamus, one with a barometer chip that needs to be
    // manually triggered, and temperature compensated, another with a barometer chip
    // that does all this internally.
    // The Thalamus library auto-detects which is in use, but the user code must
    // perform the trigering.  The library is set up so that the barometer chip that
    // doesn't need triggering simply ignores the trigering.  This makes it safe to
    // write code that supports both barometers
    
    static unsigned char temperature_counter = 0; // used to count the number of samples before updating the temperature compensation
	 
    if(temperature_counter++ < 50) { // note: temperature_counter is incremented AFTER it is compared with 50
        // Get raw barometric pressure reading in Pascals, when it reads 0
        float pressure = GetBaroPressure();

        // There is an I2C or sensor error if 0 is returned, so only update altitude when pressure is greater than 0
        if(pressure > 0) {	
            // Run an LPF filter on the barometer data
            alt.pressure *= (1-LPF_BARO);
            alt.pressure +=  LPF_BARO * pressure;									
			
            //Scale to Metres
            alt.baro = Pressure2Alt(alt.pressure);
            
            // Output processed data over telemetry
            ilink_altitude.baro = alt.baro;
        }
        
        // trigger new reading, unless temperature_counter is equal to 50, in which case trigger new temperature reading (note: this is only used on Thalamus with barometr chips that need this, the rest of the time, this does nothing, so it is safe to leave this in the code
        if(temperature_counter < 50) TrigBaro();
        else TrigBaroTemp();
    }
    else {
        GetBaroTemp();
        temperature_counter = 0;
		TrigBaro();
    }
}

void read_ultrasound(void) {			
	// Get ultrasound data and scale to mm??
	// TODO: Scale to m (everywhere should use SI units)
	ultra = (UltraGetNewRawData()) * 0.17;
	
	
	if(ultra > 0)  {
		ultraLoss = 0;
		
		// We run an LPF filter on the ultrasound readings
		alt.ultra *= (1-LPF_ULTRA);
		alt.ultra += LPF_ULTRA * ultra;
		
		// Output the ultrasound altitude
		ilink_altitude.ultra = alt.ultra;

		alt.ult_conf += (1-alt.ult_conf)*0.1;
		
	}
	// if ultra = 0 then there isn't valid data
	// TODO: Improve ultrasound confidence estimator
	else {
		ultraLoss++;
		if(ultraLoss > ULTRA_OVTH) ultraLoss = ULTRA_OVTH;

		alt.ult_conf += (0-alt.ult_conf)*0.1;	
	}

	
			
}

void trig_batt_voltage(void) {
	ADCTrigger(CHN7);
}

void read_batt_voltage(void) {
	// Because the factor is 6325/1024, we can do this in integer maths by right-shifting 10 bits instead of dividing by 1024.
	unsigned short battV = (ADCGet() * 6325) >> 10; 
	// battVoltage in milivolts
	batteryVoltage *= 0.99f;
	batteryVoltage += 0.01f * (float)battV;
	ilink_thalstat.battVoltage = battV;
}
void convert_ori(volatile signed short * X, volatile signed short * Y, volatile signed short * Z, signed short * data) {
    switch((unsigned char)ORI) {
        default:
        case 3: // NAVY EDITION
            *X = data[0];
            *Y = -data[2];
            *Z = data[1];
			break;
        case 4: // R10+ or Navy uside down
            *X = data[0];
            *Y = data[2];
            *Z = -data[1];
            break;
        case 5: // R10
            *X = data[0];
            *Y = data[1];
            *Z = data[2];
            break;
    }
}
			
void read_gyr_sensors(void) {
	signed short data[4];
	if(GetGyro(data)) {
		// Read raw Gyro data and assign it to the correct axes based on calibrated orientation
        convert_ori(&Gyro.X.raw, &Gyro.Y.raw, &Gyro.Z.raw, data);
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

void read_acc_sensors(void) {
	float sumsqu;
	signed short data[4];
    
    if(GetAccel(data)) {
        // Get raw Accelerometer data and assign it to the correct axes based on calibrated orientation
        convert_ori(&Accel.X.raw, &Accel.Y.raw, &Accel.Z.raw, data);
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

void read_mag_sensors(void) {
	float sumsqu, temp1, temp2, temp3;
	signed short data[4];
	
	if(GetMagneto(data)) {
		// Get raw magnetometer data and assign it to the correct axes based on calibrated orientation
        convert_ori(&Mag.X.raw, &Mag.Y.raw, &Mag.Z.raw, data);
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
