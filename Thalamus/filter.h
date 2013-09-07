
void filter_gps_baro(){

	// GPS Altitude in Metres
	alt.gps = ilink_gpsfly.altitude;
	
	// Old Barometer which is too poor to use for altitude hold
	if (FUNCBaro_type == 1) {
		if (gps_valid == 1) {
			Filt_GPS_K = 1; 
			Filt_baroK = 0; 
		}
		else {
			Filt_GPS_K = 0;
			Filt_baroK = 0; 
		}
	}
	// New barometer
	if (FUNCBaro_type == 2) {
		if (gps_valid == 1) {
			Filt_GPS_K = 0.2; 
			Filt_baroK = 0.8; 
		}
		else {
			Filt_GPS_K = 0;
			Filt_baroK = 1; 
		}
	}
	ilink_debug.debug1 = alt.gps;
	ilink_debug.debug2 = alt.baro;
	
	// Merge Barometer and GPS Data
	alt.filtered += Filt_GPS_K * (alt.gps - alt.filtered);
	alt.filtered += Filt_baroK * (alt.baro - alt.filtered);
	
	alt.vel = -ilink_gpsfly.velD;
	
	ilink_debug.debug0 = alt.filtered;

	
}

void a_h_r_s(){

// ****************************************************************************
// *** ATTITUDE HEADING REFERENCE SYSTEM
// ****************************************************************************
	
	// CREATE THE MEASURED ROTATION MATRIX //
	
	//Both mag and acc are normalized in their Read functions
	
	// Compose the Measured Rotation Matrix from Accelerometer and Magnetometer Vectors
	//  Global Z axis in Local Frame/ RM Third Row - Accelerometer (already normalised)
	
	RM7 = Accel.X.value;
	RM8 = Accel.Y.value;
	RM9 = Accel.Z.value;
	
	// Global Y axis in local frame/ RM Second Row - Acclerometer cross Magnetometer
	RM4 = Accel.Y.value*(Mag.Z.value) - (Accel.Z.value)*Mag.Y.value;
	RM5 = (Accel.Z.value)*Mag.X.value - Accel.X.value*(Mag.Z.value);
	RM6 = Accel.X.value*Mag.Y.value - Accel.Y.value*Mag.X.value;
	// Normalise
	float temp = sqrt(RM4*RM4 + RM5*RM5 + RM6*RM6);
	RM4 = RM4/temp;
	RM5 = RM5/temp;
	RM6 = RM6/temp;
	
	//  Global X axis in Local Frame/ RM First Row - Y axis cross Z axis
	RM1 = RM5*RM9 - RM6*RM8;
	RM2 = RM6*RM7 - RM4*RM9;
	RM3 = RM4*RM8 - RM5*RM7;
	// Normalise
	temp = sqrt(RM1*RM1 + RM2*RM2 + RM3*RM3);
	RM1 = RM1/temp;
	RM2= RM2/temp;
	RM3 = RM3/temp;
	

	// CREATE THE ESTIMATED ROTATION MATRIX //
	// The measured quaternion updates the estimated quaternion via the Gyro.*.error terms applied to the gyros
	float g1 = (Gyro.X.value - Gyro.X.error*DRIFT_AccelKp)/(float)FAST_RATE;
	float g2 = (Gyro.Y.value - Gyro.Y.error*DRIFT_AccelKp)/(float)FAST_RATE;
	float g3 = (Gyro.Z.value - Gyro.Z.error*DRIFT_MagKp)/(float)FAST_RATE;
	
	// Increment the Estimated Rotation Matrix by the Gyro Rate
	//First row is M1, M2, M3 - World X axis in local frame
	M1 = M1 + g3*M2 - g2*M3;
	M2 = M2 - g3*M1 + g1*M3;
	M3 = M3 + g2*M1 - g1*M2;
	
	//Second row is M4, M5, M6 - World Y axis in local frame
	M4 = M4 + g3*M5 - g2*M6;
	M5 = M5 - g3*M4 + g1*M6;
	M6 = M6 + g2*M4 - g1*M5;
	
	// We use the dot product and both X and Y axes to adjust any lack of orthogonality between the two.
	float MX_dot_MY = M1*M4 + M2*M5 + M3*M6;
	M1 = M1 - M4*(MX_dot_MY/2);
	M2 = M2 - M5*(MX_dot_MY/2);
	M3 = M3 - M6*(MX_dot_MY/2);
	
	float sumsqu = finvSqrt(M1*M1 + M2*M2 + M3*M3);
	M1 = M1*sumsqu;
	M2 = M2*sumsqu;
	M3 = M3*sumsqu;
	
	M4 = M4 - M1*(MX_dot_MY/2);
	M5 = M5 - M2*(MX_dot_MY/2);
	M6 = M6 - M3*(MX_dot_MY/2);
	
	sumsqu = finvSqrt(M4*M4 + M5*M5 + M6*M6);
	M4 = M4*sumsqu;
	M5 = M5*sumsqu;
	M6 = M6*sumsqu;
	
	//We find the Z axis by calculating the cross product of X and Y
	M7 = M2*M6 - M3*M5;
	M8 = M3*M4 - M1*M6;
	M9 = M1*M5 - M2*M4;
	
	sumsqu = finvSqrt(M7*M7 + M8*M8 + M9*M9);
	M7 = M7*sumsqu;
	M8 = M8*sumsqu;
	M9 = M9*sumsqu;
	
	// CALCULATE GYRO BIAS //
	// The gyro errores adjust the estimated matrix towards the measured rotation matrix.
	// Use x and y components of the Cross Product between the z vector from each Rotation Matrix for Gyro Biases
	Gyro.X.error = RM9*M8 - RM8*M9;
	Gyro.Y.error = RM7*M9 - RM9*M7;
	
	// Use z component of the cross product between the x vector from each rotation matrix to create the Z gyro error
	Gyro.Z.error = RM2*M1 - RM1*M2;


	// CALCULATE THE ESTIMATED QUATERNION //
	float trace = M1 + M5 + M9;
	if( trace > 0 ) {
		float s = 0.5f / sqrt(trace + 1.0f);
		q1 = 0.25f / s;
		q2 = ( M6 - M8 ) * s;
		q3 = ( M7 - M3 ) * s;
		q4 = ( M2 - M4 ) * s;
	} 
	else {
		if ( M1 > M5 && M1 > M9 ) {
			float s = 2.0f * sqrt( 1.0f + M1 - M5 - M9);
			q1 = (M6 - M8 ) / s;
			q2 = 0.25f * s;
			q3 = (M4 + M2 ) / s;
			q4 = (M7 + M3 ) / s;
		} 
		else if (M5 > M9) {
			float s = 2.0f * sqrt( 1.0f + M5 - M1 - M9);
			q1 = (M7 - M3 ) / s;
			q2 = (M4 + M2 ) / s;
			q3 = 0.25f * s;
			q4 = (M8 + M6 ) / s;
		} 
		else {
			 float s = 2.0f * sqrt( 1.0f + M9 - M1 - M5 );
			q1 = (M2 - M4 ) / s;
			q2 = (M7 + M3 ) / s;
			q3 = (M8 + M6 ) / s;
			q4 = 0.25f * s;
		}
	}	
	q1 = q1;
	q2 = -q2;
	q3 = -q3;
	q4 = -q4;

	// renormalise using fast inverse sq12re root
	sumsqu = finvSqrt(q1*q1 + q2*q2 + q3*q3 + q4*q4);
	q1 *= sumsqu;
	q2 *= sumsqu;
	q3 *= sumsqu;
	q4 *= sumsqu;
		
	
	// CALCULATE THE EULER ANGLES //
	// precalculated values for optimisation
	float q12 = q1 * q2;
	float q13 = q1 * q3;
	float q22 = q2 * q2;
	float q23 = q2 * q4;
	float q33 = q3 * q3;
	float q34 = q3 * q4;
	float q44 = q4 * q4;
	float q22Pq33 = q22 + q33;
	float Tq13Mq23 = 2 * (q13 - q23);
	float Tq34Pq12 = 2 * (q34 + q12);
	// avoid gimbal lock at singularity points
	if (Tq13Mq23 == 1) {
		psiAngle = 2 * fatan2(q2, q1);
		thetaAngle = M_PI_2;
		phiAngle = 0;
	}
	else if (Tq13Mq23 == -1) {
		psiAngle = -2 * fatan2(q2, q1);
		thetaAngle = - M_PI_2;
		phiAngle = 0;
	}
	else {
		thetaAngle = fasin(Tq13Mq23);	
		phiAngle = fatan2(Tq34Pq12, (1 - 2*q22Pq33));  
		psiAngle = fatan2((2*(q1 * q4 + q2 * q3)), (1 - 2*(q33 + q44)));  
	}
	

	// Output angles over telemetry
	ilink_attitude.roll = phiAngle; 
	ilink_attitude.pitch = thetaAngle;
	ilink_attitude.yaw = psiAngle;
}