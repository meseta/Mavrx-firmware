// void control_gimbal() {

// }

// void control_led() {

// }



void control_throttle()	{

	static float gpsThrottle = 0;
	static float ultraThrottle = 0;
		
			
	/////////////////  PID ALTITUDE CONTROL ////////////////////////////
	//run ultrasound and gps in parallell, and select which one drives higher
	// This prevents Hypo dumping the craft onto the ground when it decreases the GPS demand too low
	// It also drags the craft out of the Ultrasound hold when Hypo asks for an increase in GPS altitude.
	
	//Hysteresis has been implemented
	static float ultTouchdownHyst = 0;
	if (alt.ult_conf > (0.90 - ultTouchdownHyst)) {
		
		ultTouchdownHyst = 10;

		// When we enter the confidence range, we store the ultrasound altitude and attempt to hold it.
		// TODO: Work out what happens if move out of the ultrasound confidence zone again.
		if (got_setpoint == 0) {
			targetZ_ult = alt.ultra;
			got_setpoint = 1;
			oldUltra = alt.ultra;
		}

		float targetAltVel = 0;
			
		
		// If the allowLand flag is set and we have confidence in the ultrasound readings, we lower the craft onto the ground by decrementing the ultrasound altitude.
		if (ilink_gpsfly.flags & 0x02) {
			targetZ_ult -= 100 * (1/(float)FAST_RATE); // -100 mm/second
			// targetAltVel = -0.2;
		}

		//Ultrasound derived PID controller
		float ULT_errP = targetZ_ult - alt.ultra;
		ULT_KerrI += ULTRA_Ki * (1/(float)FAST_RATE) * ULT_errP;
		float ULT_errD = targetAltVel - (alt.ultra - oldUltra) * (float)FAST_RATE;
		oldUltra = alt.ultra;
		// Collecting the PID terms
		ultraThrottle = ULTRA_Kp * ULT_errP + ULT_KerrI + ULTRA_Kd * ULT_errD;
			

	} 
	
	else {
		got_setpoint = 0;
		ultraThrottle = 0;
		ultTouchdownHyst = 0;
	}


	//Baro/GPS derived PID controller
	//TODO: Hypo should set ilink_gpsfly.altitudeDemand to zero when it recieves the airborne flag from Thalamus
	// float GPS_errP = ilink_gpsfly.altitudeDemand + (alt_tkoff + 0.8) - alt.filtered;
	float GPS_errP = ilink_gpsfly.altitudeDemand - alt.filtered;
	// float GPS_errP = alt_tkoff - alt.filtered;
	GPS_KerrI += GPS_ALTKi * (1/(float)FAST_RATE) * GPS_errP;
	// float GPS_errD = ilink_gpsfly.altitudeDemandVel - alt.vel; TODO: Set this back when COde finished
	float GPS_errD = 0.0 - alt.vel;
	// Collecting the PID terms
	gpsThrottle = GPS_ALTKp * GPS_errP + GPS_KerrI + GPS_ALTKd * GPS_errD;

	// If Thalamus is allowed to overwrite throttle
	if (thal_throt_cont == 1) {
		//Use largest throttle output, and cross-feed the integrals for step free transition
		if (ultraThrottle > gpsThrottle){
			GPS_KerrI = ULT_KerrI;
			throttle = ultraThrottle;
		} else {
			ULT_KerrI = GPS_KerrI;
			throttle = gpsThrottle;			
		}
	}
		

	// If Thalamus is allowed to shut off the motors
	if (thal_motor_off == 1) {
	
		///////////// Set Airborne if takeoff value is read/////////////////
		if (alt.ultra > ULTRA_TKOFF) airborne = 1;
		
		///////////////////////////////// LANDING MOTOR SHUT OFF, 1-Ultrasound Driven  2-GPS Driven ////////////////////////
		//1 - Ultrasound driven
		// If ultrasound reading is valid and less than landing threshold (ULTRA_LD_TD) and we are airborne
		// then increase landing counter
		static int ult_landing = 0;
		if ((ultra > 0) && (ultra < ULTRA_LND) && (airborne == 1)) ult_landing++;
		// If consecutive run of readings is broken, reset the landing counter.
		else ult_landing = 0;
		
		//If the above is satisfied consecutively more than ULT_LD_DT times then shut down the motors
		if(ult_landing > ULTRA_DTCT) { 
			airborne = 0;
			// Prevent the throttle value from rising again before the throttle stick is returned to the bottom
			hold_thro_off = 1;
			throttle = 0;
			//Reset Launcher Button State
			flapState = 0;
			//Reset Ultrasound setpoint aquisition flag
			got_setpoint = 0;
		}

		// 2 - GPS driven
		// If the GPS altitude integral stays maxed out at minimum value, increment the landing counter
		static int gps_landing = 0;
		if ((GPS_KerrI == -1000) && (alt.vel < 1) && (alt.vel > -1)) gps_landing++;
		// If consecutive run of readings is broken, reset the landing counter.
		else gps_landing = 0;
		
		//If the above is satisfied consecutively more than ULTRA_DTCT times then shut down the motors
		if(gps_landing > ULTRA_DTCT) { 
			airborne = 0;
			// Prevent the throttle value from rising again before the throttle stick is returned to the bottom
			hold_thro_off = 1;
			throttle = 0;
			//Reset Launcher Button State
			flapState = 0;
			//Reset Ultrasound setpoint aquisition flag
			got_setpoint = 0;
		}
		
	}	
	
		
		
}
	







void control_motors(){	
    // Orientation Referencing
    switch((unsigned char)ORI) {
        default:
		case 3: // NAVY EDITION passes the pitch and roll demands straight through unchanged
			attitude_demand_body.pitch = user.pitch;
			attitude_demand_body.roll =  user.roll;			
			break;
		case 5:	// R10 Kickstarter
		
			// Stops yawing on slippery ground to ensure magneto compensation is correct.
			if (throttle < 380) attitude_demand_body.yaw = -psiAngle;
			
			// We have to rotate the frame of reference as Thalamus is mounted at 45 degrees.
			attitude_demand_body.pitch = fsin(0.78539816339744830962+M_PI_2) * user.pitch - fsin(0.78539816339744830962) * user.roll;
			attitude_demand_body.roll = fsin(0.78539816339744830962) * user.pitch + fsin(0.78539816339744830962+M_PI_2) * user.roll;
			break;
		
    }

	// This section of code applies some throttle increase with high tilt angles to help reduce descent on angle increase
	float M9temp;
	if (M9 > 0) M9temp = M9;
	else M9temp = -M9;
	throttle_angle = ((throttle / M9temp) - throttle); 
	if (throttle_angle < 0) throttle_angle = 0;

	static float rollold = 0;
	static float pitchold = 0;
	static float yawold = 0;
	
	// This section of code limits the rate at which the craft is allowed to track angle demand changes
	if ((attitude_demand_body.pitch - pitchold) > PITCH_SPL_set) attitude_demand_body.pitch = pitchold + PITCH_SPL_set;
	if ((attitude_demand_body.pitch - pitchold) < -PITCH_SPL_set) attitude_demand_body.pitch = pitchold - PITCH_SPL_set;
	pitchold = attitude_demand_body.pitch;
	if ((attitude_demand_body.roll - rollold) > ROLL_SPL_set) attitude_demand_body.roll = rollold + ROLL_SPL_set;
	if ((attitude_demand_body.roll - rollold) < -ROLL_SPL_set) attitude_demand_body.roll = rollold - ROLL_SPL_set;
	rollold = attitude_demand_body.roll;
	if ((attitude_demand_body.yaw - yawold) > YAW_SPL_set) attitude_demand_body.yaw = yawold + YAW_SPL_set;
	if ((attitude_demand_body.yaw - yawold) < -YAW_SPL_set) attitude_demand_body.yaw = yawold - YAW_SPL_set;
	yawold = attitude_demand_body.yaw;
	
	// This part of the code makes the yaw demand loop around and not exceed Pi or -Pi bounds
	if(attitude_demand_body.yaw > M_PI) {
		attitude_demand_body.yaw -= M_TWOPI;
		yawold -= M_TWOPI;
	}
	else if(attitude_demand_body.yaw < -M_PI) {
		attitude_demand_body.yaw += M_TWOPI;
		yawold += M_TWOPI;
	}
			
			
	// This part of the code sets the maximum angle the quadrotor can go to in the pitch and roll axes
	if(attitude_demand_body.pitch > LIM_ANGLE) attitude_demand_body.pitch = LIM_ANGLE;	
	if(attitude_demand_body.pitch < -LIM_ANGLE) attitude_demand_body.pitch = -LIM_ANGLE;
	if(attitude_demand_body.roll > LIM_ANGLE) attitude_demand_body.roll = LIM_ANGLE;
	if(attitude_demand_body.roll < -LIM_ANGLE) attitude_demand_body.roll = -LIM_ANGLE;

	// Create the demand derivative (demand and external rotations are split) term for the attitude motor control
	static float rollDemandOld = 0;
	static float pitchDemandOld = 0;
	static float yawDemandOld = 0;

	float deltaPitch = (attitude_demand_body.pitch - pitchDemandOld);	
	float deltaRoll = (attitude_demand_body.roll - rollDemandOld);
	float deltaYaw = (attitude_demand_body.yaw - yawDemandOld);
	if(deltaYaw > M_PI) {
		deltaYaw -= M_TWOPI;
	}
	else if(deltaYaw < -M_PI) {
		deltaYaw += M_TWOPI;
	}

	pitchDemandOld = attitude_demand_body.pitch;
	rollDemandOld = attitude_demand_body.roll;
	yawDemandOld = attitude_demand_body.yaw;
	
	
	
	// Use the Current and demanded angles to create the error for the proportional part of the PID loop
	float pitcherror = attitude_demand_body.pitch + thetaAngle;
	float rollerror = attitude_demand_body.roll - phiAngle;
	float yawerror = attitude_demand_body.yaw + psiAngle; 	
	
	// This section ensures that on swapping between -PI and PI, 
	// the craft always takes the shortest route to the desired angle
	if(pitcherror > M_PI) pitcherror -= M_TWOPI;
	else if(pitcherror < -M_PI) pitcherror += M_TWOPI;
	if(rollerror > M_PI) rollerror -= M_TWOPI;
	else if(rollerror < -M_PI) rollerror += M_TWOPI;  
	if(yawerror > M_PI) yawerror -= M_TWOPI;
	else if(yawerror < -M_PI) yawerror += M_TWOPI;
	
	// Creating the integral for the motor PID
	// TODO: Is this the cause of poor leveling on takeoff? (took out wierd throttle dependent rates)
	//TODO: Check to see if added Yaw integral solves yaw offsets in autonomous flight
	static float pitchIntegral = 0;
	static float rollIntegral = 0;
	static float yawIntegral = 0;
	
	if (rcInput[RX_THRO] - throttletrim > OFFSTICK){
		pitchIntegral += pitcherror;
		rollIntegral += rollerror;
		yawIntegral += yawerror;
	} else {
		pitchIntegral = 0;
		rollIntegral = 0;
		yawIntegral = 0;
	}
	
	
	// Detune at high throttle - We turn the tunings down at high throttle to prevent oscillations
	// happening on account of the higher energy input to the system
	float throttlefactor = throttle/MAXSTICK;
	if(throttlefactor > 1) throttlefactor = 1;	   
	float detunefactor = 1-(throttlefactor * DETUNE);
	float thisPITCH_Kd = PITCH_Kd;
	float thisPITCH_Kdd = PITCH_Kdd * detunefactor;
	float thisROLL_Kd = ROLL_Kd;
	float thisROLL_Kdd = ROLL_Kdd * detunefactor;
	float thisPITCH_Ki = PITCH_Ki;
	float thisROLL_Ki = ROLL_Ki;
	
			
	// Attitude control PID Assembly - We use a proportional, derivative, and integral on pitch roll and yaw
	// we add a double derivative term for pitch and roll.
	static float oldGyroValuePitch = 0;
	static float oldGyroValueRoll = 0;

	float pitchcorrection = -((float)Gyro.Y.value - PITCH_Boost*deltaPitch) * thisPITCH_Kd;
	pitchcorrection += -thisPITCH_Kdd*((float)Gyro.Y.value - oldGyroValuePitch);
	pitchcorrection += -PITCH_Kp*pitcherror;
	pitchcorrection += -thisPITCH_Ki*pitchIntegral;

	float rollcorrection = -((float)Gyro.X.value - ROLL_Boost*deltaRoll) * thisROLL_Kd;
	rollcorrection += -thisROLL_Kdd*((float)Gyro.X.value - oldGyroValueRoll);
	rollcorrection += ROLL_Kp*rollerror;
	rollcorrection += thisROLL_Ki*rollIntegral;

	float yawcorrection = -((float)Gyro.Z.value + YAW_Boost*deltaYaw) * YAW_Kd; 
	yawcorrection += -YAW_Kp*yawerror;
	// TODO: Check direction of yaw integral
	yawcorrection += -YAW_Ki*yawIntegral;
	
	// If the craft is upsidedown, turn off yaw control until control brings it back upright.
	// TODO: Test this code
	if (M9 < 0) {
		yawcorrection = 0;
	}
	
	oldGyroValuePitch = (float)Gyro.Y.value;
	oldGyroValueRoll = (float)Gyro.X.value;
   
	//Assigning the PID results to the correct motors
	// TODO: fill these in properly
    switch((unsigned char)ORI) {
        default:
        case 3: // NAVY EDITION
            motorN = pitchcorrection + rollcorrection;
            motorE = pitchcorrection - rollcorrection;
            motorS = -pitchcorrection - rollcorrection;
            motorW = -pitchcorrection + rollcorrection;
            motorN -= yawcorrection;
            motorE += yawcorrection;
            motorS -= yawcorrection;
            motorW += yawcorrection;
            break;
		case 5: // R10
            motorN = pitchcorrection;
            motorE = -rollcorrection;
            motorS = -pitchcorrection;
            motorW = rollcorrection;
			
            motorN -= yawcorrection;
            motorE += yawcorrection;
            motorS -= yawcorrection;
            motorW += yawcorrection;
            break;
    }
	
	// We run an LPF filter on the outputs to ensure they aren't too noisey and don't demand changes too quickly
	// This seems to reduce power consumption a little and ESC heating a little also
	motorNav *= LPF_OUT;
	motorNav += (1-LPF_OUT) * motorN;		
	motorEav *= LPF_OUT;
	motorEav += (1-LPF_OUT) * motorE;		
	motorSav *= LPF_OUT;
	motorSav += (1-LPF_OUT) * motorS;		
	motorWav *= LPF_OUT;
	motorWav += (1-LPF_OUT) * motorW;


	// Combine attitude stabilisation demands from PID loop with throttle demands
	tempN = (signed short)motorNav + (signed short)throttle + THROTTLEOFFSET + (signed short)throttle_angle;
	tempE = (signed short)motorEav + (signed short)throttle + THROTTLEOFFSET + (signed short)throttle_angle;
	tempS = (signed short)motorSav + (signed short)throttle + THROTTLEOFFSET + (signed short)throttle_angle;
	tempW = (signed short)motorWav + (signed short)throttle + THROTTLEOFFSET + (signed short)throttle_angle;
	
		
	// TODO: Add Auto Land on rxLoss!
	if (((rcInput[RX_THRO] - throttletrim) <  OFFSTICK) || (hold_thro_off > 0)) {
		
		// Set Airborne = 0
		airborne = 0;
		
		// Set throttle off
		throttle = 0;
		
		//Reset Launcher Button State
		flapState = 0;
					
		// Reset Important variables
		motorN = 0;
		motorE = 0;
		motorS = 0;
		motorW = 0;
		motorNav = 0;
		motorEav = 0;
		motorSav = 0;
		motorWav = 0;
		// Reseting the yaw demand to the actual yaw angle continuously helps stop yawing happening on takeoff
		attitude_demand_body.yaw = -psiAngle;

		
		// Reset the throttle hold variable, this prevents reactivation of the throttle until 
		// the input is dropped and re-applied
		if (rcInput[RX_THRO] - throttletrim  < OFFSTICK) hold_thro_off = 0;
		
		// If the craft is armed, set the PWM channels to the PWM value corresponding to off!
		if(armed) PWMSetNESW(THROTTLEOFFSET, THROTTLEOFFSET, THROTTLEOFFSET, THROTTLEOFFSET);
		
		// Output the motor PWM demand on the telemetry link
		ilink_outputs0.channel[0] = THROTTLEOFFSET;
		ilink_outputs0.channel[1] = THROTTLEOFFSET;
		ilink_outputs0.channel[2] = THROTTLEOFFSET;
		ilink_outputs0.channel[3] = THROTTLEOFFSET;
		

	}
	else if(armed) {
		
		float temp;
		
		// Limit the maximum throttle output to a percentage of the highest throttle available 
		// to allow additional throttle for manouevering
		if(throttle > MAXTHROTTLE*MAXTHROTTLEPERCENT) throttle = MAXTHROTTLE*MAXTHROTTLEPERCENT;
		
		// Set the PWM channels. Maximum of MAXTHROTTLE + THROTTLEOFFSET, MINMUM OF IDLETHROTTLE + THROTTLE OFFSET
		// Throttle offset offsets the throttle readings (which start at 0) to the PWM values (in ms?) which need to start at around 1000
		temp = tempN;
		if(temp > (MAXTHROTTLE + THROTTLEOFFSET)) temp = (MAXTHROTTLE + THROTTLEOFFSET);
		else if(temp < (IDLETHROTTLE + THROTTLEOFFSET)) temp = (IDLETHROTTLE + THROTTLEOFFSET);
		PWMSetN(temp);
		ilink_outputs0.channel[0] = temp;
		
		temp = tempE;
		if(temp > (MAXTHROTTLE + THROTTLEOFFSET)) temp = (MAXTHROTTLE + THROTTLEOFFSET);
		else if(temp < (IDLETHROTTLE + THROTTLEOFFSET)) temp = (IDLETHROTTLE + THROTTLEOFFSET);
		PWMSetE(temp);
		ilink_outputs0.channel[1] = temp;
		
		temp = tempS;
		if(temp > (MAXTHROTTLE + THROTTLEOFFSET)) temp = (MAXTHROTTLE + THROTTLEOFFSET);
		else if(temp < (IDLETHROTTLE + THROTTLEOFFSET)) temp = (IDLETHROTTLE + THROTTLEOFFSET);
		PWMSetS(temp);
		ilink_outputs0.channel[2] = temp;
		
		temp = tempW;
		if(temp > (MAXTHROTTLE + THROTTLEOFFSET)) temp = (MAXTHROTTLE + THROTTLEOFFSET);
		else if(temp < (IDLETHROTTLE + THROTTLEOFFSET)) temp = (IDLETHROTTLE + THROTTLEOFFSET);
		PWMSetW(temp);
		ilink_outputs0.channel[3] = temp;
		ilink_outputs0.isNew = 1; 

	}
}