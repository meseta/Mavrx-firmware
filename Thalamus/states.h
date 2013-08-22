void state_machine(){


// ****************************************************************************
// ****************************************************************************
// *** disarmed
// ****************************************************************************
// ****************************************************************************


	if (state == disarmed) {
	
		///////////////////// STATE SWITCHING ///////////////////////////////

		// if left stick bottom and right stick bottom left then calibrate orientation
		if  (((rcInput[RX_THRO] - throttletrim) <  OFFSTICK)  &&  (rcInput[RX_ELEV] < MINTHRESH) && (rcInput[RX_AILE] < MINTHRESH)) calibrate_ori();
		
		// if left stick bottom and right stick bottom right then calibrate magnetometer
		if  (((rcInput[RX_THRO] - throttletrim) <  OFFSTICK)  &&  (rcInput[RX_ELEV] < MINTHRESH) && (rcInput[RX_AILE] > MAXTHRESH)) calibrate_mag();
		
		// if left stick bottom right, right stick top left and switch = 0 then switch to manual mode with GPS inactive
		if  (((rcInput[RX_THRO] - throttletrim) <  OFFSTICK)  && (rcInput[RX_RUDD] > MAXTHRESH)  &&  (rcInput[RX_ELEV] > MAXTHRESH) && (rcInput[RX_AILE] < MINTHRESH) && (auxState == 0)) {
			state = manual;
			arm();
		}
		
		// if left stick bottom middle, right stick top left, switch = 0 and GPS is active then switch to manual mode with GPS active
		if  (((rcInput[RX_THRO] - throttletrim) <  OFFSTICK)  && (rcInput[RX_RUDD] < MAXTHRESH)  && (rcInput[RX_RUDD] > MINTHRESH)  &&  (rcInput[RX_ELEV] > MAXTHRESH) && (rcInput[RX_AILE] < MINTHRESH) && (auxState == 0)  &&  (gps_valid == 1)) {
			state = manual_gps;
			arm();
		}
		
		// if left stick bottom right, right stick bottom right, switch = 1 and GPS is active  then switch to full auto
		if  (((rcInput[RX_THRO] - throttletrim) <  OFFSTICK)  && (rcInput[RX_RUDD] > MAXTHRESH)  &&  (rcInput[RX_ELEV] < MINTHRESH) && (rcInput[RX_AILE] > MAXTHRESH) && (auxState == 1)  &&  (gps_valid == 1)) {
			state = auto;
			arm();
		}
		
	}


	
// ****************************************************************************
// ****************************************************************************
// *** manual
// ****************************************************************************
// ****************************************************************************

	if (state == manual) {
			
		///////////////////// STATE SWITCHING ///////////////////////////////
		
		// if left stick bottom and right stick bottom then switch to Disarmed
		if  (((rcInput[RX_THRO] - throttletrim) <  OFFSTICK)  &&  (rcInput[RX_ELEV] < MINTHRESH)) {
			state = disarmed;
			disarm();
		}
		
		// if we loose gps validity then switch into full manual mode
		if (gps_valid == 0) state = manual;

		
		///////////////////// OPERATION ///////////////////////////////
		
		// In manual mode, set pitch and roll demands based on the user commands collected from the rx unit
		attitude_demand_body.pitch = -((float)MIDSTICK - (float)rcInput[RX_ELEV])*PITCH_SENS; 
		attitude_demand_body.roll = ((float)MIDSTICK - (float)rcInput[RX_AILE])*ROLL_SENS;
		float tempf = -(float)(yawtrim - rcInput[RX_RUDD])*YAW_SENS; 						
		throttle = rcInput[RX_THRO] - throttletrim;
		
		
		// A yaw rate is demanded by the rudder input, not an absolute angle.
		// This code increments the demanded angle at a rate proportional to the rudder input
		if(fabsf(tempf) > YAW_DEADZONE) {
			attitude_demand_body.yaw += tempf;
			if(attitude_demand_body.yaw > M_PI) {
				attitude_demand_body.yaw -= M_TWOPI;
			}
			else if(attitude_demand_body.yaw < -M_PI) {
				attitude_demand_body.yaw += M_TWOPI;
			}
		}
		
		///////////////////// VARIABLE SETTING FOR SEAMLESS TRANSITIONS ///////////////////////////////

	}

	
// ****************************************************************************
// ****************************************************************************
// *** manual_gps
// ****************************************************************************
// ****************************************************************************


	if (state == manual_gps) {
			
		///////////////////// STATE SWITCHING ///////////////////////////////
		
		// if left stick bottom and right stick bottom then switch to Disarmed
		if  (((rcInput[RX_THRO] - throttletrim) <  OFFSTICK)  &&  (rcInput[RX_ELEV] < MINTHRESH)) {
			state = disarmed;
			disarm();
		}
		
		// if we loose gps validity then switch into full manual mode
		if (gps_valid == 0) state = manual;

		
		///////////////////// OPERATION ///////////////////////////////
		
		// In manual mode, set pitch and roll demands based on the user commands collected from the rx unit
		attitude_demand_body.pitch = -((float)MIDSTICK - (float)rcInput[RX_ELEV])*PITCH_SENS; 
		attitude_demand_body.roll = ((float)MIDSTICK - (float)rcInput[RX_AILE])*ROLL_SENS;
		float tempf = -(float)(yawtrim - rcInput[RX_RUDD])*YAW_SENS; 						
		throttle = rcInput[RX_THRO] - throttletrim;
		
		
		// A yaw rate is demanded by the rudder input, not an absolute angle.
		// This code increments the demanded angle at a rate proportional to the rudder input
		if(fabsf(tempf) > YAW_DEADZONE) {
			attitude_demand_body.yaw += tempf;
			if(attitude_demand_body.yaw > M_PI) {
				attitude_demand_body.yaw -= M_TWOPI;
			}
			else if(attitude_demand_body.yaw < -M_PI) {
				attitude_demand_body.yaw += M_TWOPI;
			}
		}
		
		// In position hold we take attitude demands from Hypo
		attitude_demand_body.pitch = fsin(-psiAngle+M_PI_2) * ilink_gpsfly.northDemand - fsin(-psiAngle) * ilink_gpsfly.eastDemand;
		attitude_demand_body.roll = fsin(-psiAngle) * ilink_gpsfly.northDemand + fsin(-psiAngle+M_PI_2) * ilink_gpsfly.eastDemand;
		
		if (ilink_gpsfly.headingDemand == 42.0f){
			attitude_demand_body.yaw = user.yaw;
        }
        else {
			attitude_demand_body.yaw = ilink_gpsfly.headingDemand;
		}

		
		///////////////////// VARIABLE SETTING FOR SEAMLESS TRANSITIONS ///////////////////////////////

	}


// ****************************************************************************
// ****************************************************************************
// *** auto
// ****************************************************************************
// ****************************************************************************


	if (state == auto) {
		
		///////////////////// STATE SWITCHING ///////////////////////////////
		
		// if left stick bottom and right stick bottom then switch to Disarmed
		if  (((rcInput[RX_THRO] - throttletrim) <  OFFSTICK)  &&  (rcInput[RX_ELEV] < MINTHRESH)) {
			state = disarmed;
			disarm();
		}
		
		// if we loose gps validity then switch into manual mode
		if (gps_valid == 0) state = manual;
		
		// if the switch is put to zero, switch into manual_gps mode
		if (auxState == 0) state = manual_gps;
		
		///////////////////// OPERATION ///////////////////////////////
		
		// In manual mode, set pitch and roll demands based on the user commands collected from the rx unit
	
		
		///////////////////// VARIABLE SETTING FOR SEAMLESS TRANSITIONS ///////////////////////////////
		
	}




	
}

/*
void control_throttle(){

	//PID states
	static float GPS_KerrI = 0;
	static float ULT_KerrI = 0;
	static float targetZ_ult = 0;
	static unsigned char got_setpoint = 0; //bool
	static float oldUltra = 0;

	//Manual Mode
	// if (MODE_ST == MODE_MANUAL)	{
	if (auxState == 0)	{
		
		//TODO: Must set airborne or not in all modes
		// Throttle comes straight from userinput
		throttle = user.throttle;
		if (throttle < 0) throttle = 0;
		
		//We store key variables to ensure a stepless transition.
		GPS_KerrI = throttle;
		ULT_KerrI = throttle;

		if (alt.ult_conf > 80) {
			targetZ_ult = alt.ultra;
			oldUltra = alt.ultra;
			got_setpoint = 1;
		}
		
	}

	static float gpsThrottle = 0;
	static float ultraThrottle = 0;
	static float alt_tkoff = 0;
	//debug
		ilink_debug.debug1 = alt.filtered;
		//debug
		ilink_debug.debug2 = (alt_tkoff + 0.8);

	//Autonomous Mode
	// if (MODE_ST == MODE_AUTO)	{
	if (auxState == 1)	{	
		
		//When not airborne
		if (airborne == 0) {
			////////////////// TAKEOFF CONTROL  ///////////////////////
			//TODO: Switch over when Hypo requests active if (takeoff == 1) {
			// If take off requested
			if ((flapState == 1)
			//TODO: ADD CHECK ON GPS CONFIDENCE
			// && (GPS CONFIDENCE is GOOD)
			// and throttlestick in middle
			&& ((rcInput[RX_THRO] - throttletrim) > 320) && ((rcInput[RX_THRO] - throttletrim) < 420))	{
				//If still on the ground (throttle zero), record altitude
				if (throttle == 0) {				
					alt_tkoff = alt.filtered;
					throttle += 200;
				}
				// Increase throttle
				throttle += 0.2;

				if ((alt.filtered > alt_tkoff + 0.8 ) || (alt.ultra > ULTRA_TKOFF)) { 
					// just taken off, set airborne to 1 and remember takeoff throttle
					airborne = 1;
					ULT_KerrI = throttle;
					GPS_KerrI = throttle;
					targetZ_ult = alt.ultra;				
				}
			}
		}
	
		
			
		if (airborne == 1) {
		
			
			/////////////////  PID ALTITUDE CONTROL ////////////////////////////
			//run ultrasound and gps in parallell, and select which one drives higher
			// This prevents Hypo dumping the craft onto the ground when it decreases the GPS demand too low
			// It also drags the craft out of the Ultrasound hold when Hypo asks for an increase in GPS altitude.
			
			//TODO: Oskar, explain what your Hysteresis does
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
				
				// If the allowLand flag is set, we lower the craft onto the ground by decrementing the ultrasound altitude.
				//TODO: Replace with allowLand
				// if (ilink_gpsfly.allowLand) {
				if (flapState == 0) {
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
			float GPS_errP = (alt_tkoff + 0.8) - alt.filtered;
			GPS_KerrI += GPS_ALTKi * (1/(float)FAST_RATE) * GPS_errP;
			// float GPS_errD = ilink_gpsfly.altitudeDemandVel - alt.vel; TODO: Set this back when COde finished
			float GPS_errD = 0.0 - alt.vel;
			// Collecting the PID terms
			gpsThrottle = GPS_ALTKp * GPS_errP + GPS_KerrI + GPS_ALTKd * GPS_errD;


			//Use largest throttle output, and cross-feed the integrals for step free transition
			if (ultraThrottle > gpsThrottle){
				GPS_KerrI = ULT_KerrI;
				throttle = ultraThrottle;
			} else {
				ULT_KerrI = GPS_KerrI;
				throttle = gpsThrottle;
				
			}


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
			
			// TODO: Consider accelerometer based shutdown on landing if GPS driven landing causes too much bounce
		}
		
		
	}
	

}
*/