void state_machine()	{

	static unsigned char auto_lock = 0;
	
// ****************************************************************************
// ****************************************************************************
// *** disarmed
// ****************************************************************************
// ****************************************************************************


	if (state == disarmed) {
	
	
		if ((rxLoss < 50) && (rxFirst != 0)) {
	
			///////////////////// STATE SWITCHING ///////////////////////////////
			
			// if left stick bottom right, right stick top left and switch = 0 then switch to manual mode with GPS inactive
			if  (((rcInput[RX_THRO] - throttletrim) <  OFFSTICK)  && (rcInput[RX_RUDD] < MINTHRESH)  &&  (rcInput[RX_ELEV] > MAXTHRESH) && (rcInput[RX_AILE] > MAXTHRESH) && (auxState == 0)) {
				state = manual;
				arm();
			}
			
			// if left stick bottom middle, right stick top left, switch = 0 and GPS is active then switch to manual mode with GPS active
			if  (((rcInput[RX_THRO] - throttletrim) <  OFFSTICK)  && (rcInput[RX_RUDD] < MAXTHRESH)  && (rcInput[RX_RUDD] > MINTHRESH)  &&  (rcInput[RX_ELEV] > MAXTHRESH) && (rcInput[RX_AILE] > MAXTHRESH) && (auxState == 0)  &&  (gps_valid == 1)) {
				state = manual_gps;
				arm();
			}
			
			// if left stick bottom middle, right stick top right, switch = 1 and GPS is active  then switch to full auto
			if  (((rcInput[RX_THRO] - throttletrim) <  OFFSTICK)  && (rcInput[RX_RUDD] < MAXTHRESH)  && (rcInput[RX_RUDD] > MINTHRESH)  &&  (rcInput[RX_ELEV] > MAXTHRESH) && (rcInput[RX_AILE] < MINTHRESH) && (auxState == 1)  &&  (gps_valid == 1)) {
				state = auto;
				arm();
			}
			
			///////////////////// OPERATION ///////////////////////////////
			
			auto_lock = 0;
			
			// if left stick bottom and right stick bottom left then calibrate orientation
			if  (((rcInput[RX_THRO] - throttletrim) <  OFFSTICK)  &&  (rcInput[RX_ELEV] < MINTHRESH) && (rcInput[RX_AILE] > MAXTHRESH)) calibrate_ori();
			
			// if left stick bottom and right stick bottom right then calibrate magnetometer
			if  (((rcInput[RX_THRO] - throttletrim) <  OFFSTICK)  &&  (rcInput[RX_ELEV] < MINTHRESH) && (rcInput[RX_AILE] < MINTHRESH)) calibrate_mag();

			
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
		
		
		///////////////////// OPERATION ///////////////////////////////
		
		auto_lock = 0;
		
		ROLL_SPL_set = ROLL_SPL;
		PITCH_SPL_set = PITCH_SPL;
		YAW_SPL_set = YAW_SPL;
		
		// In manual mode, set pitch and roll demands based on the user commands collected from the rx unit
		attitude_demand_body.pitch = -((float)MIDSTICK - (float)rcInput[RX_ELEV])*PITCH_SENS; 
		attitude_demand_body.roll = ((float)MIDSTICK - (float)rcInput[RX_AILE])*ROLL_SENS;
		float tempf = -(float)(yawtrim - rcInput[RX_RUDD])*YAW_SENS; 		
		
		throttle = rcInput[RX_THRO] - throttletrim;
		
		
		// A yaw rate is demanded by the rudder input, not an absolute angle.
		// This code increments the demanded angle at a rate proportional to the rudder input
		if(fabsf(tempf) > YAW_DEADZONE) {
			attitude_demand_body.yaw += tempf;
			
		}
		
		

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
		
		// If we are responding to user control
		if (flapState == 0) {
		
			auto_lock = 0;
			// And Thalamus can't control the throttle
			thal_throt_cont = 0;				
			// And Thalamus isn't allowed to turn the motors off
			thal_motor_off = 0;
		
			ROLL_SPL_set = ROLL_SPL;
			PITCH_SPL_set = PITCH_SPL;
			YAW_SPL_set = YAW_SPL;
		
		
			// In manual mode, set pitch and roll demands based on the user commands collected from the rx unit
			attitude_demand_body.pitch = -((float)MIDSTICK - (float)rcInput[RX_ELEV])*PITCH_SENS; 
			attitude_demand_body.roll = ((float)MIDSTICK - (float)rcInput[RX_AILE])*ROLL_SENS;
			throttle = rcInput[RX_THRO] - throttletrim;	
			
			//We store key variables to ensure a stepless transition into other states
			// For both the GPS
			GPS_KerrI = throttle;
			ULT_KerrI = throttle;
			// and the ultrasound
			if (alt.ult_conf > 80) {
				targetZ_ult = alt.ultra;
				oldUltra = alt.ultra;
				got_setpoint = 1;
			}
		}
		
		// The pilot has control of yaw		
		float tempf = -(float)(yawtrim - rcInput[RX_RUDD])*YAW_SENS; 									
		// A yaw rate is demanded by the rudder input, not an absolute angle.
		// This code increments the demanded angle at a rate proportional to the rudder input
		if(fabsf(tempf) > YAW_DEADZONE) {
			attitude_demand_body.yaw += tempf;
		}
		
		// if the throttle is greater than 0
		if (throttle > 0) {	
			// and if we are position holding or flying home, Hypo has control
			if ((flapState == 2) || (flapState == 1)) {
						
				// then Hypo controls attitude.
				attitude_demand_body.pitch = fsin(-psiAngle+M_PI_2) * ilink_gpsfly.northDemand - fsin(-psiAngle) * ilink_gpsfly.eastDemand;
				attitude_demand_body.roll = fsin(-psiAngle) * ilink_gpsfly.northDemand + fsin(-psiAngle+M_PI_2) * ilink_gpsfly.eastDemand;
				
				// And Thalamus controls the throttle
				thal_throt_cont = 0;
				
				// And Thalamus is allowed to turn the motors off
				thal_motor_off = 0;
			}
		}
	

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
		
		// if we loose gps validity then immediately drop throttle level to indicate user should switch to manual mode and lock out the auto code loop
		if (gps_valid == 0) {
			throttle -= 200;
			auto_lock = 1;
		}
		
		// if the switch is put to zero, switch into manual_gps mode and lock out the auto code loop
		if (auxState == 0) {
			state = manual_gps;
			auto_lock = 1;
		}
		
		///////////////////// OPERATION ///////////////////////////////
		if (auto_lock == 0) {
			//When not airborne
			if (airborne == 0) {
				////////////////// TAKEOFF CONTROL  ///////////////////////
				// If gps valid
				if ((gps_valid == 1)
				// and throttlestick in middle
				&& ((rcInput[RX_THRO] - throttletrim) > 320) && ((rcInput[RX_THRO] - throttletrim) < 420))	{
					//If still on the ground (throttle zero), record altitude
					if (throttle == 0) {				
						alt_tkoff = alt.filtered;
						throttle += 200;
					}
					// Increase throttle
					throttle += 0.2;

					if ((alt.filtered > alt_tkoff + 0.5 ) || (alt.ultra > ULTRA_TKOFF)) { 
						// just taken off, set airborne to 1 and remember takeoff throttle
						airborne = 1;
						ULT_KerrI = throttle;
						GPS_KerrI = throttle;
						targetZ_ult = alt.ultra;
						// Tell Hypo to commence auto flight
						ilink_gpsreq.request = 2;
						ilink_gpsreq.sequence++;
					}
				}
			}
		
			
				
			if (airborne == 1) {
				

				// then Hypo controls attitude.
				attitude_demand_body.pitch = fsin(-psiAngle+M_PI_2) * ilink_gpsfly.northDemand - fsin(-psiAngle) * ilink_gpsfly.eastDemand;
				attitude_demand_body.roll = fsin(-psiAngle) * ilink_gpsfly.northDemand + fsin(-psiAngle+M_PI_2) * ilink_gpsfly.eastDemand;
				
				// The pilot has control of yaw		
				float tempf = -(float)(yawtrim - rcInput[RX_RUDD])*YAW_SENS; 									
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
		
				// If Hypo is transmitting yaw demands we allow it to overwrite the pilots yaw demands
				if (!(ilink_gpsfly.flags & 0x04)) attitude_demand_body.yaw =  ilink_gpsfly.headingDemand;
					
				
				// And Thalamus controls the throttle
				thal_throt_cont = 1;
				
				// And Thalamus is allowed to turn the motors off
				thal_motor_off = 1;
				
		
			}
			
		}

	}
	
}
