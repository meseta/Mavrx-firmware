/*!
\file Thalamus/states.c
\brief The State Machine (and Arm/Disarm functions)

The state machine deals with which mode the craft is currently in.

\author Henry Fletcher
*/

#include "all.h"

// State
unsigned char armed=0;					/*!< State variable boolean for armed/disarmed */
unsigned char state=STATE_DISARMED;		/*!< State variable for state machine */
unsigned char airborne=0;				/*!< State variable for being ariborne or not */


/*!
\brief This is the state machine

*/
void state_machine()	{
	static unsigned char auto_lock = 0;
	static unsigned int throttle_on_count = 0;
	static unsigned int throttle_off_count = 0;
	static unsigned char auxStateIgnore = 0;
	float tempf;
	
	switch(state) {
		// ****************************************************************************
		// *** disarmed
		// ****************************************************************************
		default:
			state = STATE_DISARMED;
			// fallthrough!
		case STATE_DISARMED:
			// Key Parameters that need resetting/ setting whilst Disarmed
			auto_lock = 0;
			airborne = 0;
			throttle = 0;
			throttle_off_count = 0;
			throttle_on_count = 0;
		
			if((rxLoss < RX_PANIC) && (rxFirst != 0)) {
				///////////////////// OPERATION ///////////////////////////////
				// if left stick bottom and right stick bottom left then calibrate orientation
/*[ . ][.  ]*/	if(rcInput[RX_THRO] - throttletrim <  OFFSTICK && rcInput[RX_ELEV] < MINTHRESH && rcInput[RX_AILE] > MAXTHRESH) calibrate_ori();
				
				// if left stick bottom and right stick bottom right then calibrate magnetometer
/*[ . ][  .]*/	if(rcInput[RX_THRO] - throttletrim <  OFFSTICK && rcInput[RX_ELEV] < MINTHRESH && rcInput[RX_AILE] < MINTHRESH) calibrate_mag();

				///////////////////// STATE SWITCHING ///////////////////////////////
				// if left stick bottom right, right stick top left and switch = 0 then switch to manual mode with GPS inactive
/*[  .][`  ]*/	if(rcInput[RX_THRO] - throttletrim <  OFFSTICK && rcInput[RX_RUDD] < MINTHRESH && rcInput[RX_ELEV] > MAXTHRESH && rcInput[RX_AILE] > MAXTHRESH && auxState == 0) {
					if(ORI == detect_ori()) { 
						arm();
						state = STATE_MANUAL;
						// We hold the throttle off in case someone knocks it up after arming, the motors won't start until they have reduced the stick to zero and put it up again	
						hold_thro_off = 1;
					}
				}
				
				// if left stick bottom middle, right stick top left, switch = 0 and GPS is active then switch to manual mode with GPS active
/*[ . ][`  ]*/	if(rcInput[RX_THRO] - throttletrim <  OFFSTICK && rcInput[RX_RUDD] < MAXTHRESH && rcInput[RX_RUDD] > MINTHRESH && rcInput[RX_ELEV] > MAXTHRESH && rcInput[RX_AILE] > MAXTHRESH && auxState == 0 && gps_valid == 1) {
					if(ORI == detect_ori()) {                    
						arm();
						// Offset Barometer
						alt.barobias = (alt.gps - alt.baro);
						// Request that Hypo stores Return to Arm location
						ilink_gpsreq.request = 1;
						ilink_gpsreq.sequence++;
						// Set the New State
						state = STATE_MANUAL_GPS;
						// We hold the throttle off in case someone knocks it up after arming, the motors won't start until they have reduced the stick to zero and put it up again	
						hold_thro_off = 1;
					}
				}
				
				// if left stick bottom middle, right stick top right, switch = 1 and GPS is active  then switch to full auto
				// OR ARM request from Hypo
/*[ . ][  `]*/	if(rcInput[RX_THRO] - throttletrim <  OFFSTICK && gps_valid == 1 &&
				((rcInput[RX_RUDD] < MAXTHRESH && rcInput[RX_RUDD] > MINTHRESH && rcInput[RX_ELEV] > MAXTHRESH && rcInput[RX_AILE] < MINTHRESH && auxState == 1)
				|| (ilink_thalctrl_rx.isNew == 1 && ilink_thalctrl_rx.command == THALCTRL_ARM))) {
					if(ilink_thalctrl_rx.command == THALCTRL_ARM) {
						ilink_thalctrl_rx.isNew = 0;
						auxStateIgnore = 1;
					}
					if(ORI == detect_ori()) {                   
						arm();
						// Offset Barometer
						alt.barobias = (alt.gps - alt.baro);
						// Request that Hypo stores Return to Arm location
						ilink_gpsreq.request = 1;
						ilink_gpsreq.sequence++;
						// Set the New State
						state = STATE_AUTO;
						// We hold the throttle off in case someone knocks it up after arming, the motors won't start until they have reduced the stick to zero and put it up again	
						hold_thro_off = 1;
					}
				}
			}
		break;

		// ****************************************************************************
		// *** manual without GPS
		// ****************************************************************************
		case STATE_MANUAL:
			///////////////////// STATE SWITCHING ///////////////////////////////
			
			// if left stick bottom and right stick bottom then switch to Disarmed
			if(rcInput[RX_THRO] - throttletrim < OFFSTICK && (rcInput[RX_ELEV] < MINTHRESH || (ilink_thalctrl_rx.isNew == 1 && ilink_thalctrl_rx.command == THALCTRL_DISARM))) {
				if(ilink_thalctrl_rx.command == THALCTRL_DISARM) ilink_thalctrl_rx.isNew = 0;
				state = STATE_DISARMED;
				disarm();
			}
			
			// Auto Disarm
			if(throttle == 0) {
				throttle_on_count = 0;
				throttle_off_count++;
				// If throttle is off for 15 seconds
				if(throttle_off_count > SLOW_RATE*15) {
					state = STATE_DISARMED;
					disarm();
				}
			}
			
			///////////////////// OPERATION ///////////////////////////////
			
			// Airborne timer to enable auto land. Airborne must be set else autoland will not work.
			// Note that this code will auto throttle off/ "land" if you stay on the ground too long with the motors spinning and with an ultrasound module plugged in.
			if(throttle > 0) {
				throttle_off_count = 0;
				// If throttle is on for 6 seconds
				if(throttle_on_count > SLOW_RATE*6) {
					airborne = 1;
				}
				else {
					throttle_on_count++;
				}
			}	
			
			// Thalamus is allowed to turn the motors off
			thal_motor_off = 1;
			auto_lock = 0;
			
			ROLL_SPL_set = ROLL_SPL;
			PITCH_SPL_set = PITCH_SPL;
			YAW_SPL_set = YAW_SPL;
			
			// In manual mode, set pitch and roll demands based on the user commands collected from the rx unit
			user.pitch = -((float)MIDSTICK - (float)rcInput[RX_ELEV])*PITCH_SENS; 
			user.roll = ((float)MIDSTICK - (float)rcInput[RX_AILE])*ROLL_SENS;
			tempf = -(float)(yawtrim - rcInput[RX_RUDD])*YAW_SENS; 		
			
			throttle = ((signed int)rcInput[RX_THRO] - (signed int)throttletrim) * MAXTHROTTLE / ((signed int)MAXSTICK - (signed int)throttletrim); // all this should fit within signed ints. results in throttle between 0 and MAXTHROTTLE
			
			// A yaw rate is demanded by the rudder input, not an absolute angle.
			// This code increments the demanded angle at a rate proportional to the rudder input
			if(fabsf(tempf) > YAW_DEADZONE) {
				attitude_demand_body.yaw += tempf;
			}
		break;

		// ****************************************************************************
		// *** manual_gps
		// ****************************************************************************
		case STATE_MANUAL_GPS:
			///////////////////// STATE SWITCHING ///////////////////////////////
			
			// if left stick bottom and right stick bottom then switch to Disarmed
			if(rcInput[RX_THRO] - throttletrim <  OFFSTICK && (rcInput[RX_ELEV] < MINTHRESH || (ilink_thalctrl_rx.isNew == 1 && ilink_thalctrl_rx.command == THALCTRL_DISARM))) {
				if(ilink_thalctrl_rx.command == THALCTRL_DISARM) ilink_thalctrl_rx.isNew = 0;
				state = STATE_DISARMED;
				disarm();
			}
			
			// if we loose gps validity then switch into full manual mode
			if(gps_valid == 0) state = STATE_MANUAL;
			
			// If the switch is flicked, we go into AUTO mode
			if((auxState == 1 && auxStateIgnore == 0) || (ilink_thalctrl_rx.isNew == 1 && ilink_thalctrl_rx.command == THALCTRL_STATE_AUTO)) {
				if(ilink_thalctrl_rx.command == THALCTRL_STATE_AUTO) {
					ilink_thalctrl_rx.isNew = 0;
					auxStateIgnore = 1; // set ignore status otherwise mode jumps back immediately
				}
				state = STATE_AUTO;
				
				// request hold from Hypo
				ilink_gpsreq.request = 3;
				ilink_gpsreq.sequence++;
				// set button state to 1
				flapState = 1;		
			}
			if(auxState == 0) auxStateIgnore = 0; // when switch is flicked, reset ignore status

			// Auto Disarm
			if(throttle == 0) {
				throttle_off_count++;
				// If throttle is off for 15 seconds
				if(throttle_off_count > (SLOW_RATE*15)) {
					throttle_off_count = 0;
					throttle_on_count = 0;
					state = STATE_DISARMED;
					disarm();
					
				}
			}
			
			///////////////////// OPERATION ///////////////////////////////
			
			// Airborne timer to enable auto land. Airborne must be set else autoland will not work.
			// Note that this code will auto throttle off/ "land" if you stay on the ground too long with the motors spinning and with an ultrasound module plugged in.
			if(throttle > 0) {
				throttle_off_count = 0;
				throttle_on_count++;
				// If throttle is on for 6 seconds
				if(throttle_on_count > (SLOW_RATE*6)) {
					throttle_on_count = 0;
					airborne = 1;
				}
			}
			
			// Flap State determines whether we are free manual flying (0) Position Hold (1) or Heading Home (2)
			if(flapState == 0) {
				auto_lock = 0;
				// And Thalamus can't control the throttle
				thal_throt_cont = 0;				
				// Thalamus is allowed to turn the motors off
				thal_motor_off = 1;
				
				//We store key variables to ensure a stepless transition into other states
				// For both the GPS
				GPS_KerrI = throttle;
				ULT_KerrI = throttle;
				// and the ultrasound
				if(alt.ult_conf > 80) {
					targetZ_ult = alt.ultra;
					oldUltra = alt.ultra;
					got_setpoint = 1;
				}
			
				ROLL_SPL_set = ROLL_SPL;
				PITCH_SPL_set = PITCH_SPL;
				YAW_SPL_set = YAW_SPL;
			}
			
			// We always set pitch, roll and throttle from manual demands, but they can be overwritten in other situations
			// In manual mode, set pitch and roll demands based on the user commands collected from the rx unit
			user.pitch = -((float)MIDSTICK - (float)rcInput[RX_ELEV])*PITCH_SENS; 
			user.roll = ((float)MIDSTICK - (float)rcInput[RX_AILE])*ROLL_SENS;
			
			throttle = ((signed int)rcInput[RX_THRO] - (signed int)throttletrim) * (MAXTHROTTLE * MAXTHROTTLEPERCENT) / ((signed int)MAXSTICK - (signed int)throttletrim); // all this should fit within signed ints. results in throttle between 0 and MAXTHROTTLE*MAXTHROTTLEPERCENT
				
			// The pilot has control of yaw		
			tempf = -(float)(yawtrim - rcInput[RX_RUDD])*YAW_SENS; 									
			// A yaw rate is demanded by the rudder input, not an absolute angle.
			// This code increments the demanded angle at a rate proportional to the rudder input
			if(fabsf(tempf) > YAW_DEADZONE) {
				attitude_demand_body.yaw += tempf;
			}
			
			// and if we are position holding or flying home, Hypo has control
			if((flapState == 2) || (flapState == 1)) {
				
				// We make the craft change attitude slower when flying autonomously
				ROLL_SPL_set = 0.002;
				PITCH_SPL_set = 0.002;
				YAW_SPL_set = YAW_SPL/2;
				
				// then Hypo controls attitude.
				user.pitch = fsin(-psiAngle+M_PI_2) * ilink_gpsfly.northDemand - fsin(-psiAngle) * ilink_gpsfly.eastDemand;
				user.roll = fsin(-psiAngle) * ilink_gpsfly.northDemand + fsin(-psiAngle+M_PI_2) * ilink_gpsfly.eastDemand;
				
				// And Thalamus controls the throttle
				thal_throt_cont = 1;
				
				// And Thalamus is allowed to turn the motors off
				thal_motor_off = 1;
			}
		break;

		// ****************************************************************************
		// *** auto
		// ****************************************************************************
		case STATE_AUTO:
			
			///////////////////// STATE SWITCHING ///////////////////////////////
			
			// if left stick bottom and right stick bottom then switch to Disarmed
			if(rcInput[RX_THRO] - throttletrim <  OFFSTICK && (rcInput[RX_ELEV] < MINTHRESH || (ilink_thalctrl_rx.isNew == 1 && ilink_thalctrl_rx.command == THALCTRL_DISARM))) {
				if(ilink_thalctrl_rx.command == THALCTRL_DISARM) ilink_thalctrl_rx.isNew = 0;
				state = STATE_DISARMED;
				disarm();
			}
			
			// if we loose gps validity then immediately switch to manual without GPS mode
			if(gps_valid == 0) {
				state = STATE_MANUAL;
				auto_lock = 1;
			}
			
			// if the switch is put to zero, switch into manual_gps mode and lock out the auto code loop
			if((auxState == 0 && auxStateIgnore == 0) || (ilink_thalctrl_rx.isNew == 1 && ilink_thalctrl_rx.command == THALCTRL_STATE_MANUAL)) {
				if(ilink_thalctrl_rx.command == THALCTRL_STATE_MANUAL) {
					ilink_thalctrl_rx.isNew = 0;
					auxStateIgnore = 1; // set ignore status otherwise mode jumps back immediately
				}
				state = STATE_MANUAL_GPS;
				// ensure full manual control is engaged
				flapState = 0;
				//set idle mode on Hypo if we are in Manual MODE
				ilink_gpsreq.request = 7;
				ilink_gpsreq.sequence++;
				// and lock out the auto code loop
				auto_lock = 1;
			}
			if(auxState == 1) auxStateIgnore = 0; // when switch is flicked, reset ignore status
			
			// Auto Disarm
			if(throttle == 0) {
				throttle_off_count++;
				// If throttle is off for 20 seconds
				if(throttle_off_count > (SLOW_RATE*20)) {
					throttle_off_count = 0;
					throttle_on_count = 0;
					state = STATE_DISARMED;
					disarm();
					
				}
			}
			
			///////////////////// OPERATION ///////////////////////////////
			if(auto_lock == 0) {
				//When not airborne
				if(airborne == 0) {
				
					// Demand 0 radians pitch and roll
					user.pitch = 0;
					user.roll = 0;
				
					// Launch proceedure is scripted, don't let Thalamus interfere with the throttle levels.
					thal_throt_cont = 0;
					thal_motor_off = 0;
					
					
					////////////////// TAKEOFF CONTROL  ///////////////////////
					// If gps valid
					if((gps_valid == 1)
					// and throttlestick in middle
					&& ((rcInput[RX_THRO] - throttletrim) > 350) && ((rcInput[RX_THRO] - throttletrim) < 450))	{
						//If still on the ground (throttle zero), record altitude
						if((throttle >= 0) && (throttle < 200)) {				
							alt_tkoff = alt.filtered;
							throttle += 300;
						}
						// Increase throttle
						throttle += 0.4;
						
						if((alt.filtered > (alt_tkoff + 3)) || (alt.ultra > ULTRA_TKOFF)) { 
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
				else if(airborne == 1) {
					// then Hypo controls attitude.
					user.pitch = fsin(-psiAngle+M_PI_2) * ilink_gpsfly.northDemand - fsin(-psiAngle) * ilink_gpsfly.eastDemand;
					user.roll = fsin(-psiAngle) * ilink_gpsfly.northDemand + fsin(-psiAngle+M_PI_2) * ilink_gpsfly.eastDemand;
					
					// The pilot has control of yaw		
					tempf = -(float)(yawtrim - rcInput[RX_RUDD])*YAW_SENS; 									
					// A yaw rate is demanded by the rudder input, not an absolute angle.
					// This code increments the demanded angle at a rate proportional to the rudder input
					if(fabsf(tempf) > YAW_DEADZONE) {
						attitude_demand_body.yaw += tempf;
					}
			
					//if(!(ilink_gpsfly.flags & 0x04)) attitude_demand_body.yaw =  ilink_gpsfly.headingDemand;
					
					// We make the craft change attitude slower when flying autonomously
					ROLL_SPL_set = 0.002;
					PITCH_SPL_set = 0.002;
					YAW_SPL_set = YAW_SPL/2;
						
					
					// And Thalamus controls the throttle
					thal_throt_cont = 1;
					
					// And Thalamus is allowed to turn the motors off
					thal_motor_off = 1;
				}
			}
		break;
		
		
	}
	
	
	ilink_thalstat.flightStatus = state; // output state on telemetry
}

/*!
\brief Arms the craft.

A short gyro calibration is taken at the beginning of the function.
An animation is played on the motors
*/
void arm(void) {

	if(CAL_AUTO > 0) {
		calibrate_gyr_temporary(1);
	}
	
	PWMSetNESW(THROTTLEOFFSET, THROTTLEOFFSET, THROTTLEOFFSET, THROTTLEOFFSET);

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
	
	armed = 1;
	
	ilink_thalstat.systemStatus = THALSTAT_SYSTEMSTATUS_ACTIVE;
}

/*!
\brief Disarms the craft.

An animation is played on the motors
*/
void disarm(void) {
	if(armed) {
		PWMSetNESW(THROTTLEOFFSET, THROTTLEOFFSET, THROTTLEOFFSET, THROTTLEOFFSET);
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
	
	ilink_thalstat.systemStatus = THALSTAT_SYSTEMSTATUS_STANDBY;
}