// ****************************************************************************
// *** Arm, Disarm and Calibrate
// ****************************************************************************

void read_sticks(){
	///////////////////////// REGULAR MODE ///////////////////////////////////
	if (MODE_ST == 1) { 
	
	
		// Arm, Disarm and Calibrate
		if(rcInput[RX_THRO] - throttletrim <  OFFSTICK && rxFirst != 0) {
			if(rxLoss < 50) {
				if(rcInput[RX_AILE] < MAXTHRESH && rcInput[RX_AILE] > MINTHRESH) {
					if(rcInput[RX_ELEV] > MAXTHRESH) {
						// Arm
						if(zeroThrotCounter++ > ZEROTHROTMAX) {
							zeroThrotCounter = 0;

							Arm();
						}
					}
					else if(rcInput[RX_ELEV] < MINTHRESH) {
						// Disarm
						if(zeroThrotCounter++ > ZEROTHROTMAX) {
							zeroThrotCounter = 0;
							Disarm();
						}
					}
					else {
						zeroThrotCounter = 0;
					}
				}
				else if(rcInput[RX_ELEV] < MAXTHRESH && rcInput[RX_ELEV] > MINTHRESH) {
					if(rcInput[RX_AILE] > MAXTHRESH) {
						// Request gyro calibration
						// TODO: This needs to be swapped for orientation calibration
						if(zeroThrotCounter++ > ZEROTHROTMAX) {
							zeroThrotCounter = 0;
							CalibrateGyro();
						}
					}
					else if(rcInput[RX_AILE] < MINTHRESH) {
						// Request magneto calibration
						if(zeroThrotCounter++ > ZEROTHROTMAX) {
							zeroThrotCounter = 0;
							CalibrateMagneto();
						}
					}
				}
				
			}
			else {
				zeroThrotCounter = 0;
			}
  
		}
		
		// General Flight Demands
		else {	
			// In manual mode, set pitch and roll demands based on the user commands collected from the rx unit
			pitch.demand = -((float)MIDSTICK - (float)rcInput[RX_ELEV])*PITCH_SENS; 
			roll.demand = ((float)MIDSTICK - (float)rcInput[RX_AILE])*ROLL_SENS;
			float tempf = -(float)(yawtrim - rcInput[RX_RUDD])*YAW_SENS; 						
			throttle = rcInput[RX_THRO] - throttletrim;
			if (throttle < 0) throttle = 0;
			
			// Pitch and roll demands can be remapped to different variables if the orientation of the 
			// board in the craft is non standard or if the pitch and roll demands need to vary with position
			pitchDemandSpin = pitch.demand;
			rollDemandSpin = roll.demand;
			
			// A yaw rate is demanded by the rudder input, not an absolute angle.
			// This code increments the demanded angle at a rate proportional to the rudder input
			if(fabsf(tempf) > YAW_DEADZONE) {
				yaw.demand += tempf;
				if(yaw.demand > M_PI) {
					yaw.demand -= M_TWOPI;
					yaw.demandOld -= M_TWOPI;
				}
				else if(yaw.demand < -M_PI) {
					yaw.demand += M_TWOPI;
					yaw.demandOld += M_TWOPI;
				}
			} 
		}
		
	
	}
}

void ReadRXInput(void) {			

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
		}
		
		// Controller's aux or gear switch (Can be switched permanently either way)
		if(rcInput[RX_AUX1] > MIDSTICK) {
            // TEMPORARY FOR TESTING, switches into GPS mode
            if(auxState != 0) {
                ilink_thalctrl_tx.command = 0x91;
                if(ILinkSendMessage(ID_ILINK_THALCTRL, (unsigned short *) &ilink_thalctrl_tx, sizeof(ilink_thalctrl_tx)/2 - 1)) {
                    auxState = 0;
                }
            }
		}
		else {
            // TEMPORARY FOR TESTING, switches into GPS mode
            if(auxState != 1) {
                ilink_thalctrl_tx.command = 0x90;
                if(ILinkSendMessage(ID_ILINK_THALCTRL, (unsigned short *) &ilink_thalctrl_tx, sizeof(ilink_thalctrl_tx)/2 - 1)) {
                    auxState = 1;
                }
            }
		}
		
		
		// Controller's flap switch/ button (mechanically sprung return)
		if(rcInput[RX_FLAP] > MIDSTICK) {
			if((flapState == 0) && (flpswitch == 0)) {
				flapState = 1;
				flpswitch = 1;
			}
			if((flapState == 1) && (flpswitch == 0)) {
				flapState = 0;
				flpswitch = 1;
			}
			
		}
		else {
			flpswitch = 0;
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