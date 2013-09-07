

struct paramStorage_struct paramStorage[] = {
	{"DRIFT_AKp",		   0.2f},
	{"DRIFT_MKp",	  		0.2f},   
	#define DRIFT_AccelKp   paramStorage[0].value	
	#define DRIFT_MagKp	 paramStorage[1].value  
  
	{"LPF_ULTRA",	   0.05f},  
	#define LPF_ULTRA 		paramStorage[2].value

	{"YAW_SEN",	 0.0001f},	 
	{"PITCH_SEN",	0.0022f},	
	{"ROLL_SEN",	 0.0022f},  
	{"YAW_DZN",	  0.001f},	
	#define YAW_SENS		paramStorage[3].value
	#define PITCH_SENS	  paramStorage[4].value
	#define ROLL_SENS	   paramStorage[5].value
	#define YAW_DEADZONE	paramStorage[6].value 
	
	{"PITCH_Kp",	  400.0f},	 
	{"PITCH_Ki",		2.0f},	
	{"PITCH_Kd",	  100.0f},	 
	{"PITCH_Kdd",	1500.0f},
	{"PITCH_Bst",	 0.0f},	  
	{"PITCH_De",	  0.999f},	
	#define PITCH_Kp		paramStorage[7].value 
	#define PITCH_Ki		paramStorage[8].value
	#define PITCH_Kd		paramStorage[9].value 
	#define PITCH_Kdd	   paramStorage[10].value 
	#define PITCH_Boost	 paramStorage[11].value 
	#define PITCH_De		paramStorage[12].value 

	{"ROLL_Kp",	   400.0f},	   
	{"ROLL_Ki",		 2.0f},	  
	{"ROLL_Kd",	   100.0f},		
	{"ROLL_Kdd",	 1500.0f},	 		
	{"ROLL_Bst",	   0.00f},					
	{"ROLL_De",	   0.999f},
	#define ROLL_Kp		 paramStorage[13].value  
	#define ROLL_Ki		 paramStorage[14].value 
	#define ROLL_Kd		 paramStorage[15].value 
	#define ROLL_Kdd		paramStorage[16].value 	
	#define ROLL_Boost	  paramStorage[17].value 
	#define ROLL_De		 paramStorage[18].value

	{"YAW_Kp",		1000.0f},	 
	{"YAW_Kd",		250.0f},  
	{"YAW_Bst",		0.00f},   
	#define YAW_Kp		  paramStorage[19].value 
	#define YAW_Kd		  paramStorage[20].value 
	#define YAW_Boost	   paramStorage[21].value 	

	// Mode
	{"state",	   STATE_DISARMED},  //
	#define state 		paramStorage[22].value

	//Limits
	{"LIM_ANGLE",	   0.35f},  // Roll and Pitch Angle Limit in Radians
	{"LIM_ALT",		 1000.0f},  // Altitude Limit in mm when in Ultrasound Mode
	#define LIM_ANGLE 		paramStorage[23].value
	#define LIM_ALT 		paramStorage[24].value

	// Magneto Correction
	{"CAL_MAGN1",	 0.001756f},	  
	{"CAL_MAGN2",   0.00008370f},		
	{"CAL_MAGN3",   0.00005155f},	 	
	{"CAL_MAGN5",	 0.001964f},			
	{"CAL_MAGN6",   0.00002218f},	 
	{"CAL_MAGN9",	 0.001768f},	 
	{"CAL_MAGM1",	   0.0f},	 
	{"CAL_MAGM2",		0.0f},	 	
	{"CAL_MAGM3",		0.0f},  
	#define MAGCOR_N1	   paramStorage[25].value 
	#define MAGCOR_N2	   paramStorage[26].value 		
	#define MAGCOR_N3	   paramStorage[27].value 		
	#define MAGCOR_N5	   paramStorage[28].value 
	#define MAGCOR_N6	   paramStorage[29].value 
	#define MAGCOR_N9	   paramStorage[30].value 
	#define MAGCOR_M1	   paramStorage[31].value 		
	#define MAGCOR_M2	   paramStorage[32].value 
	#define MAGCOR_M3	   paramStorage[33].value

	// Ultrasound
	{"ULTRA_Kp",		0.05f},
	{"ULTRA_Kd",		0.1f},
	{"ULTRA_Ki",		0.00001f},
	{"ULTRA_De",	  	0.9999f},
	{"ULTRA_TKOFF",   	200.0f}, 
	{"ULTRA_LND",   	150.0f}, 
	#define ULTRA_Kp		paramStorage[34].value 
	#define ULTRA_Kd		paramStorage[35].value 
	#define ULTRA_Ki		paramStorage[36].value 
	#define ULTRA_De		paramStorage[37].value
	#define ULTRA_TKOFF	 paramStorage[38].value 
	#define ULTRA_LND	   paramStorage[39].value

	// TODO: I don't think these should be tunable parameters should they? Remember that the gyros are calibrated on every Arm
	{"CAL_GYROX",   0.0f},
	{"CAL_GYROY",   0.0f},
	{"CAL_GYROZ",   0.0f},
	#define CAL_GYROX	   paramStorage[40].value 
	#define CAL_GYROY	   paramStorage[41].value 
	#define CAL_GYROZ	   paramStorage[42].value 
	
	{"DETUNE",			0.2f},
	#define DETUNE		paramStorage[43].value
	
	{"LIM_RATE",			100.0f},
	#define LIM_RATE		paramStorage[44].value   
	{"LIM_ULTRA",			4.0f},
	#define LIM_ULTRA		paramStorage[45].value
	
	{"ULTRA_DRMP",	 3.0f}, 
	{"ULTRA_DTCT",	 6.0f},
	#define ULTRA_DRMP	  paramStorage[46].value
	#define ULTRA_DTCT	  paramStorage[47].value
	
	{"LIM_THROT", 		0.3f},
	#define LIM_THROT		paramStorage[48].value
	
	{"ULTRA_OVDEC",		0.01f},
	#define ULTRA_OVDEC		paramStorage[49].value
	
	{"ULTRA_DEAD",		100},
	#define ULTRA_DEAD		paramStorage[50].value
	
	{"ULTRA_OVTH",		40},
	#define ULTRA_OVTH		paramStorage[51].value
  
	
	{"CAL_AUTO", 1.0f},
	#define CAL_AUTO		paramStorage[52].value	
	
	
	{"LPF_OUT",	   0.6f},  
	#define LPF_OUT 		paramStorage[53].value
	
	
	{"BAT_LOW",		 11000.0f},
	{"BAT_CRIT",		10000.0f},
	#define BATT_LOW		paramStorage[54].value
	#define BATT_CRIT	   paramStorage[55].value
	
	{"ULTRA_OFFSET",		 350},
	#define ULTRA_OFFSET		paramStorage[56].value
	
	{"ROLL_SPL",		 0.02},
	#define ROLL_SPL		paramStorage[57].value
	{"PITCH_SPL",		 0.02},
	#define PITCH_SPL		paramStorage[58].value
	
	// TODO: Tune Yaw integral
	{"YAW_Ki",		 0.0},
	#define YAW_Ki		paramStorage[59].value
	{"YAW_De",		 1.0},
	#define YAW_De		paramStorage[60].value

	{"Filt_GPS_K",		 1.0},
	#define Filt_GPS_K		paramStorage[61].value

	{"LPF_BARO",   0.05},
	 #define LPF_BARO  paramStorage[62].value

	{"GPS_ALTKp", 5.0f},
    {"GPS_ALTKi", 0.0001f},
    {"GPS_ALTDe", 1.0f},
    {"GPS_ALTKd", 20.0f},
    #define GPS_ALTKp paramStorage[63].value
    #define GPS_ALTKi paramStorage[64].value
    #define GPS_ALTDe paramStorage[65].value
    #define GPS_ALTKd paramStorage[66].value

    {"Filt_baroK",		 0.0},
	#define Filt_baroK		paramStorage[67].value
	
	{"YAW_SPL",		 0.04},
	#define YAW_SPL		paramStorage[68].value
    
	{"ORI",		 0.00},
	#define ORI		paramStorage[69].value
	
};

// Initialiser function: sets everything up.
void setup() {	
	
	throttle_angle = 0;
	// *** Startup PWM
		// the ESC will beep when it doesn not receive a PWM signal, we use this to indicate that the craft is powered but disarmed
		// however, the ESC will not beep the first time it is powered, so we start up the PWM here and turn it off quickly so that
		// under ALL circumstances the ESC will beep when the craft is disarmed.
		PWMInit(PWM_ALL);
		PWMSetNESW(THROTTLEOFFSET, THROTTLEOFFSET, THROTTLEOFFSET, THROTTLEOFFSET);
	
	// *** LED setup
		LEDInit(PLED | VLED);
		LEDOff(PLED | VLED);
		
		flashPLED = 0;
		flashVLED = 0;
		flashRLED = 0;
	
		armed = 0;
		calib = 0;
		zeroThrotCounter = 0;
	
	// *** Timers and couters6
		rxLoss = 50;
		ultraLoss = ULTRA_OVTH + 1;
		sysMS = 0;
		sysUS = 0;
		SysTickInit();  // SysTick enable (default 1ms)
		

	// *** Parameters
		paramCount = sizeof(paramStorage)/20;
		eeprom_load_all();
		paramSendCount = paramCount;
		paramSendSingle = 0;
	
	// *** Establish ILink
		ilink_thalstat.sensorStatus = 1; // set ilink status to boot
		ilink_thalstat.flightMode = (0x1 << 0); // attitude control
		ilink_identify.deviceID = WHO_AM_I;
		ilink_identify.firmVersion = FIRMWARE_VERSION;
		ILinkInit(SLAVE);
	
	// *** Initialise input
		throttle = 0;
		rxFirst = 0;
		auxState = 0;
		RXInit();
		state = STATE_DISARMED;
				
	// *** Initialise Ultrasound
		UltraInit();

	// *** Battery sensor
		ADCInit(CHN7);
		ADCTrigger(CHN7);
		Delay(1);
		ilink_thalstat.battVoltage = (ADCGet() * 6325) >> 10; // Because the factor is 6325/1024, we can do this in integer maths by right-shifting 10 bits instead of dividing by 1024.
		batteryVoltage = 0;
		
	// *** Calibrate Sensors
		Delay(500);
		SensorInit();
		sensor_zero();
		
		TrigBaroTemp(); // get parometer temperature for temperature compensation.
		Delay(15);
		GetBaroTemp();
		
		
	// *** quaternion AHRS init
		q1 = 1;
		q2 = 0;
		q3 = 0;
		q4 = 0;
		
	// *** Rotation matrix Initialisation
		M1 = 1;
		M2 = 0;
		M3 = 0;
		M4 = 0;
		M5 = 1;
		M6 = 0;
		M7 = 0;
		M8 = 0;
		M9 = 1;
		
	// *** Rotation matrix Initialisation
		RM1 = 1;
		RM2 = 0;
		RM3 = 0;
		RM4 = 0;
		RM5 = 1;
		RM6 = 0;
		RM7 = 0;
		RM8 = 0;
		RM9 = 1;
		
		
		// Temporarily store drift correction multipliers
		float tempAccelKp = DRIFT_AccelKp;
		float tempMagKp = DRIFT_MagKp;
		
		// Set high confidence in accelerometer/magneto to rotate AHRS to initial heading
		DRIFT_MagKp = 20;
		DRIFT_AccelKp = 20;
		
		//Timer for AHRS
		Timer0Init(59);
		Timer0Match0(1200000/FAST_RATE, INTERRUPT | RESET);
		Delay(1000);
	   
		// Now Set them back to their original values
		DRIFT_MagKp = tempMagKp;
		DRIFT_AccelKp = tempAccelKp;
	
		slowSoftscale = 0;
	
		//Initialise PWM outputs
		motorN = 0;
		motorE = 0;
		motorS = 0;
		motorW = 0;
		
		motorNav = 0;
		motorEav = 0;
		motorSav = 0;
		motorWav = 0;
		
		Gyro.Y.error = 0;
		Gyro.X.error = 0;
		Gyro.Z.error = 0;
		
		// We start with throttle hold on in case the user has forgotten to lower their throttle stick
		hold_thro_off = 1;
		
	// *** Initialise timers and loops
		PWMSetNESW(0, 0, 0, 0);
		RITInitms(1000/MESSAGE_LOOP_HZ);
		flashPLED = 0;
		LEDOff(PLED);
		
		gps_valid = 0;
		alt.barobias = 0;
		
		ROLL_SPL_set = ROLL_SPL;
		PITCH_SPL_set = PITCH_SPL;
		YAW_SPL_set = YAW_SPL;		
		
		// gps.request
		// 0 = Nothing
		// 1 = set home
		// 2 = take off
		// 3 = hold/ pause
		// 4 = resume/ go
		// 5 = land
		// 6 = go home and land
		// After the request variable is changed, the sequence variable must be incremented by one to make Hypo read the request
		ilink_gpsreq.request = 0;
		ilink_gpsreq.sequence = 0;
		
}