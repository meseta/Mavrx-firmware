// Initialiser function: sets everything up.
void setup() {	
	
	throttle_angle = 0;
	
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
		//PWMInit(PWM_NESW);
		PWMInit(PWM_X | PWM_Y);
		RITInitms(1000/MESSAGE_LOOP_HZ);
		flashPLED = 0;
		LEDOff(PLED);
		
		gps_valid = 0;
		
}