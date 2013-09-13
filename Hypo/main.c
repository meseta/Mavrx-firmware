/*!
\file Hypo/main.c
\brief Hypo main file

On bootup, a startup function is run (part of the function library, and 
not	documented in this code.  The startuf function after initialising 
the hardware proceeds to invoke the function setup() followed by loop().

This file contains setup() and loop(), as well as some of the interrupt 
service routines that are set to run repeatedly.

\author Yuan Gao
*/

#include "all.h"

/*!
\brief This is the setup function, sets everything up.

This is the first user function called after the initialisation is complete.
All the hardware peripherals and stuff are initialised here.
*/
void setup() {
    // *** LED setup
    LEDInit(PLED);
    LEDOn(PLED);
	
	// *** EEPROM and params
	MAV_ID = ReadUIDHash() & 0xff; // get last byte of hash and use as default ID
	eeprom_load_all();
	
    // *** XBee and MAVLink
    allowTransmit = 1;
    XBeeInit();
    MAVLinkInit();
    MAVSendHeartbeat();

    // *** GPS
    GPSInit();
    GPSSetRate(ID_NAV_POSLLH, 1); // every one nav solutions (i.e. as fast as possible)
    GPSSetRate(ID_NAV_STATUS, 3); // every three nav solutions
    GPSSetRate(ID_NAV_VELNED, 1);
    
    mavlink_sys_status.onboard_control_sensors_present |= MAVLINK_SENSOR_GPS | MAVLINK_CONTROL_Z | MAVLINK_CONTROL_XY;
    mavlink_sys_status.onboard_control_sensors_enabled |= MAVLINK_SENSOR_GPS;
    mavlink_sys_status.onboard_control_sensors_health = mavlink_sys_status.onboard_control_sensors_enabled;

    // *** Establish ILink and Look for Thalamus
    ILinkInit(2000);
    XBeeInhibit();
    ILinkPoll(ID_ILINK_CLEARBUF); // forces Thalamus to clear its output buffers
	unsigned int i;
	for(i=0; i< 512; i++) { // force flush anyway
		SSP0Byte(0xffff);
	}
	ILinkPoll(ID_ILINK_IDENTIFY);
    XBeeAllow();
	
    // *** Things start happening as soon as RIT is enabled!
    RITInitms(1000/MESSAGE_LOOP_HZ);  // RIT at 25Hz
    LEDOff(PLED);
    LEDInit(VLED);
	Port0Init(PIN13 | PIN14 | PIN15 | PIN16);
	Port0SetOut(PIN13 | PIN14 | PIN15 | PIN16);
}

/*!
\brief Main loop (idle loop, doesn't do much)

This is the main loop, the processor sits in here when it's not dealing with 
interrupts.  This loop's only purpose right now is to deal with button presses,
and XBee modem status
*/
void loop() {
    if(idleCount < IDLE_MAX) idleCount++; // this is the counter for CPU idle time
    
    if(PRGBlankTimer == 0) {
        if(PRGTimer > 3000) {
            flashVLED = 0;
            LEDOn(VLED);
        }
        
        if(PRGPushTime > 3000) {
            flashVLED = 0xffffffffUL;
            XBeeJoin();
            flashVLED = 5;
            
            PRGPushTime = 0;
            PRGTimer = 0;
            PRGBlankTimer = 100;
        }
    }
    
    if(xbee_modem_status.isNew) {
        xbee_modem_status.isNew = 0;
        if(xbee_modem_status.status == 2) {
            allowTransmit = 1;
        }
        else if(xbee_modem_status.status == 3) {
            allowTransmit = 0;
        }
    }
}

/*!
\brief System Tick Timer, deals with timing, flashing, pushing.

This function is triggered by interrupt every 1ms, its purpose is to keep
timings, flash LEDs, time button pushes, and miscellaneous other timing needs
*/
void SysTickInterrupt(void) {
    // deal with button(s)
    if(PRGBlankTimer) {
        PRGBlankTimer--;
    }
    else {
        if(PRGPoll() == 0) {
            PRGTimer++;
            PRGLastState = 0;
        }
        else {
            if(PRGLastState == 0) {
                PRGPushTime = PRGTimer;
            }
            PRGTimer = 0;
            PRGLastState = 1;
        }
    }
    
    sysMS ++;
    sysUS += 1000;
    
    if(waypointLoiterTimer < 0xffffffff) waypointLoiterTimer ++;
    
    // deal with flashing LEDs
    if(sysMS % 100 == 0) {
        if(flashVLED > 0) {
            flashVLED--;
            LEDToggle(VLED);
            if(flashVLED == 0) LEDOff(VLED);
        }
    }
}

/*!
\brief Repetitive Interrput Timer, this is the main action loop.

This function is triggered by interrupt every few tens of ms (actual speed, is 
defined by MESSAGE_LOOP_HZ).
*/
void RITInterrupt(void) {
	static unsigned char heartbeatCounter=0;

    // *** Watchdogs
    // Incoming heartbeat watchdog
    heartbeatWatchdog++;
    if(heartbeatWatchdog >= MESSAGE_LOOP_HZ*XBEE_PANIC) {
        // heartbeat from ground control lost
        heartbeatWatchdog = MESSAGE_LOOP_HZ*(XBEE_PANIC+1); // prevent overflow
        LEDOn(PLED);
    }
    else {
        LEDOff(PLED);
    }
	
    // GPS watchdog
    gpsWatchdog++;
    if(gpsWatchdog >= MESSAGE_LOOP_HZ*GPS_PANIC) {
        // GPS signal loss/no-fix
        gpsWatchdog = MESSAGE_LOOP_HZ*(GPS_PANIC+1); // prevent overflow
        mavlink_gps_raw_int.fix_type = 0;
        mavlink_gps_raw_int.satellites_visible = 0;
        gpsFixed = 0;
    }
        
    // Thalamus watchdog
    thalWatchdog++;
    if(thalWatchdog >= MESSAGE_LOOP_HZ*THAL_PANIC) {
        // thalamus signal loss/no-fix
		// should probably output something intelligible here to signify thalamus lost
        //thalWatchdog = MESSAGE_LOOP_HZ*(THAL_PANIC+1); // prevent overflow
        thalWatchdog = 0;
		thalAvailable = 0;
        XBeeInhibit();
        ILinkPoll(ID_ILINK_IDENTIFY);
        XBeeAllow();
    }
    
	// *** Outgoing heartbeat
    heartbeatCounter++;
    if(heartbeatCounter >= MESSAGE_LOOP_HZ) { // 1Hz loop
        heartbeatCounter = 0;
        MAVSendHeartbeat();
        
        unsigned short load = (1000*(IDLE_MAX - idleCount))/IDLE_MAX; // Idle load is calculated here as this loop runs at 1Hz
        if(mavlink_sys_status.load == 0) mavlink_sys_status.load = load;
        mavlink_sys_status.load *= IDLE_SPRF;
        mavlink_sys_status.load += (1-IDLE_SPRF)*load;
        
        idleCount = 0;
    }
	
	// *** Process ILink
    XBeeInhibit();
    ILinkFetchData();
    XBeeAllow();
    
	// Telemetry
    if(allowTransmit) {
		paramater_transmit();
        mavlink_telemetry();
		mavlink_messages();
    }
	
	// *** Process ILink
	XBeeInhibit();
	ILinkFetchData();
	XBeeAllow();
	
	// GPS
	static unsigned short gpsFetchCounter = 0;
	gpsFetchCounter++;
	if(gpsFetchCounter > MESSAGE_LOOP_HZ/10) {
		gpsFetchCounter = 0; // fetch GPS at 10Hz
		// *** Process GPS
		XBeeInhibit(); // XBee input needs to be inhibited while processing GPS to avoid disrupting the I2C
		GPSFetchData();
		XBeeAllow();
	}
    gps_navigate();
	
    // *** Process ILink
    XBeeInhibit();
    ILinkFetchData();
    XBeeAllow();
}

