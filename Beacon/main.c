/*!
\file Beacon/main.c
\brief Beacon main file

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
	//MAV_ID = ReadUIDHash() & 0xff; // get last byte of hash and use as default ID
	
    // *** XBee and MAVLink
    XBeeInit();
    /*MAVLinkInit();
    MAVSendHeartbeat();*/

    // *** GPS
    GPSInit();
    GPSSetRate(ID_NAV_POSLLH, 1); // every one nav solutions (i.e. as fast as possible)
    GPSSetRate(ID_NAV_STATUS, 3); // every three nav solutions
    GPSSetRate(ID_NAV_VELNED, 1);
	
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
        //flashVLED = 5;
    }
	
    // GPS watchdog
    gpsWatchdog++;
    if(gpsWatchdog >= MESSAGE_LOOP_HZ*GPS_PANIC) {
        // GPS signal loss/no-fix
        gpsWatchdog = MESSAGE_LOOP_HZ*(GPS_PANIC+1); // prevent overflow
        //mavlink_gps_raw_int.fix_type = 0;
        //mavlink_gps_raw_int.satellites_visible = 0;
        //gpsFixed = 0;
    }
    
	// *** Outgoing heartbeat
    heartbeatCounter++;
    if(heartbeatCounter >= MESSAGE_LOOP_HZ) { // 1Hz loop
        heartbeatCounter = 0;
        //MAVSendHeartbeat();
    }
    
	// Telemetry
    if(allowTransmit) {
		//paramater_transmit();
        //mavlink_telemetry();
		//mavlink_messages();
    }
	
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
}

