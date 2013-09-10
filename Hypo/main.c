#include "all.h"

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
    ILinkInit(4000);
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
}

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

void RITInterrupt(void) {
    heartbeatCounter++;
    heartbeatWatchdog++;
    gpsWatchdog++;
    statusCounter++;
    thalWatchdog++;
    rawSensorStreamCounter++;
    extStatusStreamCounter++;
    rcChannelCounter++;
    rawControllerCounter++;
    positionStreamCounter++;
    extra1ChannelCounter++;
    extra2ChannelCounter++;
    extra3ChannelCounter++;
    waypointTimer++;
    
    gpsSendCounter++;
	
    // *** Watchdogs
    // Incoming heartbeat watchdog
    if(heartbeatWatchdog >= MESSAGE_LOOP_HZ*XBEE_PANIC) {
        // heartbeat from ground control lost
        heartbeatWatchdog = MESSAGE_LOOP_HZ*(XBEE_PANIC+1); // prevent overflow
        //flashVLED = 5;
    }
    
    // GPS watchdog
    if(gpsWatchdog >= MESSAGE_LOOP_HZ*GPS_PANIC) {
        // GPS signal loss/no-fix
        gpsWatchdog = MESSAGE_LOOP_HZ*(GPS_PANIC+1); // prevent overflow
        mavlink_gps_raw_int.fix_type = 0;
        mavlink_gps_raw_int.satellites_visible = 0;
        gpsFixed = 0;
    }
        
    // Thalamus watchdog
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
    
	// *** MAVLink messages
    if(heartbeatCounter >= MESSAGE_LOOP_HZ) { // 1Hz loop
        heartbeatCounter = 0;
        MAVSendHeartbeat();
        
        unsigned short load = (1000*(IDLE_MAX - idleCount))/IDLE_MAX; // Idle load is calculated here as this loop runs at 1Hz
        if(mavlink_sys_status.load == 0) mavlink_sys_status.load = load;
        mavlink_sys_status.load *= IDLE_SPRF;
        mavlink_sys_status.load += (1-IDLE_SPRF)*load;
        
        idleCount = 0;
    }
	
    // *** Process GPS
    XBeeInhibit(); // XBee input needs to be inhibited while processing GPS to avoid disrupting the I2C
    GPSFetchData();
    XBeeAllow();

    gps_navigate();
    
    if(allowTransmit) {
		paramater_transmit();
        mavlink_telemetry();
    }

    // *** Process ILink
    XBeeInhibit();
    ILinkFetchData();
    XBeeAllow();
	
}

