#include "thal.h"
#include "mavlink.h"

// *** Config stuff
#define FIRMWARE_VERSION    1           // Firmware version
#define MESSAGE_LOOP_HZ     50          // Max frequency of message loop in Hz (keep this number low, like around 25)
#define XBEE_PANIC          3           // Number of seconds after missing heartbeat from GCS before considering XBee fail
#define GPS_PANIC           3           // Number of seconds after no message from GPS before considering GPS fail
#define THAL_PANIC          3           // Number of seconds after no message from Thalamus before considering Thalamus fail
#define IDLE_SPRF           0.9         // Some filtering on the CPU load
#define IDLE_MAX            0x1ff480    // Set this to the number that the idleCounter counts up to every second.  This is used to measure the CPU load - when there are no interrupts (i.e. processor not "busy"), the idleCounter is being incremented

#define MAX_WAYPOINTS       20 // TODO: maybe move waypoints into EEPROM
#define WAYPOINT_HOME       MAX_WAYPOINTS+1 // there is one waypoint storage location that is never used between Waypoint home and the last waypoint, this is in case I screw up with the waypoint management and accidentally try to send the craft back to home by mistake
#define WAYPOINT_TEMP       MAX_WAYPOINTS+2
#define WAYPOINT_TIMEOUT    500/MESSAGE_LOOP_HZ // value = timeout in ms / message_loop_hz
#define WAYPOINT_TRIES      5   // number of times to retry waypoint
#define WAYPOINT_MIN_RADIUS 1
#define WAYPOINT_SAFE_HEIGHT 2  // altitude above home to use


// *** Status stuff
volatile unsigned int allowTransmit;

// *** LEDs and buttons stuff
volatile unsigned int flashVLED;
volatile unsigned int PRGTimer;
volatile unsigned int PRGLastState;
volatile unsigned int PRGPushTime;
volatile unsigned int PRGBlankTimer;

unsigned int counter = 0;

// *** Timers and counters
unsigned int sysMS;
unsigned long long sysUS;
unsigned int idleCount;
unsigned short heartbeatWatchdog;
unsigned short gpsWatchdog;
unsigned short thalWatchdog;

// *** MAVLINK stuff
unsigned char mavlinkID;

// Sent messages
mavlink_status_t mavlink_status;
mavlink_message_t mavlink_tx_msg;
mavlink_heartbeat_t mavlink_heartbeat;
mavlink_sys_status_t mavlink_sys_status;
mavlink_gps_raw_int_t mavlink_gps_raw_int;
mavlink_raw_imu_t mavlink_raw_imu;
mavlink_scaled_imu_t mavlink_scaled_imu;
mavlink_attitude_t mavlink_attitude;
mavlink_command_ack_t mavlink_command_ack;
mavlink_param_value_t mavlink_param_value;
mavlink_rc_channels_raw_t mavlink_rc_channels_raw;
mavlink_rc_channels_scaled_t mavlink_rc_channels_scaled;
mavlink_servo_output_raw_t mavlink_servo_output_raw;
mavlink_gps_global_origin_t mavlink_gps_global_origin;

mavlink_named_value_float_t mavlink_named_value_float;
mavlink_named_value_int_t mavlink_named_value_int;
mavlink_debug_vect_t mavlink_debug_vect;
mavlink_debug_t mavlink_debug;
mavlink_statustext_t mavlink_statustext;

mavlink_mission_request_t mavlink_mission_request;
mavlink_mission_item_t mavlink_mission_item;
mavlink_mission_ack_t mavlink_mission_ack;
mavlink_mission_item_reached_t mavlink_mission_item_reached;


// Received messages
mavlink_message_t mavlink_rx_msg;
mavlink_request_data_stream_t mavlink_request_data_stream;
mavlink_command_long_t mavlink_command_long;
mavlink_param_request_list_t mavlink_param_request_list;
mavlink_param_set_t mavlink_param_set;
mavlink_param_request_read_t mavlink_param_request_read;
mavlink_manual_control_t mavlink_manual_control;
mavlink_mission_count_t mavlink_mission_count;
mavlink_set_gps_global_origin_t mavlink_set_gps_global_origin;
mavlink_mission_request_list_t mavlink_mission_request_list;
mavlink_mission_count_t mavlink_mission_count;
mavlink_mission_clear_all_t mavlink_mission_clear_all;
mavlink_mission_set_current_t mavlink_mission_set_current;
mavlink_mission_current_t mavlink_mission_current;
mavlink_set_mode_t mavlink_set_mode;

// Disabled messages
// mavlink_global_position_setpoint_int_t
// 
// mavlink_raw_pressure_t
// mavlink_vfr_hud_t

mavlink_mission_request_list_t mavlink_mission_request_list;

// Variables
unsigned char mavlink_message_buf[MAVLINK_MAX_PACKET_LEN];
unsigned short mavlink_message_len;

// Timers
unsigned char dataRate[MAV_DATA_STREAM_ENUM_END];
unsigned char heartbeatCounter;
unsigned short extra3ChannelCounter;
unsigned short extra2ChannelCounter;
unsigned short extra1ChannelCounter;
unsigned short rcChannelCounter;
unsigned short rawControllerCounter;
unsigned short positionStreamCounter;
unsigned short extStatusStreamCounter;
unsigned short rawSensorStreamCounter;

unsigned int gpsSendCounter, posupdate;

// Functions
void MAVLinkInit(void);
void MAVSendHeartbeat(void);
void MAVSendFloat(char * name, float value);
void MAVSendInt(char * name, int value);
void MAVSendVector(char * name, float valX, float valY, float valZ);
void MAVSendText(unsigned char severity, char * text);
void MAVLinkParse(unsigned char UARTData);

// *** GPS stuff
gps_nav_posecef_t gps_nav_posecef;
gps_nav_posllh_t gps_nav_posllh;
gps_nav_status_t gps_nav_status;
gps_nav_sol_t gps_nav_sol;
gps_nav_velned_t gps_nav_velned;
gps_nav_timeutc_t gps_nav_timeutc;

// *** ILink stuff
ilink_identify_t ilink_identify;
ilink_thalstat_t ilink_thalstat;
ilink_thalctrl_t ilink_thalctrl_rx;
ilink_imu_t ilink_rawimu;
ilink_imu_t ilink_scaledimu;
ilink_altitude_t ilink_altitude;
ilink_attitude_t ilink_attitude;
ilink_thalparam_t ilink_thalparam_rx;
ilink_thalparam_t ilink_thalparam_tx;
ilink_thalpareq_t ilink_thalpareq;
ilink_iochan_t ilink_inputs0;
ilink_iochan_t ilink_outputs0;
ilink_atdemand_t ilink_atdemand;
ilink_gpsfly_t ilink_gpsfly;
ilink_debug_t ilink_debug;

typedef struct paramBuffer_struct {
    char name[16];
    float value;
    unsigned short id;
} paramBuffer_t;

#define PARAMBUFFER_SIZE    50
paramBuffer_t paramBuffer[PARAMBUFFER_SIZE];
unsigned int paramPointer = PARAMBUFFER_SIZE;

// *** Xbee stuff
xbee_modem_status_t xbee_modem_status;
xbee_at_command_t xbee_at_command;
xbee_at_response_t xbee_at_response;
xbee_receive_packet_t xbee_receive_packet;
xbee_transmit_request_t xbee_transmit_request;

// ****** waypoints
typedef struct {

    unsigned char command;
    unsigned char autocontinue;
    float param1;
    float param2;
    float param3;
    float param4;
    float x;
    float y;
    float z;
} waypointStruct;

waypointStruct waypoint[MAX_WAYPOINTS+3];

unsigned short waypointCurrent, waypointCount, waypointReceiveIndex;
unsigned char waypointTries, waypointValid, waypointHomeValid, waypointGo, waypointReached;
unsigned short waypointTimer;
unsigned int waypointLoiter;
unsigned char waypointProviderID, waypointProviderComp;
float waypointPhase;

float lat_diff_i;
float lon_diff_i;

float GPS_Kp = 10.0f;
float GPS_Ki = 0.0001f;
float GPS_Kd = 20.0f;

unsigned int gpsFixed;
unsigned int gpsChange;

unsigned int horizontalHold;
float horizontalHoldLon;
float horizontalHoldLat;

// ****************************************************************************
// *** Initialiseation
// ****************************************************************************

void setup() {
    // *** LED setup
    LEDInit(PLED);
    LEDOn(PLED);
    flashVLED = 0;
    
    // *** Timers and couters
    heartbeatWatchdog = 0;
    gpsWatchdog = 0;
    thalWatchdog = 0;
    sysMS = 0;
    sysUS = 0;
    idleCount = 0;
    
    // *** XBee and MAVLink
    allowTransmit = 1;
    XBeeInit();
    MAVLinkInit();
    heartbeatCounter = 0;
    MAVSendHeartbeat();

    // *** GPS
    GPSInit();
    GPSSetRate(ID_NAV_POSLLH, 5);
    GPSSetRate(ID_NAV_STATUS, 5);
    GPSSetRate(ID_NAV_VELNED, 5);
    
    lat_diff_i = 0;
    lon_diff_i = 0;
    horizontalHold = 0;
    gpsFixed = 0;
    
    mavlink_sys_status.onboard_control_sensors_present |= MAVLINK_SENSOR_GPS | MAVLINK_CONTROL_Z | MAVLINK_CONTROL_XY;
    mavlink_sys_status.onboard_control_sensors_enabled |= MAVLINK_SENSOR_GPS;
    mavlink_sys_status.onboard_control_sensors_health = mavlink_sys_status.onboard_control_sensors_enabled;

    // *** Waypoints
    waypointCurrent = 0;
    waypointGo = 0;
    waypointHomeValid = 0;
    waypointCount = 0;
    waypointValid = 0;
    waypointTimer = 0;
    waypointReceiveIndex = 0;
    
    // *** Establish ILink and Look for Thalamus
    ILinkInit(6000);
    XBeeInhibit();
    ILinkPoll(ID_ILINK_CLEARBUF); // forces Thalamus to clear its output buffers
	ILinkPoll(ID_ILINK_IDENTIFY);
    XBeeAllow();
	
    // *** Things start happening as soon as RIT is enabled!
    RITInitms(1000/MESSAGE_LOOP_HZ);  // RIT at 25Hz
    LEDOff(PLED);
    LEDInit(VLED);
}

void MAVLinkInit() {
    mavlinkID = 10;
    
    mavlink_heartbeat.type = MAV_TYPE_QUADROTOR;
    mavlink_heartbeat.autopilot = MAV_AUTOPILOT_GENERIC;
    mavlink_heartbeat.base_mode = MAV_MODE_PREFLIGHT;
    mavlink_heartbeat.system_status = MAV_STATE_BOOT;
    mavlink_heartbeat.mavlink_version = MAVLINK_VERSION;
    
    mavlink_sys_status.onboard_control_sensors_present = 0;
    mavlink_sys_status.onboard_control_sensors_enabled = 0;
    mavlink_sys_status.onboard_control_sensors_health = 0;
    mavlink_sys_status.load = 0;
    mavlink_sys_status.voltage_battery = 0;
    mavlink_sys_status.current_battery = -1;
    mavlink_sys_status.battery_remaining = -1;
    
    dataRate[MAV_DATA_STREAM_EXTENDED_STATUS] = 2;
    dataRate[MAV_DATA_STREAM_POSITION] = 5;
    dataRate[MAV_DATA_STREAM_RAW_SENSORS] = 0;
    dataRate[MAV_DATA_STREAM_RAW_CONTROLLER] = 0;
    dataRate[MAV_DATA_STREAM_RC_CHANNELS] = 0;
    dataRate[MAV_DATA_STREAM_POSITION] = 0;
    dataRate[MAV_DATA_STREAM_EXTRA1] = 0;
    dataRate[MAV_DATA_STREAM_EXTRA2] = 0;
    dataRate[MAV_DATA_STREAM_EXTRA3] = 0;
}

// ****************************************************************************
// *** Main loop and timer loops
// ****************************************************************************

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
    
    if(waypointLoiter < 0xffffffff) waypointLoiter ++;
    
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
        XBeeInhibit();
        ILinkPoll(ID_ILINK_IDENTIFY);
        XBeeAllow();
    }
    
    // *** Status and GPS
    
        
    // *** Process GPS
    XBeeInhibit(); // XBee input needs to be inhibited while processing GPS to avoid disrupting the I2C
    GPSFetchData();
    XBeeAllow();
    
    if(gpsSendCounter >= MESSAGE_LOOP_HZ/5) {
        gpsSendCounter = 0;
        
        
        if(gps_nav_status.isNew) {
            gpsChange = 1;
            gps_nav_status.isNew = 0;
            gpsWatchdog = 0;

            if(gps_nav_status.flags & 0x1) { // fix is valid
                mavlink_gps_raw_int.fix_type = gps_nav_status.gpsFix;
                gpsFixed = 1;
            }
            else {
                mavlink_gps_raw_int.fix_type = 0;
                gpsFixed = 0;
            }
            //mavlink_gps_raw_int.satellites_visible = gps_nav_sol.numSV;
        }
        
        if(gps_nav_posllh.isNew) {
            gpsChange = 1;
            gps_nav_posllh.isNew = 0;
            
            mavlink_gps_raw_int.lat = gps_nav_posllh.lat;
            mavlink_gps_raw_int.lon = gps_nav_posllh.lon;
            mavlink_gps_raw_int.alt = gps_nav_posllh.hMSL;
            mavlink_gps_raw_int.eph = gps_nav_posllh.hAcc / 10;
            mavlink_gps_raw_int.epv = gps_nav_posllh.vAcc / 10;
        }
        
        if(gps_nav_velned.isNew) {
            gpsChange = 1;
            gps_nav_velned.isNew = 0;
            
            mavlink_gps_raw_int.vel = gps_nav_velned.gSpeed;
            mavlink_gps_raw_int.cog = gps_nav_velned.heading / 100; // because GPS assumes cog IS heading.
        }
        

        
        // send GPS position
        if(posupdate == 1 && gpsFixed == 1) {
            posupdate = 0;
            
            float craftX = gps_nav_posllh.lat / 10000000.0f;
            float craftY = gps_nav_posllh.lon / 10000000.0f;
            float craftZ = (float)gps_nav_posllh.hMSL/ 1000.0f;
            
            float targetX;
            float targetY;
            float targetZ;
            float targetYaw;
            
            if(horizontalHold == 1) { // request horizontal hold
                horizontalHoldLat = craftX;
                horizontalHoldLon = craftY;
                horizontalHold = 2; // now in hold
            }
            
            if(horizontalHold == 2) { // with hold
                targetX = horizontalHoldLat;
                targetY = horizontalHoldLon;
                targetZ = craftZ;
                targetYaw = 42.0f;
            }
            else if((waypointCurrent == WAYPOINT_HOME && waypointHomeValid == 1) || (waypointCurrent < waypointCount && waypointValid == 1)) { // with 
                targetX = waypoint[waypointCurrent].x;
                targetY = waypoint[waypointCurrent].y;
                targetZ = waypoint[waypointCurrent].z;
                targetYaw = waypoint[waypointCurrent].param4 * 0.01745329251994329577f; // param4 is yaw angle, degrees to radian conversion M_PI / 180.0f = 0.01745329251994329577...
            }
            else {
                targetX = horizontalHoldLat;
                targetY = horizontalHoldLon;
                targetZ = craftZ;
                targetYaw = 42.0f;
            }
                
            float lat_diff = (double)(targetX - craftX) * (double)111194.92664455873734580834; // 111194.92664455873734580834f is radius of earth and deg-rad conversion: 6371000*PI()/180
            float lon_diff = (double)(targetY - craftY) * (double)111194.92664455873734580834 * fcos((float)((double)craftX*(double)0.01745329251994329577)); // 0.01745329251994329577f is deg-rad conversion PI()/180
            float alt_diff = (float)(targetZ - craftZ);
            
            lat_diff_i += lat_diff;
            lon_diff_i += lon_diff;

            ilink_gpsfly.northDemand = GPS_Kp*lat_diff + GPS_Ki*lat_diff_i + GPS_Kd*gps_nav_velned.velN;
            ilink_gpsfly.eastDemand = GPS_Kp*lon_diff + GPS_Ki*lon_diff_i + GPS_Kd* gps_nav_velned.velE;
            ilink_gpsfly.headingDemand = targetYaw;
            ilink_gpsfly.altitudeDemand = targetZ;
            ilink_gpsfly.altitude = craftZ;
            ilink_gpsfly.vAcc = (float)gps_nav_posllh.vAcc / 1000.0f; // we think this is 1 sigma
            ilink_gpsfly.velD = (float)gps_nav_velned.velD / 100.0f;

            XBeeInhibit();
            ILinkSendMessage(ID_ILINK_GPSFLY, (unsigned short *) & ilink_gpsfly, sizeof(ilink_gpsfly)/2-1);
            XBeeAllow();
            
            
            if(horizontalHold == 0 && ((waypointCurrent == WAYPOINT_HOME && waypointHomeValid == 1) || (waypointCurrent < waypointCount && waypointValid == 1))) {
                float radius = waypoint[waypointCurrent].param2; // param2 is radius in QGroumdcontrol 1.0.1
                if(radius < 1) radius = 1;
                
                //float lat_diff2 = lat_diff; // for orbit phase calculation
                //float lon_diff2 = lon_diff;
                
                // assume cube of sides 2*radius rather than a sphere for target detection
                if(lat_diff < 0) lat_diff = -lat_diff;
                if(lon_diff < 0) lon_diff = -lon_diff;
                if(alt_diff < 0) alt_diff = -alt_diff;
                
                if(lat_diff < radius && lon_diff < radius && alt_diff < radius) {
                
                    // target reached
                    if(waypointReached == 0) {
                        waypointReached = 1;
                        waypointLoiter = 0;
                        
                        mavlink_mission_item_reached.seq = waypointCurrent;

                        mavlink_msg_mission_item_reached_encode(mavlinkID, MAV_COMP_ID_SYSTEM_CONTROL, &mavlink_tx_msg, &mavlink_mission_item_reached);
                        mavlink_message_len = mavlink_msg_to_send_buffer(mavlink_message_buf, &mavlink_tx_msg);
                        XBeeInhibit(); // XBee input needs to be inhibited before transmitting as some incomming messages cause UART responses which could disrupt XBeeWriteCoordinator if it is interrupted.
                        XBeeWriteCoordinator(mavlink_message_buf, mavlink_message_len);
                        XBeeAllow();
                        
                        // waypointPhase = fatan2(-lon_diff2, -lat_diff2);
                        
                    }
                }
                
                if(waypointReached == 1) {
                    switch(waypoint[waypointCurrent].command) {
                        case MAV_CMD_NAV_WAYPOINT:
                            if(waypointLoiter >= waypoint[waypointCurrent].param1) { // Param1 in this case is wait time
                                if(waypointCurrent < waypointCount) {
                                    waypointCurrent ++;
                                    waypointReached = 0;
                                    waypointLoiter = 0;
                                }
                            }
                            
                            /*ilink_payldctrl.camRoll = 0;
                            ilink_payldctrl.camPitch = 0;
                            ilink_payldctrl.camYaw = waypoint[waypointCurrent].param4 * 0.01745329251994329577f; // param4 is yaw angle, degrees to radian conversion M_PI / 180.0f = 0.01745329251994329577...;
                            ilink_payldctrl.controlMask = 0b100;
                            XBeeInhibit();
                            ILinkSendMessage(ID_ILINK_PAYLDCTRL, (unsigned short *) & ilink_payldctrl, sizeof(ilink_payldctrl)/2-1);
                            XBeeAllow();*/
                            
                            // TODO: tween yaw between waypoints
                            
                            break;
                        
                        case MAV_CMD_NAV_LOITER_UNLIM:
                        case MAV_CMD_NAV_LOITER_TIME:
                        case MAV_CMD_NAV_LOITER_TURNS:
                            // attempting to achieve a speed of 3m/s
                            //waypointPhase += 0.1;
                            
                            // don't need to do anything to hold position
                            // TODO: loiter radius/time/turns
                            
                            /*ilink_payldctrl.camRoll = 0;
                            ilink_payldctrl.camPitch = 0;
                            ilink_payldctrl.camYaw = waypoint[waypointCurrent].param4 * M_PI / 180.0f;
                            ilink_payldctrl.controlMask = 0b100;
                            XBeeInhibit();
                            ILinkSendMessage(ID_ILINK_PAYLDCTRL, (unsigned short *) & ilink_payldctrl, sizeof(ilink_payldctrl)/2-1);
                            XBeeAllow();*/
                            break;
                            
                            
                        case MAV_CMD_NAV_RETURN_TO_LAUNCH:
                            waypointCurrent = WAYPOINT_HOME;
                            waypointReached = 0;
                            break;
                            
                        case MAV_CMD_NAV_LAND:
                            //ilink_position.state = 0; // LAND NOW
                            break;
                    }
                }
            }
        }
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
    
    if(allowTransmit) {
        if(paramPointer < PARAMBUFFER_SIZE) {
            unsigned int i;
            // shunt this along to GCS
            for(i=0; i<MAVLINK_MSG_NAMED_VALUE_FLOAT_FIELD_NAME_LEN; i++) {
                mavlink_param_value.param_id[i] = paramBuffer[paramPointer].name[i];
                if(paramBuffer[paramPointer].name[i] == '\0') break;
            }
            mavlink_param_value.param_value = paramBuffer[paramPointer].value;
            mavlink_param_value.param_count = ilink_thalparam_rx.paramCount; // this value shouldn't change
            mavlink_param_value.param_index = paramBuffer[paramPointer].id;
            mavlink_param_value.param_type = MAV_PARAM_TYPE_REAL32;
            
            paramPointer++;
            
            mavlink_msg_param_value_encode(mavlinkID, MAV_COMP_ID_SYSTEM_CONTROL, &mavlink_tx_msg, &mavlink_param_value);
            mavlink_message_len = mavlink_msg_to_send_buffer(mavlink_message_buf, &mavlink_tx_msg);
            XBeeInhibit(); // XBee input needs to be inhibited before transmitting as some incomming messages cause UART responses which could disrupt XBeeWriteCoordinator if it is interrupted.
            XBeeWriteCoordinator(mavlink_message_buf, mavlink_message_len);
            XBeeAllow();
        }
        else if(waypointReceiveIndex < waypointCount) {
            if(waypointTimer > WAYPOINT_TIMEOUT) {
                mavlink_mission_request.seq = waypointReceiveIndex;
                mavlink_mission_request.target_system = waypointProviderID;
                mavlink_mission_request.target_component = waypointProviderComp;
                mavlink_msg_mission_request_encode(mavlinkID, MAV_COMP_ID_MISSIONPLANNER, &mavlink_tx_msg, &mavlink_mission_request);
                mavlink_message_len = mavlink_msg_to_send_buffer(mavlink_message_buf, &mavlink_tx_msg);
                XBeeWriteCoordinator(mavlink_message_buf, mavlink_message_len);
                
                waypointTimer = 0;
                waypointTries++;
            }
            if(waypointTries > waypointTries) { // timeout failure
                waypointCount = 0;
                MAVSendText(255, "Receiving Waypoint timeout");
            }
        }
        else if(ilink_thalctrl_rx.isNew) {
            // TODO translate mavlink command to thalctrl
            /*ilink_thalctrl_rx.isNew = 0;
            if(ilink_thalctrl_rx.command == MAVLINK_MSG_ID_COMMAND_LONG) {
                mavlink_command_ack.result = 0;
                mavlink_command_ack.command = ilink_thalctrl_rx.data;
                mavlink_msg_command_ack_encode(mavlinkID, MAV_COMP_ID_SYSTEM_CONTROL, &mavlink_tx_msg, &mavlink_command_ack);
                mavlink_message_len = mavlink_msg_to_send_buffer(mavlink_message_buf, &mavlink_tx_msg);
                XBeeWriteCoordinator(mavlink_message_buf, mavlink_message_len);
            }*/
        }
        else if(dataRate[MAV_DATA_STREAM_RAW_SENSORS] && rawSensorStreamCounter >= MESSAGE_LOOP_HZ/dataRate[MAV_DATA_STREAM_RAW_SENSORS]) {
            rawSensorStreamCounter = 0;
            
            if(ilink_rawimu.isNew) {
                ilink_rawimu.isNew = 0;
                mavlink_raw_imu.xacc = ilink_rawimu.xAcc;
                mavlink_raw_imu.yacc = ilink_rawimu.yAcc;
                mavlink_raw_imu.zacc = ilink_rawimu.zAcc;
                mavlink_raw_imu.xgyro = ilink_rawimu.xGyro;
                mavlink_raw_imu.ygyro = ilink_rawimu.yGyro;
                mavlink_raw_imu.zgyro = ilink_rawimu.zGyro;
                mavlink_raw_imu.xmag = ilink_rawimu.xMag;
                mavlink_raw_imu.ymag = ilink_rawimu.yMag;
                mavlink_raw_imu.zmag = ilink_rawimu.zMag;
                
                mavlink_msg_raw_imu_encode(mavlinkID, MAV_COMP_ID_SYSTEM_CONTROL, &mavlink_tx_msg, &mavlink_raw_imu);
                mavlink_message_len = mavlink_msg_to_send_buffer(mavlink_message_buf, &mavlink_tx_msg);
                XBeeInhibit(); // XBee input needs to be inhibited before transmitting as some incomming messages cause UART responses which could disrupt XBeeWriteCoordinator if it is interrupted.
                XBeeWriteCoordinator(mavlink_message_buf, mavlink_message_len);
                XBeeAllow();
            }
            XBeeInhibit();
            ILinkPoll(ID_ILINK_RAWIMU);
            XBeeAllow();
            
        }
        else if(dataRate[MAV_DATA_STREAM_EXTENDED_STATUS] && extStatusStreamCounter >= MESSAGE_LOOP_HZ/dataRate[MAV_DATA_STREAM_EXTENDED_STATUS]) {
            extStatusStreamCounter = 0;
            // GPS_STATUS, CONTROL_STATUS, AUX_STATUS
            
            // Sys status
            if(ilink_thalstat.isNew) {
                ilink_thalstat.isNew = 0;
                
                switch(ilink_thalstat.sensorStatus & 0x7) {
                    case 0:
                        mavlink_heartbeat.system_status = MAV_STATE_UNINIT;
                        mavlink_heartbeat.base_mode = MAV_MODE_PREFLIGHT;
                        break;
                    case 1:
                        mavlink_heartbeat.system_status = MAV_STATE_BOOT;
                        mavlink_heartbeat.base_mode = MAV_MODE_PREFLIGHT;
                        break;
                    case 2:
                        mavlink_heartbeat.system_status = MAV_STATE_CALIBRATING;
                        mavlink_heartbeat.base_mode = MAV_MODE_PREFLIGHT;
                        break;
                    case 3:
                        mavlink_heartbeat.system_status = MAV_STATE_STANDBY;
                        mavlink_heartbeat.base_mode &= ~MAV_MODE_FLAG_DECODE_POSITION_SAFETY;
                        break;
                    case 4:
                        mavlink_heartbeat.system_status = MAV_STATE_ACTIVE; 
                        mavlink_heartbeat.base_mode |= MAV_MODE_FLAG_DECODE_POSITION_SAFETY;
                        break;
                    case 5:     mavlink_heartbeat.system_status = MAV_STATE_CRITICAL;        break;
                    default:    mavlink_heartbeat.system_status = MAV_STATE_UNINIT;         break;
                }
                
                if(ilink_thalstat.sensorStatus & (0x1 << 3)) mavlink_sys_status.onboard_control_sensors_enabled |= MAVLINK_SENSOR_ACCEL;
                else mavlink_sys_status.onboard_control_sensors_enabled &= ~MAVLINK_SENSOR_ACCEL;
                if(ilink_thalstat.sensorStatus & (0x1 << 4)) mavlink_sys_status.onboard_control_sensors_enabled |= MAVLINK_SENSOR_GYRO;
                else mavlink_sys_status.onboard_control_sensors_enabled &= ~MAVLINK_SENSOR_GYRO;
                if(ilink_thalstat.sensorStatus & (0x1 << 5)) mavlink_sys_status.onboard_control_sensors_enabled |= MAVLINK_SENSOR_MAGNETO;
                else mavlink_sys_status.onboard_control_sensors_enabled &= ~MAVLINK_SENSOR_MAGNETO;
                if(ilink_thalstat.sensorStatus & (0x1 << 6)) mavlink_sys_status.onboard_control_sensors_enabled |= MAVLINK_SENSOR_BARO;
                else mavlink_sys_status.onboard_control_sensors_enabled &= ~MAVLINK_SENSOR_BARO;
                
                if(ilink_thalstat.flightMode & (0x1 << 0)) {
                    mavlink_sys_status.onboard_control_sensors_enabled |= MAVLINK_CONTROL_ATTITUDE;
                    mavlink_heartbeat.base_mode |= MAV_MODE_FLAG_DECODE_POSITION_STABILIZE;
                }
                else {
                    mavlink_sys_status.onboard_control_sensors_enabled &= ~MAVLINK_CONTROL_ATTITUDE;
                    mavlink_heartbeat.base_mode &= ~MAV_MODE_FLAG_DECODE_POSITION_STABILIZE;
                }
                if(ilink_thalstat.flightMode & (0x1 << 1)) mavlink_sys_status.onboard_control_sensors_enabled |= MAVLINK_CONTROL_ANGLERATE;
                else mavlink_sys_status.onboard_control_sensors_enabled &= ~MAVLINK_CONTROL_ANGLERATE;
                if(ilink_thalstat.flightMode & (0x1 << 2)) mavlink_sys_status.onboard_control_sensors_enabled |= MAVLINK_CONTROL_YAW;
                else mavlink_sys_status.onboard_control_sensors_enabled &= ~MAVLINK_CONTROL_YAW;
                if(ilink_thalstat.flightMode & (0x1 << 3)) mavlink_sys_status.onboard_control_sensors_enabled |= MAVLINK_CONTROL_Z;
                else mavlink_sys_status.onboard_control_sensors_enabled &= ~MAVLINK_CONTROL_Z;
                if(ilink_thalstat.flightMode & (0x1 << 4)) {
                    mavlink_heartbeat.base_mode |= MAV_MODE_FLAG_DECODE_POSITION_GUIDED;
                }
                else {
                    mavlink_heartbeat.base_mode &= ~MAV_MODE_FLAG_DECODE_POSITION_GUIDED;
                }
                
                mavlink_sys_status.onboard_control_sensors_health = mavlink_sys_status.onboard_control_sensors_enabled;
                mavlink_sys_status.voltage_battery = ilink_thalstat.battVoltage;
            }
            XBeeInhibit();
            ILinkPoll(ID_ILINK_THALSTAT);
            XBeeAllow();
            
            // Note: system load was calculated in the Heartbeat as it is on an invariable 1Hz loop)
            mavlink_msg_sys_status_encode(mavlinkID, MAV_COMP_ID_SYSTEM_CONTROL, &mavlink_tx_msg, &mavlink_sys_status);
            mavlink_message_len = mavlink_msg_to_send_buffer(mavlink_message_buf, &mavlink_tx_msg);
            XBeeInhibit(); // XBee input needs to be inhibited before transmitting as some incomming messages cause UART responses which could disrupt XBeeWriteCoordinator if it is interrupted.
            XBeeWriteCoordinator(mavlink_message_buf, mavlink_message_len);
            XBeeAllow();
            
        }
        else if(dataRate[MAV_DATA_STREAM_RC_CHANNELS] && rcChannelCounter >= MESSAGE_LOOP_HZ/dataRate[MAV_DATA_STREAM_RC_CHANNELS]) {
            // RC_CHANNELS_SCALED, RC_CHANNELS_RAW, SERVO_OUTPUT_RAW
             rcChannelCounter= 0;
             
            if(ilink_outputs0.isNew) {
                ilink_outputs0.isNew = 0;
                mavlink_servo_output_raw.time_usec = sysUS;
                mavlink_servo_output_raw.servo1_raw = ilink_outputs0.channel[0];
                mavlink_servo_output_raw.servo2_raw = ilink_outputs0.channel[1];
                mavlink_servo_output_raw.servo3_raw = ilink_outputs0.channel[2];
                mavlink_servo_output_raw.servo4_raw = ilink_outputs0.channel[3];
                mavlink_servo_output_raw.servo5_raw = ilink_outputs0.channel[4];
                mavlink_servo_output_raw.servo6_raw = ilink_outputs0.channel[5];
                mavlink_servo_output_raw.servo7_raw = 0;
                mavlink_servo_output_raw.servo8_raw = 0;
                mavlink_servo_output_raw.port = 0;
                
                /*mavlink_msg_servo_output_raw_encode(mavlinkID, MAV_COMP_ID_SYSTEM_CONTROL, &mavlink_tx_msg, &mavlink_servo_output_raw);
                mavlink_message_len = mavlink_msg_to_send_buffer(mavlink_message_buf, &mavlink_tx_msg);
                XBeeInhibit(); // XBee input needs to be inhibited before transmitting as some incomming messages cause UART responses which could disrupt XBeeWriteCoordinator if it is interrupted.
                XBeeWriteCoordinator(mavlink_message_buf, mavlink_message_len);
                XBeeAllow();*/
                //MAVSendVector("OUTPUT0", ilink_outputs0.channel[0], ilink_outputs0.channel[1], ilink_outputs0.channel[2]);
                //MAVSendVector("OUTPUT1", ilink_outputs0.channel[3], ilink_outputs0.channel[4], ilink_outputs0.channel[5]);
                MAVSendInt("MOTOR_N", ilink_outputs0.channel[0]);
                MAVSendInt("MOTOR_E", ilink_outputs0.channel[1]);
                MAVSendInt("MOTOR_S", ilink_outputs0.channel[2]);
                MAVSendInt("MOTOR_W", ilink_outputs0.channel[3]);
            
            }
            XBeeInhibit();
            ILinkPoll(ID_ILINK_OUTPUTS0);
            XBeeAllow();
                
            if(ilink_inputs0.isNew) {
                ilink_inputs0.isNew = 0;
                mavlink_rc_channels_raw.time_boot_ms = sysMS;
                mavlink_rc_channels_raw.chan1_raw = ilink_inputs0.channel[0];
                mavlink_rc_channels_raw.chan2_raw = ilink_inputs0.channel[1];
                mavlink_rc_channels_raw.chan3_raw = ilink_inputs0.channel[2];
                mavlink_rc_channels_raw.chan4_raw = ilink_inputs0.channel[3];
                mavlink_rc_channels_raw.chan5_raw = ilink_inputs0.channel[4];
                mavlink_rc_channels_raw.chan6_raw = ilink_inputs0.channel[5];
                mavlink_rc_channels_raw.chan7_raw = 0;
                mavlink_rc_channels_raw.chan8_raw = 0;
                mavlink_rc_channels_raw.port = 0;
                mavlink_rc_channels_raw.rssi = 255;
                
                mavlink_msg_rc_channels_raw_encode(mavlinkID, MAV_COMP_ID_SYSTEM_CONTROL, &mavlink_tx_msg, &mavlink_rc_channels_raw);
                mavlink_message_len = mavlink_msg_to_send_buffer(mavlink_message_buf, &mavlink_tx_msg);
                XBeeInhibit(); // XBee input needs to be inhibited before transmitting as some incomming messages cause UART responses which could disrupt XBeeWriteCoordinator if it is interrupted.
                XBeeWriteCoordinator(mavlink_message_buf, mavlink_message_len);
                XBeeAllow();
                /*
                mavlink_rc_channels_scaled.time_boot_ms = sysMS;
                mavlink_rc_channels_scaled.chan1_scaled = (signed int)ilink_inputs0.channel[0] * 11.8;
                mavlink_rc_channels_scaled.chan2_scaled = ((signed int)ilink_inputs0.channel[1] - (signed int)511) * 29.4;
                mavlink_rc_channels_scaled.chan3_scaled = ((signed int)ilink_inputs0.channel[2] - (signed int)511) * 29.4;
                mavlink_rc_channels_scaled.chan4_scaled = ((signed int)ilink_inputs0.channel[3] - (signed int)511) * 29.4;
                if(ilink_inputs0.channel[4] < 500) mavlink_rc_channels_scaled.chan5_scaled = 0;
                else mavlink_rc_channels_scaled.chan5_scaled = 10000;
                if(ilink_inputs0.channel[5] < 500) mavlink_rc_channels_scaled.chan6_scaled = 0;
                else mavlink_rc_channels_scaled.chan6_scaled = 10000;
                mavlink_rc_channels_scaled.chan7_scaled = 0;
                mavlink_rc_channels_scaled.chan8_scaled = 0;
                mavlink_rc_channels_scaled.port = 0;
                mavlink_rc_channels_scaled.rssi = 255;
                
                mavlink_msg_rc_channels_scaled_encode(mavlinkID, MAV_COMP_ID_SYSTEM_CONTROL, &mavlink_tx_msg, &mavlink_rc_channels_scaled);
                mavlink_message_len = mavlink_msg_to_send_buffer(mavlink_message_buf, &mavlink_tx_msg);
                XBeeInhibit(); // XBee input needs to be inhibited before transmitting as some incomming messages cause UART responses which could disrupt XBeeWriteCoordinator if it is interrupted.
                XBeeWriteCoordinator(mavlink_message_buf, mavlink_message_len);
                XBeeAllow();*/
            }
            XBeeInhibit();
            ILinkPoll(ID_ILINK_INPUTS0);
            XBeeAllow();
             
        }
        else if(dataRate[MAV_DATA_STREAM_RAW_CONTROLLER] && rawControllerCounter >= MESSAGE_LOOP_HZ/dataRate[MAV_DATA_STREAM_RAW_CONTROLLER]) {
            //ATTITUDE_CONTROLLER_OUTPUT, POSITION_CONTROLLER_OUTPUT, NAV_CONTROLLER_OUTPUT
            rawControllerCounter = 0;
            
            if(ilink_attitude.isNew) {
                ilink_attitude.isNew = 0;
                mavlink_attitude.time_boot_ms = sysMS;
                mavlink_attitude.roll = ilink_attitude.roll;
                mavlink_attitude.pitch = ilink_attitude.pitch;
                mavlink_attitude.yaw = ilink_attitude.yaw;
                mavlink_attitude.rollspeed = ilink_attitude.rollRate;
                mavlink_attitude.pitchspeed = ilink_attitude.pitchRate;
                mavlink_attitude.yawspeed = ilink_attitude.yawRate;
                
                mavlink_msg_attitude_encode(mavlinkID, MAV_COMP_ID_SYSTEM_CONTROL, &mavlink_tx_msg, &mavlink_attitude);
                mavlink_message_len = mavlink_msg_to_send_buffer(mavlink_message_buf, &mavlink_tx_msg);
                XBeeInhibit(); // XBee input needs to be inhibited before transmitting as some incomming messages cause UART responses which could disrupt XBeeWriteCoordinator if it is interrupted.
                XBeeWriteCoordinator(mavlink_message_buf, mavlink_message_len);
                XBeeAllow();
            }
            XBeeInhibit();
            ILinkPoll(ID_ILINK_ATTITUDE);
            XBeeAllow();
            
        }
        else if(dataRate[MAV_DATA_STREAM_POSITION] && positionStreamCounter >= MESSAGE_LOOP_HZ/dataRate[MAV_DATA_STREAM_POSITION]) {
            positionStreamCounter= 0;

            if(gpsChange) {
                mavlink_gps_raw_int.time_usec = sysUS;
                
                mavlink_msg_gps_raw_int_encode(mavlinkID, MAV_COMP_ID_SYSTEM_CONTROL, &mavlink_tx_msg, &mavlink_gps_raw_int);
                mavlink_message_len = mavlink_msg_to_send_buffer(mavlink_message_buf, &mavlink_tx_msg);
                XBeeInhibit(); // XBee input needs to be inhibited before transmitting as some incomming messages cause UART responses which could disrupt XBeeWriteCoordinator if it is interrupted.
                XBeeWriteCoordinator(mavlink_message_buf, mavlink_message_len);
                XBeeAllow();
            }
        }
        else if(dataRate[MAV_DATA_STREAM_EXTRA1] && extra1ChannelCounter >= MESSAGE_LOOP_HZ/dataRate[MAV_DATA_STREAM_EXTRA1]) {
            extra1ChannelCounter = 0;
                    
            if(ilink_scaledimu.isNew) {
                ilink_scaledimu.isNew = 0;
                mavlink_scaled_imu.xacc = ilink_scaledimu.xAcc;
                mavlink_scaled_imu.yacc = ilink_scaledimu.yAcc;
                mavlink_scaled_imu.zacc = ilink_scaledimu.zAcc;
                mavlink_scaled_imu.xgyro = ilink_scaledimu.xGyro;
                mavlink_scaled_imu.ygyro = ilink_scaledimu.yGyro;
                mavlink_scaled_imu.zgyro = ilink_scaledimu.zGyro;
                mavlink_scaled_imu.xmag = ilink_scaledimu.xMag;
                mavlink_scaled_imu.ymag = ilink_scaledimu.yMag;
                mavlink_scaled_imu.zmag = ilink_scaledimu.zMag;
                
                mavlink_msg_scaled_imu_encode(mavlinkID, MAV_COMP_ID_SYSTEM_CONTROL, &mavlink_tx_msg, &mavlink_scaled_imu);
                mavlink_message_len = mavlink_msg_to_send_buffer(mavlink_message_buf, &mavlink_tx_msg);
                XBeeInhibit(); // XBee input needs to be inhibited before transmitting as some incomming messages cause UART responses which could disrupt XBeeWriteCoordinator if it is interrupted.
                XBeeWriteCoordinator(mavlink_message_buf, mavlink_message_len);
                XBeeAllow();
            }
            XBeeInhibit();
            ILinkPoll(ID_ILINK_SCALEDIMU);
            XBeeAllow();
            
        }
        else if(dataRate[MAV_DATA_STREAM_EXTRA2] && extra2ChannelCounter > MESSAGE_LOOP_HZ/dataRate[MAV_DATA_STREAM_EXTRA2]) {
            extra2ChannelCounter = 0;
            
            if(ilink_altitude.isNew) {
                ilink_altitude.isNew = 0;
                MAVSendFloat("ALT_ULTRA",  ilink_altitude.ultra);
                MAVSendFloat("ALT_BARO",  ilink_altitude.baro);
                MAVSendFloat("ALT_FILT",  ilink_altitude.filtered);
            }
            XBeeInhibit();
            ILinkPoll(ID_ILINK_ALTITUDE);
            XBeeAllow();
        }
        else if(dataRate[MAV_DATA_STREAM_EXTRA3] && extra3ChannelCounter > MESSAGE_LOOP_HZ/dataRate[MAV_DATA_STREAM_EXTRA3]) {
            extra3ChannelCounter = 0;
            
            if(ilink_debug.isNew) {
                ilink_debug.isNew = 0;
                MAVSendFloat("DEBUG0",  ilink_debug.debug0);
                MAVSendFloat("DEBUG1",  ilink_debug.debug1);
                MAVSendFloat("DEBUG2",  ilink_debug.debug2);
                MAVSendFloat("DEBUG3",  ilink_debug.debug3);
                MAVSendFloat("DEBUG4",  ilink_debug.debug4);
                MAVSendFloat("DEBUG5",  ilink_debug.debug5);
                MAVSendFloat("DEBUG6",  ilink_debug.debug6);
                MAVSendFloat("DEBUG7",  ilink_debug.debug7);
            }
            
            XBeeInhibit();
            ILinkPoll(ID_ILINK_DEBUG);
            XBeeAllow();
            
        }
    }
    
    // *** Process ILink
    /*XBeeInhibit();
    ILinkFetchData();
    XBeeAllow();*/

    
    // *** Process ILink
    XBeeInhibit();
    ILinkFetchData();
    XBeeAllow();
}

// ****************************************************************************
// *** Communications
// ****************************************************************************

// *** Mavlink messages
void MAVSendHeartbeat(void) {
    //if(allowTransmit) {
        mavlink_msg_heartbeat_encode(mavlinkID, MAV_COMP_ID_SYSTEM_CONTROL, &mavlink_tx_msg, &mavlink_heartbeat);
        mavlink_message_len = mavlink_msg_to_send_buffer(mavlink_message_buf, &mavlink_tx_msg);
        XBeeInhibit(); // XBee input needs to be inhibited before transmitting as some incomming messages cause UART responses which could disrupt XBeeWriteCoordinator if it is interrupted.
        XBeeWriteCoordinator(mavlink_message_buf, mavlink_message_len);
        XBeeAllow();
    //}
}

void MAVSendFloat(char * name, float value) {
    if(allowTransmit) {
        unsigned int i;
        for(i=0; i<MAVLINK_MSG_NAMED_VALUE_FLOAT_FIELD_NAME_LEN; i++) {
            mavlink_named_value_float.name[i] = name[i];
            if(name[i] == '\0') break;
        }
        mavlink_named_value_float.time_boot_ms = sysMS;
        mavlink_named_value_float.value = value;
        mavlink_msg_named_value_float_encode(mavlinkID, MAV_COMP_ID_SYSTEM_CONTROL, &mavlink_tx_msg, &mavlink_named_value_float);
        mavlink_message_len = mavlink_msg_to_send_buffer(mavlink_message_buf, &mavlink_tx_msg);
        XBeeInhibit();
        XBeeWriteCoordinator(mavlink_message_buf, mavlink_message_len);
        XBeeAllow();
    }
}

void MAVSendInt(char * name, int value) {
    if(allowTransmit) {
        unsigned int i;
        for(i=0; i<MAVLINK_MSG_NAMED_VALUE_INT_FIELD_NAME_LEN; i++) {
            mavlink_named_value_int.name[i] = name[i];
            if(name[i] == '\0') break;
        }
        mavlink_named_value_int.time_boot_ms = sysMS;
        mavlink_named_value_int.value = value;
        mavlink_msg_named_value_int_encode(mavlinkID, MAV_COMP_ID_SYSTEM_CONTROL, &mavlink_tx_msg, &mavlink_named_value_int);
        mavlink_message_len = mavlink_msg_to_send_buffer(mavlink_message_buf, &mavlink_tx_msg);
        XBeeInhibit();
        XBeeWriteCoordinator(mavlink_message_buf, mavlink_message_len);
        XBeeAllow();
    }
}

void MAVSendVector(char * name, float valX, float valY, float valZ) {
    if(allowTransmit) {
        unsigned int i;
        for(i=0; i<MAVLINK_MSG_DEBUG_VECT_FIELD_NAME_LEN; i++) {
            mavlink_debug_vect.name[i] = name[i];
            if(name[i] == '\0') break;
        }
        mavlink_debug_vect.time_usec = sysUS;
        mavlink_debug_vect.x = valX;
        mavlink_debug_vect.y = valY;
        mavlink_debug_vect.z = valZ;
        mavlink_msg_debug_vect_encode(mavlinkID, MAV_COMP_ID_SYSTEM_CONTROL, &mavlink_tx_msg, &mavlink_debug_vect);
        mavlink_message_len = mavlink_msg_to_send_buffer(mavlink_message_buf, &mavlink_tx_msg);
        XBeeInhibit();
        XBeeWriteCoordinator(mavlink_message_buf, mavlink_message_len);
        XBeeAllow();
    }
}

void MAVSendText(unsigned char severity, char * text) {
    if(allowTransmit) {
        unsigned int i;
        mavlink_statustext.severity = severity;
        for(i=0; i<MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN; i++) {
            mavlink_statustext.text[i] = text[i];
            if(text[i] == '\0') break;
        }
        mavlink_msg_statustext_encode(mavlinkID, MAV_COMP_ID_SYSTEM_CONTROL, &mavlink_tx_msg, &mavlink_statustext);
        mavlink_message_len = mavlink_msg_to_send_buffer(mavlink_message_buf, &mavlink_tx_msg);
        XBeeInhibit();
        XBeeWriteCoordinator(mavlink_message_buf, mavlink_message_len);
        XBeeAllow();
    }
}

// XBee interrupt (for MAVLink)
void XBeeMessage(unsigned char id, unsigned char * buffer, unsigned short length) {
    unsigned char * ptr = 0;
    unsigned int j;
    
    switch(id) {
        case ID_XBEE_MODEMSTATUS:
            ptr = (unsigned char *) &xbee_modem_status;
            xbee_modem_status.isNew = 1;
            break;
        case ID_XBEE_ATRESPONSE:
            ptr = (unsigned char *) &xbee_at_response;
            xbee_at_response.isNew = 1;
            xbee_at_response.varLen = length - 4;
            break;
        case ID_XBEE_RECEIVEPACKET:
            /*ptr = (unsigned char *) &xbee_receive_packet;
            xbee_receive_packet.isNew = 1;
            xbee_receive_packet.varLen = length - 11;*/
            
            // bypass copy, send direct to MAVLINK Parse
            for(j=11; j<length; j++) {
                MAVLinkParse(buffer[j]);
            }
            break;
    }
    
    if(ptr) {
        for(j=0; j<length; j++) {
            ptr[j] = buffer[j];
        }
    }
}

void MAVLinkParse(unsigned char UARTData) {
    if(mavlink_parse_char(MAVLINK_COMM_0, UARTData, &mavlink_rx_msg, &mavlink_status)) {
        //mavlinkSendDebugV("MSGID", mavlink_rx_msg.msgid, 0, 0);
        switch(mavlink_rx_msg.msgid) {
             case MAVLINK_MSG_ID_HEARTBEAT:
                // count heartbeat messages
                heartbeatWatchdog = 0;
                allowTransmit = 1;
                break;
             case MAVLINK_MSG_ID_MANUAL_CONTROL:
                mavlink_msg_manual_control_decode(&mavlink_rx_msg, &mavlink_manual_control);
                if(mavlink_manual_control.target == mavlinkID) {
                    ilink_atdemand.roll = mavlink_manual_control.roll;
                    ilink_atdemand.pitch = mavlink_manual_control.pitch;
                    ilink_atdemand.yaw = mavlink_manual_control.yaw;
                    ilink_atdemand.thrust = mavlink_manual_control.thrust;
                    ILinkSendMessage(ID_ILINK_ATDEMAND, (unsigned short *) & ilink_atdemand, sizeof(ilink_atdemand)/2-1);
                }
                break;
            case MAVLINK_MSG_ID_SET_MODE:
                mavlink_msg_set_mode_decode(&mavlink_rx_msg, &mavlink_set_mode);
                if (mavlink_set_mode.target_system == mavlinkID) {
                    //mavlink_heartbeat.base_mode = mavlink_set_mode.base_mode;
                    //mavlink_heartbeat.custom_mode = mavlink_set_mode.custom_mode;
                    
                    /*ilink_thalctrl_rx.command = MAVLINK_MSG_ID_SET_MODE;
                    ilink_thalctrl_rx.data = mavlink_set_mode.base_mode;
                    ILinkSendMessage(ID_ILINK_THALCTRL, (unsigned short *) & ilink_thalctrl_rx, sizeof(ilink_thalctrl_rx)/2-1);*/
                }
                break;
            case MAVLINK_MSG_ID_COMMAND_LONG:
                // actions!
                mavlink_msg_command_long_decode(&mavlink_rx_msg, &mavlink_command_long);
                if (mavlink_command_long.target_system == mavlinkID) {
                    
                    
                    switch(mavlink_command_long.command) {
                        case 0: // custom 0, reset
                            mavlink_command_ack.result = 0;
                            mavlink_command_ack.command = mavlink_command_long.command;
                            mavlink_msg_command_ack_encode(mavlinkID, MAV_COMP_ID_SYSTEM_CONTROL, &mavlink_tx_msg, &mavlink_command_ack);
                            mavlink_message_len = mavlink_msg_to_send_buffer(mavlink_message_buf, &mavlink_tx_msg);
                            XBeeWriteCoordinator(mavlink_message_buf, mavlink_message_len);
                            Reset();
                            break;
                        //case MAV_CMD_NAV_WAYPOINT:
                            // param1 Hold time in decimal seconds.
                            // Acceptance radius in meters
                            //  0 to pass through the WP, if > 0 radius in meters to pass by WP
                            // Positive value for clockwise orbit, negative value for counter-clockwise orbit
                            // Desired yaw angle at MISSION (rotary wing)
                            //| Latitude| Longitude| Altitude|
                            
                            // use this for FOLLOW-ME mode (or without mission planner)
                         //   break;
                            
                        // MAV_CMD_NAV_LOITER_UNLIM=17, // Loiter around this MISSION an unlimited amount of time |Empty| Empty| Radius around MISSION, in meters. If positive loiter clockwise, else counter-clockwise| Desired yaw angle.| Latitude| Longitude| Altitude|  
                        // MAV_CMD_NAV_LOITER_TURNS=18, // Loiter around this MISSION for X turns |Turns| Empty| Radius around MISSION, in meters. If positive loiter clockwise, else counter-clockwise| Desired yaw angle.| Latitude| Longitude| Altitude|  
                        // MAV_CMD_NAV_LOITER_TIME=19, // Loiter around this MISSION for X seconds |Seconds (decimal)| Empty| Radius around MISSION, in meters. If positive loiter clockwise, else counter-clockwise| Desired yaw angle.| Latitude| Longitude| Altitude|  
                        // MAV_CMD_NAV_RETURN_TO_LAUNCH=20, // Return to launch location |Empty| Empty| Empty| Empty| Empty| Empty| Empty|  
                        // MAV_CMD_NAV_ROI=80, // Sets the region of interest (ROI) |Region of intereset mode. (see MAV_ROI enum)| MISSION index/ target ID. (see MAV_ROI enum)| ROI index (allows a vehicle to manage multiple ROI's)| Empty| x the location of the fixed ROI (see MAV_FRAME)| y| z|  
                        // MAV_CMD_NAV_PATHPLANNING=81, // Control autonomous path planning on the MAV. |0: Disable local obstacle avoidance / local path planning (without resetting map), 1: Enable local path planning, 2: Enable and reset local path planning| 0: Disable full path planning (without resetting map), 1: Enable, 2: Enable and reset map/occupancy grid, 3: Enable and reset planned route, but not occupancy grid| Empty| Yaw angle at goal, in compass degrees, [0..360]| Latitude/X of goal| Longitude/Y of goal| Altitude/Z of goal|  
                        
                        // MAV_CMD_CONDITION_DELAY=112, // Delay mission state machine. |Delay in seconds (decimal)| Empty| Empty| Empty| Empty| Empty| Empty|  
                        // MAV_CMD_CONDITION_CHANGE_ALT=113, // Ascend/descend at rate.  Delay mission state machine until desired altitude reached. |Descent / Ascend rate (m/s)| Empty| Empty| Empty| Empty| Empty| Finish Altitude|  
                        // MAV_CMD_CONDITION_DISTANCE=114, // Delay mission state machine until within desired distance of next NAV point. |Distance (meters)| Empty| Empty| Empty| Empty| Empty| Empty|  
                        // MAV_CMD_CONDITION_YAW=115, // Reach a certain target angle. |target angle: [0-360], 0 is north| speed during yaw change:[deg per second]| direction: negative: counter clockwise, positive: clockwise [-1,1]| relative offset or absolute angle: [ 1,0]| Empty| Empty| Empty|  
                        // MAV_CMD_CONDITION_LAST=159, // NOP - This command is only used to mark the upper limit of the CONDITION commands in the enumeration |Empty| Empty| Empty| Empty| Empty| Empty| Empty|  
                        // MAV_CMD_DO_SET_MODE=176, // Set system mode. |Mode, as defined by ENUM MAV_MODE| Empty| Empty| Empty| Empty| Empty| Empty|  
                        // MAV_CMD_DO_JUMP=177, // Jump to the desired command in the mission list.  Repeat this action only the specified number of times |Sequence number| Repeat count| Empty| Empty| Empty| Empty| Empty|  
                        // MAV_CMD_DO_CHANGE_SPEED=178, // Change speed and/or throttle set points. |Speed type (0=Airspeed, 1=Ground Speed)| Speed  (m/s, -1 indicates no change)| Throttle  ( Percent, -1 indicates no change)| Empty| Empty| Empty| Empty|  
                        // case MAV_CMD_DO_SET_HOME:
                            
                        // MAV_CMD_DO_SET_PARAMETER=180, // Set a system parameter.  Caution!  Use of this command requires knowledge of the numeric enumeration value of the parameter. |Parameter number| Parameter value| Empty| Empty| Empty| Empty| Empty|  
                        // MAV_CMD_DO_SET_RELAY=181, // Set a relay to a condition. |Relay number| Setting (1=on, 0=off, others possible depending on system hardware)| Empty| Empty| Empty| Empty| Empty|  
                        // MAV_CMD_DO_REPEAT_RELAY=182, // Cycle a relay on and off for a desired number of cyles with a desired period. |Relay number| Cycle count| Cycle time (seconds, decimal)| Empty| Empty| Empty| Empty|  
                        // MAV_CMD_DO_SET_SERVO=183, // Set a servo to a desired PWM value. |Servo number| PWM (microseconds, 1000 to 2000 typical)| Empty| Empty| Empty| Empty| Empty|  
                        // MAV_CMD_DO_REPEAT_SERVO=184, // Cycle a between its nominal setting and a desired PWM for a desired number of cycles with a desired period. |Servo number| PWM (microseconds, 1000 to 2000 typical)| Cycle count| Cycle time (seconds)| Empty| Empty| Empty|  
                        // MAV_CMD_DO_CONTROL_VIDEO=200, // Control onboard camera system. |Camera ID (-1 for all)| Transmission: 0: disabled, 1: enabled compressed, 2: enabled raw| Transmission mode: 0: video stream, >0: single images every n seconds (decimal)| Recording: 0: disabled, 1: enabled compressed, 2: enabled raw| Empty| Empty| Empty|  

                        // MAV_CMD_PREFLIGHT_CALIBRATION=241, // Trigger calibration. This command will be only accepted if in pre-flight mode. |Gyro calibration: 0: no, 1: yes| Magnetometer calibration: 0: no, 1: yes| Ground pressure: 0: no, 1: yes| Radio calibration: 0: no, 1: yes| Empty| Empty| Empty|  
                        // MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS=242, // Set sensor offsets. This command will be only accepted if in pre-flight mode. |Sensor to adjust the offsets for: 0: gyros, 1: accelerometer, 2: magnetometer, 3: barometer, 4: optical flow| X axis offset (or generic dimension 1), in the sensor's raw units| Y axis offset (or generic dimension 2), in the sensor's raw units| Z axis offset (or generic dimension 3), in the sensor's raw units| Generic dimension 4, in the sensor's raw units| Generic dimension 5, in the sensor's raw units| Generic dimension 6, in the sensor's raw units|  

                        case MAV_CMD_PREFLIGHT_STORAGE:
                            if(mavlink_command_long.param1 == 0) ilink_thalpareq.reqType = 3; // read all
                            else ilink_thalpareq.reqType = 2; // save all
                            ILinkSendMessage(ID_ILINK_THALPAREQ, (unsigned short *) & ilink_thalpareq, sizeof(ilink_thalpareq)/2-1);
                            break;
                            
                        case MAV_CMD_NAV_LAND:
                        case MAV_CMD_NAV_TAKEOFF:
                        case MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN: // KILL UAS
                            /*ilink_thalctrl_rx.command = MAVLINK_MSG_ID_COMMAND_LONG;
                            ilink_thalctrl_rx.data = mavlink_command_long.command;
                            ILinkSendMessage(ID_ILINK_THALCTRL, (unsigned short *) & ilink_thalctrl_rx, sizeof(ilink_thalctrl_rx)/2-1);*/
                            break;
                        
                        case MAV_CMD_OVERRIDE_GOTO:
                            if(mavlink_command_long.param1 == MAV_GOTO_DO_HOLD) {
                                waypointGo = 0;
                            }
                            else if(mavlink_command_long.param1 == MAV_GOTO_DO_CONTINUE) {
                                if(waypointGo == 1) {
                                    if(waypoint[waypointCurrent].command == MAV_CMD_NAV_LOITER_UNLIM && waypointCurrent < waypointCount) {
                                        waypointCurrent++;
                                    }
                                }
                                waypointGo = 1;
                                waypointReached = 0;
                            }
                            break;
                        
                        
                        default:
                            mavlink_command_ack.result = MAV_CMD_ACK_ERR_NOT_SUPPORTED;
                            mavlink_command_ack.command = mavlink_command_long.command;
                            mavlink_msg_command_ack_encode(mavlinkID, MAV_COMP_ID_SYSTEM_CONTROL, &mavlink_tx_msg, &mavlink_command_ack);
                            mavlink_message_len = mavlink_msg_to_send_buffer(mavlink_message_buf, &mavlink_tx_msg);
                            XBeeWriteCoordinator(mavlink_message_buf, mavlink_message_len);
                            break; 
                    }               
                    break;
                }
                break;
            case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
                // request send of all parameters
				ilink_thalpareq.reqType = 0; // request all
				ILinkSendMessage(ID_ILINK_THALPAREQ, (unsigned short *) & ilink_thalpareq, sizeof(ilink_thalpareq)/2-1);
                break;
            case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
                // request send of all parameters
                mavlink_msg_param_request_read_decode(&mavlink_rx_msg, &mavlink_param_request_read);
				
				ilink_thalpareq.reqType = 1; // request one
				unsigned short i;
				for(i=0; i<MAVLINK_MSG_NAMED_VALUE_FLOAT_FIELD_NAME_LEN; i++) {
					ilink_thalpareq.paramName[i] = mavlink_param_request_read.param_id[i];
					if(mavlink_param_set.param_id[i] == '\0') break;
				}
				ilink_thalpareq.paramID = mavlink_param_request_read.param_index;
				ILinkSendMessage(ID_ILINK_THALPAREQ, (unsigned short *) & ilink_thalpareq, sizeof(ilink_thalpareq)/2-1);
                break;
            case MAVLINK_MSG_ID_PARAM_SET:
                // request set parameter
                mavlink_msg_param_set_decode(&mavlink_rx_msg, &mavlink_param_set);
                if(mavlink_param_set.target_system == mavlinkID) {
                    
					unsigned short i;
					for(i=0; i<MAVLINK_MSG_NAMED_VALUE_FLOAT_FIELD_NAME_LEN; i++) {
						ilink_thalparam_tx.paramName[i] = mavlink_param_set.param_id[i];
						if(mavlink_param_set.param_id[i] == '\0') break;
					}
					
					ilink_thalparam_tx.paramID = 0;
					ilink_thalparam_tx.paramValue = mavlink_param_set.param_value;
					ilink_thalparam_tx.paramCount = 0;
					ILinkSendMessage(ID_ILINK_THALPARAM, (unsigned short *) &ilink_thalparam_tx, sizeof(ilink_thalparam_tx)/2 - 1);
                }
                break;
            case MAVLINK_MSG_ID_SET_GPS_GLOBAL_ORIGIN:
                mavlink_msg_set_gps_global_origin_decode(&mavlink_rx_msg, &mavlink_set_gps_global_origin);
                if (mavlink_set_gps_global_origin.target_system == mavlinkID) {
                        waypoint[WAYPOINT_HOME].command = MAV_CMD_NAV_LAND;
                        waypoint[WAYPOINT_HOME].autocontinue = 0;
                        waypoint[WAYPOINT_HOME].param1 = 0;
                        waypoint[WAYPOINT_HOME].param2 = 0;
                        waypoint[WAYPOINT_HOME].param3 = 0;
                        waypoint[WAYPOINT_HOME].param4 = 0;
                        waypoint[WAYPOINT_HOME].x = (float)mavlink_set_gps_global_origin.latitude / 10000000.0f;
                        waypoint[WAYPOINT_HOME].y = (float)mavlink_set_gps_global_origin.longitude / 10000000.0f;
                        waypoint[WAYPOINT_HOME].z = (float)mavlink_set_gps_global_origin.altitude / 1000.0f;
                        waypointHomeValid = 1;
                }
            case MAVLINK_MSG_ID_MISSION_CLEAR_ALL:
                mavlink_msg_mission_clear_all_decode(&mavlink_rx_msg, &mavlink_mission_clear_all);
                if (mavlink_mission_clear_all.target_system == mavlinkID) {
                    waypointCurrent = 0;
                    waypointCount = 0;
                    waypointValid = 0;

                    mavlink_mission_ack.type = MAV_MISSION_ACCEPTED;
                    mavlink_msg_mission_ack_encode(mavlinkID, MAV_COMP_ID_SYSTEM_CONTROL, &mavlink_tx_msg, &mavlink_mission_ack);
                    mavlink_message_len = mavlink_msg_to_send_buffer(mavlink_message_buf, &mavlink_tx_msg);
                    XBeeWriteCoordinator(mavlink_message_buf, mavlink_message_len);
                }
                break;
            case MAVLINK_MSG_ID_MISSION_SET_CURRENT:
                mavlink_msg_mission_set_current_decode(&mavlink_rx_msg, &mavlink_mission_set_current);
                if (mavlink_mission_set_current.target_system == mavlinkID) {
                    waypointCurrent = mavlink_mission_set_current.seq;
                    mavlink_mission_current.seq = waypointCurrent;
                    mavlink_msg_mission_current_encode(mavlinkID, MAV_COMP_ID_MISSIONPLANNER, &mavlink_tx_msg, &mavlink_mission_current);
                    mavlink_message_len = mavlink_msg_to_send_buffer(mavlink_message_buf, &mavlink_tx_msg);
                    XBeeWriteCoordinator(mavlink_message_buf, mavlink_message_len);     
                }
                break;
            case MAVLINK_MSG_ID_MISSION_COUNT:
                mavlink_msg_mission_count_decode(&mavlink_rx_msg, &mavlink_mission_count);
                if (mavlink_mission_count.target_system == mavlinkID) {
                    waypointCount = mavlink_mission_count.count;
                    waypointReceiveIndex = 0;
                    waypointTimer = WAYPOINT_TIMEOUT; // set waypoint timeout to timed out so that request is immediate
                    waypointTries = 0;
                    waypointValid = 0;  // invalidate waypoint storage until full set is received
                    
                    waypointProviderID = mavlink_rx_msg.sysid;
                    waypointProviderComp = mavlink_rx_msg.compid;
                }
                break;
            case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:
                mavlink_msg_mission_request_list_decode(&mavlink_rx_msg, &mavlink_mission_request_list);
                if (mavlink_mission_request_list.target_system == mavlinkID) {
                    if(waypointValid == 0) {
                        mavlink_mission_count.count = 0;
                    }
                    else {
                        mavlink_mission_count.count = waypointCount;
                    }
                    
                    mavlink_mission_count.target_system = mavlink_rx_msg.sysid;
                    mavlink_mission_count.target_component = mavlink_rx_msg.compid;
                    mavlink_msg_mission_count_encode(mavlinkID, MAV_COMP_ID_SYSTEM_CONTROL, &mavlink_tx_msg, &mavlink_mission_count);
                    mavlink_message_len = mavlink_msg_to_send_buffer(mavlink_message_buf, &mavlink_tx_msg);
                    XBeeWriteCoordinator(mavlink_message_buf, mavlink_message_len);
                }
                break;
            case MAVLINK_MSG_ID_MISSION_REQUEST:
                mavlink_msg_mission_request_decode(&mavlink_rx_msg, &mavlink_mission_request);
                if (mavlink_mission_request.target_system == mavlinkID) {
                    if(waypointValid != 0) {
                        mavlink_mission_item.target_system = mavlink_rx_msg.sysid;
                        mavlink_mission_item.target_component = mavlink_rx_msg.compid;    

                        mavlink_mission_item.seq = mavlink_mission_request.seq;
                        mavlink_mission_item.frame = MAV_FRAME_GLOBAL;
                        mavlink_mission_item.command = waypoint[mavlink_mission_request.seq].command;
                        mavlink_mission_item.autocontinue = waypoint[mavlink_mission_request.seq].autocontinue;
                        mavlink_mission_item.param1 = waypoint[mavlink_mission_request.seq].param1;
                        mavlink_mission_item.param2 = waypoint[mavlink_mission_request.seq].param2;
                        mavlink_mission_item.param3 = waypoint[mavlink_mission_request.seq].param3;
                        mavlink_mission_item.param4 = waypoint[mavlink_mission_request.seq].param4;
                        mavlink_mission_item.x = waypoint[mavlink_mission_request.seq].x;
                        mavlink_mission_item.y = waypoint[mavlink_mission_request.seq].y;
                        mavlink_mission_item.z = waypoint[mavlink_mission_request.seq].z;
                        
                        if(waypointCurrent == mavlink_mission_request.seq) {
                            mavlink_mission_item.current = 1;
                        }
                        else {
                            mavlink_mission_item.current = 0;
                        }
                            
                        mavlink_msg_mission_item_encode(mavlinkID, MAV_COMP_ID_MISSIONPLANNER, &mavlink_tx_msg, &mavlink_mission_item);
                        mavlink_message_len = mavlink_msg_to_send_buffer(mavlink_message_buf, &mavlink_tx_msg);
                        XBeeWriteCoordinator(mavlink_message_buf, mavlink_message_len);
                    }
                }
                break;
            case MAVLINK_MSG_ID_MISSION_ITEM:
                mavlink_msg_mission_item_decode(&mavlink_rx_msg, &mavlink_mission_item);
                if (mavlink_mission_item.target_system == mavlinkID) {
                    mavlink_mission_ack.type = MAV_MISSION_ERROR;
                    if(mavlink_mission_item.frame == MAV_FRAME_GLOBAL) {
                        if(mavlink_mission_item.seq < MAX_WAYPOINTS) {
                            //waypoint[mavlink_mission_item.seq].frame = mavlink_mission_item.frame;
                            waypoint[mavlink_mission_item.seq].command = mavlink_mission_item.command;
                            waypoint[mavlink_mission_item.seq].autocontinue = mavlink_mission_item.autocontinue;
                            waypoint[mavlink_mission_item.seq].param1 = mavlink_mission_item.param1;
                            waypoint[mavlink_mission_item.seq].param2 = mavlink_mission_item.param2;
                            waypoint[mavlink_mission_item.seq].param3 = mavlink_mission_item.param3;
                            waypoint[mavlink_mission_item.seq].param4 = mavlink_mission_item.param4;
                            waypoint[mavlink_mission_item.seq].x = mavlink_mission_item.x;
                            waypoint[mavlink_mission_item.seq].y = mavlink_mission_item.y;
                            waypoint[mavlink_mission_item.seq].z = mavlink_mission_item.z;
                            
                            if(mavlink_mission_item.current == 1) {
                                waypointCurrent = mavlink_mission_item.seq;
                            }
                        
                            waypointReceiveIndex++;
                            waypointTimer = WAYPOINT_TIMEOUT; // set waypoint timeout to timed out so that request is immediate
                            waypointTries = 0;
                            mavlink_mission_ack.target_system = mavlink_rx_msg.sysid;
                            mavlink_mission_ack.target_component = mavlink_rx_msg.compid;
                            if(waypointReceiveIndex >= waypointCount) {
                                mavlink_mission_ack.type = MAV_MISSION_ACCEPTED;
                                waypointValid = 1;
                            }
                            else if(waypointReceiveIndex >= MAX_WAYPOINTS) {
                                mavlink_mission_ack.type = MAV_MISSION_NO_SPACE;
                            }
                        }
                        else {
                            mavlink_mission_ack.type = MAV_MISSION_NO_SPACE;
                        }
                    }
                    else {
                        mavlink_mission_ack.type = MAV_MISSION_UNSUPPORTED_FRAME;
                    }
                    
                    mavlink_msg_mission_ack_encode(mavlinkID, MAV_COMP_ID_MISSIONPLANNER, &mavlink_tx_msg, &mavlink_mission_ack);
                    mavlink_message_len = mavlink_msg_to_send_buffer(mavlink_message_buf, &mavlink_tx_msg);
                    XBeeWriteCoordinator(mavlink_message_buf, mavlink_message_len);
                }
                break;
            case MAVLINK_MSG_ID_MISSION_WRITE_PARTIAL_LIST:
                mavlink_mission_ack.type = MAV_MISSION_UNSUPPORTED;
                mavlink_msg_mission_ack_encode(mavlinkID, MAV_COMP_ID_MISSIONPLANNER, &mavlink_tx_msg, &mavlink_mission_ack);
                mavlink_message_len = mavlink_msg_to_send_buffer(mavlink_message_buf, &mavlink_tx_msg);
                XBeeWriteCoordinator(mavlink_message_buf, mavlink_message_len);
                break;
            case MAVLINK_MSG_ID_MISSION_ACK:
                //ignored
                break;
                
            case MAVLINK_MSG_ID_REQUEST_DATA_STREAM:
                // Sets the output data rates
                mavlink_msg_request_data_stream_decode(&mavlink_rx_msg, &mavlink_request_data_stream);
                if (mavlink_request_data_stream.target_system == mavlinkID) {
                    if(mavlink_request_data_stream.req_message_rate > 255) mavlink_request_data_stream.req_message_rate = 255;
                    dataRate[mavlink_request_data_stream.req_stream_id] = mavlink_request_data_stream.req_message_rate;
                }
                break;
            default:
                MAVSendInt("CMDIGNORE", mavlink_rx_msg.msgid);
                break;
        }
        //if(mavlink_rx_msg.msgid != 0) MAVSendInt("CMD", mavlink_rx_msg.msgid);
    }
}


// *** GPS messages
void GPSMessage(unsigned short id, unsigned char * buffer, unsigned short length) {
    unsigned char * ptr = 0;
    unsigned short j;
    
    switch(id) {
        case ID_NAV_POSECEF: ptr = (unsigned char *) &gps_nav_posecef; break;
        case ID_NAV_POSLLH: ptr = (unsigned char *) &gps_nav_posllh; posupdate = 1; break;
        case ID_NAV_STATUS: ptr = (unsigned char *) &gps_nav_status; break;
        case ID_NAV_SOL: ptr = (unsigned char *) &gps_nav_sol; break;
        case ID_NAV_VELNED: ptr = (unsigned char *) &gps_nav_velned; break;
        case ID_NAV_TIMEUTC: ptr = (unsigned char *) &gps_nav_timeutc; break;
    }
    
    if(ptr) {
        for(j=0; j<length; j++) {
            ptr[j] = buffer[j];
        }
        ptr[j] = 1; // this is the "isNew" byte
    }
}

void ILinkMessage(unsigned short id, unsigned short * buffer, unsigned short length) {
    unsigned short * ptr = 0;
    unsigned int j;
    
    switch(id) {
        case ID_ILINK_IDENTIFY: ptr = (unsigned short *) &ilink_identify; break;
        case ID_ILINK_THALSTAT: ptr = (unsigned short *) &ilink_thalstat; break;
        case ID_ILINK_THALCTRL: ptr = (unsigned short *) &ilink_thalctrl_rx; break;
        case ID_ILINK_RAWIMU: ptr = (unsigned short *) &ilink_rawimu; break;
        case ID_ILINK_SCALEDIMU: ptr = (unsigned short *) &ilink_scaledimu; break;
        case ID_ILINK_ALTITUDE: ptr = (unsigned short *) &ilink_altitude; break;
        case ID_ILINK_ATTITUDE: ptr = (unsigned short *) &ilink_attitude; break;
        case ID_ILINK_THALPARAM: ptr = (unsigned short *) &ilink_thalparam_rx; break;
        case ID_ILINK_INPUTS0: ptr = (unsigned short *) &ilink_inputs0; break;
        case ID_ILINK_OUTPUTS0: ptr = (unsigned short *) &ilink_outputs0; break;
        case ID_ILINK_THALPAREQ: ptr = (unsigned short *) &ilink_thalpareq; break;
        case ID_ILINK_DEBUG: ptr = (unsigned short *) &ilink_debug; break;
    }
    
    if(ptr) {
        for(j=0; j<length; j++) {
            ptr[j] = buffer[j];
        }
        ptr[j] = 1; // this is the "isNew" byte
        
        switch(id) {
            case ID_ILINK_THALCTRL:
                switch(ilink_thalctrl_rx.command) {
                    case 0x90: // horizontal hold
                        if(horizontalHold == 0) {
                            horizontalHold = 1;
                        }
                        break;
                        
                    case 0x91: // horizontal releas
                        horizontalHold = 0;
                        break;
                }
            
                break;
            case ID_ILINK_THALPARAM: // store parameters in buffer
                if(paramPointer > 0) {
                    paramPointer--;
                    paramBuffer[paramPointer].value = ilink_thalparam_rx.paramValue;
                    paramBuffer[paramPointer].id = ilink_thalparam_rx.paramID;
                    for(j=0; j<16; j++) {
                        paramBuffer[paramPointer].name[j] = ilink_thalparam_rx.paramName[j];
                        if(ilink_thalparam_rx.paramName[j] == '\r') break;
                    }
                }
                break;
            case ID_ILINK_THALPAREQ:
                if(ilink_thalpareq.reqType == 2) {
                    // output something intelligible to user to signify successful EEPROM save
                }
                break;
			case ID_ILINK_IDENTIFY:
				if(ilink_identify.firmVersion == FIRMWARE_VERSION && ilink_identify.deviceID == I_AM_THALAMUS) {
					mavlink_sys_status.onboard_control_sensors_present |= MAVLINK_SENSOR_GYRO | MAVLINK_SENSOR_ACCEL | MAVLINK_SENSOR_MAGNETO | MAVLINK_SENSOR_BARO | MAVLINK_CONTROL_ANGLERATE | MAVLINK_CONTROL_ATTITUDE | MAVLINK_CONTROL_YAW | MAVLINK_CONTROL_Z;
					mavlink_sys_status.onboard_control_sensors_health = mavlink_sys_status.onboard_control_sensors_enabled;
				}
				break;
        }
		
		thalWatchdog = 0;
    }
}


