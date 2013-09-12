/*!
\file Beacon/telemetry.c
\brief Deals with telemetry and MAVLink

\author Yuan Gao
*/


#include "all.h"

unsigned char mavlinkID;	/*!< This contains the craft's MAVID, by default it is generated from the chip ID, but is adjusted in parameters */

// Sent messages
mavlink_ping_t mavlink_ping;                                    /*!< Ping */
mavlink_status_t mavlink_status;								/*!< Craft status */
mavlink_message_t mavlink_tx_msg;								/*!< Transmitted message  */
mavlink_heartbeat_t mavlink_heartbeat;							/*!< Heartbeat  */
mavlink_sys_status_t mavlink_sys_status;						/*!< System status  */
mavlink_gps_raw_int_t mavlink_gps_raw_int;						/*!< Raw GPS  */
mavlink_raw_imu_t mavlink_raw_imu;								/*!< Raw IMU  */
mavlink_scaled_imu_t mavlink_scaled_imu;						/*!< Scaled IMU  */
mavlink_attitude_t mavlink_attitude;							/*!< Attitude  */
mavlink_command_ack_t mavlink_command_ack;						/*!< Command acknowledgement  */
mavlink_param_value_t mavlink_param_value;						/*!< Parameters  */
mavlink_rc_channels_raw_t mavlink_rc_channels_raw;				/*!< RC raw data  */
mavlink_rc_channels_scaled_t mavlink_rc_channels_scaled;		/*!< RC scaled data  */
mavlink_servo_output_raw_t mavlink_servo_output_raw;			/*!< Servo output data  */
mavlink_gps_global_origin_t mavlink_gps_global_origin;			/*!< GPS home location  */
mavlink_vfr_hud_t mavlink_vfr_hud;								/*!< VFR HUD  */

mavlink_named_value_float_t mavlink_named_value_float;			/*!< Debug float  */
mavlink_named_value_int_t mavlink_named_value_int;				/*!< Debug int  */
mavlink_debug_vect_t mavlink_debug_vect;						/*!< Debug vector  */
mavlink_statustext_t mavlink_statustext;						/*!< Status text  */
mavlink_mission_request_t mavlink_mission_request;				/*!< Mission request  */
mavlink_mission_item_t mavlink_mission_item;					/*!< Mission waypoint item  */
mavlink_mission_ack_t mavlink_mission_ack;						/*!< Mission acknowledgement  */
mavlink_mission_item_reached_t mavlink_mission_item_reached;	/*!< Mission waypoint reached notification  */

// Received messages
mavlink_message_t mavlink_rx_msg;								/*!< Receieved message  */
mavlink_request_data_stream_t mavlink_request_data_stream;		/*!< Data stream rate request  */
mavlink_command_long_t mavlink_command_long;					/*!< Commands  */
//mavlink_param_request_list_t mavlink_param_request_list;		/*!< Parameter read all reuest  */
mavlink_param_set_t mavlink_param_set;							/*!< Parameter set request  */
mavlink_param_request_read_t mavlink_param_request_read;		/*!< Parameter read request  */
mavlink_manual_control_t mavlink_manual_control;				/*!< Manual control  */
mavlink_mission_count_t mavlink_mission_count;					/*!< Mission count  */
mavlink_set_gps_global_origin_t mavlink_set_gps_global_origin;	/*!< Set Home  */
mavlink_mission_request_list_t mavlink_mission_request_list;	/*!< Mission request  */
mavlink_mission_clear_all_t mavlink_mission_clear_all;			/*!< Mission clear  */
mavlink_mission_set_current_t mavlink_mission_set_current;		/*!< Mission set current  */
mavlink_mission_current_t mavlink_mission_current;				/*!< Mission current  */
mavlink_set_mode_t mavlink_set_mode;							/*!< Set craft mode  */

unsigned char mavlink_message_buf[MAVLINK_MAX_PACKET_LEN];		/*!< Mavlink message buffer */
unsigned short mavlink_message_len;								/*!< Mavlink message buffer length */
// Timers
unsigned char dataRate[MAV_DATA_STREAM_ENUM_END];	/*!< Data stream rates */

// /*!
// \brief Deals with sending all the Mavlink telemetry
// */
void mavlink_telemetry(void) {
	static unsigned short rawSensorStreamCounter=0;
	static unsigned short extStatusStreamCounter=0;
	static unsigned short rcChannelCounter=0;
	static unsigned short rawControllerCounter=0;
	static unsigned short positionStreamCounter=0;
	static unsigned short extra1ChannelCounter=0;
	static unsigned short extra2ChannelCounter=0;
	static unsigned short extra3ChannelCounter=0;

    rawSensorStreamCounter++;
    extStatusStreamCounter++;
    rcChannelCounter++;
    rawControllerCounter++;
    positionStreamCounter++;
    extra1ChannelCounter++;
    extra2ChannelCounter++;
    extra3ChannelCounter++;
	
	if(dataRate[MAV_DATA_STREAM_POSITION] && positionStreamCounter >= MESSAGE_LOOP_HZ/dataRate[MAV_DATA_STREAM_POSITION]) {
		positionStreamCounter= 0;

		mavlink_gps_raw_int.time_usec = sysUS;
		
		mavlink_msg_gps_raw_int_encode(mavlinkID, MAV_COMP_ID_GPS, &mavlink_tx_msg, &mavlink_gps_raw_int);
		mavlink_message_len = mavlink_msg_to_send_buffer(mavlink_message_buf, &mavlink_tx_msg);
		XBeeInhibit(); // XBee input needs to be inhibited before transmitting as some incomming messages cause UART responses which could disrupt XBeeWriteCoordinator if it is interrupted.
		XBeeWriteCoordinator(mavlink_message_buf, mavlink_message_len);
		XBeeAllow();
	}
	
	if(dataRate[MAV_DATA_STREAM_RAW_CONTROLLER] && rawControllerCounter >= MESSAGE_LOOP_HZ/dataRate[MAV_DATA_STREAM_RAW_CONTROLLER]) {
		rawControllerCounter = 0;
		
		// Nothing here
	}
	
	if(dataRate[MAV_DATA_STREAM_RAW_SENSORS] && rawSensorStreamCounter >= MESSAGE_LOOP_HZ/dataRate[MAV_DATA_STREAM_RAW_SENSORS]) {
		rawSensorStreamCounter = 0;
		
		// Nothing here
	}
	
	if(dataRate[MAV_DATA_STREAM_EXTENDED_STATUS] && extStatusStreamCounter >= MESSAGE_LOOP_HZ/dataRate[MAV_DATA_STREAM_EXTENDED_STATUS]) {
		extStatusStreamCounter = 0;
		
        mavlink_sys_status.voltage_battery = 0;
            
		mavlink_msg_sys_status_encode(mavlinkID, MAV_COMP_ID_SYSTEM_CONTROL, &mavlink_tx_msg, &mavlink_sys_status);
		mavlink_message_len = mavlink_msg_to_send_buffer(mavlink_message_buf, &mavlink_tx_msg);
		XBeeInhibit(); // XBee input needs to be inhibited before transmitting as some incomming messages cause UART responses which could disrupt XBeeWriteCoordinator if it is interrupted.
		XBeeWriteCoordinator(mavlink_message_buf, mavlink_message_len);
		XBeeAllow();
	}
	
	if(dataRate[MAV_DATA_STREAM_RC_CHANNELS] && rcChannelCounter >= MESSAGE_LOOP_HZ/dataRate[MAV_DATA_STREAM_RC_CHANNELS]) {
		// RC_CHANNELS_SCALED, RC_CHANNELS_RAW, SERVO_OUTPUT_RAW
		 rcChannelCounter= 0;
		 
		// Nothing here
	}
	
	if(dataRate[MAV_DATA_STREAM_EXTRA1] && extra1ChannelCounter >= MESSAGE_LOOP_HZ/dataRate[MAV_DATA_STREAM_EXTRA1]) {
		extra1ChannelCounter = 0;
				
		// Nothing here
	}
	
	if(dataRate[MAV_DATA_STREAM_EXTRA2] && extra2ChannelCounter > MESSAGE_LOOP_HZ/dataRate[MAV_DATA_STREAM_EXTRA2]) {
		extra2ChannelCounter = 0;
		
		// Nothing here
	}
	
	if(dataRate[MAV_DATA_STREAM_EXTRA3] && extra3ChannelCounter > MESSAGE_LOOP_HZ/dataRate[MAV_DATA_STREAM_EXTRA3]) {
		extra3ChannelCounter = 0;
		
		// Nothing here
	}
}

/*!
\brief Initialise mavlink stuff
*/
void MAVLinkInit() {
    mavlinkID = (unsigned char) MAV_ID;
	if(mavlinkID == 255) mavlinkID = 254; // 255 reserved for ground control
    if(mavlinkID == 0) mavlinkID = 1; // 0 reserved for broadcast
	
    mavlink_heartbeat.type = MAV_TYPE_GCS;
    mavlink_heartbeat.autopilot = MAV_AUTOPILOT_INVALID;
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
    dataRate[MAV_DATA_STREAM_RAW_SENSORS] = 0;
    dataRate[MAV_DATA_STREAM_RAW_CONTROLLER] = 0;
    dataRate[MAV_DATA_STREAM_RC_CHANNELS] = 0;
    dataRate[MAV_DATA_STREAM_POSITION] = 5;
    dataRate[MAV_DATA_STREAM_EXTRA1] = 0;
    dataRate[MAV_DATA_STREAM_EXTRA2] = 0;
    dataRate[MAV_DATA_STREAM_EXTRA3] = 0;
}

/*!
\brief Deals with receiving Mavlink

This function is triggered by the Mavlink parser, which is triggered by the
Xbee parser, which is ultimately triggered by the UART RX ISR
*/
void MAVSendHeartbeat(void) {
    //if(allowTransmit) {
        mavlink_msg_heartbeat_encode(mavlinkID, MAV_COMP_ID_SYSTEM_CONTROL, &mavlink_tx_msg, &mavlink_heartbeat);
        mavlink_message_len = mavlink_msg_to_send_buffer(mavlink_message_buf, &mavlink_tx_msg);
        XBeeInhibit(); // XBee input needs to be inhibited before transmitting as some incomming messages cause UART responses which could disrupt XBeeWriteCoordinator if it is interrupted.
        XBeeWriteCoordinator(mavlink_message_buf, mavlink_message_len);
        XBeeAllow();
    //}
}

/*!
\brief Sends a debug float
*/
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
/*!
\brief Sends a debug int
*/
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
/*!
\brief Sends a debug vector consisting of three floats
*/
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
/*!
\brief Sends a debug text
*/
void MAVSendText(unsigned char severity, char * text) {
	MAVSendTextFrom(severity, text, MAV_COMP_ID_SYSTEM_CONTROL);
}
/*!
\brief Sends a debug text from a specific component
*/
void MAVSendTextFrom(unsigned char severity, char * text, unsigned char from) {
    if(allowTransmit) {
        unsigned int i;
        mavlink_statustext.severity = severity;
        for(i=0; i<MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN; i++) {
            mavlink_statustext.text[i] = text[i];
            if(text[i] == '\0') break;
        }
        mavlink_msg_statustext_encode(mavlinkID, from, &mavlink_tx_msg, &mavlink_statustext);
        mavlink_message_len = mavlink_msg_to_send_buffer(mavlink_message_buf, &mavlink_tx_msg);
        XBeeInhibit();
        XBeeWriteCoordinator(mavlink_message_buf, mavlink_message_len);
        XBeeAllow();
    }
}

/*!
\brief XBee interrupt, this is triggered by the UART ISR, and decodes XBee packets
*/
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

/*!
\brief Parses MAVLink data
*/
void MAVLinkParse(unsigned char UARTData) {
	unsigned int i, j, match;
    if(mavlink_parse_char(MAVLINK_COMM_0, UARTData, &mavlink_rx_msg, &mavlink_status)) {
        //MAVSendInt("ID", mavlink_rx_msg.msgid);
        switch(mavlink_rx_msg.msgid) {
            case MAVLINK_MSG_ID_HEARTBEAT:
                // count heartbeat messages
                heartbeatWatchdog = 0;
                allowTransmit = 1;
                break;
            
            case MAVLINK_MSG_ID_PING:
                // PING is a special case, as only pings with target system set to zero is responded to,
                // and results are breadcast on mesh instead of to coordinator.
                if(mavlink_msg_ping_get_target_system(&mavlink_rx_msg) == 0) { // only respond to pings with target system set to zero
                    mavlink_msg_ping_decode(&mavlink_rx_msg, &mavlink_ping);
                    mavlink_ping.time_usec = sysUS;
                    mavlink_ping.target_component = mavlinkID;
                    mavlink_ping.target_component = MAV_COMP_ID_SYSTEM_CONTROL;
                    XBeeWriteBroadcast(mavlink_message_buf, mavlink_message_len); // BROADCAST
                }
                break;
            case MAVLINK_MSG_ID_COMMAND_LONG:
                // actions!
                if(mavlink_msg_command_long_get_target_system(&mavlink_rx_msg) == mavlinkID) {
                    mavlink_msg_command_long_decode(&mavlink_rx_msg, &mavlink_command_long);
                    switch(mavlink_command_long.command) {
                        case 0: // custom 0, reset
							MAVSendText(MAV_SEVERITY_WARNING, "WARNING: User requested RESET!");
							Delay(100);
                            Reset();
                            break;
							
						case 1: // custom 1
							break;
							
                        case MAV_CMD_PREFLIGHT_STORAGE:
                            if(mavlink_command_long.param1 == 0) { // read all
								eeprom_load_all();
                                MAVSendTextFrom(MAV_SEVERITY_INFO, "EEPROM Load OK", MAV_COMP_ID_SYSTEM_CONTROL);
                            }
                            else {
								eeprom_save_all();
                                MAVSendTextFrom(MAV_SEVERITY_INFO, "EEPROM Saved OK", MAV_COMP_ID_SYSTEM_CONTROL);
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
            case MAVLINK_MSG_ID_PARAM_REQUEST_LIST: // request send of all parameters
                if(mavlink_msg_param_request_list_get_target_system(&mavlink_rx_msg) == mavlinkID) {
                    //mavlink_msg_param_request_list_decode(&mavlink_rx_msg, &mavlink_param_request_list);
                    // no need to decode, doesn't conatian anything interesting
                    paramSendCount = 0;
                    paramSendSingle = 0;
                }
				break;
            case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
                // request send of one parameters
				if(mavlink_msg_param_request_read_get_target_system(&mavlink_rx_msg) == mavlinkID) {
                    mavlink_msg_param_request_read_decode(&mavlink_rx_msg, &mavlink_param_request_read);
					if(mavlink_param_request_read.target_component == MAV_COMP_ID_MISSIONPLANNER) {
						// local params
						for (i=0; i<paramCount; i++){
							match = 1;
							for (j=0; j<MAVLINK_MSG_NAMED_VALUE_FLOAT_FIELD_NAME_LEN; j++) {
								if(paramStorage[i].name[j] !=  mavlink_param_request_read.param_id[i]) {
									match = 0;
									break;
								}
								if(paramStorage[i].name[j] == '\0') break;
							}
							
							if(match == 1) {
								// when a match is found get the iD
								paramSendCount = i;
								paramSendSingle = 1;
								break;
							}
						}
					}
				}
				break;
            case MAVLINK_MSG_ID_PARAM_SET:
                // request set parameter
                if(mavlink_msg_param_set_get_target_system(&mavlink_rx_msg) == mavlinkID) {
                    mavlink_msg_param_set_decode(&mavlink_rx_msg, &mavlink_param_set);
                        if(mavlink_param_set.target_component == MAV_COMP_ID_MISSIONPLANNER) {
						// local params
						for (i=0; i<paramCount; i++){
							match = 1;
							for (j=0; j<MAVLINK_MSG_NAMED_VALUE_FLOAT_FIELD_NAME_LEN; j++) {
								if(paramStorage[i].name[j] !=  mavlink_param_set.param_id[j]) {
									match = 0;
									break;
								}
								if(paramStorage[i].name[j] == '\0') break;
							}
							
							if(match == 1) {
								// when a match is found, save it to paramStorage
								paramStorage[i].value = mavlink_param_set.param_value;
								
								// then order the value to be sent out again using the param send engine
								// but deal with cases where it's already in the process of sending out data
								if(paramSendCount < paramCount) {
									// parameter engine currently sending out data
									if(paramSendCount >= i) {
										// if parameter engine already sent out this now-changed data, redo this one, otherwise no action needed
										paramSendCount = i;
									}
								}
								else {
									// parameter engine not currently sending out data, so send single parameter
									paramSendCount = i;
									paramSendSingle = 1;
								}
								break;
							}
						}
					}
                }
                break;
            case MAVLINK_MSG_ID_REQUEST_DATA_STREAM:
                // Sets the output data rates
                if(mavlink_msg_request_data_stream_get_target_system(&mavlink_rx_msg) == mavlinkID) {
                    mavlink_msg_request_data_stream_decode(&mavlink_rx_msg, &mavlink_request_data_stream);
                    if(mavlink_request_data_stream.req_message_rate > 255) mavlink_request_data_stream.req_message_rate = 255;
                    dataRate[mavlink_request_data_stream.req_stream_id] = mavlink_request_data_stream.req_message_rate;
                }
                break;
            default:
                //MAVSendInt("CMDIGNORE", mavlink_rx_msg.msgid);
                break;
        }
    }
}