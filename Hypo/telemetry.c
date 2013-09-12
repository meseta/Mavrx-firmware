/*!
\file Hypo/telemetry.c
\brief Deals with telemetry and MAVLink

\author Yuan Gao
*/


#include "all.h"

unsigned char mavlinkID;	/*!< This contains the craft's MAVID, by default it is generated from the chip ID, but is adjusted in parameters */

// Sent messages
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
mavlink_param_request_list_t mavlink_param_request_list;		/*!< Parameter read all reuest  */
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

// Xbee stuff
xbee_modem_status_t xbee_modem_status;							/*!< XBee modem status */
xbee_at_command_t xbee_at_command;								/*!< XBee commands */
xbee_at_response_t xbee_at_response;							/*!< XBee command response */
xbee_receive_packet_t xbee_receive_packet;						/*!< XBee received packet */
xbee_transmit_request_t xbee_transmit_request;					/*!< XBee transmit packet */

/*!
\brief Deals with sending all the Mavlink telemetry
*/
void mavlink_telemetry(void) {
	static unsigned short waypointTimer=0;
	static unsigned short rawSensorStreamCounter=0;
	static unsigned short extStatusStreamCounter=0;
	static unsigned short rcChannelCounter=0;
	static unsigned short rawControllerCounter=0;
	static unsigned short positionStreamCounter=0;
	static unsigned short extra1ChannelCounter=0;
	static unsigned short extra2ChannelCounter=0;
	static unsigned short extra3ChannelCounter=0;
	
    waypointTimer++;
	
    rawSensorStreamCounter++;
    extStatusStreamCounter++;
    rcChannelCounter++;
    rawControllerCounter++;
    positionStreamCounter++;
    extra1ChannelCounter++;
    extra2ChannelCounter++;
    extra3ChannelCounter++;
	

	if(waypointReceiveIndex < waypointCount) {
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
			MAVSendTextFrom(MAV_SEVERITY_NOTICE, "NOTICE: Receiving waypoint timeout!", MAV_COMP_ID_MISSIONPLANNER);
		}
	}
	
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
		//ATTITUDE_CONTROLLER_OUTPUT, POSITION_CONTROLLER_OUTPUT, NAV_CONTROLLER_OUTPUT
		rawControllerCounter = 0;
		
		if(ilink_attitude.isNew) {
			Port0Toggle(PIN13);
			ilink_attitude.isNew = 0;
			mavlink_attitude.time_boot_ms = sysMS;
			mavlink_attitude.roll = ilink_attitude.roll;
			mavlink_attitude.pitch = ilink_attitude.pitch;
			mavlink_attitude.yaw = ilink_attitude.yaw;
			mavlink_attitude.rollspeed = ilink_attitude.rollRate;
			mavlink_attitude.pitchspeed = ilink_attitude.pitchRate;
			mavlink_attitude.yawspeed = ilink_attitude.yawRate;
			
			mavlink_msg_attitude_encode(mavlinkID, MAV_COMP_ID_IMU, &mavlink_tx_msg, &mavlink_attitude);
			mavlink_message_len = mavlink_msg_to_send_buffer(mavlink_message_buf, &mavlink_tx_msg);
			XBeeInhibit(); // XBee input needs to be inhibited before transmitting as some incomming messages cause UART responses which could disrupt XBeeWriteCoordinator if it is interrupted.
			XBeeWriteCoordinator(mavlink_message_buf, mavlink_message_len);
			XBeeAllow();
		}
		Port0Toggle(PIN14);
		XBeeInhibit();
		ILinkPoll(ID_ILINK_ATTITUDE);
		XBeeAllow();
		
	}
	
	if(dataRate[MAV_DATA_STREAM_RAW_SENSORS] && rawSensorStreamCounter >= MESSAGE_LOOP_HZ/dataRate[MAV_DATA_STREAM_RAW_SENSORS]) {
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
			
			mavlink_msg_raw_imu_encode(mavlinkID, MAV_COMP_ID_IMU, &mavlink_tx_msg, &mavlink_raw_imu);
			mavlink_message_len = mavlink_msg_to_send_buffer(mavlink_message_buf, &mavlink_tx_msg);
			XBeeInhibit(); // XBee input needs to be inhibited before transmitting as some incomming messages cause UART responses which could disrupt XBeeWriteCoordinator if it is interrupted.
			XBeeWriteCoordinator(mavlink_message_buf, mavlink_message_len);
			XBeeAllow();
		}
		XBeeInhibit();
		ILinkPoll(ID_ILINK_RAWIMU);
		XBeeAllow();
		
	}
	
	if(dataRate[MAV_DATA_STREAM_EXTENDED_STATUS] && extStatusStreamCounter >= MESSAGE_LOOP_HZ/dataRate[MAV_DATA_STREAM_EXTENDED_STATUS]) {
		extStatusStreamCounter = 0;
		// GPS_STATUS, CONTROL_STATUS, AUX_STATUS
		
		// Sys status
		if(ilink_thalstat.isNew) {
			ilink_thalstat.isNew = 0;
			
			// do a conversion between ILINK values and MAVLINK for sensor status
			mavlink_sys_status.onboard_control_sensors_enabled = 0;
			if(ilink_thalstat.sensorStatus & (0x1 << 0)) mavlink_sys_status.onboard_control_sensors_enabled |= MAVLINK_SENSOR_GYRO;
			if(ilink_thalstat.sensorStatus & (0x1 << 1)) mavlink_sys_status.onboard_control_sensors_enabled |= MAVLINK_SENSOR_ACCEL;
			if(ilink_thalstat.sensorStatus & (0x1 << 2)) mavlink_sys_status.onboard_control_sensors_enabled |= MAVLINK_SENSOR_MAGNETO;
			if(ilink_thalstat.sensorStatus & (0x1 << 3)) mavlink_sys_status.onboard_control_sensors_enabled |= MAVLINK_SENSOR_BARO;
			mavlink_sys_status.onboard_control_sensors_health = mavlink_sys_status.onboard_control_sensors_enabled;
			
			// do a conversion between ILINK values and MAVLINK for system status
			switch(ilink_thalstat.systemStatus) {
				default:
				case THALSTAT_SYSTEMSTATUS_UNINIT:	mavlink_heartbeat.system_status = MAV_STATE_UNINIT;			break;
				case THALSTAT_SYSTEMSTATUS_BOOT:	mavlink_heartbeat.system_status = MAV_STATE_BOOT;			break;
				case THALSTAT_SYSTEMSTATUS_CALIB:	mavlink_heartbeat.system_status = MAV_STATE_CALIBRATING;	break;
				case THALSTAT_SYSTEMSTATUS_STANDBY:	mavlink_heartbeat.system_status = MAV_STATE_STANDBY;		break;
				case THALSTAT_SYSTEMSTATUS_ACTIVE:	mavlink_heartbeat.system_status = MAV_STATE_ACTIVE; 		break;
				case THALSTAT_SYSTEMSTATUS_CRITICAL:mavlink_heartbeat.system_status = MAV_STATE_CRITICAL;		break;
			}
			mavlink_heartbeat.system_status = MAV_STATE_ACTIVE; 
			// do a conversion between ILINK values and MAVLINK for flight status, refer to Thalamus/inc/state.h for details
			switch(ilink_thalstat.flightStatus) {
				default:
				case 0: // uninitialised
				case 1: // disarmed
					mavlink_heartbeat.base_mode = MAV_MODE_PREFLIGHT;
					break;
				case 2: // manual
				case 3: // manual with GPS
					mavlink_heartbeat.base_mode = MAV_MODE_STABILIZE_ARMED;
					break;
				case 4: // simplicity
					mavlink_sys_status.onboard_control_sensors_enabled |= MAVLINK_CONTROL_YAW;
					mavlink_heartbeat.base_mode = MAV_MODE_STABILIZE_ARMED;
					break;
				case 5: // auto
					mavlink_sys_status.onboard_control_sensors_enabled |= MAVLINK_CONTROL_ATTITUDE | MAVLINK_CONTROL_YAW | MAVLINK_CONTROL_Z;
					mavlink_heartbeat.base_mode = MAV_MODE_GUIDED_ARMED;
					break;
				case 6: // acro mode
					mavlink_sys_status.onboard_control_sensors_enabled |= MAVLINK_CONTROL_ANGLERATE;
					mavlink_heartbeat.base_mode = MAV_MODE_MANUAL_ARMED;
					break;
			}
					
			mavlink_sys_status.voltage_battery = ilink_thalstat.battVoltage;
			mavlink_vfr_hud.throttle = ilink_thalstat.throttle;
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
		
		mavlink_msg_vfr_hud_encode(mavlinkID, MAV_COMP_ID_SYSTEM_CONTROL,  &mavlink_tx_msg, &mavlink_vfr_hud);
		mavlink_message_len = mavlink_msg_to_send_buffer(mavlink_message_buf, &mavlink_tx_msg);
		XBeeInhibit(); // XBee input needs to be inhibited before transmitting as some incomming messages cause UART responses which could disrupt XBeeWriteCoordinator if it is interrupted.
		XBeeWriteCoordinator(mavlink_message_buf, mavlink_message_len);
		XBeeAllow();
	}
	
	if(dataRate[MAV_DATA_STREAM_RC_CHANNELS] && rcChannelCounter >= MESSAGE_LOOP_HZ/dataRate[MAV_DATA_STREAM_RC_CHANNELS]) {
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
			
			mavlink_msg_servo_output_raw_encode(mavlinkID, MAV_COMP_ID_SYSTEM_CONTROL, &mavlink_tx_msg, &mavlink_servo_output_raw);
			mavlink_message_len = mavlink_msg_to_send_buffer(mavlink_message_buf, &mavlink_tx_msg);
			XBeeInhibit(); // XBee input needs to be inhibited before transmitting as some incomming messages cause UART responses which could disrupt XBeeWriteCoordinator if it is interrupted.
			XBeeWriteCoordinator(mavlink_message_buf, mavlink_message_len);
			XBeeAllow();
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
			
			// mavlink_rc_channels_scaled.time_boot_ms = sysMS;
			// mavlink_rc_channels_scaled.chan1_scaled = (signed int)ilink_inputs0.channel[0] * 11.8;
			// mavlink_rc_channels_scaled.chan2_scaled = ((signed int)ilink_inputs0.channel[1] - (signed int)511) * 29.4;
			// mavlink_rc_channels_scaled.chan3_scaled = ((signed int)ilink_inputs0.channel[2] - (signed int)511) * 29.4;
			// mavlink_rc_channels_scaled.chan4_scaled = ((signed int)ilink_inputs0.channel[3] - (signed int)511) * 29.4;
			// if(ilink_inputs0.channel[4] < 500) mavlink_rc_channels_scaled.chan5_scaled = 0;
			// else mavlink_rc_channels_scaled.chan5_scaled = 10000;
			// if(ilink_inputs0.channel[5] < 500) mavlink_rc_channels_scaled.chan6_scaled = 0;
			// else mavlink_rc_channels_scaled.chan6_scaled = 10000;
			// mavlink_rc_channels_scaled.chan7_scaled = 0;
			// mavlink_rc_channels_scaled.chan8_scaled = 0;
			// mavlink_rc_channels_scaled.port = 0;
			// mavlink_rc_channels_scaled.rssi = 255;
			
			// mavlink_msg_rc_channels_scaled_encode(mavlinkID, MAV_COMP_ID_SYSTEM_CONTROL, &mavlink_tx_msg, &mavlink_rc_channels_scaled);
			// mavlink_message_len = mavlink_msg_to_send_buffer(mavlink_message_buf, &mavlink_tx_msg);
			// XBeeInhibit(); // XBee input needs to be inhibited before transmitting as some incomming messages cause UART responses which could disrupt XBeeWriteCoordinator if it is interrupted.
			// XBeeWriteCoordinator(mavlink_message_buf, mavlink_message_len);
			// XBeeAllow();
		}
		XBeeInhibit();
		ILinkPoll(ID_ILINK_INPUTS0);
		XBeeAllow();
		 
	}
	
	if(dataRate[MAV_DATA_STREAM_EXTRA1] && extra1ChannelCounter >= MESSAGE_LOOP_HZ/dataRate[MAV_DATA_STREAM_EXTRA1]) {
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
			
			mavlink_msg_scaled_imu_encode(mavlinkID, MAV_COMP_ID_IMU, &mavlink_tx_msg, &mavlink_scaled_imu);
			mavlink_message_len = mavlink_msg_to_send_buffer(mavlink_message_buf, &mavlink_tx_msg);
			XBeeInhibit(); // XBee input needs to be inhibited before transmitting as some incomming messages cause UART responses which could disrupt XBeeWriteCoordinator if it is interrupted.
			XBeeWriteCoordinator(mavlink_message_buf, mavlink_message_len);
			XBeeAllow();
		}
		XBeeInhibit();
		ILinkPoll(ID_ILINK_SCALEDIMU);
		XBeeAllow();
		
	}
	
	if(dataRate[MAV_DATA_STREAM_EXTRA2] && extra2ChannelCounter > MESSAGE_LOOP_HZ/dataRate[MAV_DATA_STREAM_EXTRA2]) {
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
	
	if(dataRate[MAV_DATA_STREAM_EXTRA3] && extra3ChannelCounter > MESSAGE_LOOP_HZ/dataRate[MAV_DATA_STREAM_EXTRA3]) {
		extra3ChannelCounter = 0;
		
		if(ilink_debug.isNew) {
			ilink_debug.isNew = 0;
			MAVSendVector("DEBUG012", ilink_debug.debug0, ilink_debug.debug1, ilink_debug.debug2);
			/*MAVSendFloat("DEBUG0",  ilink_debug.debug0);
			MAVSendFloat("DEBUG1",  ilink_debug.debug1);
			MAVSendFloat("DEBUG2",  ilink_debug.debug2);*/
			// MAVSendFloat("DEBUG3",  ilink_debug.debug3);
			// MAVSendFloat("DEBUG4",  ilink_debug.debug4);
			// MAVSendFloat("DEBUG5",  ilink_debug.debug5);
			// MAVSendFloat("DEBUG6",  ilink_debug.debug6);
			// MAVSendFloat("DEBUG7",  ilink_debug.debug7);
		}
		
		XBeeInhibit();
		ILinkPoll(ID_ILINK_DEBUG);
		XBeeAllow();
		
	}
}

/*!
\brief Deals with sending Mavlink messages
*/
void mavlink_messages(void) {
	// remote messages
	if(ilink_thalctrl_rx.isNew) {
		ilink_thalctrl_rx.isNew = 0;
		switch(ilink_thalctrl_rx.command) {
			case THALCTRL_EEPROM_SAVEOK:
				/*mavlink_command_ack.result = MAV_MISSION_ACCEPTED;
				mavlink_command_ack.command = MAV_CMD_PREFLIGHT_STORAGE;
				mavlink_msg_command_ack_encode(mavlinkID, MAV_COMP_ID_SYSTEM_CONTROL, &mavlink_tx_msg, &mavlink_command_ack);
				mavlink_message_len = mavlink_msg_to_send_buffer(mavlink_message_buf, &mavlink_tx_msg);
				XBeeWriteCoordinator(mavlink_message_buf, mavlink_message_len);*/
				MAVSendTextFrom(MAV_SEVERITY_INFO, "EEPROM Saved OK", MAV_COMP_ID_SYSTEM_CONTROL);
				break;
			case THALCTRL_EEPROM_LOADOK:
				/*mavlink_command_ack.result = MAV_MISSION_ACCEPTED;
				mavlink_command_ack.command = MAV_CMD_PREFLIGHT_STORAGE;
				mavlink_msg_command_ack_encode(mavlinkID, MAV_COMP_ID_SYSTEM_CONTROL, &mavlink_tx_msg, &mavlink_command_ack);
				mavlink_message_len = mavlink_msg_to_send_buffer(mavlink_message_buf, &mavlink_tx_msg);
				XBeeWriteCoordinator(mavlink_message_buf, mavlink_message_len);*/
				MAVSendTextFrom(MAV_SEVERITY_INFO, "EEPROM Load OK", MAV_COMP_ID_SYSTEM_CONTROL);
				break;
			case THALCTRL_ORIOK:	MAVSendTextFrom(MAV_SEVERITY_INFO, "Craft orientation calibration ok", MAV_COMP_ID_IMU);	break;
			case THALCTRL_ORIBAD:	MAVSendTextFrom(MAV_SEVERITY_WARNING, "WARNING: Craft orientation mismatch!", MAV_COMP_ID_IMU);	break;
			case THALCTRL_RXLOST:	MAVSendTextFrom(MAV_SEVERITY_WARNING, "WARNING: Radio transmitter signal lost!", MAV_COMP_ID_IMU);	break;
			case THALCTRL_RXFOUND:	MAVSendTextFrom(MAV_SEVERITY_INFO, "Radio transmitter signal found", MAV_COMP_ID_IMU);	break;
			case THALCTRL_BATTLOW:	MAVSendTextFrom(MAV_SEVERITY_WARNING, "WARNING: Battery low!", MAV_COMP_ID_SYSTEM_CONTROL);	break;
			case THALCTRL_BATTCRITICAL:	MAVSendTextFrom(MAV_SEVERITY_CRITICAL, "CRITICAL: Batter critical!", MAV_COMP_ID_SYSTEM_CONTROL);	break;
		}
	}
}

/*!
\brief Initialise mavlink stuff
*/
void MAVLinkInit() {
    mavlinkID = (unsigned char) MAV_ID;
	if(mavlinkID == 255) mavlinkID = 0; // 255 reserved for ground control
	
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
             case MAVLINK_MSG_ID_MANUAL_CONTROL:
				// QGROUNDCONTROL BUG: gamepad control don't work
                /*mavlink_msg_manual_control_decode(&mavlink_rx_msg, &mavlink_manual_control);
                if(mavlink_manual_control.target == mavlinkID) {
                    ilink_mancon.roll = mavlink_manual_control.roll;
                    ilink_mancon.pitch = mavlink_manual_control.pitch;
                    ilink_mancon.yaw = mavlink_manual_control.yaw;
                    ilink_mancon.thrust = mavlink_manual_control.thrust;
                    //ILinkSendMessage(ID_ILINK_MANCON, (unsigned short *) & ilink_mancon, sizeof(ilink_mancon)/2-1);
                }*/
                break;
            case MAVLINK_MSG_ID_SET_MODE:
                mavlink_msg_set_mode_decode(&mavlink_rx_msg, &mavlink_set_mode);
                if(mavlink_set_mode.target_system == mavlinkID) {
					//MAVSendVector("MOD", mavlink_set_mode.custom_mode, mavlink_set_mode.base_mode, 0);
					// QGROUNDCONTROL BUG: "Control" doesn't work, so only allowing ARM/Disarm into auto mode here.
						//MAVSendVector("MOD", mavlink_set_mode.custom_mode, mavlink_set_mode.base_mode, 0);
					if(mavlink_set_mode.base_mode & MAV_MODE_FLAG_DECODE_POSITION_SAFETY) {
						ilink_thalctrl_tx.command = THALCTRL_ARM;
						ILinkSendMessage(ID_ILINK_THALCTRL, (unsigned short *) & ilink_thalctrl_tx, sizeof(ilink_thalctrl_tx)/2-1);
					}
					else {
						ilink_thalctrl_tx.command = THALCTRL_DISARM;
						ILinkSendMessage(ID_ILINK_THALCTRL, (unsigned short *) & ilink_thalctrl_tx, sizeof(ilink_thalctrl_tx)/2-1);
					}
				}
                break;
            case MAVLINK_MSG_ID_COMMAND_LONG:
                // actions!
                mavlink_msg_command_long_decode(&mavlink_rx_msg, &mavlink_command_long);
                if(mavlink_command_long.target_system == mavlinkID) {
                    switch(mavlink_command_long.command) {
                        case 0: // custom 0, reset
                            // reset remote
							ilink_thalctrl_tx.command = THALCTRL_RESET;
							ILinkSendMessage(ID_ILINK_THALCTRL, (unsigned short *) & ilink_thalctrl_tx, sizeof(ilink_thalctrl_tx)/2-1);
							
							ILinkFetchData();
							
							/*mavlink_command_ack.result = MAV_MISSION_ACCEPTED;
                            mavlink_command_ack.command = mavlink_command_long.command;
                            mavlink_msg_command_ack_encode(mavlinkID, MAV_COMP_ID_SYSTEM_CONTROL, &mavlink_tx_msg, &mavlink_command_ack);
                            mavlink_message_len = mavlink_msg_to_send_buffer(mavlink_message_buf, &mavlink_tx_msg);
                            XBeeWriteCoordinator(mavlink_message_buf, mavlink_message_len);*/
							
							MAVSendText(MAV_SEVERITY_WARNING, "WARNING: User requested RESET!");
							Delay(100);
                            Reset();
                            break;
							
						case 1: // custom 1
							break;
							
                        case MAV_CMD_PREFLIGHT_STORAGE:
                            if(mavlink_command_long.param1 == 0) { // read all
								eeprom_load_all();
								ilink_thalctrl_tx.command = THALCTRL_EEPROM_LOAD;
								ILinkSendMessage(ID_ILINK_THALCTRL, (unsigned short *) & ilink_thalctrl_tx, sizeof(ilink_thalctrl_tx)/2-1);
							}
                            else {
								eeprom_save_all();
								ilink_thalctrl_tx.command = THALCTRL_EEPROM_SAVE;
								ILinkSendMessage(ID_ILINK_THALCTRL, (unsigned short *) & ilink_thalctrl_tx, sizeof(ilink_thalctrl_tx)/2-1);
							}
                            break;
                            
                        case MAV_CMD_NAV_LAND:
							gps_action = 6; // RTL
							break;
                        case MAV_CMD_NAV_TAKEOFF:
							gps_action = 2; // Takeoff
                        case MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN: // KILL UAS
                            /*ilink_thalctrl_rx.command = MAVLINK_MSG_ID_COMMAND_LONG;
                            ilink_thalctrl_rx.data = mavlink_command_long.command;
                            ILinkSendMessage(ID_ILINK_THALCTRL, (unsigned short *) & ilink_thalctrl_rx, sizeof(ilink_thalctrl_rx)/2-1);*/
                            break;
                        
                        case MAV_CMD_OVERRIDE_GOTO:
							// QGROUNDCONTROL BUG: "Land now" issues a "hold" instruction
							// QGROUNDCONTROL BUG: On gamepad, "Go Home" issues a "hold" instruction
							
                            if(mavlink_command_long.param1 == MAV_GOTO_DO_HOLD) { 
                                gps_action = 3; // hold
                            }
                            else if(mavlink_command_long.param1 == MAV_GOTO_DO_CONTINUE) {
                                gps_action = 4;
                            }
							/*mavlink_command_ack.result = MAV_MISSION_ACCEPTED;
                            mavlink_command_ack.command = mavlink_command_long.command;
                            mavlink_msg_command_ack_encode(mavlinkID, MAV_COMP_ID_MISSIONPLANNER, &mavlink_tx_msg, &mavlink_command_ack);
                            mavlink_message_len = mavlink_msg_to_send_buffer(mavlink_message_buf, &mavlink_tx_msg);
                            XBeeWriteCoordinator(mavlink_message_buf, mavlink_message_len);*/
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
				// remote params
				ilink_thalctrl_tx.command = THALCTRL_EEPROM_READALL;
				ILinkSendMessage(ID_ILINK_THALCTRL, (unsigned short *) & ilink_thalctrl_tx, sizeof(ilink_thalctrl_tx)/2-1);
                // local params
				paramSendCount = 0;
				paramSendSingle = 0;
				paramWaitForRemote = 1;
				break;
            case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
                // request send of one parameters
                mavlink_msg_param_request_read_decode(&mavlink_rx_msg, &mavlink_param_request_read);
			
				if(mavlink_param_request_read.target_system == mavlinkID) {
					if(mavlink_param_request_read.target_component == MAV_COMP_ID_IMU) {
						// remote params
						
						for(i=0; i<MAVLINK_MSG_NAMED_VALUE_FLOAT_FIELD_NAME_LEN; i++) {
							ilink_thalparam_tx.paramName[i] = mavlink_param_request_read.param_id[i];
							if(mavlink_param_request_read.param_id[i] == '\0') break;
						}
						ilink_thalparam_tx.paramID = mavlink_param_request_read.param_index;
						ilink_thalparam_tx.paramCount = 0; // set 1 to request parameter
						ILinkSendMessage(ID_ILINK_THALPARAM, (unsigned short *) &ilink_thalparam_tx, sizeof(ilink_thalparam_tx)/2 - 1);
					}
					else if(mavlink_param_request_read.target_component == MAV_COMP_ID_MISSIONPLANNER) {
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
                mavlink_msg_param_set_decode(&mavlink_rx_msg, &mavlink_param_set);
                if(mavlink_param_set.target_system == mavlinkID) {
                    
					if(mavlink_param_set.target_component == MAV_COMP_ID_IMU) {
						// remote params
						for(i=0; i<MAVLINK_MSG_NAMED_VALUE_FLOAT_FIELD_NAME_LEN; i++) {
							ilink_thalparam_tx.paramName[i] = mavlink_param_set.param_id[i];
							if(mavlink_param_set.param_id[i] == '\0') break;
						}
						
						ilink_thalparam_tx.paramID = 0;
						ilink_thalparam_tx.paramValue = mavlink_param_set.param_value;
						ilink_thalparam_tx.paramCount = 1; // set 1 to write parameter
						ILinkSendMessage(ID_ILINK_THALPARAM, (unsigned short *) &ilink_thalparam_tx, sizeof(ilink_thalparam_tx)/2 - 1);
					}
					else if(mavlink_param_set.target_component == MAV_COMP_ID_MISSIONPLANNER) {
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
            case MAVLINK_MSG_ID_SET_GPS_GLOBAL_ORIGIN:
                mavlink_msg_set_gps_global_origin_decode(&mavlink_rx_msg, &mavlink_set_gps_global_origin);
                if(mavlink_set_gps_global_origin.target_system == mavlinkID) {
                        home_X = (double)mavlink_set_gps_global_origin.latitude / 10000000.0d;
                        home_Y = (double)mavlink_set_gps_global_origin.longitude / 10000000.0d;
                        home_Z = (double)mavlink_set_gps_global_origin.altitude / 1000.0d;
                        home_valid = 1;
                }
            case MAVLINK_MSG_ID_MISSION_CLEAR_ALL:
                mavlink_msg_mission_clear_all_decode(&mavlink_rx_msg, &mavlink_mission_clear_all);
                if(mavlink_mission_clear_all.target_system == mavlinkID) {
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
                if(mavlink_mission_set_current.target_system == mavlinkID) {
                    waypointCurrent = mavlink_mission_set_current.seq;
                    mavlink_mission_current.seq = waypointCurrent;
                    mavlink_msg_mission_current_encode(mavlinkID, MAV_COMP_ID_MISSIONPLANNER, &mavlink_tx_msg, &mavlink_mission_current);
                    mavlink_message_len = mavlink_msg_to_send_buffer(mavlink_message_buf, &mavlink_tx_msg);
                    XBeeWriteCoordinator(mavlink_message_buf, mavlink_message_len);     
                }
                break;
            case MAVLINK_MSG_ID_MISSION_COUNT:
                mavlink_msg_mission_count_decode(&mavlink_rx_msg, &mavlink_mission_count);
                if(mavlink_mission_count.target_system == mavlinkID) {
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
                if(mavlink_mission_request_list.target_system == mavlinkID) {
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
                if(mavlink_mission_request.target_system == mavlinkID) {
                    if(waypointValid != 0) {
                        mavlink_mission_item.target_system = mavlink_rx_msg.sysid;
                        mavlink_mission_item.target_component = mavlink_rx_msg.compid;    

                        mavlink_mission_item.seq = mavlink_mission_request.seq;
                        mavlink_mission_item.frame = waypoint[mavlink_mission_request.seq].frame;
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
                if(mavlink_mission_item.target_system == mavlinkID) {
                    mavlink_mission_ack.type = MAV_MISSION_ERROR;
                    if(mavlink_mission_item.frame == MAV_FRAME_GLOBAL) {
                        if(mavlink_mission_item.seq < MAX_WAYPOINTS) {
                            waypoint[mavlink_mission_item.seq].frame = mavlink_mission_item.frame;
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
                /*mavlink_mission_ack.type = MAV_MISSION_UNSUPPORTED;
                mavlink_msg_mission_ack_encode(mavlinkID, MAV_COMP_ID_MISSIONPLANNER, &mavlink_tx_msg, &mavlink_mission_ack);
                mavlink_message_len = mavlink_msg_to_send_buffer(mavlink_message_buf, &mavlink_tx_msg);
                XBeeWriteCoordinator(mavlink_message_buf, mavlink_message_len);*/
				// ignored
                break;
            case MAVLINK_MSG_ID_MISSION_ACK:
                //ignored
                break;
                
            case MAVLINK_MSG_ID_REQUEST_DATA_STREAM:
                // Sets the output data rates
                mavlink_msg_request_data_stream_decode(&mavlink_rx_msg, &mavlink_request_data_stream);
                if(mavlink_request_data_stream.target_system == mavlinkID) {
                    if(mavlink_request_data_stream.req_message_rate > 255) mavlink_request_data_stream.req_message_rate = 255;
                    dataRate[mavlink_request_data_stream.req_stream_id] = mavlink_request_data_stream.req_message_rate;
                }
                break;
            default:
                //MAVSendInt("CMDIGNORE", mavlink_rx_msg.msgid);
                break;
        }
        //if(mavlink_rx_msg.msgid != 0) MAVSendInt("CMD", mavlink_rx_msg.msgid);
    }
}