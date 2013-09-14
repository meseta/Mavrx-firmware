/*!
\file Hypo/inc/telemetry.h
\brief MAVLink stuff

\author Yuan Gao
*/

#ifndef __TELEMETRY_H__
#define __TELEMETRY_H__

extern unsigned char mavlinkID;

extern mavlink_ping_t mavlink_ping;

extern mavlink_status_t mavlink_status;
extern mavlink_message_t mavlink_tx_msg;
extern mavlink_heartbeat_t mavlink_heartbeat;
extern mavlink_sys_status_t mavlink_sys_status;
extern mavlink_gps_raw_int_t mavlink_gps_raw_int;
extern mavlink_raw_imu_t mavlink_raw_imu;
extern mavlink_scaled_imu_t mavlink_scaled_imu;
extern mavlink_attitude_t mavlink_attitude;
extern mavlink_command_ack_t mavlink_command_ack;
extern mavlink_param_value_t mavlink_param_value;
extern mavlink_rc_channels_raw_t mavlink_rc_channels_raw;
extern mavlink_rc_channels_scaled_t mavlink_rc_channels_scaled;
extern mavlink_servo_output_raw_t mavlink_servo_output_raw;
extern mavlink_gps_global_origin_t mavlink_gps_global_origin;

extern mavlink_named_value_float_t mavlink_named_value_float;
extern mavlink_named_value_int_t mavlink_named_value_int;
extern mavlink_debug_vect_t mavlink_debug_vect;
extern mavlink_statustext_t mavlink_statustext;

extern mavlink_mission_request_t mavlink_mission_request;
extern mavlink_mission_item_t mavlink_mission_item;
extern mavlink_mission_ack_t mavlink_mission_ack;
extern mavlink_mission_item_reached_t mavlink_mission_item_reached;
extern mavlink_vfr_hud_t mavlink_vfr_hud;

extern mavlink_message_t mavlink_rx_msg;
extern mavlink_request_data_stream_t mavlink_request_data_stream;
extern mavlink_command_long_t mavlink_command_long;
extern mavlink_param_set_t mavlink_param_set;
extern mavlink_param_request_read_t mavlink_param_request_read;
extern mavlink_manual_control_t mavlink_manual_control;
extern mavlink_mission_count_t mavlink_mission_count;
extern mavlink_set_gps_global_origin_t mavlink_set_gps_global_origin;
extern mavlink_global_position_int_t mavlink_global_position_int_beacon;
extern mavlink_mission_request_list_t mavlink_mission_request_list;
extern mavlink_mission_count_t mavlink_mission_count;
extern mavlink_mission_clear_all_t mavlink_mission_clear_all;
extern mavlink_mission_set_current_t mavlink_mission_set_current;
extern mavlink_mission_current_t mavlink_mission_current;
extern mavlink_set_mode_t mavlink_set_mode;

extern mavlink_mission_request_list_t mavlink_mission_request_list;

extern unsigned char mavlink_message_buf[MAVLINK_MAX_PACKET_LEN];
extern unsigned short mavlink_message_len;
extern unsigned char dataRate[MAV_DATA_STREAM_ENUM_END];

extern unsigned char waypointProviderID, waypointProviderComp;
extern unsigned short waypointTimer;

void mavlink_telemetry(void);
void mavlink_messages(void);

void MAVLinkInit(void);
void MAVSendHeartbeat(void);
void MAVSendFloat(char * name, float value);
void MAVSendInt(char * name, int value);
void MAVSendVector(char * name, float valX, float valY, float valZ);
void MAVSendText(unsigned char severity, char * text);
void MAVSendTextFrom(unsigned char severity, char * text, unsigned char from);
void MAVLinkParse(unsigned char UARTData);

#endif