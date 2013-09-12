/*!
\file Beacon/inc/telemetry.h
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
extern mavlink_command_ack_t mavlink_command_ack;
extern mavlink_param_value_t mavlink_param_value;

extern mavlink_named_value_float_t mavlink_named_value_float;
extern mavlink_named_value_int_t mavlink_named_value_int;
extern mavlink_debug_vect_t mavlink_debug_vect;
extern mavlink_statustext_t mavlink_statustext;

extern mavlink_message_t mavlink_rx_msg;
extern mavlink_request_data_stream_t mavlink_request_data_stream;
extern mavlink_command_long_t mavlink_command_long;
extern mavlink_param_set_t mavlink_param_set;
extern mavlink_param_request_read_t mavlink_param_request_read;
extern mavlink_manual_control_t mavlink_manual_control;

extern mavlink_set_mode_t mavlink_set_mode;

extern unsigned char mavlink_message_buf[MAVLINK_MAX_PACKET_LEN];
extern unsigned short mavlink_message_len;

extern unsigned char dataRate[MAV_DATA_STREAM_ENUM_END];

void mavlink_telemetry(void);

void MAVLinkInit(void);
void MAVSendHeartbeat(void);
void MAVSendFloat(char * name, float value);
void MAVSendInt(char * name, int value);
void MAVSendVector(char * name, float valX, float valY, float valZ);
void MAVSendText(unsigned char severity, char * text);
void MAVSendTextFrom(unsigned char severity, char * text, unsigned char from);
void MAVLinkParse(unsigned char UARTData);

#endif