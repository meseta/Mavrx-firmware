/*!
\file Beacon/inc/telemetry.h
\brief MAVLink stuff

\author Yuan Gao
*/

#ifndef __TELEMETRY_H__
#define __TELEMETRY_H__

extern unsigned char mavlinkID;

extern mavlink_ping_t mavlink_ping;
extern mavlink_message_t mavlink_tx_msg;
extern mavlink_heartbeat_t mavlink_heartbeat;

extern mavlink_command_long_t mavlink_command_long;

extern mavlink_sys_status_t mavlink_sys_status;
extern mavlink_status_t mavlink_status;
extern mavlink_message_t mavlink_rx_msg;
extern mavlink_manual_control_t mavlink_manual_control;
extern mavlink_set_mode_t mavlink_set_mode;

extern unsigned char mavlink_message_buf[MAVLINK_MAX_PACKET_LEN];
extern unsigned short mavlink_message_len;

extern unsigned char craftValid;
extern unsigned long long craftSourceAddress;
extern unsigned char craftNetworkAddress;

void MAVLinkInit(void);
void MAVSendHeartbeat(void);
void MAVLinkParse(unsigned char UARTData);

#endif