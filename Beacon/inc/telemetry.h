/*!
\file Beacon/inc/telemetry.h
\brief MAVLink stuff

\author Yuan Gao
*/

#ifndef __TELEMETRY_H__
#define __TELEMETRY_H__

extern unsigned char mavlinkID;

extern mavlink_message_t mavlink_tx_msg;

extern mavlink_heartbeat_t mavlink_heartbeat_tx;
extern mavlink_command_long_t mavlink_command_long;
extern mavlink_manual_control_t mavlink_manual_control;
extern mavlink_global_position_int_t mavlink_global_position_int;
extern mavlink_set_mode_t mavlink_set_mode;

extern mavlink_ping_t mavlink_ping;
extern mavlink_message_t mavlink_rx_msg;
extern mavlink_status_t mavlink_status;
extern mavlink_heartbeat_t mavlink_heartbeat_rx;
extern mavlink_sys_status_t mavlink_sys_status;
extern mavlink_gps_raw_int_t mavlink_gps_raw_int;
extern mavlink_statustext_t mavlink_statustext;

extern unsigned char mavlink_message_buf[MAVLINK_MAX_PACKET_LEN];
extern unsigned short mavlink_message_len;

/*! \brief Craft data storage structure */
typedef struct craft_data_struct {
	unsigned char valid;                /*!< Whether craft is valid or not */
    unsigned char ID;                   /*!< MAVID */
    unsigned char IDValid;              /*!< MAVID valid */
    unsigned long long sourceAddress;   /*!< Xbee long address */
    unsigned short networkAddress;      /*!< XBee short address */
    
    unsigned char connected;            /*!< Bool: craft connected */
    
    unsigned char baseMode;             /*!< System mode */
    unsigned char systemStatus;         /*!< System status */
    
    unsigned short battVoltage;         /*!< Battery voltage */
    
	signed int lat;	                    /*!< Latitude */
	signed int lon;	                    /*!< Longitude */
	signed int alt;	                    /*!< Altitude (hMSL) */
	unsigned char fix;	                /*!< GPS fix info */
    
    unsigned char statusText[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN]; /*!< status text */
    unsigned char statusTextNew;        /*!< New flag for the status text */
    unsigned char statusTextSeverity;   /*!< Severity of the status text */
} craft_data_t;

extern craft_data_t craft;

void MAVLinkInit(void);
void MAVSendHeartbeat(void);
void MAVLinkParse(unsigned char UARTData);

#endif