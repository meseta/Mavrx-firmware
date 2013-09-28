/*!
\file Hypo/inc/params.h
\brief Parameters

\author Yuan Gao
*/

#ifndef __PARAMS_H__
#define __PARAMS_H__

#include "mavlink.h"
#include "thal.h"
#define PARAMNAMELEN	MAVLINK_MSG_NAMED_VALUE_FLOAT_FIELD_NAME_LEN	/*!< Maximum length of param name used by MAVLink */

/*! \brief Parameter storage structure */
typedef struct paramStorage_struct {
	char name[PARAMNAMELEN];	/*!< Parameter name */
	float value;				/*!< Parameter value */
} PACKED paramStorage_t;

extern struct paramStorage_struct paramStorage[];
#define MAV_ID   		paramStorage[0].value	/*!< Mavlink ID */
#define GPS_SAFE_ALT    paramStorage[1].value	/*!< Minimum safe altitude above home position in m */
#define GPS_MAX_ANGLE   paramStorage[2].value  	/*!< Maximum attitude demanded in radians */
#define GPS_MAX_SPEED   paramStorage[3].value	/*!< Maximum travel speed in m/s */
#define GPS_MAX_ROTATE  paramStorage[4].value	/*!< Maximum rotation rate in rad/s */
#define GPS_MAX_ALTDIFF paramStorage[5].value	/*!< Maximum altitude demanded in m */
#define GPS_MIN_RADIUS	paramStorage[6].value	/*!< Default (minimum) detection radius for waypoint in meters */
#define GPS_Kp			paramStorage[7].value	/*!< GPS navigation P gain */
#define GPS_Ki			paramStorage[8].value	/*!< GPS navigation I gain */
#define GPS_Kd			paramStorage[9].value	/*!< GPS navigation D gain */
#define GPS_ORBRADIUS   paramStorage[10].value	/*!< GPS navigation D gain */
#define GPS_LAND_SPEED  paramStorage[11].value	/*!< Reduced speed for when near the ground during landing */
#define GPS_LAND_THRES  paramStorage[12].value	/*!< Threshold above landing for the reduced speed */

extern unsigned int paramCount;
extern unsigned int paramSendCount;
extern unsigned char paramSendSingle;

/*! \brief Parameter buffer structure for buffering parametrs from ilink*/
typedef struct paramBuffer_struct {
    char name[PARAMNAMELEN];	/*!< Parameter name */
    float value;				/*!< Parameter value */
    unsigned short id;			/*!< Parameter ID */
} PACKED paramBuffer_t;

#define PARAMBUFFER_SIZE 50		/*!< Parameter buffer size */
extern paramBuffer_t paramBuffer[PARAMBUFFER_SIZE];
extern unsigned int paramPointer;
extern unsigned char paramWaitForRemote;

#define EEPROM_OFFSET   0	/*!< EEPROM Offset used for moving the EEPROM values around storage (wear levelling I guess) */
#define EEPROM_VERSION	10 	/*!< version of variables in EEPROM, change this value to invalidate EEPROM contents and restore defaults */

void eeprom_load_all(void);
void eeprom_save_all(void);

void paramater_transmit(void);

#endif