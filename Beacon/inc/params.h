/*!
\file Beacon/inc/params.h
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

extern unsigned int paramCount;
extern unsigned int paramSendCount;
extern unsigned char paramSendSingle;

#define EEPROM_OFFSET   0	/*!< EEPROM Offset used for moving the EEPROM values around storage (wear levelling I guess) */
#define EEPROM_VERSION	10 	/*!< version of variables in EEPROM, change this value to invalidate EEPROM contents and restore defaults */

void eeprom_load_all(void);
void eeprom_save_all(void);

#endif