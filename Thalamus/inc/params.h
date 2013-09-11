/*!
\file Thalamus/inc/params.h
\brief Tuning parameters

\author Yuan Gao
\author Henry Fletcher
\author Oskar Weigl

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
#define DRIFT_AccelKp   paramStorage[0].value	/*!< Gyro drift compensation from Accelerometer */
#define DRIFT_MagKp	 	paramStorage[1].value	/*!< Gyro drift compensation from Accelerometer */
#define YAW_SENS		paramStorage[2].value	/*!< Yaw control sensitivity */
#define PITCH_SENS	  	paramStorage[3].value	/*!< Pitch control sensitivity */
#define ROLL_SENS	   	paramStorage[4].value	/*!< Roll control sensitivity */
#define YAW_DEADZONE	paramStorage[5].value	/*!< Yaw control deadzone */
#define PITCH_Kp		paramStorage[6].value	/*!< Pitch P gain */
#define PITCH_Ki		paramStorage[7].value	/*!< Pitch I gain */
#define PITCH_Kd		paramStorage[8].value	/*!< Pitch D gain */
#define PITCH_Kdd	   	paramStorage[9].value	/*!< Pitch DD gain */
#define PITCH_Boost	 	paramStorage[10].value	/*!< Pitch conttrol boost gain */
#define PITCH_De		paramStorage[11].value	/*!< Pitch integral decay */
#define ROLL_Kp		 	paramStorage[12].value	/*!< Roll P gain */
#define ROLL_Ki		 	paramStorage[13].value	/*!< Roll I gain */
#define ROLL_Kd			paramStorage[14].value	/*!< Roll D gain */
#define ROLL_Kdd		paramStorage[15].value	/*!< Roll DD gain */
#define ROLL_Boost	  	paramStorage[16].value	/*!< Roll control boost gain */
#define ROLL_De		 	paramStorage[17].value	/*!< Roll integral decay */
#define YAW_Kp		  	paramStorage[18].value	/*!< Yaw P gain */
#define YAW_Kd		  	paramStorage[19].value	/*!< Yaw D gain */
#define YAW_Boost	   	paramStorage[20].value	/*!< Yaw control boost gain */
#define YAW_Ki			paramStorage[21].value	/*!< Yaw I gain */
#define YAW_De			paramStorage[22].value	/*!< Yaw integral decay */
#define LIM_ANGLE 		paramStorage[23].value	/*!< Pitch/roll angle limit */
#define LIM_ROLL		paramStorage[24].value	/*!< Roll rate limit */
#define LIM_PITCH		paramStorage[25].value	/*!< Roll rate limit */
#define LIM_YAW			paramStorage[26].value	/*!< Yaw rate limit */
#define LPF_BARD		paramStorage[27].value	/*!< Barometer derivative filter */
#define LPF_OUT 		paramStorage[28].value	/*!< Output filters */
#define LPF_ULTRA 		paramStorage[29].value	/*!< Ultrasound filter */
#define MAGCOR_N1	   	paramStorage[30].value	/*!< Magneto soft iron calibration matrix element 1,1 */
#define MAGCOR_N2	   	paramStorage[31].value	/*!< Magneto soft iron calibration matrix element 1,2 */
#define MAGCOR_N3	   	paramStorage[32].value	/*!< Magneto soft iron calibration matrix element 1,3 */
#define MAGCOR_N5	   	paramStorage[33].value	/*!< Magneto soft iron calibration matrix element 2,2 */
#define MAGCOR_N6	   	paramStorage[34].value	/*!< Magneto soft iron calibration matrix element 2,3 */
#define MAGCOR_N9	   	paramStorage[35].value	/*!< Magneto soft iron calibration matrix element 3,3 */
#define MAGCOR_M1	   	paramStorage[36].value	/*!< Magneto hard iron calibration vector element 1 */
#define MAGCOR_M2	   	paramStorage[37].value	/*!< Magneto hard iron calibration vector element 2 */
#define MAGCOR_M3	   	paramStorage[38].value	/*!< Magneto hard iron calibration vector element 3 */
#define ULTRA_Kp		paramStorage[39].value	/*!< Ultrasound P gain */
#define ULTRA_Kd		paramStorage[40].value	/*!< Ultrasound D gain */
#define ULTRA_Ki		paramStorage[41].value	/*!< Ultrasound I gain */
#define ULTRA_De		paramStorage[42].value	/*!< Ultrasound integral decay */
#define ULTRA_TKOFF	 	paramStorage[43].value	/*!< Ultrasound takeoff threshold */
#define ULTRA_LND	   	paramStorage[44].value	/*!< Ultrasound land threshold */
#define ULTRA_DTCT	  	paramStorage[45].value	/*!< Number of ultrasound measurements for landing detection */
#define ULTRA_OVTH		paramStorage[46].value	/*!< Number of ultrasound measurements for considering ultrasound loss */
#define GPS_ALTKp 		paramStorage[47].value	/*!< GPS altitude P gain */
#define GPS_ALTKi 		paramStorage[48].value	/*!< GPS altitude I gain */
#define GPS_ALTDe 		paramStorage[49].value	/*!< GPS altitude integral decay */
#define GPS_ALTKd 		paramStorage[50].value	/*!< GPS altitude D gain */
#define DETUNE			paramStorage[51].value	/*!< Detune at high throttle */
#define BATT_LOW		paramStorage[52].value	/*!< Battery low voltage */
#define BATT_CRIT	   	paramStorage[53].value	/*!< Battery critical voltage */
#define ORI				paramStorage[54].value	/*!< Craft orientation/configuration */

extern unsigned int paramCount;
extern unsigned int paramSendCount;
extern unsigned char paramSendSingle;

#define EEPROM_OFFSET   0	/*!< EEPROM Offset used for moving the EEPROM values around storage (wear levelling I guess) */
#define EEPROM_VERSION	10 	/*!< version of variables in EEPROM, change this value to invalidate EEPROM contents and restore defaults */

void eeprom_load_all(void);
void eeprom_save_all(void);

#endif