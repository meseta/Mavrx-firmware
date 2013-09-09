#ifndef __PARAMS_H__
#define __PARAMS_H__

#define PARAMNAMELEN	10

/*! \brief Parameter storage structure */
typedef struct paramStorage_struct {
	char name[PARAMNAMELEN];	/*!< Parameter name */
	float value;				/*!< Parameter value */
} paramStorage_t;

extern struct paramStorage_struct paramStorage[];
#define DRIFT_AccelKp   paramStorage[0].value
#define DRIFT_MagKp	 	paramStorage[1].value
#define LPF_ULTRA 		paramStorage[2].value
#define YAW_SENS		paramStorage[3].value
#define PITCH_SENS	  	paramStorage[4].value
#define ROLL_SENS	   	paramStorage[5].value
#define YAW_DEADZONE	paramStorage[6].value
#define PITCH_Kp		paramStorage[7].value
#define PITCH_Ki		paramStorage[8].value
#define PITCH_Kd		paramStorage[9].value
#define PITCH_Kdd	   	paramStorage[10].value
#define PITCH_Boost	 	paramStorage[11].value
#define PITCH_De		paramStorage[12].value
#define ROLL_Kp		 	paramStorage[13].value
#define ROLL_Ki		 	paramStorage[14].value
#define ROLL_Kd			paramStorage[15].value
#define ROLL_Kdd		paramStorage[16].value
#define ROLL_Boost	  	paramStorage[17].value
#define ROLL_De		 	paramStorage[18].value
#define YAW_Kp		  	paramStorage[19].value
#define YAW_Kd		  	paramStorage[20].value
#define YAW_Boost	   	paramStorage[21].value
#define LPF_BARD		paramStorage[22].value
#define LIM_ANGLE 		paramStorage[23].value
#define LIM_ALT 		paramStorage[24].value
#define MAGCOR_N1	   	paramStorage[25].value
#define MAGCOR_N2	   	paramStorage[26].value
#define MAGCOR_N3	   	paramStorage[27].value
#define MAGCOR_N5	   	paramStorage[28].value
#define MAGCOR_N6	   	paramStorage[29].value
#define MAGCOR_N9	   	paramStorage[30].value
#define MAGCOR_M1	   	paramStorage[31].value
#define MAGCOR_M2	   	paramStorage[32].value
#define MAGCOR_M3	   	paramStorage[33].value
#define ULTRA_Kp		paramStorage[34].value
#define ULTRA_Kd		paramStorage[35].value
#define ULTRA_Ki		paramStorage[36].value
#define ULTRA_De		paramStorage[37].value
#define ULTRA_TKOFF	 	paramStorage[38].value
#define ULTRA_LND	   	paramStorage[39].value
#define CAL_GYROX	   	paramStorage[40].value
#define CAL_GYROY	   	paramStorage[41].value
#define CAL_GYROZ	   	paramStorage[42].value
#define DETUNE			paramStorage[43].value
#define LIM_RATE		paramStorage[44].value
#define LIM_ULTRA		paramStorage[45].value
#define ULTRA_DRMP	  	paramStorage[46].value
#define ULTRA_DTCT	  	paramStorage[47].value
#define LIM_THROT		paramStorage[48].value
#define ULTRA_OVDEC		paramStorage[49].value
#define ULTRA_DEAD		paramStorage[50].value
#define ULTRA_OVTH		paramStorage[51].value
#define CAL_AUTO		paramStorage[52].value
#define LPF_OUT 		paramStorage[53].value
#define BATT_LOW		paramStorage[54].value
#define BATT_CRIT	   	paramStorage[55].value
#define ULTRA_OFFSET	paramStorage[56].value
#define ROLL_SPL		paramStorage[57].value
#define PITCH_SPL		paramStorage[58].value
#define YAW_Ki			paramStorage[59].value
#define YAW_De			paramStorage[60].value
#define Filt_GPS_K		paramStorage[61].value
#define LPF_BARO  		paramStorage[62].value
#define GPS_ALTKp 		paramStorage[63].value
#define GPS_ALTKi 		paramStorage[64].value
#define GPS_ALTDe 		paramStorage[65].value
#define GPS_ALTKd 		paramStorage[66].value
#define Filt_baroK		paramStorage[67].value
#define YAW_SPL			paramStorage[68].value
#define ORI				paramStorage[69].value

extern unsigned int paramCount;
extern unsigned int paramSendCount;
extern unsigned char paramSendSingle;


// EEPROM STUFF
#define EEPROM_MAX_PARAMS   100 // this should be greater than or equal to the above number of parameters
#define EEPROM_OFFSET   0 // EEPROM Offset used for moving the EEPROM values around storage (wear levelling I guess)
#define EEPROM_VERSION	10 // version of variables in EEPROM, change this value to invalidate EEPROM contents and restore defaults

// *** This function loads all parameters from EEPROM.  First it loads the
// parameters into temporary storage to verify the checksum.  Only if the
// checksums are correct will the function update the parameters in RAM.
void eeprom_load_all(void);
void eeprom_save_all(void);

// deprecated functions
void eeprom_load_all_old(void);
void eeprom_save_all_old(void);


#endif