/*!
\file Thalamus/params.c
\brief Tuning parameters

\author Yuan Gao
\author Henry Fletcher
\author Oskar Weigl

*/

#include "all.h"

/*!
\brief This large array contains all the tuning parameter defaults.

Tuning parameters are stored in EEPROM, this array contains the defaults, 
which are overwritten by the EEPROM values on startup if the EEPROM checksum
is valid.
*/
struct paramStorage_struct paramStorage[] = {
	[ 0] = {"DRIFT_AKp",       0.2f}, 
    [ 1] = {"DRIFT_MKp",       0.2f}, 
    [ 2] = {"LPF_ULTRA",      0.05f},  
    [ 3] = {"YAW_SEN",      0.0001f},     
    [ 4] = {"PITCH_SEN",    0.0022f},    
    [ 5] = {"ROLL_SEN",     0.0022f},  
    [ 6] = {"YAW_DZN",       0.001f},    
    [ 7] = {"PITCH_Kp",      1000.0f},     
    [ 8] = {"PITCH_Ki",        0.0f},    
    [ 9] = {"PITCH_Kd",      1000.0f},     
    [10] = {"PITCH_Kdd",    1000.0f},
    [11] = {"PITCH_Bst",       0.0f},      
    [12] = {"PITCH_De",      0.999f},
    [13] = {"ROLL_Kp",       1000.0f},       
    [14] = {"ROLL_Ki",         0.0f},      
    [15] = {"ROLL_Kd",       1000.0f},        
    [16] = {"ROLL_Kdd",     1000.0f},             
    [17] = {"ROLL_Bst",       0.00f},                    
    [18] = {"ROLL_De",       0.999f},
    [19] = {"YAW_Kp",       1000.0f},     
    [20] = {"YAW_Kd",       1000.0f},  
    [21] = {"YAW_Bst",        0.00f},
    [22] = {"LPF_BARD",       0.01f},
    [23] = {"LIM_ANGLE",      0.35f},  // Roll and Pitch Angle Limit in Radians
    [24] = {"LIM_ALT",      1000.0f},  // Altitude Limit in mm when in Ultrasound Mode
    [25] = {"CAL_MAGN1",  0.001756f},      
    [26] = {"CAL_MAGN2", 0.0000837f},        
    [27] = {"CAL_MAGN3",0.00005155f},         
    [28] = {"CAL_MAGN5",  0.001964f},            
    [29] = {"CAL_MAGN6",0.00002218f},     
    [30] = {"CAL_MAGN9",  0.001768f},     
    [31] = {"CAL_MAGM1",       0.0f},     
    [32] = {"CAL_MAGM2",       0.0f},         
    [33] = {"CAL_MAGM3",       0.0f}, 
    [34] = {"ULTRA_Kp",       0.05f},
    [35] = {"ULTRA_Kd",        0.1f},
    [36] = {"ULTRA_Ki",    0.00001f},
    [37] = {"ULTRA_De",     0.9999f},
    [38] = {"ULTRA_TKOF",   200.0f}, 
    [39] = {"ULTRA_LND",     150.0f}, 
    [40] = {"CAL_GYROX",       0.0f},
    [41] = {"CAL_GYROY",       0.0f},
    [42] = {"CAL_GYROZ",       0.0f},
    [43] = {"DETUNE",          0.2f},
    [44] = {"LIM_RATE",      100.0f}, 
    [45] = {"LIM_ULTRA",       4.0f},
    [46] = {"ULTRA_DRMP",      3.0f}, 
    [47] = {"ULTRA_DTCT",      6.0f},
    [48] = {"LIM_THROT",       0.3f},
    [49] = {"ULTRA_OVDC",    0.01f},
    [50] = {"ULTRA_DEAD",    100.0f},
    [51] = {"ULTRA_OVTH",     40.0f},
    [52] = {"CAL_AUTO",        1.0f},
    [53] = {"LPF_OUT",         0.6f},
    [54] = {"BAT_LOW",     11000.0f},
    [55] = {"BAT_CRIT",    10000.0f},
    [56] = {"ULTRA_OFST",  350.0f},
    [57] = {"ROLL_SPL",       0.02f},
    [58] = {"PITCH_SPL",      0.02f},
    [59] = {"YAW_Ki",          0.0f},
    [60] = {"YAW_De",          1.0f},
    [61] = {"Filt_GPS_K",      1.0f},
    [62] = {"LPF_BARO",       0.05f},
    [63] = {"GPS_ALTKp",       5.0f},
    [64] = {"GPS_ALTKi",    0.0001f},
    [65] = {"GPS_ALTDe",       1.0f},
    [66] = {"GPS_ALTKd",      20.0f},
    [67] = {"Filt_baroK",      0.0f},
    [68] = {"YAW_SPL",        0.04f},
    [69] = {"ORI",            0.00f},
};

unsigned char paramSendSingle = 0; /*!< boolean to control whether to send a single param or not */
unsigned int paramCount = sizeof(paramStorage)/(PARAMNAMELEN+4);  /*!< contains the number of parametrs */

/*! \brief used to count the ID of parametr to be sent
Needs to be initialised to paramCount so that parameters aren't sent on startup
*/
unsigned int paramSendCount = sizeof(paramStorage)/(PARAMNAMELEN+4);



/*!
\brief Loads all parametrs from EEPRM (this is deprecated)

First it loads the parameters into temporary storage to verify the checksum.  
Only if the checksums are correct will the function update the parameters in RAM
*/
void eeprom_load_all_old(void) {
	unsigned char chkA, chkB;
	unsigned int i;
	unsigned char * ptr;
	float tempStorage[EEPROM_MAX_PARAMS+1];
	
	// Read EEPROM into some temporarily allocated space
	EEPROMRead(EEPROM_OFFSET, (unsigned char *)&tempStorage, paramCount * 4 + 2);
	
	// Calculate the Fletcher-16 checksum
	chkA=EEPROM_VERSION;
	chkB=EEPROM_VERSION;
	ptr = (unsigned char *)&tempStorage;
	for(i=0; i<paramCount*4; i++) {
		chkA += ptr[i];
		chkB += chkA;
	}
	
	// Verify the checksum is valid (at this point i points to the correct elements for chkA and chkB)
	if(chkA == ptr[i+1] && chkB == ptr[i]) {
		// For valid data, load into parameters in RAM
		for(i=0; i<paramCount; i++) {
			paramStorage[i].value = tempStorage[i];
		}
	}
}


/*!
\brief Saves all parameters to EEPRM (this is deprecated)

*/
void eeprom_save_all_old(void) {
	unsigned char chkA, chkB;
	unsigned int i;
	unsigned char * ptr;
	float tempStorage[EEPROM_MAX_PARAMS+1];
	
	// Save parameters into temporary space
	
	chkA=EEPROM_VERSION;
	chkB=EEPROM_VERSION;
	for(i=0; i<paramCount && i<EEPROM_MAX_PARAMS; i++) {
		tempStorage[i] = paramStorage[i].value;
		ptr = (unsigned char *)&(tempStorage[i]);
		chkA += ptr[0];
		chkB += chkA;
		chkA += ptr[1];
		chkB += chkA;
		chkA += ptr[2];
		chkB += chkA;
		chkA += ptr[3];
		chkB += chkA;
	}
	
	ptr = (unsigned char *)&(tempStorage[i]);
	ptr[0] = chkA;
	ptr[1] = chkB;
	
	EEPROMWrite(EEPROM_OFFSET, (unsigned char *)&tempStorage, paramCount * 4);
}


/*!
\brief Does the same as eeprom_load_all_old() but uses less RAM

Optimised to use less RAM but at the expense of more EEPROM read operations.
Should be safer to use because of the lower RAM usage
*/
void eeprom_load_all(void) {
	unsigned char chkA, chkB;
	unsigned int i;
	unsigned char * ptr;
	float tempStorage[1];
	
    chkA=EEPROM_VERSION;
    chkB=EEPROM_VERSION;
    
    // Verify EEPRM
    for(i=0; i<paramCount && i<EEPROM_MAX_PARAMS; i++) {
        EEPROMRead(EEPROM_OFFSET + i*4, (unsigned char *)&(tempStorage[0]), 4);
        
        // Calculate the Fletcher-16 checksum
        ptr = (unsigned char *)&(tempStorage[0]);
		chkA += ptr[0];
		chkB += chkA;
		chkA += ptr[1];
		chkB += chkA;
		chkA += ptr[2];
		chkB += chkA;
		chkA += ptr[3];
		chkB += chkA;
    }
	
	// If the checksum is valid, read out values
	if(chkA == EEPROMReadByte(EEPROM_OFFSET + paramCount*4) && chkB == EEPROMReadByte(EEPROM_OFFSET + paramCount*4 + 1)) {
		// For valid data, load into parameters in RAM
		for(i=0; i<paramCount && i<EEPROM_MAX_PARAMS; i++) {
            EEPROMRead(EEPROM_OFFSET + i*4, (unsigned char *)&(paramStorage[i].value), 4);
		}
	}
}

/*!
\brief Does the same as eeprom_save_all_old() but uses less RAM

Optimised to use less RAM but at the expense of more EEPROM read operations.
should be better to use because of better EEPROM wear (assuming that the
microcontroller doesn't automatically detect writing the same data to EEPROM)
*/
void eeprom_save_all(void) {
	unsigned char chkA, chkB;
	unsigned int i;
	unsigned char * ptr;
	float tempStorage[1];
	
	chkA=EEPROM_VERSION;
	chkB=EEPROM_VERSION;
    
	for(i=0; i<paramCount && i<EEPROM_MAX_PARAMS; i++) {
        EEPROMRead(EEPROM_OFFSET + i*4, (unsigned char *)&(tempStorage[0]), 4);
        if(tempStorage[0] != paramStorage[i].value) EEPROMWrite(EEPROM_OFFSET + i*4, (unsigned char *)&(paramStorage[i].value), 4);
		ptr = (unsigned char *)&(paramStorage[i].value);
		chkA += ptr[0];
		chkB += chkA;
		chkA += ptr[1];
		chkB += chkA;
		chkA += ptr[2];
		chkB += chkA;
		chkA += ptr[3];
		chkB += chkA;
	}
	
	// for some reason single-byte writing does not work
	ptr = (unsigned char *)&(tempStorage[0]);
	ptr[0] = chkA;
	ptr[1] = chkB;
	
	EEPROMWrite(EEPROM_OFFSET + paramCount * 4, (unsigned char *)&tempStorage, 2);
}
