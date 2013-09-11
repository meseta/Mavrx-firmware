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
    [ 2] = {"YAW_SEN",      0.0001f},     
    [ 3] = {"PITCH_SEN",    0.0022f},    
    [ 4] = {"ROLL_SEN",     0.0022f},  
    [ 5] = {"YAW_DZN",       0.001f},    
    [ 6] = {"PITCH_Kp",      400.0f},     
    [ 7] = {"PITCH_Ki",        2.0f},    
    [ 8] = {"PITCH_Kd",      100.0f},     
    [ 9] = {"PITCH_Kdd",    1500.0f},
    [10] = {"PITCH_Bst",       0.0f},      
    [11] = {"PITCH_De",      0.999f},
    [12] = {"ROLL_Kp",       400.0f},       
    [13] = {"ROLL_Ki",         2.0f},      
    [14] = {"ROLL_Kd",       100.0f},        
    [15] = {"ROLL_Kdd",     1500.0f},             
    [16] = {"ROLL_Bst",       0.00f},                    
    [17] = {"ROLL_De",       0.999f},
    [18] = {"YAW_Kp",       1000.0f},     
    [19] = {"YAW_Kd",        250.0f},  
    [20] = {"YAW_Bst",        0.00f},
    [21] = {"YAW_Ki",          0.0f},
    [22] = {"YAW_De",          1.0f},
    [23] = {"LIM_ANGLE",      0.35f},
    [24] = {"LIM_ROLL",       0.02f},
    [25] = {"LIM_PITCH",      0.02f},
    [26] = {"LIM_YAW",        0.04f},
    [27] = {"LPF_BARD",       0.01f},
    [28] = {"LPF_OUT",         0.6f},
    [29] = {"LPF_ULTRA",      0.05f},  
    [30] = {"CAL_MAGN1",  0.001756f},      
    [31] = {"CAL_MAGN2", 0.0000837f},        
    [32] = {"CAL_MAGN3",0.00005155f},         
    [33] = {"CAL_MAGN5",  0.001964f},            
    [34] = {"CAL_MAGN6",0.00002218f},     
    [35] = {"CAL_MAGN9",  0.001768f},     
    [36] = {"CAL_MAGM1",       0.0f},     
    [37] = {"CAL_MAGM2",       0.0f},         
    [38] = {"CAL_MAGM3",       0.0f}, 
    [39] = {"ULTRA_Kp",       0.05f},
    [40] = {"ULTRA_Kd",        0.1f},
    [41] = {"ULTRA_Ki",    0.00001f},
    [42] = {"ULTRA_De",     0.9999f},
    [43] = {"ULTRA_TKOF",   200.0f}, 
    [44] = {"ULTRA_LND",     150.0f}, 
    [45] = {"ULTRA_DTCT",      6.0f},
    [46] = {"ULTRA_OVTH",     40.0f},
	[47] = {"GPS_ALTKp",       5.0f},
    [48] = {"GPS_ALTKi",    0.0001f},
    [49] = {"GPS_ALTDe",       1.0f},
    [50] = {"GPS_ALTKd",      20.0f},
    [51] = {"DETUNE",          0.2f},
    [52] = {"BAT_LOW",     11000.0f},
    [53] = {"BAT_CRIT",    10000.0f},
    [54] = {"ORI",            0.00f},
};

unsigned char paramSendSingle = 0; /*!< boolean to control whether to send a single param or not */
unsigned int paramCount = sizeof(paramStorage)/(PARAMNAMELEN+4);  /*!< contains the number of parametrs */

/*! \brief used to count the ID of parametr to be sent
Needs to be initialised to paramCount so that parameters aren't sent on startup
*/
unsigned int paramSendCount = sizeof(paramStorage)/(PARAMNAMELEN+4);


/*!
\brief Loads all parametrs from EEPRM
*/
void eeprom_load_all(void) {
	unsigned char chkA, chkB;
	unsigned int i;
	unsigned char * ptr;
	float tempStorage[1];
	
    chkA=EEPROM_VERSION;
    chkB=EEPROM_VERSION;
    
    // Verify EEPRM
    for(i=0; i<paramCount; i++) {
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
		for(i=0; i<paramCount; i++) {
            EEPROMRead(EEPROM_OFFSET + i*4, (unsigned char *)&(paramStorage[i].value), 4);
		}
	}
}

/*!
\brief Saves all the parameters to EEPROM
*/
void eeprom_save_all(void) {
	unsigned char chkA, chkB;
	unsigned int i;
	unsigned char * ptr;
	float tempStorage[1];
	
	chkA=EEPROM_VERSION;
	chkB=EEPROM_VERSION;
    
	for(i=0; i<paramCount; i++) {
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
