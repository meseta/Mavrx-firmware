#include "params.h"
#include "thal.h"






struct paramStorage_struct paramStorage[] = {
	[ 0] = {"MAV_ID",         0.0f}, 
    [ 1] = {"GPS_SAFALT",     20.0f}, 
    [ 2] = {"GPS_MXANGL",    0.35f}, 
    [ 3] = {"GPS_MAXSPD",     1.0f}, 
    [ 4] = {"GPS_MAXROT",     1.5f}, 
    [ 5] = {"GPS_MAXADF",     3.0f}, 
    [ 6] = {"GPS_MINRAD",     2.0f}, 
};


unsigned int paramCount = sizeof(paramStorage)/20;
unsigned int paramSendCount = sizeof(paramStorage)/20; // needs to be initialised to paramCount so that parameters aren't sent on startup
unsigned char paramSendSingle = 0;

// *** This function loads all parameters from EEPROM.  First it loads the
// parameters into temporary storage to verify the checksum.  Only if the
// checksums are correct will the function update the parameters in RAM.
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

// *** This function saves all parameters to EEPROM.
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


// does the same as eeprom_load_all_old() but uses less RAM but many more EEPROM read operations
// should be safer to use because of lower RAM usage
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

// does the same as eeprom_save_all_old(), involves fewer EEPROM write operations and less RAM usage but many more EEPROM read operations
// should be better to use because of better EEPROM wear (assuming that the microcontroller doesn't automatically detect writing the same data to EEPROM)
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