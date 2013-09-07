// ****************************************************************************
// *** EEPROM Functions
// ****************************************************************************

#include "eeprom.h"
#include "thal.h"

unsigned int paramCount;
unsigned int paramSendCount;
unsigned char paramSendSingle;

// *** This function loads all parameters from EEPROM.  First it loads the
// parameters into temporary storage to verify the checksum.  Only if the
// checksums are correct will the function update the parameters in RAM.
void eeprom_load_all(void) {
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
	if(chkA == ptr[i] && chkB == ptr[i+1]) {
		// For valid data, load into parameters in RAM
		for(i=0; i<paramCount; i++) {
			paramStorage[i].value = tempStorage[i];
		}
	}
}

// *** This function saves all parameters to EEPROM.
void eeprom_save_all(void) {
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
	
	EEPROMWrite(EEPROM_OFFSET, (unsigned char *)&tempStorage, paramCount * 4 + 2);
}


// does the same as eeprom_load_all() but uses less RAM but many more EEPROM read operations
// should be safer to use because of lower RAM usage
void eeprom_load_all2(void) {
	unsigned char chkA, chkB;
	unsigned int i;
	unsigned char * ptr;
	float tempStorage;
	
    chkA=EEPROM_VERSION;
    chkB=EEPROM_VERSION;
    
    // Verify EEPRM
    for(i=0; i<paramCount && i<EEPROM_MAX_PARAMS; i++) {
        EEPROMRead(EEPROM_OFFSET + i*4, (unsigned char *)&tempStorage, 4);
        
        // Calculate the Fletcher-16 checksum
        ptr = (unsigned char *)&tempStorage;
        for(i=0; i<4; i++) {
            chkA += ptr[i];
            chkB += chkA;
        }
    }
	
	// If the checksum is valid, read out values
	if(chkA == EEPROMReadByte(EEPROM_OFFSET + paramCount*4) && chkB == EEPROMReadByte(EEPROM_OFFSET + paramCount*4 + 1)) {
		// For valid data, load into parameters in RAM
		for(i=0; i<paramCount && i<EEPROM_MAX_PARAMS; i++) {
            EEPROMRead(EEPROM_OFFSET + i*4, (unsigned char *)&(paramStorage[i].value), 4);
		}
	}
}

// does the same as eeprom_save_all, involves fewer EEPROM write operations and less RAM usage but many more EEPROM read operations
// should be better to use because of better EEPROM wear (assuming that the microcontroller doesn't automatically detect writing the same data to EEPROM)
void eeprom_save_all2(void) {
	unsigned char chkA, chkB;
	unsigned int i;
	unsigned char * ptr;
	float tempStorage;
	
	chkA=EEPROM_VERSION;
	chkB=EEPROM_VERSION;
    
	for(i=0; i<paramCount && i<EEPROM_MAX_PARAMS; i++) {
        EEPROMRead(EEPROM_OFFSET + i*4, (unsigned char *)&tempStorage, 4);
        if(tempStorage != paramStorage[i].value) EEPROMWrite(EEPROM_OFFSET + i*4, (unsigned char *)&(paramStorage[i].value), 4);
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
	
    EEPROMWriteByte(EEPROM_OFFSET + paramCount*4, chkA);
    EEPROMWriteByte(EEPROM_OFFSET + paramCount*4, chkB);
}