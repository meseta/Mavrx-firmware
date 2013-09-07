// ****************************************************************************
// *** EEPROM Functions
// ****************************************************************************

#ifndef __EEPROM_H__
#define __EEPROM_H__

#define EEPROM_MAX_PARAMS   100 // this should be greater than or equal to the above number of parameters
#define EEPROM_OFFSET   0 // EEPROM Offset used for moving the EEPROM values around storage (wear levelling I guess)
#define EEPROM_VERSION	10 // version of variables in EEPROM, change this value to invalidate EEPROM contents and restore defaults

extern unsigned int paramCount;
extern unsigned int paramSendCount;
extern unsigned char paramSendSingle;

typedef struct paramStorage_struct {
	char name[16];
	float value;
} paramStorage_t;

extern struct paramStorage_struct paramStorage[];

// *** This function loads all parameters from EEPROM.  First it loads the
// parameters into temporary storage to verify the checksum.  Only if the
// checksums are correct will the function update the parameters in RAM.
void eeprom_load_all(void);
void eeprom_save_all(void);

// deprecated functions
void eeprom_load_all_old(void);
void eeprom_save_all_old(void);

#endif