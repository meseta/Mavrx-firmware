#ifndef __PARAMS_H__
#define __PARAMS_H__

typedef struct paramStorage_struct {
	char name[16];
	float value;
} paramStorage_t;

extern struct paramStorage_struct paramStorage[];
#define MAV_ID   		paramStorage[0].value
#define GPS_SAFE_ALT    paramStorage[1].value	// Minimum safe altitude above home position in m
#define GPS_MAX_ANGLE   paramStorage[2].value  	// Maximum attitude demanded in radians
#define GPS_MAX_SPEED   paramStorage[3].value	// Maximum travel speed in m/s
#define GPS_MAX_ROTATE  paramStorage[4].value	// Maximum rotation rate in rad/s    
#define GPS_MAX_ALTDIFF paramStorage[5].value	// Maximum altitude demanded in m
#define GPS_MIN_RADIUS	paramStorage[6].value	// Default (minimum) detection radius for waypoint in meters



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