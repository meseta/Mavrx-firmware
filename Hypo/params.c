/*!
\file Hypo/params.c
\brief Tuning parameters

\author Yuan Gao

*/

#include "all.h"

/*!
\brief This large array contains all the tuning parameter defaults.

Tuning parameters are stored in EEPROM, this array contains the defaults, 
which are overwritten by the EEPROM values on startup if the EEPROM checksum
is valid.
*/
struct paramStorage_struct paramStorage[] = {
	[ 0] = {"MAV_ID",         0.0f}, 
    [ 1] = {"GPS_SAFALT",     20.0f}, 
    [ 2] = {"GPS_MXANGL",    0.35f}, 
    [ 3] = {"GPS_MAXSPD",     1.0f}, 
    [ 4] = {"GPS_MAXROT",     1.5f}, 
    [ 5] = {"GPS_MAXADF",     3.0f}, 
    [ 6] = {"GPS_MINRAD",     2.0f},  
    [ 7] = {"GPS_Kp",        0.03f}, 
    [ 8] = {"GPS_Ki",        0.00f}, 
    [ 9] = {"GPS_Kd",         0.1f}, 
    [ 10] = {"GPS_ORBRAD",     3.0f},
};

unsigned char paramSendSingle = 0; /*!< boolean to control whether to send a single param or not */
unsigned int paramCount = sizeof(paramStorage)/(PARAMNAMELEN+4);  /*!< contains the number of parametrs */

/*! \brief used to count the ID of parametr to be sent
Needs to be initialised to paramCount so that parameters aren't sent on startup
*/
unsigned int paramSendCount = sizeof(paramStorage)/(PARAMNAMELEN+4);

paramBuffer_t paramBuffer[PARAMBUFFER_SIZE];	/*!< Parameter buffer storage */
unsigned int paramPointer = PARAMBUFFER_SIZE;	/*!< Parameter buffer storage space */
unsigned char paramWaitForRemote = 0;			/*!< Boolean for determining whether to wait for the remote device to transmit its parameters */

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

/*!
\brief deals with the transmission of parameters via MAVLink
*/
void paramater_transmit(void) {
	unsigned int i;

	if(paramPointer < PARAMBUFFER_SIZE) {
		// shunt this along to GCS
		for(i=0; i<PARAMNAMELEN; i++) {
			mavlink_param_value.param_id[i] = paramBuffer[paramPointer].name[i];
			if(paramBuffer[paramPointer].name[i] == '\0') break;
		}
		mavlink_param_value.param_value = paramBuffer[paramPointer].value;
		mavlink_param_value.param_count = ilink_thalparam_rx.paramCount;
		mavlink_param_value.param_index = paramBuffer[paramPointer].id;
		mavlink_param_value.param_type = MAV_PARAM_TYPE_REAL32;
		
		paramPointer++;
		
		mavlink_msg_param_value_encode(mavlinkID, MAV_COMP_ID_IMU, &mavlink_tx_msg, &mavlink_param_value);
		mavlink_message_len = mavlink_msg_to_send_buffer(mavlink_message_buf, &mavlink_tx_msg);
		XBeeInhibit(); // XBee input needs to be inhibited before transmitting as some incomming messages cause UART responses which could disrupt XBeeWriteCoordinator if it is interrupted.
		XBeeWriteCoordinator(mavlink_message_buf, mavlink_message_len);
		XBeeAllow();
		paramWaitForRemote = 0;
	}
	else if(paramSendCount < paramCount && ((thalAvailable == 1 && paramWaitForRemote == 0) || thalAvailable == 0)) {
		// these extra conditions are a small hack: qgroundcontrol doesn't seem to want to display the received status for the second component to transmit parameters, so the above line forces Hypo to transmit after Thalamus since Hypo is generally more reliable
		unsigned short thisParam = paramSendCount; // store this to avoid race hazard since paramSendCount can change outside this interrupt
		
		for(i=0; i<PARAMNAMELEN; i++) {
			mavlink_param_value.param_id[i] = paramStorage[thisParam].name[i];
			if(paramStorage[thisParam].name[i] == '\0') break;
		}
		
		mavlink_param_value.param_value = paramStorage[thisParam].value;
		mavlink_param_value.param_count = paramCount; // this value shouldn't change
		mavlink_param_value.param_index = thisParam;
		mavlink_param_value.param_type = MAV_PARAM_TYPE_REAL32;
		
		mavlink_msg_param_value_encode(mavlinkID, MAV_COMP_ID_MISSIONPLANNER, &mavlink_tx_msg, &mavlink_param_value);
		mavlink_message_len = mavlink_msg_to_send_buffer(mavlink_message_buf, &mavlink_tx_msg);
		
		XBeeInhibit(); // XBee input needs to be inhibited before transmitting as some incomming messages cause UART responses which could disrupt XBeeWriteCoordinator if it is interrupted.
		XBeeWriteCoordinator(mavlink_message_buf, mavlink_message_len);
		XBeeAllow();
		
		if(paramSendSingle) {
			paramSendSingle = 0;
			paramSendCount = paramCount;
		}
		else {
			paramSendCount = thisParam+1;
		}
	}
}