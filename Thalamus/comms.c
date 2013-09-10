/*!
\file Thalamus/comms.c
\brief Ilink communication functions

\author Yuan Gao
*/

#include "all.h"


ilink_identify_t 	ilink_identify={0};			/**< Identity */
ilink_thalstat_t 	ilink_thalstat={0};			/**< Thalamus status */
ilink_thalctrl_t 	ilink_thalctrl_rx={0};		/**< Thalamus control receive */
ilink_thalctrl_t 	ilink_thalctrl_tx={0};		/**< Thalamus control transmit */
ilink_imu_t 		ilink_rawimu={0};			/**< Telemetry: raw IMU */
ilink_imu_t 		ilink_scaledimu={0};		/**< Telemetry: scaled IMU */
ilink_altitude_t 	ilink_altitude={0};			/**< Telemetry: altitude */
ilink_attitude_t 	ilink_attitude={0};			/**< Telemetry: attitude */
ilink_thalparam_t 	ilink_thalparam_tx={0};		/**< Transmit parameters */
ilink_thalparam_t 	ilink_thalparam_rx={0};		/**< Receive parametrs */
ilink_thalpareq_t 	ilink_thalpareq={0};		/**< Parameter request control */
ilink_iochan_t 		ilink_inputs0={{0}};		/**< Telemetry: inputs */
ilink_iochan_t 		ilink_outputs0={{0}};		/**< Telemetry: outputs */
ilink_gpsfly_t 		ilink_gpsfly={0};			/**< GPS autofly PID stuff */
ilink_gpsreq_t 		ilink_gpsreq={0};			/**< GPS control */		
ilink_debug_t 		ilink_debug={0};			/**< Debugging parameters */
ilink_mancon_t 		ilink_mancon={0};			/**< External control over SPI */

/*!
\brief Deals with message requests over ilink for messages received

This function is triggered by the SPI interrupt handler.  When a message is 
polled for, this function is triggered, and this function loads the relevant
message from the structs into the send buffer using ILinkSendMessage()

\param id the Message ID that is being requested
*/
void ILinkMessageRequest(unsigned short id) {
	unsigned short * ptr = 0;
	unsigned short maxlength = 0;
	
	switch(id) {
		case ID_ILINK_IDENTIFY:	  ptr = (unsigned short *) &ilink_identify;   maxlength = sizeof(ilink_identify)/2 - 1; break;
        case ID_ILINK_THALCTRL:  ptr = (unsigned short *) &ilink_thalctrl_tx;   maxlength = sizeof(ilink_thalctrl_tx)/2 - 1;   break;
		case ID_ILINK_THALSTAT:	 ptr = (unsigned short *) &ilink_thalstat;   maxlength = sizeof(ilink_thalstat)/2 - 1;   break;
		case ID_ILINK_RAWIMU:	   ptr = (unsigned short *) &ilink_rawimu;	 maxlength = sizeof(ilink_rawimu)/2 - 1;	 break;
		case ID_ILINK_SCALEDIMU:	ptr = (unsigned short *) &ilink_scaledimu;  maxlength = sizeof(ilink_scaledimu)/2 - 1;  break;
		case ID_ILINK_ALTITUDE:	 ptr = (unsigned short *) &ilink_altitude;   maxlength = sizeof(ilink_altitude)/2 - 1;   break;
		case ID_ILINK_ATTITUDE:	 ptr = (unsigned short *) &ilink_attitude;   maxlength = sizeof(ilink_attitude)/2 - 1;   break;
		case ID_ILINK_INPUTS0:   ptr = (unsigned short *) &ilink_inputs0;  maxlength = sizeof(ilink_inputs0)/2 - 1;  break;
		case ID_ILINK_OUTPUTS0:	 ptr = (unsigned short *) &ilink_outputs0;   maxlength = sizeof(ilink_outputs0)/2 - 1;   break;
		case ID_ILINK_DEBUG:	 ptr = (unsigned short *) &ilink_debug;   maxlength = sizeof(ilink_debug)/2 - 1;   break;
		case ID_ILINK_GPSREQ:	 ptr = (unsigned short *) &ilink_gpsreq;   maxlength = sizeof(ilink_gpsreq)/2 - 1;   break;
		case ID_ILINK_CLEARBUF:
			FUNCILinkTxBufferPushPtr = 0;
			FUNCILinkTxBufferPopPtr = 0;
			break;
	}

	if(ptr) {
		ILinkSendMessage(id, ptr, maxlength);
	}
}

/*!
\brief Deals with message received over ilink

When a message arrives over ilink, this function is triggered by the SPI ISR,
this function then shunts the data from the buffer into the relevant structs

\param id the Message ID that is being received
\param buffer pointer to buffer containing the message
\param length the length of the message
*/
void ILinkMessage(unsigned short id, unsigned short * buffer, unsigned short length) {
	unsigned short * ptr = 0;
	unsigned int i, j;
	
	switch(id) {
		case ID_ILINK_THALPAREQ: ptr = (unsigned short *) &ilink_thalpareq; break;
		case ID_ILINK_THALPARAM: ptr = (unsigned short *) &ilink_thalparam_rx; break;
		case ID_ILINK_THALCTRL: ptr = (unsigned short *) &ilink_thalctrl_rx; break;
		case ID_ILINK_GPSFLY: ptr = (unsigned short *) &ilink_gpsfly; break;
	}
	
	if(ptr) {
		for(i=0; i<length; i++) {
			ptr[i] = buffer[i];
		}
		ptr[i] = 1; // this should be the isNew byte
	}
	
	switch(id) {
		case ID_ILINK_THALPAREQ:
			ilink_thalpareq.isNew = 0;
			switch(ilink_thalpareq.reqType) {
				case 1: // get one
					if(ilink_thalpareq.paramID == 0xffff) {

						for (i=0; i<paramCount; i++){
							unsigned char match = 1;
							for (j=0; j<PARAMNAMELEN; j++) {
								if (paramStorage[i].name[j] !=  ilink_thalparam_rx.paramName[j]) {

									match = 0;
									break;
								}
								if (paramStorage[i].name[j] == '\0') break;
							}
							
							if(match == 1) {
								// when a match is found get the iD
								paramSendCount = i;
								paramSendSingle = 1;
								break;
							}
						}
					}

					else {
						paramSendCount = ilink_thalpareq.paramID;
						paramSendSingle = 1;
					}
					break;
				case 2: // save all
					eeprom_save_all();
					ilink_thalpareq.isNew = 1;
					ilink_thalctrl_rx.command = MAVLINK_MSG_ID_COMMAND_LONG;
					ilink_thalctrl_rx.data = MAV_CMD_PREFLIGHT_STORAGE;
					//ILinkSendMessage(ID_ILINK_THALCTRL, (unsigned short *) &ilink_thalctrl_rx, sizeof(ilink_thalctrl_rx)/2 - 1);
					break;
				case 3: // reload all
					eeprom_load_all();
					ilink_thalpareq.isNew = 1;
					ilink_thalctrl_rx.command = MAVLINK_MSG_ID_COMMAND_LONG;
					ilink_thalctrl_rx.data = MAV_CMD_PREFLIGHT_STORAGE;
					//ILinkSendMessage(ID_ILINK_THALCTRL, (unsigned short *) &ilink_thalctrl_rx, sizeof(ilink_thalctrl_rx)/2 - 1);
					// fall through to get all
				default:
				case 0: // get all
					paramSendCount = 0;
					paramSendSingle = 0;
					break;
				}
			break;
		case ID_ILINK_THALCTRL:
            switch(ilink_thalctrl_rx.command) {
                case 0:
                    break;
            }
            break;
		case ID_ILINK_THALPARAM:
			// match up received parameter with stored parameter.
			for (i=0; i<paramCount; i++){
				unsigned char match = 1;
				for (j=0; j<PARAMNAMELEN; j++) {
					if (paramStorage[i].name[j] !=  ilink_thalparam_rx.paramName[j]) {
						match = 0;
						break;
					}
					if (paramStorage[i].name[j] == '\0') break;
				}
				
				if(match == 1) {
					// when a match is found, save it to paramStorage
					paramStorage[i].value = ilink_thalparam_rx.paramValue;
					
					// then order the value to be sent out again using the param send engine
					// but deal with cases where it's already in the process of sending out data
					if(paramSendCount < paramCount) {
						// parameter engine currently sending out data
						if(paramSendCount >= i) {
							// if parameter engine already sent out this now-changed data, redo this one, otherwise no action needed
							paramSendCount = i;
						}
					}
					else {
						// parameter engine not currently sending out data, so send single parameter
						paramSendCount = i;
						paramSendSingle = 1;
					}
					break;
				}
			}
			break;
	}
}