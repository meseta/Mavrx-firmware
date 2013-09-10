#include "all.h"

// *** ILink stuff
ilink_identify_t ilink_identify;
ilink_thalstat_t ilink_thalstat;
ilink_thalctrl_t ilink_thalctrl_rx;
ilink_thalctrl_t ilink_thalctrl_tx;
ilink_imu_t ilink_rawimu;
ilink_imu_t ilink_scaledimu;
ilink_altitude_t ilink_altitude;
ilink_attitude_t ilink_attitude;
ilink_thalparam_t ilink_thalparam_rx;
ilink_thalparam_t ilink_thalparam_tx;
ilink_iochan_t ilink_inputs0;
ilink_iochan_t ilink_outputs0;
ilink_gpsfly_t ilink_gpsfly;
ilink_gpsreq_t ilink_gpsreq;
ilink_debug_t ilink_debug;
ilink_mancon_t ilink_mancon;


void ILinkMessage(unsigned short id, unsigned short * buffer, unsigned short length) {
    unsigned short * ptr = 0;
    unsigned int j;
    
    switch(id) {
        case ID_ILINK_IDENTIFY: ptr = (unsigned short *) &ilink_identify; break;
        case ID_ILINK_THALSTAT: ptr = (unsigned short *) &ilink_thalstat; break;
        case ID_ILINK_THALCTRL: ptr = (unsigned short *) &ilink_thalctrl_rx; break;
        case ID_ILINK_RAWIMU: ptr = (unsigned short *) &ilink_rawimu; break;
        case ID_ILINK_SCALEDIMU: ptr = (unsigned short *) &ilink_scaledimu; break;
        case ID_ILINK_ALTITUDE: ptr = (unsigned short *) &ilink_altitude; break;
        case ID_ILINK_ATTITUDE: ptr = (unsigned short *) &ilink_attitude; break;
        case ID_ILINK_THALPARAM: ptr = (unsigned short *) &ilink_thalparam_rx; break;
        case ID_ILINK_INPUTS0: ptr = (unsigned short *) &ilink_inputs0; break;
        case ID_ILINK_OUTPUTS0: ptr = (unsigned short *) &ilink_outputs0; break;
        case ID_ILINK_DEBUG: ptr = (unsigned short *) &ilink_debug; break;
        case ID_ILINK_GPSREQ: ptr = (unsigned short *) &ilink_gpsreq; break;
    }
    
    if(ptr) {
        for(j=0; j<length; j++) {
            ptr[j] = buffer[j];
        }
        ptr[j] = 1; // this is the "isNew" byte
        switch(id) {
            case ID_ILINK_GPSREQ:
                __NOP(); // this fixes some weird compiler bug that occurs when putting a static after a case
                static unsigned short gpsreq_lastsequence = 0;
                if(ilink_gpsreq.sequence > gpsreq_lastsequence) { 
                    gpsreq_lastsequence = ilink_gpsreq.sequence;
                    
                    if(ilink_gpsreq.request == 0xffff) { // reset sequence number
                        gpsreq_lastsequence = 0;
                    }
                    else if(ilink_gpsreq.request < 0x8000) { // standard commands are < 0x8000
                        gps_action = ilink_gpsreq.request;
                    }
                    else { // sequence commands, explicitely set waypoint number
                        if((ilink_gpsreq.request & 0x7fff) < waypointCount) {
                            waypointCurrent = ilink_gpsreq.request & 0x7fff;
                        }
                        else {
                            waypointCurrent = waypointCount - 1;
                        }
                        gps_action = 4;
                    }
                }
                break;
            case ID_ILINK_THALPARAM: // store parameters in buffer
                if(paramPointer > 0) {
                    paramPointer--;
                    paramBuffer[paramPointer].value = ilink_thalparam_rx.paramValue;
                    paramBuffer[paramPointer].id = ilink_thalparam_rx.paramID;
                    for(j=0; j<PARAMNAMELEN; j++) {
                        paramBuffer[paramPointer].name[j] = ilink_thalparam_rx.paramName[j];
                        if(ilink_thalparam_rx.paramName[j] == '\r') break;
                    }
                }
                break;
			case ID_ILINK_IDENTIFY:
				if(ilink_identify.firmVersion == FIRMWARE_VERSION && ilink_identify.deviceID == I_AM_THALAMUS) {
					mavlink_sys_status.onboard_control_sensors_present |= MAVLINK_SENSOR_GYRO | MAVLINK_SENSOR_ACCEL | MAVLINK_SENSOR_MAGNETO | MAVLINK_SENSOR_BARO | MAVLINK_CONTROL_ANGLERATE | MAVLINK_CONTROL_ATTITUDE | MAVLINK_CONTROL_YAW | MAVLINK_CONTROL_Z;
					mavlink_sys_status.onboard_control_sensors_health = mavlink_sys_status.onboard_control_sensors_enabled;
				}
				break;
        }
		
		thalAvailable = 1;
		thalWatchdog = 0;
    }
}
