/*!
\file Beacon/gps.c
\brief GPS waypoint flying

\author Yuan Gao

*/

#include "all.h"

gps_nav_posecef_t gps_nav_posecef;		/*!< GPS Position in ECEF (not used) */
gps_nav_posllh_t gps_nav_posllh;		/*!< GPS Position in LLH */
gps_nav_status_t gps_nav_status;		/*!< GPS navigation status */
gps_nav_sol_t gps_nav_sol;				/*!< GPS navigation solution */
gps_nav_velned_t gps_nav_velned;		/*!< GPS velocity in NED */
gps_nav_timeutc_t gps_nav_timeutc;		/*!< GPS time in UTC */

unsigned int gpsSendCounter;			/*!< GPS loop counter */
unsigned char gpsFixed=0;				/*!< Boolean: whether GPS is fixed */

/*!
\brief Communications with the uBlox GPS

This function is called by the GPS processing function that needs to be called
at regular intervals to poll the GPS for data over I2C
*/
void GPSMessage(unsigned short id, unsigned char * buffer, unsigned short length) {
    unsigned char * ptr = 0;
    unsigned short j;

    switch(id) {
        case ID_NAV_POSECEF: ptr = (unsigned char *) &gps_nav_posecef; break;
        case ID_NAV_POSLLH: ptr = (unsigned char *) &gps_nav_posllh; break;
        case ID_NAV_STATUS: ptr = (unsigned char *) &gps_nav_status; break;
        case ID_NAV_SOL: ptr = (unsigned char *) &gps_nav_sol; break;
        case ID_NAV_VELNED: ptr = (unsigned char *) &gps_nav_velned; break;
        case ID_NAV_TIMEUTC: ptr = (unsigned char *) &gps_nav_timeutc; break;
    }

    if(ptr) {
        for(j=0; j<length; j++) {
            ptr[j] = buffer[j];
        }
        ptr[j] = 1; // this is the "isNew" byte
    }
}
