/*!
\file Beacon/inc/gps.h
\brief GPS navigation stuff

\author Yuan Gao
*/

#ifndef __GPS_H__
#define __GPS_H__

extern gps_nav_posecef_t gps_nav_posecef;
extern gps_nav_posllh_t gps_nav_posllh;
extern gps_nav_status_t gps_nav_status;
extern gps_nav_sol_t gps_nav_sol;
extern gps_nav_velned_t gps_nav_velned;
extern gps_nav_timeutc_t gps_nav_timeutc;

extern unsigned int gpsSendCounter;
extern unsigned char gpsFixed;

/*! \brief GPS data storage structure */
typedef struct gps_data_struct {
	unsigned char valid;    /*!< Whether GPS data is valid */
	signed int lat;	        /*!< Latitude */
	signed int lon;	        /*!< Longitude */
	signed int alt;	        /*!< Altitude (hMSL) */
	signed int velN;	    /*!< North velocity */
	signed int velE;	    /*!< East velocity */
	signed int velD;	    /*!< Down velocity */
	signed int heading;	    /*!< Heading */
} gps_data;

extern gps_data gps;

void gps_process(void);

#endif