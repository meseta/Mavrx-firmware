/*!
\file Hypo/inc/gps.h
\brief GPS navigation stuff

\author Yuan Gao
*/

#ifndef __GPS_H__
#define __GPS_H__

#define MAX_WAYPOINTS       40		/*!< Maximum waypoint size, increase this, but beware of running out of RAM */
#define WAYPOINT_HOME       MAX_WAYPOINTS	/*!< Reserve home waypoint */
#define WAYPOINT_TIMEOUT    500/MESSAGE_LOOP_HZ /*!< timeout for waypoint receiving */

#define INTMODE_OFF         0 		/*!< horizontal is off (maximum), veritcal is off (maximum) */
#define INTMODE_VERTICAL    1 		/*!< horizontal is off (maximum), vertical interpolates up to the target */
#define INTMODE_HORIZONTAL  2 		/*!< horizontal interpolates, vertical is off (maximum) */
#define INTMODE_3D          3 		/*!< horizontal interpolates, vertical interpolates */
#define INTMODE_DOWN        4 		/*!< horizontal is off (maximum), vertical decrements indefinately */
#define INTMODE_UP_AND_GO   5 		/*!< same as UP, but auto-switches to waypoint on completion */
#define INTMODE_SEQUENCE_UP 6 		/*!< same as UP, but auto-switches to SEQUENCE_NORMAL on completion */
#define INTMODE_SEQUENCE_3D 7 		/*!< same as 3D, but auto-switches to SEQUENCE_DOWN on completion */
#define INTMODE_SEQUENCE_DOWN   8 	/*!< same as DOWN, but with allow_land explicitely set */

/*! \brief Waypoint storage structure */
typedef struct {
    unsigned char command;			/*!< waypoint command */
    unsigned char autocontinue;		/*!< autocontinue comamnd */
    float param1;					/*!< parameter 1 (see MAVLink spec) */
    float param2;					/*!< parameter 2 (see MAVLink spec)  */
    float param3;					/*!< parameter 3 (see MAVLink spec)  */
    float param4;					/*!< parameter 4 (see MAVLink spec)  */
    double x;						/*!< waypoint x location (latitude) */
    double y;						/*!< waypoint y location (longitude) */
    double z;						/*!< waypoint z location (altutude) */
    unsigned char frame;            /*!< waypoint frome */    
} waypointStruct;

extern gps_nav_posecef_t gps_nav_posecef;
extern gps_nav_posllh_t gps_nav_posllh;
extern gps_nav_status_t gps_nav_status;
extern gps_nav_sol_t gps_nav_sol;
extern gps_nav_velned_t gps_nav_velned;
extern gps_nav_timeutc_t gps_nav_timeutc;

extern unsigned int gpsSendCounter;

extern waypointStruct waypoint[MAX_WAYPOINTS];
extern double home_X;
extern double home_Y;
extern double home_Z;
extern unsigned char home_valid;

extern unsigned short waypointCurrent, waypointCount, waypointReceiveIndex;
extern unsigned char waypointTries, waypointValid, waypointGo, waypointReached;
extern unsigned short waypointTimer;
extern unsigned int waypointLoiterTimer;
extern unsigned char waypointProviderID, waypointProviderComp;

extern unsigned char gpsFixed;
extern unsigned char gps_action;

void gps_navigate(void);

#endif