#ifndef __GPS_H__
#define __GPS_H__

#define MAX_WAYPOINTS       20
#define WAYPOINT_HOME       MAX_WAYPOINTS
#define WAYPOINT_TIMEOUT    500/MESSAGE_LOOP_HZ // value = timeout in ms / message_loop_hz

typedef struct {
    unsigned char command;
    unsigned char autocontinue;
    float param1;
    float param2;
    float param3;
    float param4;
    double x;
    double y;
    double z;
} waypointStruct;


// *** GPS stuff
extern gps_nav_posecef_t gps_nav_posecef;
extern gps_nav_posllh_t gps_nav_posllh;
extern gps_nav_status_t gps_nav_status;
extern gps_nav_sol_t gps_nav_sol;
extern gps_nav_velned_t gps_nav_velned;
extern gps_nav_timeutc_t gps_nav_timeutc;


extern unsigned int gpsSendCounter;

// ****** waypoints

extern waypointStruct waypoint[MAX_WAYPOINTS];
extern double home_X;
extern double home_Y;
extern double home_Z;
extern float home_valid;


extern unsigned short waypointCurrent, waypointCount, waypointReceiveIndex;
extern unsigned char waypointTries, waypointValid, waypointGo, waypointReached;
extern unsigned short waypointTimer;
extern unsigned int waypointLoiterTimer;
extern unsigned char waypointProviderID, waypointProviderComp;
extern float waypointPhase;

extern float lat_diff_i;
extern float lon_diff_i;

extern unsigned char gpsFixed;
extern unsigned char gps_action;

void gps_navigate(void);

#endif