/*!
\file Hypo/gps.c
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

waypointStruct waypoint[MAX_WAYPOINTS];	/*!< Waypoint storage */
double home_lat;							/*!< Home X waypoint */
double home_lon;							/*!< Home Y waypoint */
double home_alt;							/*!< Home Z waypoint */
unsigned char home_valid=0;				/*!< Boolean: whether home waypoint is valid */

unsigned short waypointCurrent=0;		/*!< Current waypoint */
unsigned short waypointCount=0;			/*!< Number of waypoints */
unsigned short waypointReceiveIndex=0;	/*!< Index for receiving waypoints */
unsigned char waypointTries;			/*!< Number of attemts to get waypoint*/
unsigned char waypointValid=0;			/*!< Boolean: Whether waypoints are vaild */
unsigned char waypointGo=0;				/*!< Boolean: Waypoints execution */
unsigned char waypointReached;			/*!< Boolean: Waypoint is reached */
unsigned int waypointLoiterTimer;		/*!< Waypoint loiter time */
unsigned char waypointProviderID;		/*!< The provider device of the waypoints */
unsigned char waypointProviderComp;		/*!< The provider component of the waypoints */

unsigned short waypointTimer=0;			/*!< Waypoint loop timer */

unsigned char gpsFixed=0;				/*!< Boolean: whether GPS is fixed */
unsigned char gps_action = 0;			/*!< GPS Actions */

/*!
\brief Does the GPS navigation thing

Comes in five parts: collect GPS data, apply GPS action, GPS navigation
interpolator, GPS PID, and waypoint goal detection
*/
void gps_navigate(void) {
	static unsigned short gpsSendCounter = 0;
    gpsSendCounter++;

	if(gpsSendCounter >= MESSAGE_LOOP_HZ/5) {
        gpsSendCounter = 0;

        XBeeInhibit();
        ILinkPoll(ID_ILINK_GPSREQ);
        XBeeAllow();

        // *** Get GPS data
        static double craft_lat = 0;
        static double craft_lon = 0;
        static double craft_alt = 0;

        if(gps_nav_status.isNew) {
            gps_nav_status.isNew = 0;

            if((gps_nav_status.gpsFix == 0x03 || gps_nav_status.gpsFix == 0x04) && gps_nav_status.flags & 0x1) { // fix is 3D and valid
                mavlink_gps_raw_int.fix_type = gps_nav_status.gpsFix;
                gpsFixed = 1;
            }
            else {
                mavlink_gps_raw_int.fix_type = 0;
                gpsFixed = 0;
            }
            //mavlink_gps_raw_int.satellites_visible = gps_nav_sol.numSV;
        }

        if(gps_nav_posllh.isNew) {
            gps_nav_posllh.isNew = 0;

            if(gpsFixed == 1) {
                gpsWatchdog = 0;
                mavlink_gps_raw_int.lat = gps_nav_posllh.lat;
                mavlink_gps_raw_int.lon = gps_nav_posllh.lon;
                mavlink_gps_raw_int.alt = gps_nav_posllh.hMSL;
                mavlink_gps_raw_int.eph = gps_nav_posllh.hAcc / 10;
                mavlink_gps_raw_int.epv = gps_nav_posllh.vAcc / 10;

				mavlink_vfr_hud.alt = mavlink_gps_raw_int.alt = gps_nav_posllh.hMSL;

                craft_lat = gps_nav_posllh.lat / 10000000.0d;
                craft_lon = gps_nav_posllh.lon / 10000000.0d;
                craft_alt = (double)gps_nav_posllh.hMSL/ 1000.0d;
            }
        }

        if(gps_nav_velned.isNew) {
            gps_nav_velned.isNew = 0;

            mavlink_gps_raw_int.vel = gps_nav_velned.gSpeed;
            mavlink_gps_raw_int.cog = gps_nav_velned.heading / 100; // because GPS assumes cog IS heading.

			mavlink_vfr_hud.groundspeed = gps_nav_velned.gSpeed;
			mavlink_vfr_hud.airspeed = gps_nav_velned.speed;
			mavlink_vfr_hud.heading = gps_nav_velned.heading / 100;
			mavlink_vfr_hud.climb = (float)gps_nav_velned.velD / 100.0f;
		}

        if(gpsFixed) {
            // *** Actions
            static double target_lat, target_lon, target_alt, target_yaw;
            static double interpolator_lat, interpolator_lon, interpolator_alt, interpolator_yaw;
            static unsigned char interpolator_mode = 0;
            static unsigned char free_yaw = 1;
            static unsigned char allow_land = 0;
            static unsigned char target_set = 0;

			// 0:	do nothing
			// 1:	store home position - Thalamus requests this everytime it arms into MANUAL_GPS or AUTO mode
			// 2:	take off - sets target to GPS_SAFE_ALT above current location
			// 3:	hold/pause - sets target to current craft location
			// 4:	resume current waypoint
			// 5: 	unscheduled land - sets target to current location, and sets the incrementer mode to "landing"
			// 6: 	return to home - sets waypoint to home, unsets the "land when reached" flag
			// 7:
            switch(gps_action) {
                case 0:
                default: // do nothing
                    break;
                case 1: // store home position
                    gps_action = 0;
                    gps_set_home(craft_lat, craft_lon, craft_alt);
                    break;
                case 2: // take off - sets target to GPS_SAFE_ALT above current location
                    gps_action = 0;
                    if(home_valid == 0) {
                       gps_set_home(craft_lat, craft_lon, craft_alt);
                    }
                    target_lat = craft_lat;
                    target_lon = craft_lon;
                    target_alt = craft_alt + GPS_SAFE_ALT;
                    target_set = 1;

                    free_yaw = 1;
                    MAVSendTextFrom(MAV_SEVERITY_INFO, "Taking off to preset safe altitude", MAV_COMP_ID_MISSIONPLANNER);
                    interpolator_mode = INTMODE_VERTICAL;
                    waypointGo = 0;
                    break;
                case 3: // hold/pause - unset target to hold
                    gps_action = 0;
                    target_set = 0;
					if(waypointGo == 1) {
						waypointGo = 0;
						MAVSendTextFrom(MAV_SEVERITY_INFO, "Waypoint execution paused, position hold", MAV_COMP_ID_MISSIONPLANNER);
					}
					else {
						MAVSendTextFrom(MAV_SEVERITY_INFO, "Position hold", MAV_COMP_ID_MISSIONPLANNER);
					}
                    break;
                case 4: // resume - resume current waypoint
                    gps_action = 0;
                    if(waypointValid == 1 && waypointCurrent < waypointCount) {
						MAVSendTextFrom(MAV_SEVERITY_INFO, "Waypoint execution resumed", MAV_COMP_ID_MISSIONPLANNER);

                        // if we're already going, check if we're stuck at a LOITER_UNLIM position, and break out of it
                        if(waypointGo == 1 && waypointReached == 1 && waypoint[waypointCurrent].command == MAV_CMD_NAV_LOITER_UNLIM) {
                            waypointCurrent++;
                        }
                        waypointGo = 1;
                        waypointReached = 0;

                        if(waypoint[waypointCurrent].frame == MAV_FRAME_GLOBAL || waypoint[waypointCurrent].frame == MAV_FRAME_GLOBAL_RELATIVE_ALT) {
                            switch(waypoint[waypointCurrent].command) {
                                case MAV_CMD_NAV_LOITER_TURNS:
                                    MAVSendTextFrom(MAV_SEVERITY_WARNING, "WARNING: Loiter turns not supported!", MAV_COMP_ID_MISSIONPLANNER);
                                case MAV_CMD_NAV_WAYPOINT:
                                case MAV_CMD_NAV_LOITER_TIME:
                                case MAV_CMD_NAV_LOITER_UNLIM:
                                case MAV_CMD_NAV_LAND:
                                    target_lat = waypoint[waypointCurrent].x;
                                    target_lon = waypoint[waypointCurrent].y;
                                    target_alt = waypoint[waypointCurrent].z;
                                    if(waypoint[waypointCurrent].frame == MAV_FRAME_GLOBAL_RELATIVE_ALT) {
                                        target_alt += home_alt;
                                    }
                                    if(free_yaw == 1) { // if previous yaw state was free yaw, set interpolator to demand to avoid swinging the craft unnecessarily
                                        target_yaw = waypoint[waypointCurrent].param4 * 0.01745329251994329577f; // 0.01745329251994329577 is degrees to radian conversion
                                        interpolator_yaw = target_yaw;
                                    }
                                    else {  // otherwise assume previous target_yaw is current craft yaw and set interpolator to start there
                                        interpolator_yaw = target_yaw;
                                        target_yaw = waypoint[waypointCurrent].param4 * 0.01745329251994329577f; // 0.01745329251994329577 is degrees to radian conversion
                                    }
                                    free_yaw = 0;
                                    target_set = 1;
                                    break;
                                case MAV_CMD_NAV_RETURN_TO_LAUNCH:
                                    gps_action = 6;
                                    break;
                                default:
                                    MAVSendTextFrom(MAV_SEVERITY_WARNING, "WARNING: Waypoint command not supported", MAV_COMP_ID_MISSIONPLANNER);
                                    waypointGo = 0;
                                    target_set = 0;
                                    gps_action = 3;
                                    break;
                            }
                        }
                        else {
                            MAVSendTextFrom(MAV_SEVERITY_WARNING, "WARNING: Waypoint frame not supported", MAV_COMP_ID_MISSIONPLANNER);
                            waypointGo = 0;
                            target_set = 0;
                            gps_action = 3;
                        }
                    }
                    else {
						MAVSendTextFrom(MAV_SEVERITY_WARNING, "WARNING: No valid waypoints to resume", MAV_COMP_ID_MISSIONPLANNER);
                        waypointGo = 0;
                        target_set = 0;
                        gps_action = 3;
                    }
                    break;
                case 5: // land here - sets target to current location, and sets the incrementer mode to "landing"
                    gps_action = 0;
					MAVSendTextFrom(MAV_SEVERITY_INFO, "Landing here", MAV_COMP_ID_MISSIONPLANNER);
                    target_lat = craft_lat;
                    target_lon = craft_lon;
                    target_alt = craft_alt;
                    target_set = 1;
                    interpolator_mode = INTMODE_DOWN; // three for landing
                    waypointGo = 0;
                    free_yaw = 1;
                    break;
                case 6: // return to home - sets waypoint to home, unsets the "land when reached" flag
                    gps_action = 0;
                    if(home_valid == 1) {
						MAVSendTextFrom(MAV_SEVERITY_INFO, "Returning to home position", MAV_COMP_ID_MISSIONPLANNER);
                        target_lat = home_lat;
                        target_lon = home_lon;
                        target_alt = home_alt + GPS_SAFE_ALT;
                        interpolator_mode = INTMODE_SEQUENCE_UP; // this sets the interpolator onto the RTL sequence: rise to target altitude, fly to home position, then land
					}
                    else { // no home waypoint set, land here
						MAVSendTextFrom(MAV_SEVERITY_WARNING, "No home position set, landing here", MAV_COMP_ID_MISSIONPLANNER);
                        target_lat = craft_lat;
                        target_lon = craft_lon;
                        target_alt = craft_alt;
                        interpolator_mode = INTMODE_DOWN;
                    }

                    if(craft_alt > target_alt) target_alt = craft_alt; // if craft is heigher than target, don't bother flying to target altitude first since craft is already above it, just fly to position and land.
                    target_set = 1;
                    waypointGo = 0;
                    free_yaw = 1;
                    break;
                case 7: // go IDLE - velocity kill
                    gps_action = 0;
					MAVSendTextFrom(MAV_SEVERITY_WARNING, "WARNING: Craft going IDLE!", MAV_COMP_ID_MISSIONPLANNER);
                    target_set = 0;
                    break;
            }

            // *** Interpolator
            // work out whether to move or not based on whether pitch/roll is maxed out (horizontal), or vertical distance
            float zdiff;
            float target_speed_north = 0;
            float target_speed_east = 0;
            float target_speed_up = 0;

            if(target_set == 0) { // target not set, set target to craft for hold
                target_lat = craft_lat;
                target_lon = craft_lon;
                target_alt = craft_alt;
                interpolator_mode = INTMODE_OFF;
                free_yaw = 1;
            }
            else if(target_set == 1) { // newly set target, reset interpolator
                target_set = 2;
                interpolator_lat = craft_lat; // reset interpolator
                interpolator_lon = craft_lon;
                interpolator_alt = craft_alt;
            }
            
            switch(interpolator_mode) {
                default:
                case INTMODE_OFF:
                    interpolator_lat = target_lat;
                    interpolator_lon = target_lon;
                    interpolator_alt = target_alt;
                    interpolator_yaw = target_yaw;
                    target_speed_north = 0;
                    target_speed_east = 0;
                    target_speed_up = 0;
                    allow_land = 0;
                    break;

                case INTMODE_SEQUENCE_UP:
                case INTMODE_UP_AND_GO:
                case INTMODE_VERTICAL:
                    zdiff = interpolator_alt - craft_alt;
                    target_speed_north = 0;
                    target_speed_east = 0;
                    target_speed_up = 0;

                    // check that we're not maxed out height
                    if(zdiff  < GPS_MAX_ALTDIFF && zdiff > -GPS_MAX_ALTDIFF) {
                        float vector_Z = target_alt - interpolator_alt; // this is the vertical distance to target

                        // normalise vector
                        float sumsqu = finvSqrt(vector_Z*vector_Z); // this is one over the magnitude of the distance to target

                        // check to see if we'd overshoot the waypoint
                        sumsqu *= (GPS_MAX_SPEED/5.0f); // TODO: replace "5" with actual GPS rate.  Sumsqu now equals one over the number of iterations to reach target
                        if(sumsqu < 1) { // equivalent to if mag > speed, but we're working with 1/mag here, so mess with inequality maths.
                            vector_Z *= sumsqu; // multiplying the distance to target by one over the number of iterations to reach target yields the next step distance
                        }
                        else if(interpolator_mode == INTMODE_SEQUENCE_UP) { // if sumsqu > 1, then mag < speed (i.e. we should have arrived at waypoint now
                            // entering this means that mag < speed, so the next interpolator target should be right on the target.
                            // if that's the case, we've arrived at target, and if we're in SEQUENCE, set interpolator mode to 3D for next time
                            interpolator_mode = INTMODE_SEQUENCE_3D;
                        }
                        else if(interpolator_mode == INTMODE_UP_AND_GO) {
                            if(waypointValid == 1 && waypointCurrent < waypointCount) {
                                gps_action = 4;
                            }
                            else {
                                gps_action = 3;
                            }
                        }

                        interpolator_alt += vector_Z; // interpolator changes by next step distance
                        target_speed_up = vector_Z * 5; // target speed is next step distance times the loop rate (5Hz) Todo: change 5 to a rate

                        float yawdiff = target_yaw - interpolator_yaw; // angle to target
                        if(yawdiff > M_PI) yawdiff -= 2*M_PI;             // wrap around 2PI
                        if(yawdiff < -M_PI) yawdiff += 2*M_PI;

                        interpolator_yaw += yawdiff * sumsqu;
                    }
                    allow_land = 0;
                    break;

                case INTMODE_HORIZONTAL:
                    target_speed_north = 0;
                    target_speed_east = 0;
                    target_speed_up = 0;
                     // check that we're not maxed out on the angles
                    if(ilink_gpsfly.northDemand < GPS_MAX_ANGLE && ilink_gpsfly.northDemand > -GPS_MAX_ANGLE &&
                    ilink_gpsfly.northDemand < GPS_MAX_ANGLE && ilink_gpsfly.northDemand > -GPS_MAX_ANGLE) {

                        float vector_X = latDiff2Meters(target_lat - interpolator_lat);
                        float vector_Y = lonDiff2Meters(target_lon - interpolator_lon, interpolator_lat);

                        // normalise vector
                        float sumsqu = finvSqrt(vector_X*vector_X + vector_Y*vector_Y);

                        // check to see if we'd overshoot the waypoint
                        sumsqu *= GPS_MAX_SPEED/5.0f;
                        if(sumsqu < 1) { // equivalent to if mag > speed, but we're working with 1/mag here, so mess with inequality maths.
                            vector_X *= sumsqu;
                            vector_Y *= sumsqu;
                        }

                        interpolator_lat += meters2LatDiff(vector_X);
                        interpolator_lon += meters2LonDiff(vector_Y, interpolator_lat);
                        target_speed_north = vector_X * 5;
                        target_speed_east = vector_Y * 5;

                        float yawdiff = target_yaw - interpolator_yaw; // angle to target
                        if(yawdiff > M_PI) yawdiff -= 2*M_PI;             // wrap around 2PI
                        if(yawdiff < -M_PI) yawdiff += 2*M_PI;

                        interpolator_yaw += yawdiff * sumsqu;
                    }
                    allow_land = 0;
                    break;

                case INTMODE_SEQUENCE_3D:
                case INTMODE_3D:
                    zdiff = craft_alt - interpolator_alt;

                    target_speed_north = 0;
                    target_speed_east = 0;
                    target_speed_up = 0;
                    // check that we're not maxed out on the angles or height
                    if(ilink_gpsfly.northDemand < GPS_MAX_ANGLE && ilink_gpsfly.northDemand > -GPS_MAX_ANGLE &&
                    ilink_gpsfly.northDemand < GPS_MAX_ANGLE && ilink_gpsfly.northDemand > -GPS_MAX_ANGLE &&
                    zdiff  < GPS_MAX_ALTDIFF && zdiff > -GPS_MAX_ALTDIFF) {
                    
                        float vector_X = latDiff2Meters(target_lat - interpolator_lat);
                        float vector_Y = lonDiff2Meters(target_lon - interpolator_lon, interpolator_lat);
                        float vector_Z = target_alt - interpolator_alt;

                        // normalise vector
                        float sumsqu = finvSqrt(vector_X*vector_X + vector_Y*vector_Y + vector_Z*vector_Z);

                        // check to see if we'd overshoot the waypoint
                        sumsqu *= GPS_MAX_SPEED/5.0f;
                        if(sumsqu < 1) { // equivalent to if mag > speed, but we're working with 1/mag here, so mess with inequality maths.
                            vector_X *= sumsqu;
                            vector_Y *= sumsqu;
                            vector_Z *= sumsqu;
                        }
                        else if(interpolator_mode == INTMODE_SEQUENCE_3D) {
                            // entering this means that mag < speed, so the next interpolator target should be right on the target.
                            // if that's the case, we've arrived at target, and if we're in SEQUENCE, set interpolator mode to land for next time
                            interpolator_mode = INTMODE_SEQUENCE_DOWN;
                        }

                        
                        interpolator_lat += meters2LatDiff(vector_X);
                        interpolator_lon += meters2LonDiff(vector_Y, interpolator_lat);
                        interpolator_alt += vector_Z;
                        target_speed_north = vector_X * 5;
                        target_speed_east = vector_Y * 5;
                        target_speed_up = vector_Z * 5;

                        float yawdiff = target_yaw - interpolator_yaw; // angle to target
                        if(yawdiff > M_PI) yawdiff -= 2*M_PI;             // wrap around 2PI
                        if(yawdiff < -M_PI) yawdiff += 2*M_PI;

                        interpolator_yaw += yawdiff * sumsqu;
                    }
                    allow_land = 0;
                    break;

                case INTMODE_SEQUENCE_DOWN:
                case INTMODE_DOWN:

                    target_speed_north = 0;
                    target_speed_east = 0;
                    target_speed_up = 0;
                    zdiff = craft_alt - interpolator_alt;
                    // check that we're not maxed out height
                    if(zdiff  < GPS_MAX_ALTDIFF && zdiff > -GPS_MAX_ALTDIFF) {
                        interpolator_alt += -GPS_MAX_SPEED/5.0f;
                        target_speed_up = -GPS_MAX_SPEED;
                    }
                    allow_land = 1;

                    // no yaw control in landing mode
                    break;
            }

            // *** PID & Output
            float diff_X = latDiff2Meters(interpolator_lat - craft_lat);
            float diff_Y = lonDiff2Meters(interpolator_lon - craft_lon, craft_lat);
            float diff_Z = (float)(interpolator_alt - craft_alt);

			static float diff_X_i = 0;
			static float diff_Y_i = 0;

            diff_X_i += diff_X;
            diff_Y_i += diff_Y;

            ilink_gpsfly.northDemand = GPS_Kp*diff_X + GPS_Ki*diff_X_i + GPS_Kd*(target_speed_north - gps_nav_velned.velN / 100.0f);
            ilink_gpsfly.eastDemand = GPS_Kp*diff_Y + GPS_Ki*diff_Y_i + GPS_Kd*(target_speed_east - gps_nav_velned.velE / 100.0f);
            ilink_gpsfly.altitudeDemand = interpolator_alt;
            ilink_gpsfly.altitudeDemandVel = target_speed_up;
            ilink_gpsfly.headingDemand = interpolator_yaw;
            ilink_gpsfly.altitude = craft_alt;
            ilink_gpsfly.vAcc = (float)gps_nav_posllh.vAcc / 1000.0f; // we think this is 1 sigma
            ilink_gpsfly.velD = (float)gps_nav_velned.velD / 100.0f;
            ilink_gpsfly.flags = ((free_yaw & 0x1) << 2) | ((allow_land & 0x1) << 1) | 1;

            XBeeInhibit();
            ILinkSendMessage(ID_ILINK_GPSFLY, (unsigned short *) & ilink_gpsfly, sizeof(ilink_gpsfly)/2-1);
            ILinkPoll(ID_ILINK_GPSREQ);
            XBeeAllow();

            // *** Check targeting rules for waypoints
            if(waypointGo == 1 && waypointValid == 1 && waypointCurrent < waypointCount && waypoint[waypointCurrent].command != MAV_CMD_NAV_RETURN_TO_LAUNCH) {
                float radius = waypoint[waypointCurrent].param2; // param2 is radius in QGroumdcontrol 1.0.1
                if(radius < GPS_MIN_RADIUS) radius = GPS_MIN_RADIUS; // minimum radis

                //float diff_X2 = diff_X; // for orbit phase calculation
                //float diff_Y2 = diff_Y;

                // assume cube of sides 2*radius rather than a sphere for target detection
                if(diff_X < 0) diff_X = -diff_X;
                if(diff_Y < 0) diff_Y = -diff_Y;
                if(diff_Z < 0) diff_Z = -diff_Z;

                if(waypointReached == 0 && diff_X < radius && diff_Y < radius && diff_Z < radius) { // target reached
                    waypointReached = 1;
                    waypointLoiterTimer = 0;

                    // report waypoint reached to GCS
                    mavlink_mission_item_reached.seq = waypointCurrent;
                    mavlink_msg_mission_item_reached_encode(mavlinkID, MAV_COMP_ID_MISSIONPLANNER, &mavlink_tx_msg, &mavlink_mission_item_reached);
                    mavlink_message_len = mavlink_msg_to_send_buffer(mavlink_message_buf, &mavlink_tx_msg);
                    XBeeInhibit(); // XBee input needs to be inhibited before transmitting as some incomming messages cause UART responses which could disrupt XBeeWriteCoordinator if it is interrupted.
                    XBeeWriteCoordinator(mavlink_message_buf, mavlink_message_len);
                    XBeeAllow();

                    // waypointPhase = fatan2(-diff_Y2, -diff_X2);
                }

                if(waypointReached == 1) {
                    switch(waypoint[waypointCurrent].command) {
                        case MAV_CMD_NAV_LOITER_UNLIM:
                        default:
                            // do nothing (i.e. get stuck here until user requests to resume)
                            break;
                        case MAV_CMD_NAV_LOITER_TURNS:
                            // TODO: deal with loiter radius/turns
                            break;
                        case MAV_CMD_NAV_LOITER_TIME:
                        case MAV_CMD_NAV_WAYPOINT:
                            if(waypointLoiterTimer >= waypoint[waypointCurrent].param1) { // Param1 in this case is wait time (in milliseconds, waypointLoiterTimer incremented by SysTick)
                                if(waypointCurrent < waypointCount) {
                                    waypointCurrent ++;
                                    gps_action = 4; // loads up new waypoint
                                    waypointReached = 0;
                                    waypointLoiterTimer = 0;
                                }
                            }
                            break;
                        case MAV_CMD_NAV_LAND:
                            gps_action = 5;
                            break;
                    }
                }
            }
            else {
                free_yaw = 1;
            }
        }
        else {
            ilink_gpsfly.flags = 0; // GPS not valid


            XBeeInhibit();
            ILinkSendMessage(ID_ILINK_GPSFLY, (unsigned short *) & ilink_gpsfly, sizeof(ilink_gpsfly)/2-1);
            XBeeAllow();

        }
    }
}

/*!
\brief Stores the home waypoint
*/
void gps_set_home(double lat, double lon, double alt) {
    home_lat = lat;
    home_lon = lon;
    home_alt = alt;
    home_valid = 1;

    mavlink_gps_global_origin.latitude = lat * 10000000;
    mavlink_gps_global_origin.longitude = lon * 10000000;
    mavlink_gps_global_origin.altitude = alt * 1000;
    mavlink_msg_gps_global_origin_encode(mavlinkID, MAV_COMP_ID_MISSIONPLANNER, &mavlink_tx_msg, &mavlink_gps_global_origin);
    mavlink_message_len = mavlink_msg_to_send_buffer(mavlink_message_buf, &mavlink_tx_msg);
    XBeeInhibit();
    XBeeWriteCoordinator(mavlink_message_buf, mavlink_message_len);
    XBeeAllow();

    MAVSendTextFrom(MAV_SEVERITY_INFO, "Home position set", MAV_COMP_ID_MISSIONPLANNER);
}

/*!
\brief Converts a lattitide difference to meters

\param latdiff a difference in latitude
\retval a distance in meters that corresponds to the input latitude difference
*/
float latDiff2Meters(double latdiff) {
    return (double)latdiff * (double)111194.92664455873734580834; // 111194.92664455873734580834f is radius of earth and deg-rad conversion: 6371000*PI()/180
}

/*!
\brief Converts a longitude difference to meters (requires a reference latitude)

\param londiff a difference in longitude
\param ref_lat a the reference latitude at which the conversion takes place
\retval a distance in meters that corresponds to the input longitude difference
*/
float lonDiff2Meters(double londiff, double ref_lat) {
    return (double)londiff * (double)111194.92664455873734580834 * fcos((float)((double)ref_lat*(double)0.01745329251994329577)); // 0.01745329251994329577f is deg-rad conversion PI()/180
}

/*!
\brief Converts meters to a lattitude difference

\param meters a distance in meters
\retval a latitude difference that corresponds to the input distance in meters
*/
double meters2LatDiff(float meters) {
    return (double)meters / (double)111194.92664455873734580834; // 111194.92664455873734580834f is radius of earth and deg-rad conversion: 6371000*PI()/180
}

/*!
\brief Converts meters to a longitude difference (requires a reference latitude)

\param meters a distance in meters
\param ref_lat a the reference latitude at which the conversion takes place
\retval a longitude difference that corresponds to the input distance in meters
*/
double meters2LonDiff(float meters, double ref_lat) {
    return (double)meters / ((double)111194.92664455873734580834 * fcos((float)((double)ref_lat*(double)0.01745329251994329577))); // 0.01745329251994329577f is deg-rad conversion PI()/180
}

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
