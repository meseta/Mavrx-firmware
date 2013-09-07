#ifndef __T_MAVLINK_H__
#define __T_MAVLINK_H__

#include "mavlink/common/mavlink.h"

#define MAVLINK_SENSOR_GYRO     (0x1UL << 0)
#define MAVLINK_SENSOR_ACCEL    (0x1UL << 1)
#define MAVLINK_SENSOR_MAGNETO  (0x1UL << 2)
#define MAVLINK_SENSOR_BARO     (0x1UL << 3)
#define MAVLINK_SENSOR_DIFPRESS (0x1UL << 4)
#define MAVLINK_SENSOR_GPS      (0x1UL << 5)
#define MAVLINK_SENSOR_OPFLOW   (0x1UL << 6)
#define MAVLINK_SENSOR_VISION   (0x1UL << 7)
#define MAVLINK_SENSOR_LASER    (0x1UL << 8)
#define MAVLINK_SENSOR_EXTGND   (0x1UL << 9)
#define MAVLINK_CONTROL_ANGLERATE   (0x1UL << 10)
#define MAVLINK_CONTROL_ATTITUDE    (0x1UL << 11)
#define MAVLINK_CONTROL_YAW     (0x1UL << 12)
#define MAVLINK_CONTROL_Z       (0x1UL << 13)
#define MAVLINK_CONTROL_XY      (0x1UL << 14)
#define MAVLINK_CONTROL_MOTOR   (0x1UL << 15)

#endif