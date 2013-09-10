#ifndef __FILTER_H__
#define __FILTER_H__


// Filters/AHRS
extern float thetaAngle, phiAngle, psiAngle, psiAngleinit;

/*! \brief User demand struct */
typedef struct {
	float roll; 	/*!< user roll demand */
	float pitch;	/*!< user pitch demand */
	float yaw;		/*!< user yaw demand */
	float throttle;	/*!< user throttle demand */
} userStruct;
extern userStruct user;

/*! \brief Body demand demand struct */
typedef struct {
	float roll;		/*!< body roll demand */
	float pitch;	/*!< pitch roll demand */
	float yaw;		/*!< yaw roll demand */
} attitude_demand_body_struct;
extern attitude_demand_body_struct attitude_demand_body;

extern float q1, q2, q3, q4;
extern float M1, M2, M3, M4, M5, M6, M7, M8, M9;


void filter_gps_baro(void);
void a_h_r_s(void);

#endif