/*!
\file Thalamus/globals.c
\brief Contains all the global values
*/

#include "all.h"

// Control/PID
unsigned char got_setpoint=0; 			/*!< Boolean for indication whether to store the first ultrasound altitude when re-entering the confidence range */

unsigned char flashPLED=0;				/*!< Boolean to enable PLED flashing */
unsigned char flashVLED=0;				/*!< Boolean to enable VLED flashing */
unsigned char flashRLED=0;				/*!< Boolean to enable RLED flashing */

unsigned char thal_throt_cont;  		/*!< Boolean for Thalamus allowed to control throttle */
unsigned char thal_motor_off;  			/*!< Boolean for Thalamus allowed to turn motors off */

float GPS_KerrI;						/*!< GPS altitude integral value for PID */
float ULT_KerrI;						/*!< Ultrasound altitude integral value for PID */
float targetZ_ult;						/*!< Ultrasound target height */
float alt_tkoff;						/*!< Takeoff altitude */

float ROLL_SPL_set;						/*!< Temporary value for adjusting roll rate limits */
float PITCH_SPL_set;					/*!< Temporary value for adjusting pitch rate limits */
float YAW_SPL_set;						/*!< Temporary value for adjusting yaw rate limits */

// Filters/AHRS
float thetaAngle;						/*!< Pitch angle */
float phiAngle;							/*!< Roll angle */
float psiAngle;							/*!< Yaw angle */
float psiAngleinit;						/*!< Initial yaw angle used for simplicity mode*/

userStruct user;						/*!< User demand data */
attitude_demand_body_struct attitude_demand_body;	/*!< Body demand data */
float q1=1;								/*!< Quaternion element 1 */
float q2=0;								/*!< Quaternion element 2 */
float q3=0;								/*!< Quaternion element 3 */
float q4=0;								/*!< Quaternion element 4 */

float M1=1;								/*!< DCM element 1,1 */
float M2=0;								/*!< DCM element 1,2 */
float M3=0;								/*!< DCM element 1,3 */
float M4=0;								/*!< DCM element 2,1 */
float M5=1;								/*!< DCM element 2,2 */
float M6=0;								/*!< DCM element 2,3 */
float M7=0;								/*!< DCM element 3,1 */
float M8=0;								/*!< DCM element 3,2 */
float M9=1;								/*!< DCM element 3,3 */

// Inputs
unsigned int PRGBlankTimer;				/*!< Blanking time for button pushes */
unsigned int PRGTimer;					/*!< Timer for button pushes, continuously increments as the button is held */
unsigned int PRGPushTime;				/*!< Contains the time that a button was pushed for, populated after button is released */

unsigned short rcInput[7];				/*!< Contains RX input */
unsigned int rxLoss=1000;   			/*!< Increments if RX is not available */ // initialise to a high number to start off assuming RX is lost
unsigned int rxFirst=0;					/*!< Counts the first few RX data, used to ignore invalid initial values */
signed short yawtrim=0;					/*!< Initial yaw input trim */
signed short throttletrim=0;			/*!< Initial trottle trim */
float throttle=0;						/*!< Throttle value */
float throttle_angle=0;					/*!< Throttle adjustment when pitch/rolled */
unsigned char hold_thro_off=1;			/*!< Boolean for holding throttle off */ // We start with throttle hold on in case the user has forgotten to lower their throttle stick
unsigned char auxState=0;				/*!< RX input aux switch state */
unsigned char flapState=0;				/*!< RX input flap switch (button) state */
unsigned char rateState=0;				/*!< RX input rate switch state */
unsigned char throState=0;				/*!< RX input thro switch state */
unsigned char aileState=0;				/*!< RX input aile switch state */
unsigned char elevState=0;				/*!< RX input elev switch state */
unsigned char ruddState=0;				/*!< RX input rudd switch state */
unsigned int flapswitch;				/*!< RX input flap switch toggle */

float pitchDemandSpin;					/*!< Pitch demand rotated to body frame */
float rollDemandSpin;					/*!< Roll demand rotated to body frame */
float pitchDemandSpinold;				/*!< Pitch demand rotated to body frame old value */
float rollDemandSpinold;				/*!< Roll demand rotated to body frame old value */

unsigned char gps_valid = 0;			/*!< Boolean for whether the GPS data is valid */

float batteryVoltage;					/*!< Contains battery voltage in millivolts */
float ultra;							/*!< Contains the ultrasound reading */
float oldUltra;							/*!< Contains the previous ultrasound reading */
unsigned int ultraLoss=1000; 			/*!< Increments if ultrasound is not available */ // initialise to a high number to start off assuming ultra is lost


altStruct alt={0};						/*!< Struct for altitude data */
threeAxisSensorStructGyro Gyro={{0}};	/*!< Struct for gyro data */
threeAxisSensorStructAccel Accel={{0}};	/*!< Struct for accel data */
threeAxisSensorStructMag Mag={{0}};		/*!< Struct for magneto data */

// State
unsigned char armed=0;					/*!< State variable boolean for armed/disarmed */
unsigned char state=STATE_DISARMED;		/*!< State variable for state machine */
unsigned char airborne=0;				/*!< State variable for being ariborne or not */