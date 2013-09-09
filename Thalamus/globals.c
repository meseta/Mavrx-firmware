#include "all.h"

unsigned char flashPLED=0, flashVLED=0, flashRLED=0;

float thetaAngle, phiAngle, psiAngle, psiAngleinit;

///////////////////////////////////////// GLOBAL VARIABLE STRUCTURES /////////////////////

userStruct user;
attitude_demand_body_struct attitude_demand_body;


unsigned char armed=0;
unsigned char state=STATE_DISARMED;
/////////////////////////////////// GLOBAL VARIABLES /////////////////////////////////
// Timers and counters
unsigned int sysMS=0;
unsigned long long sysUS=0;
unsigned short RxWatchdog=0;
unsigned short UltraWatchdog=0;
unsigned short slowSoftscale=0;

// LEDs

// Quaternion and Rotation Matrix
float q1=1, q2=0, q3=0, q4=0;
float M1=1, M2=0, M3=0, M4=0, M5=1, M6=0, M7=0, M8=0, M9=1;
float RM1=1, RM2=0, RM3=0, RM4=0, RM5=1, RM6=0, RM7=0, RM8=0, RM9=1;



// Button
unsigned int PRGBlankTimer; // Blanking time for button pushes
unsigned int PRGTimer; // Timer for button pushes, continuously increments as the button is held
unsigned int PRGPushTime; // Contains the time that a button was pushed for, populated after button is released

unsigned char airborne; //boolean


unsigned char thal_throt_cont;

unsigned char thal_motor_off;


//Altitude PID states
float GPS_KerrI;
float ULT_KerrI;
float targetZ_ult;
float alt_tkoff;
float oldUltra;

float ROLL_SPL_set;
float PITCH_SPL_set;
float YAW_SPL_set;

// Control/Output
unsigned char got_setpoint=0; //bool
float pitchcorrectionav=0, rollcorrectionav=0, yawcorrectionav=0;
float motorN=0, motorE=0, motorS=0, motorW=0;
float motorNav=0, motorEav=0, motorSav=0, motorWav=0;
float tempN=0;
float tempE=0;
float tempS=0;
float tempW=0;


unsigned char gps_valid = 0;

unsigned short rcInput[7];
unsigned int rxLoss=1000; // initialise to a high number to start off assuming RX is lost
unsigned int rxFirst=0;
signed short yawtrim=0;
signed short throttletrim=0;
float throttle=0;
float throttle_angle=0;
int hold_thro_off=1;// We start with throttle hold on in case the user has forgotten to lower their throttle stick
unsigned char auxState=0, flapState=0, rateState=0, throState=0, aileState=0, elevState=0, ruddState=0;
unsigned int flapswitch;
float pitchDemandSpin;
float rollDemandSpin;
float pitchDemandSpinold;
float rollDemandSpinold;
float flpswitch;


altStruct alt={0};
float ultra;
unsigned int ultraLoss=1000; // initialise to a high number to start off assuming ultra is lost

float batteryVoltage;

threeAxisSensorStructGyro Gyro={{0}};
threeAxisSensorStructAccel Accel={{0}};
threeAxisSensorStructMag Mag={{0}};