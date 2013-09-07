#include "globals.h"

unsigned char flashPLED, flashVLED, flashRLED;

float thetaAngle, phiAngle, psiAngle, psiAngleinit;

///////////////////////////////////////// GLOBAL VARIABLE STRUCTURES /////////////////////

userStruct user;
attitude_demand_body_struct attitude_demand_body;


unsigned char armed;
unsigned char state;
/////////////////////////////////// GLOBAL VARIABLES /////////////////////////////////
// TODO: Why don't we just set all the variable here? Some of them are being set here, and some in the setup function.
// Timers and counters
unsigned int sysMS;
unsigned long long sysUS;
unsigned short RxWatchdog;
unsigned short UltraWatchdog;
unsigned short slowSoftscale;

// LEDs

// Quaternion and Rotation Matrix
float q1, q2, q3, q4;
float M1, M2, M3, M4, M5, M6, M7, M8, M9;
float RM1, RM2, RM3, RM4, RM5, RM6, RM7, RM8, RM9;



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
