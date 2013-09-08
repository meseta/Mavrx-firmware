#include "params.h"

unsigned int paramCount;
unsigned int paramSendCount;
unsigned char paramSendSingle;

struct paramStorage_struct paramStorage[] = {
	//ADDPARAM(DRIFT_AKp, 0.2f),
	[0] = {"DRIFT_AKp",		   0.2f},
	{"DRIFT_MKp",	  		0.2f}, 
	{"LPF_ULTRA",	   0.05f},  
	{"YAW_SEN",	 0.0001f},	 
	{"PITCH_SEN",	0.0022f},	
	{"ROLL_SEN",	 0.0022f},  
	{"YAW_DZN",	  0.001f},	
	{"PITCH_Kp",	  400.0f},	 
	{"PITCH_Ki",		2.0f},	
	{"PITCH_Kd",	  100.0f},	 
	{"PITCH_Kdd",	1500.0f},
	{"PITCH_Bst",	 0.0f},	  
	{"PITCH_De",	  0.999f},
	{"ROLL_Kp",	   400.0f},	   
	{"ROLL_Ki",		 2.0f},	  
	{"ROLL_Kd",	   100.0f},		
	{"ROLL_Kdd",	 1500.0f},	 		
	{"ROLL_Bst",	   0.00f},					
	{"ROLL_De",	   0.999f},
	{"YAW_Kp",		1000.0f},	 
	{"YAW_Kd",		250.0f},  
	{"YAW_Bst",		0.00f},   
	// Mode
	{"UNUSED",	   0},  //
	//Limits
	{"LIM_ANGLE",	   0.35f},  // Roll and Pitch Angle Limit in Radians
	{"LIM_ALT",		 1000.0f},  // Altitude Limit in mm when in Ultrasound Mode
	// Magneto Correction
	{"CAL_MAGN1",	 0.001756f},	  
	{"CAL_MAGN2",   0.00008370f},		
	{"CAL_MAGN3",   0.00005155f},	 	
	{"CAL_MAGN5",	 0.001964f},			
	{"CAL_MAGN6",   0.00002218f},	 
	{"CAL_MAGN9",	 0.001768f},	 
	{"CAL_MAGM1",	   0.0f},	 
	{"CAL_MAGM2",		0.0f},	 	
	{"CAL_MAGM3",		0.0f}, 
	// Ultrasound
	{"ULTRA_Kp",		0.05f},
	{"ULTRA_Kd",		0.1f},
	{"ULTRA_Ki",		0.00001f},
	{"ULTRA_De",	  	0.9999f},
	{"ULTRA_TKOFF",   	200.0f}, 
	{"ULTRA_LND",   	150.0f}, 
	{"CAL_GYROX",   0.0f},
	{"CAL_GYROY",   0.0f},
	{"CAL_GYROZ",   0.0f},
	{"DETUNE",			0.2f},
	{"LIM_RATE",			100.0f}, 
	{"LIM_ULTRA",			4.0f},
	{"ULTRA_DRMP",	 3.0f}, 
	{"ULTRA_DTCT",	 6.0f},
	{"LIM_THROT", 		0.3f},
	{"ULTRA_OVDEC",		0.01f},
	{"ULTRA_DEAD",		100},
	{"ULTRA_OVTH",		40},
	{"CAL_AUTO", 1.0f},
	
	{"LPF_OUT",	   0.6f},
	{"BAT_LOW",		 11000.0f},
	{"BAT_CRIT",		10000.0f},
	
	{"ULTRA_OFFSET",		 350},
	
	{"ROLL_SPL",		 0.02},
	{"PITCH_SPL",		 0.02},
	
	// TODO: Tune Yaw integral
	{"YAW_Ki",		 0.0},
	{"YAW_De",		 1.0},

	{"Filt_GPS_K",		 1.0},

	{"LPF_BARO",   0.05},

	{"GPS_ALTKp", 5.0f},
    {"GPS_ALTKi", 0.0001f},
    {"GPS_ALTDe", 1.0f},
    {"GPS_ALTKd", 20.0f},

    {"Filt_baroK",		 0.0},
	
	{"YAW_SPL",		 0.04},
    
	{"ORI",		 0.00},
	
};