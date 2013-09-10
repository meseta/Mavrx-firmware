// ****************************************************************************
// *** Copyright (c) 2011, Universal Air Ltd. All rights reserved.
// *** Source and binaries are released under the BSD 3-Clause license
// *** See readme_forebrain.txt files for the text of the license
// ****************************************************************************
 
#include "lpc1347.h"
#include "thal.h"
#include "config.h"
#include <stdint.h>
#include <string.h>
#include <math.h>

#if defined (__cplusplus)
extern "C" {
#endif


// ****************************************************************************
// *** Misc Functions
// ****************************************************************************

// *** In-application programmingfunctions
static const FUNCIAP FUNCIAPEntry = (FUNCIAP)0x1fff1ff1;

// *** Reprogram function
/*void Reprogram(void) {
    unsigned int command[5], result[4];
    command[0] = 57;					        // 57 is code to reinvoke ISP
    LPC_SYSCON->SYSAHBCLKDIV = 1;			    // make sure clock divider is 1
    LPC_SYSCON->SYSAHBCLKCTRL |= 0x14440;	    // turn on USB clock, Timer 32B1, GPIO, and IO blocks 
    
    __set_MSP(*((unsigned int *)0x1FFF0000));	// set pointer to bootloader ROM location

    //__set_Stack(0x10002000);
    FUNCIAPEntry(command, result);
}*/

// *** Read chip's part identifier
unsigned int ReadPID(void) {
    unsigned int command[5], result[4];
    command[0] = 54;					        // 54 is code to read PID
    FUNCIAPEntry(command, result);
    return result[0];
}

// *** Read chip's unique identifier
void ReadUID(unsigned int * uid) {
    unsigned int command[5];
    command[0] = 58;					        // 58 is code to read UID
    FUNCIAPEntry(command, uid);
}

// *** Provide a simple hash of the unique identifier to avoid user having to deal with 128 bits
// there is a chace of collision reducing 128 to 32bits, but much easier to handle
// This is a modified Berstein Hash
unsigned int ReadUIDHash(void) {
    unsigned int command[5], result[4], hash;
    command[0] = 58;					        // 58 is code to read UID
    FUNCIAPEntry(command, result);
    
    hash = result[0];
    hash *= 33;
    hash ^= result[1];
    hash *= 33;
    hash ^= result[2];
    hash *= 33;
    hash ^= result[3];
    
    return hash;
}

void EEPROMWriteByte(unsigned int address, unsigned char data) {
    unsigned char datas[1];
    EEPROMWrite(address, datas, 1);
}

unsigned char EEPROMReadByte(unsigned int address){
    unsigned char datas[1];
    EEPROMRead(address, datas, 1);
    return datas[0];
}

void EEPROMWrite(unsigned int address, unsigned char * data, unsigned int length) {
    unsigned int command[5], result[4];
    command[0] = 61;					    // 61 is code to write to eeprom
    command[1] = address+64;				// address in EEPROM, avoiding reserved 64 bytes
    command[2] = (unsigned int) data;		// address in RAM of data
    command[3] = length;					// number of bytes read
    
    if((LPC_SYSCON->MAINCLKSEL & 0x03) == 0x03) {
        command[4] = 72000;					// clock speed in kHz
    }
    else {
        command[4] = 12000;					// clock speed in kHz
    }
    
    FUNCIAPEntry(command, result);
}

void EEPROMRead(unsigned int address, unsigned char * data, unsigned int length) {
    unsigned int command[5], result[4];
    command[0] = 62;					    // 61 is code to read from eeprom
    command[1] = address+64;				// address in EEPROM, avoiding reserved 64 bytes
    command[2] = (unsigned int) data;		// address in RAM of data
    command[3] = length;					// number of bytes read
    
    if((LPC_SYSCON->MAINCLKSEL & 0x03) == 0x03) {
        command[4] = 72000;					// clock speed in kHz
    }
    else {
        command[4] = 12000;					// clock speed in kHz
    }
    FUNCIAPEntry(command, result);
}

// *** Reset function
void Reset(void) {
  __DSB();  // Wait for write ops to clear
  SCB->AIRCR  = (0x5FA << 16) | (SCB->AIRCR & (0x7 << 8)) | (0x1 << 2);
  __DSB();  // Wait for write ops to clear
  while(1); // Wait for reset
}

// *** Fast trigonometry approximation functions
float finvSqrt(float x) {
    union {
        float f;
        int i;
    } tmp;
    tmp.f = x;
    tmp.i = 0x5f3759df - (tmp.i >> 1);
    float y = tmp.f;
    return y * (1.5f - 0.5f * x * y * y);
}

float fatan2(float y, float x) {
	if (x == 0.0f) {
		if (y > 0.0f) return M_PI_2;
		if (y == 0.0f) return 0.0f;
		return -M_PI_2;
	}
	float atan;
	float z = y/x;
	if (fabsf(z) < 1.0f) {
		atan = z/(1.0f + 0.28f*z*z);
		if (x < 0.0f) {
			if (y < 0.0f) return atan - M_PI;
			return atan + M_PI;
		}
	}
	else {
		atan = M_PI_2 - z/(z*z + 0.28f);
		if (y < 0.0f) return atan - M_PI;
	}
	return atan;
}

float fasin(float x) {
    float temp, arcsin, xabs;
    xabs = fabsf(x);
    temp = M_PI_2 - (1.5707288f + (-0.2121144f + (0.0742610f - 0.0187293f*xabs)*xabs)*xabs)/finvSqrt(1-xabs);
    arcsin = copysignf(temp, x);
    return arcsin;
}

float fsin(float x) {
    const float B = 4/M_PI;
    const float C = -4/(M_PI*M_PI);

    while(x > M_PI) x-= M_TWOPI;
    while(x < -M_PI) x+= M_TWOPI;
    
    float y = B * x + C * x * fabsf(x);
    
    //  const float Q = 0.775;
        const float P = 0.225;

        y = P * (y * fabsf(y) - y) + y;   // Q * y + P * y * abs(y)
    return y;
}

float fcos(float x) {
    return fsin(x+M_PI_2);
}

// *** Random number functions

#if RAND_MERSENNE
    volatile unsigned int FUNCMersenne[MERSENNE_N];
    volatile unsigned int FUNCMersenneIndex;
    volatile unsigned int FUNCMersenneMag[2]={0x0UL, 0x9908b0dfUL};

    void RandomSeed(unsigned int seed) {
        unsigned int i;
        FUNCMersenne[0] = seed;
        for (i=1; i<623; i++) {
            FUNCMersenne[i] = (1812433253UL * (FUNCMersenne[i-1] ^ (FUNCMersenne[i-1] >> 30)) + i);
        }
        FUNCMersenneIndex = 0;
    }

    unsigned int Random(void) {
        unsigned int i, y;
        
        if (FUNCMersenneIndex > MERSENNE_N-1) {
            for (i=0;i<MERSENNE_N-MERSENNE_M;i++) {
                y = (FUNCMersenne[i]&0x80000000UL)|(FUNCMersenne[i+1]&0x7fffffffUL);
                FUNCMersenne[i] = FUNCMersenne[i+MERSENNE_M] ^ (y >> 1) ^ FUNCMersenneMag[y & 0x1UL];
            }
            for (;i<MERSENNE_N-1;i++) {
                y = (FUNCMersenne[i]&0x80000000UL)|(FUNCMersenne[i+1]&0x7fffffffUL);
                FUNCMersenne[i] = FUNCMersenne[i+(MERSENNE_M-MERSENNE_N)] ^ (y >> 1) ^ FUNCMersenneMag[y & 0x1UL];
            }
            y = (FUNCMersenne[MERSENNE_N-1]&0x80000000UL)|(FUNCMersenne[0]&0x7fffffffUL);
            FUNCMersenne[MERSENNE_N-1] = FUNCMersenne[MERSENNE_M-1] ^ (y >> 1) ^ FUNCMersenneMag[y & 0x1UL];

            FUNCMersenneIndex = 0;
        }

        // tempering
        y = FUNCMersenne[FUNCMersenneIndex];
        y ^= (y >> 11);
        y ^= (y << 7) & 2636928640UL;
        y ^= (y << 15) & 4022730752UL;
        y ^= (y >> 18);

        FUNCMersenneIndex++;
        return y;
    }
#else
    volatile unsigned int FUNCRandomNumber;
#endif

// *** Set Code Read Protection
__attribute__ ((section(".crp"))) const unsigned int CRP_WORD = CRP;


// ****************************************************************************
// *** Clock Functions
// ****************************************************************************

/*__attribute__ ((section(".after_vectors")))*/ void ClockModeXTAL(void) {
    unsigned int i;
    
    LPC_SYSCON->PDRUNCFG &= ~(0x1UL << 5);			// Power up system oscillator
    LPC_SYSCON->SYSOSCCTRL = 0;						// System oscillator setup - not bypass, 1-20MHz range
    for(i = 0; i < 200; i++) __NOP();   			// Brief delay
    LPC_SYSCON->SYSPLLCLKSEL = 0x01;   				// Select system oscillator as PLL source
    
    LPC_SYSCON->SYSPLLCTRL = 0x25;					// Select PLL divider to 6 (12MHz - 72MHz)
    LPC_SYSCON->PDRUNCFG &= ~(0x1UL << 7);          // Power up PLL
    while(!(LPC_SYSCON->SYSPLLSTAT & 0x01));	    // Wait for PLL lock
    
    LPC_SYSCON->MAINCLKSEL = 0x03;     				// Select PLL as main clock source

    //LPC_SYSCON->PDRUNCFG |= (0x1UL <<0) | (0x1UL <<1);        // Shut down IRC
}

void ClockModeIRC72(void) {
    unsigned int i;
    
    LPC_SYSCON->PDRUNCFG &= ~((0x1UL <<0) | (0x1UL <<1));     // Power up IRC oscillator
    LPC_SYSCON->SYSOSCCTRL = 0;						// System oscillator setup - not bypass, 1-20MHz range
    for(i = 0; i < 200; i++) __NOP();   			// Brief delay
    LPC_SYSCON->SYSPLLCLKSEL = 0x00;   				// Select system oscillator as PLL source
    
    LPC_SYSCON->SYSPLLCTRL = 0x25;					// Select PLL divider to 6 (12MHz - 72MHz)
    LPC_SYSCON->PDRUNCFG &= ~(0x1UL << 7);          // Power up PLL
    while(!(LPC_SYSCON->SYSPLLSTAT & 0x01));	    // Wait for PLL lock
    
    LPC_SYSCON->MAINCLKSEL = 0x03;     				// Select PLL as main clock source

    LPC_SYSCON->PDRUNCFG |= (0x1UL <<5);            // Shut down system oscillator
}

void ClockModeIRC12(void) {
    unsigned int i;
    
    LPC_SYSCON->PDRUNCFG &= ~((0x1UL <<0) | (0x1UL <<1));     // Power up IRC oscillator
    for(i = 0; i < 200; i++);						// Brief delay

    LPC_SYSCON->MAINCLKSEL = 0x00;     				// Select IRC as main clock source

    LPC_SYSCON->PDRUNCFG |= ((0x1UL <<5) | (0x1UL <<7));    // Power down system oscillator and PLL
}

// *** Delay functions
void Delay(unsigned int milliseconds) {
    // detect if Systick is enabled (and turned on) and use the relevant delay function
    #if SYSTICK_EN
        if(SysTick->CTRL & 0x1) SysTickDelay(milliseconds);
        else WaitDelay(milliseconds);
    #else
        WaitDelay(milliseconds);
    #endif
}

void WaitDelay(unsigned int milliseconds) {
	volatile unsigned int i, j;
    // Some empty loops depending on the clock setting
    if((LPC_SYSCON->MAINCLKSEL & 0x03) == 0x03) {
        // assume 72MHz operation
        for(j=0; j<milliseconds; j++) {
            for(i=0; i<6000; i++);
        }
    }
    else {
        // assume 12MHz operation
        for(j=0; j<milliseconds; j++) {
            for(i=0; i<1000; i++);
        }
    }
}

#if SYSTICK_EN
    volatile unsigned int FUNCSysTicks, FUNCTimeout;

    // *** System tick timer functions
	void SysTickInit(void) {	// this function is run automaticlly by the startup script if SysTick is enabled
		// configure the timer based on oscillator speed, these are already defined in core_cm3
        if((LPC_SYSCON->MAINCLKSEL & 0x03) == 0x03) {
            // assume 72MHz operation
            SysTick->LOAD  = ((72 * SYSTICK_US) & 0xffffff) - 1;
            SysTick->VAL   = 0;
            SysTick->CTRL  = 0x07;
            IRQPriority(SysTick_IRQn, SYSTICK_PRIORITY);
        }
        else {
            // assume 12MHz operation
            SysTick->LOAD  = ((12 * SYSTICK_US) & 0xffffff) - 1;
            SysTick->VAL   = 0;
            SysTick->CTRL  = 0x07;
            IRQPriority(SysTick_IRQn, SYSTICK_PRIORITY);
        }
		FUNCSysTicks = 0;
	}

	void SysTickStop(void) {
		SysTick->CTRL = 0x00; // Stop the SysTick timer
	}

	void SysTick_Handler(void) {
		FUNCSysTicks++; // increment tick timer variable for use with Delays
        if(FUNCTimeout>0) FUNCTimeout--;
		if(SysTickInterrupt) SysTickInterrupt();  // Run user-supplied interrupt function if available
        if(SDTick) SDTick();
	}

    // Systick-based delay
	void SysTickDelay(unsigned int milliseconds) {
		FUNCSysTicks = 0; // zero tick timer variable
		while(FUNCSysTicks < (milliseconds*1000)/SYSTICK_US); // wait until tick timer variable reaches required number
	}
    
	void SysTickUDelay(unsigned int microseconds) {
		FUNCSysTicks = 0; // zero tick timer variable
		while(FUNCSysTicks < microseconds/SYSTICK_US); // wait until tick timer variable reaches required number
	}
#endif


// *** Watchdog timer initialise
void WDTInit(unsigned int milliseconds) {
    LPC_SYSCON->SYSAHBCLKCTRL |= (0x1UL << 15);		// Enable clock to WDT block
    LPC_SYSCON->PDRUNCFG &= ~(0x1UL << 6);	// Power up the WDT oscillator
    
    #if WDT_CLK == 0
        LPC_SYSCON->WDTOSCCTRL = 0x020;		// Default rate of divider of 2, clock rate of 0.5MHz, WDT divides by 4 again = 62.5kHz clock
        LPC_WWDT->CLKSEL = 0x01;		// Select the watchdog oscillator as input
    #else
        LPC_WWDT->CLKSEL = 0x00;		// Select IRC as input
    #endif
    
    #if WDT_MODE
        // in interrupt mode 1 and 2, enable interrupts
        IRQClear(WDT_IRQn);
        IRQEnable(WDT_IRQn);
        IRQPriority(WDT_IRQn, WDT_PRIORITY);
        LPC_WWDT->MOD = 0x1;
    #else
        // in reset mode, enable reset
        LPC_WWDT->MOD = 0x3;
    #endif
    
    // Set up WDT period with minimum and maximum values
    #if WDT_CLK == 0
    if(milliseconds < 1) {
        LPC_WWDT->TC = 1;
    }
    else if(milliseconds > 268435) { // maximum 268.435 seconds
        LPC_WWDT->TC = 268435;
    }
    else {
        LPC_WWDT->TC = 62.5 * milliseconds;
    }
    #else
    if(milliseconds < 1) {
        LPC_WWDT->TC = 1;
    }
    else if(milliseconds > 5592) { // maximum 5.592 seconds
        LPC_WWDT->TC = 5592;
    }
    else {
        LPC_WWDT->TC = 3000 * milliseconds;
    }
    #endif
    
    WDTFeed(); // feed WDT
}

// *** Watchdog timer stop
void WDTStop(void) {
    LPC_WWDT->MOD &= ~0x1;
    IRQDisable(WDT_IRQn);
}

// *** Watchdog timer interrupt
void WDT_IRQHandler(void) {
    #if WHO_AM_I == I_AM_THALAMUS && RX_EN
    RXWDTInterrupt();
    #endif
    if(WDTInterrupt) WDTInterrupt();
    #if WDT_MODE == 2
        LPC_WWDT->MOD &= ~0x04;
        LPC_WWDT->MOD = 0x1;
        WDTFeed(); // feed WDT
    #endif
}

/*
// *** Clockout confiig
void ClockOut(unsigned char mode, unsigned char divider) {
    if(mode == USBSOF) {
        // Set up USB start-of-frame output (1ms)
        LPC_IOCON->PIO0_1 = 0x53;
    }
    else if((mode & 0x3) == 0) {
        // Turn clockout off
        LPC_SYSCON->CLKOUTDIV = 0;
    } 
    else {
        // Otherwise one of the other modes
        mode--;
        LPC_IOCON->PIO0_1 = 0x51;
        LPC_SYSCON->CLKOUTDIV = divider;
        LPC_SYSCON->CLKOUTCLKSEL = mode;
        LPC_SYSCON->CLKOUTUEN = 0;
        LPC_SYSCON->CLKOUTUEN = 1;
    }
}
*/

void RITInit(unsigned long long value) {
    //LPC_SYSCON->SYSAHBCLKCTRL |= (0x1UL << 25);		// Enable clock to RIT, apparently this bit doesn't exist...?

    LPC_RITIMER->COMPVAL = value & 0xffffffffUL;
    LPC_RITIMER->COMPVAL_H = value >> 32;
    
    #if RIT_RESET
        LPC_RITIMER->CTRL = 0x2;
    #else
        LPC_RITIMER->CTRL = 0;
    #endif
    
    LPC_RITIMER->CTRL |= 0x1; // clear interrupt flag
    
    RITReset();
    RITGo();
    
    
    IRQClear(RIT_IRQn);
    IRQPriority(RIT_IRQn, RIT_PRIORITY);
    IRQEnable(RIT_IRQn);
}

void RITInitms(unsigned int value) {
    if((LPC_SYSCON->MAINCLKSEL & 0x03) == 0x03) {
        // assume 72MHz operation
        if(value > 3909374676UL) value = 3909374676UL; // prevent overflow, though maximum value is something like 45 days
        RITInit((unsigned long long) value * (unsigned long long) 72000);
    }
    else {
        // assume 12MHz operation
        RITInit((unsigned long long) value * (unsigned long long) 12000);
    }
}

void RIT_IRQHandler(void) {
    if(RITInterrupt) RITInterrupt();
    LPC_RITIMER->CTRL |= 0x1;
}



/*
// ****************************************************************************
// *** Power mode Functions
// ****************************************************************************

// *** Sleep mode, peripherals remain active, and processor wakes up on interrupt
void Sleep(void) {
    LPC_PMU->PCON |= (0x1UL << 11); // Clear the deep power-down power flag
    SCB->SCR &= ~(0x1UL <<2); // Deselect deep sleep/power-down mode
    __WFI(); // Sleep and wait for interrupt
}

// *** Deep sleep, chip shuts down, wakes up on GPIO or timer
void DeepSleep(unsigned long long startpins, unsigned long long startdirection, unsigned int timer) {
    unsigned int i;
    
    // *** If timer is enabled
    if(timer) {
        // *** Use T3[3]/P1[4]
        if(timer > 0x63FF9C) timer = 0x63FF9C;
        Timer2Init(9); // 10 prescale = ms
        
        if(startdirection & PORT0PIN1) {
            // User demanded a rising edge on this pin, Timer wakeup must also make use of rising edge
            Timer2Match2(timer, OUTPUTHIGH | RESET); // set match to measure in ms
        }
        else {
            // User demanded a falling edge on this pin (or did not specify), Timer wakeup to use falling edge
            Timer2Match2(timer, OUTPUTLOW | RESET); // set match to measure in ms
        }
        startpins |= PORT0PIN1;         // P0[1] is also the T2[2] pin
        
        LPC_SYSCON->PDRUNCFG &= ~(0x1UL <<6 | 1<<9 | 0x3);	// Power up the WDT oscillator
        LPC_SYSCON->PDAWAKECFG = LPC_SYSCON->PDRUNCFG;  // Set wakeup power config
        LPC_SYSCON->WDTOSCCTRL = 0x03F;		            // Lowest rate: 0.5MHz
        LPC_SYSCON->MAINCLKSEL = 0x02;     				// Select WDT as main clock source
        LPC_SYSCON->MAINCLKUEN = 0x00;               	// Update clock source
        LPC_SYSCON->MAINCLKUEN = 0x01;
        while (!(LPC_SYSCON->MAINCLKUEN & 0x01));       // Wait for clock update
        
        LPC_SYSCON->PDRUNCFG = ~((0x1UL <<6) | (0x1UL <<2) | (0x1UL <<9)); // Switch off everything unneeded
        
        if(LPC_SYSCON->BODCTRL & (0x1UL <<4)) {
            LPC_SYSCON->PDSLEEPCFG = 0x00000FB7;        // Use brown-out detect
        }
        else {
            LPC_SYSCON->PDSLEEPCFG = 0x00000FBF;        // Don't use brown-out
        }
    }
    else {
        LPC_SYSCON->PDRUNCFG &= ~(0x3);                 // Power up the IRC
        LPC_SYSCON->PDAWAKECFG = LPC_SYSCON->PDRUNCFG;  //Set wakeup power config
        LPC_SYSCON->MAINCLKSEL = 0;                     // Select IRC as main clock source
        LPC_SYSCON->MAINCLKUEN = 0;                     // Update clock source
        LPC_SYSCON->MAINCLKUEN = 1;
        while(!(LPC_SYSCON->MAINCLKUEN & 0x1));         // Wait for clock update
        
        LPC_SYSCON->PDRUNCFG = ~(0x7 | 1<<9);
        
        if(LPC_SYSCON->BODCTRL & (0x1UL <<4)) {
            LPC_SYSCON->PDSLEEPCFG = 0x00000FF7;        // Use brown-out detect
        }
        else {
            LPC_SYSCON->PDSLEEPCFG = 0x00000FFF;        // Don't use brown-out
        }
    }

    LPC_PMU->PCON |= (0x1UL <<8);    // Clear sleep flag
    
    SCB->SCR |=	(0x1UL <<2);         // Select deep sleep/power-down mode
    
    // Set wakeup pins
    LPC_SYSCON->STARTAPRP0 = startdirection & 0xffffffff;
    LPC_SYSCON->STARTRSRP0CLR = startpins & 0xffffffff;
    LPC_SYSCON->STARTERP0 = startpins & 0xffffffff;
    LPC_SYSCON->STARTAPRP1 = (startdirection >> 32) & 0xff;
    LPC_SYSCON->STARTRSRP1CLR = (startpins >> 32) & 0xff;
    LPC_SYSCON->STARTERP1 = (startpins >> 32) & 0xff;
    
    // Set wakeup interrupts
    for(i=0; i<40; i++) {
        if(startpins & (0x1UL <<i)) {
            IRQClear(i);
            IRQEnable(i);
        }
        else {
            IRQDisable(i);
        }
    }
    
    __WFI();    // Sleep and wait for interrupt
    
    if(timer) Timer2Stop(); // Stop timer

    ClockMode(DEFAULT_CLOCK);   // Reinstate clock
}

// *** Wakeup interrupt handler
void WAKEUP_IRQHandler(void){
	LPC_SYSCON->STARTRSRP0CLR |= LPC_SYSCON->STARTERP0; // Clear wakeup pin interrupts
	LPC_SYSCON->STARTRSRP1CLR |= LPC_SYSCON->STARTERP1;
}

// *** Deep power-down mode, only wakes up on low signal on P1[4]
void PowerDown(void) {
    #if WAKEUP_HYS
        LPC_PMU->GPREG4 | = 1 << 10;    // Enable hysteresis
    #endif
    
    LPC_PMU->PCON = (0x1UL <<1) | (0x1UL <<11);   // Select deep power-down mode, and clear flag
    SCB->SCR |= (0x1UL <<2);                 // Select deep power-down/sleep mode
    
    LPC_SYSCON->PDRUNCFG &= ~(0x3);     // Turn on IRC

    __WFI();    // Sleep
}

// *** Brownout detect
void BODInit(unsigned char interruptlevel) {
    LPC_SYSCON->BODCTRL = ((interruptlevel & 0x3) << 2) | (0x1UL <<4); // Set interrupt level
    IRQClear(BOD_IRQn);     // Clear and enable BOD
    IRQEnable(BOD_IRQn);
}

// *** Stop Brownout
void BODStop(void) {
    LPC_SYSCON->BODCTRL = 0;
    IRQDisable(BOD_IRQn);
}

// *** Brownout interrupt handler
void BOD_IRQHandler(void) {
    if(BODInterrupt) BODInterrupt();
}

// *** Reset status
unsigned char ResetStatus(void) {
    unsigned char status;
    status = LPC_SYSCON->SYSRESSTAT & 0x1f; // Read reset status
    LPC_SYSCON->SYSRESSTAT = status;        // Clear reset status
    if(LPC_PMU->PCON & (0x1UL <<11)) {           // Check deep power-down flag
        LPC_PMU->PCON |= (0x1UL <<11);
        status |= POWEREDDOWN;
    }
    return status;
}
*/

// ****************************************************************************
// *** GPIO Functions
// ****************************************************************************

// *** Initialise pins as digital IO mode (note: default state is digital input)
void Port0Init(unsigned int pins) {
    unsigned int i;
    unsigned int * ptr;
    
    ptr = (unsigned int *)LPC_IOCON_BASE;
    for(i=0x1; i<(0x1UL <<24); i<<=1) {    // Set various pins to GPIO when specified, Port 0 only goes up to pin 23
        if(pins & i) *ptr = 0x90;
        ptr++;
    }
    
	if(pins & PIN0) LPC_IOCON->RESET_PIO0_0 = 0x91; // special cases
	if(pins & PIN10) LPC_IOCON->SWCLK_PIO0_10 = 0x91;
	if(pins & PIN11) LPC_IOCON->TDI_PIO0_11 = 0x91;
	if(pins & PIN12) LPC_IOCON->TMS_PIO0_12 = 0x91;
	if(pins & PIN13) LPC_IOCON->TDO_PIO0_13 = 0x91;
	if(pins & PIN14) LPC_IOCON->TRST_PIO0_14 = 0x91;
	if(pins & PIN15) LPC_IOCON->SWDIO_PIO0_15 = 0x91;

    Port0SetIn(pins);                               // Set the specified pins as input
}

void Port1Init(unsigned int pins) {
    unsigned int i;
    unsigned int * ptr;
    
    ptr = (unsigned int *)LPC_IOCON_BASE + 24;
    for(i=0x1; i>0; i<<=1) {    // Set various pins to GPIO when specified, 
        if(pins & i) *ptr = 0x90;
        ptr++;
    }
    
    Port1SetIn(pins);                               // Set the specified pins as input
}

void Port0Hysteresis(unsigned int pins, unsigned char value) {
    unsigned int i;
    unsigned int * ptr;
    
    ptr = (unsigned int *)LPC_IOCON_BASE;
    for(i=0x1; i>(0x1UL <<24); i<<=1) {    // Set various pins to GPIO when specified, Port 0 only goes up to pin 23
        if(pins & i) {
            if(value) *ptr |= (0x1UL << 5);
            else *ptr &= ~(0x1UL << 5);
        }
        ptr++;
    }
}

void Port1Hysteresis(unsigned int pins, unsigned char value) {
    unsigned int i;
    unsigned int * ptr;
    
    ptr = (unsigned int *)LPC_IOCON_BASE + 24;
    for(i=0x1; i>0; i<<=1) {    // Set various pins to GPIO when specified, 
        if(pins & i) {
            if(value) *ptr |= (0x1UL << 5);
            else *ptr &= ~(0x1UL << 5);
        }
        ptr++;
    }
}

void Port0Invert(unsigned int pins, unsigned char value) {
    unsigned int i;
    unsigned int * ptr;
    
    ptr = (unsigned int *)LPC_IOCON_BASE;
    for(i=0x1; i>(0x1UL <<24); i<<=1) {    // Set various pins to GPIO when specified, Port 0 only goes up to pin 23
        if(pins & i) {
            if(value) *ptr |= (0x1UL << 6);
            else *ptr &= ~(0x1UL << 6);
        }
        ptr++;
    }
}

void Port1Invert(unsigned int pins, unsigned char value) {
    unsigned int i;
    unsigned int * ptr;
    
    ptr = (unsigned int *)LPC_IOCON_BASE + 24;
    for(i=0x1; i>0; i<<=1) {    // Set various pins to GPIO when specified, 
        if(pins & i) {
            if(value) *ptr |= (0x1UL << 6);
            else *ptr &= ~(0x1UL << 6);
        }
        ptr++;
    }
}

void Port0Pull(unsigned int pins, unsigned char value) {
    unsigned int i;
    unsigned int * ptr;
    
    ptr = (unsigned int *)LPC_IOCON_BASE;
    for(i=0x1; i>(0x1UL <<24); i<<=1) {    // Set various pins to GPIO when specified, Port 0 only goes up to pin 23
        if(pins & i) {
            *ptr &= ~(0x3UL << 3);
            *ptr |= (value << 6);
        }
        ptr++;
    }
}

void Port1Pull(unsigned int pins, unsigned char value) {
    unsigned int i;
    unsigned int * ptr;
    
    ptr = (unsigned int *)LPC_IOCON_BASE + 24;
    for(i=0x1; i>0; i<<=1) {    // Set various pins to GPIO when specified, 
        if(pins & i) {
            *ptr &= ~(0x3UL << 3);
            *ptr |= (value << 6);
        }
        ptr++;
    }
}

/*
// *** Configure interrupts on a pin (note: default state is interrupts disabled)
void Port0SetInterrupt(unsigned short pins, unsigned char mode) {
    // turn on and configure interrupt for specified pins if mode isn't OFF, otherwise disable
    if(mode) {
        LPC_GPIO0->IE |= pins;  // enable interrupts
        switch(mode) {
            case FALLING:
                LPC_GPIO0->IS &= ~(pins);   // edge or level sensitive setting
                LPC_GPIO0->IBE &= ~(pins);  // both or single edges setting
                LPC_GPIO0->IEV &= ~(pins);  // rising/falling edge or high/low level setting
                break;
            case RISING:
                LPC_GPIO0->IS &= ~(pins);
                LPC_GPIO0->IBE &= ~(pins);
                LPC_GPIO0->IEV |= pins;
                break;
            case BOTH:
                LPC_GPIO0->IS &= ~(pins);
                LPC_GPIO0->IBE |= pins;
                //LPC_GPIO0->IEV &= ~(pins);    // doesn't matter
                break;
            case LOW:
                LPC_GPIO0->IS |= pins;
                LPC_GPIO0->IBE &= ~(pins);
                LPC_GPIO0->IEV &= ~(pins);
                break;
            case HIGH:
                LPC_GPIO0->IS |= pins;
                LPC_GPIO0->IBE &= ~(pins);
                LPC_GPIO0->IEV |= pins;
                break;
            default:
                LPC_GPIO0->IE &= ~(pins);
        }
    }
    else {
        LPC_GPIO0->IE &= ~(pins);
    }
    
    // check if there are any interrupts enabled interrupts, or disable if there are none
    if(LPC_GPIO0->IE & 0xfff) {
        IRQClear(EINT0_IRQn);
        IRQEnable(EINT0_IRQn);
        IRQPriority(EINT0_IRQn, PORT0_PRIORITY);
    }
    else {
        IRQDisable(EINT0_IRQn);
    }
}

void Port1SetInterrupt(unsigned short pins, unsigned char mode) {
    // turn on and configure interrupt for specified pins if mode isn't OFF, otherwise disable
    if(mode) {
        LPC_GPIO1->IE |= pins;  // enable interrupts
        switch(mode) {
            case FALLING:
                LPC_GPIO1->IS &= ~(pins);   // edge or level sensitive setting
                LPC_GPIO1->IBE &= ~(pins);  // both or single edges setting
                LPC_GPIO1->IEV &= ~(pins);  // rising/falling edge or high/low level setting
                break;
            case RISING:
                LPC_GPIO1->IS &= ~(pins);
                LPC_GPIO1->IBE &= ~(pins);
                LPC_GPIO1->IEV |= pins;
                break;
            case BOTH:
                LPC_GPIO1->IS &= ~(pins);
                LPC_GPIO1->IBE |= pins;
                //LPC_GPIO1->IEV &= ~(pins);    // doesn't matter
                break;
            case LOW:
                LPC_GPIO1->IS |= pins;
                LPC_GPIO1->IBE &= ~(pins);
                LPC_GPIO1->IEV &= ~(pins);
                break;
            case HIGH:
                LPC_GPIO1->IS |= pins;
                LPC_GPIO1->IBE &= ~(pins);
                LPC_GPIO1->IEV |= pins;
                break;
            default:
                LPC_GPIO1->IE &= ~(pins);
        }
    }
    else {
        LPC_GPIO1->IE &= ~(pins);
    }
    
    // check if there are any interrupts enabled interrupts, or disable if there are none
    if(LPC_GPIO1->IE & 0xfff) {
        IRQClear(EINT1_IRQn);
        IRQEnable(EINT1_IRQn);
        IRQPriority(EINT1_IRQn, PORT1_PRIORITY);
    }
    else {
        IRQDisable(EINT1_IRQn);
    }
}

void Port2SetInterrupt(unsigned short pins, unsigned char mode) {
    // turn on and configure interrupt for specified pins if mode isn't OFF, otherwise disable
    if(mode) {
        LPC_GPIO2->IE |= pins;  // enable interrupts
        switch(mode) {
            case FALLING:
                LPC_GPIO2->IS &= ~(pins);   // edge or level sensitive setting
                LPC_GPIO2->IBE &= ~(pins);  // both or single edges setting
                LPC_GPIO2->IEV &= ~(pins);  // rising/falling edge or high/low level setting
                break;
            case RISING:
                LPC_GPIO2->IS &= ~(pins);
                LPC_GPIO2->IBE &= ~(pins);
                LPC_GPIO2->IEV |= pins;
                break;
            case BOTH:
                LPC_GPIO2->IS &= ~(pins);
                LPC_GPIO2->IBE |= pins;
                //LPC_GPIO2->IEV &= ~(pins);    // doesn't matter
                break;
            case LOW:
                LPC_GPIO2->IS |= pins;
                LPC_GPIO2->IBE &= ~(pins);
                LPC_GPIO2->IEV &= ~(pins);
                break;
            case HIGH:
                LPC_GPIO2->IS |= pins;
                LPC_GPIO2->IBE &= ~(pins);
                LPC_GPIO2->IEV |= pins;
                break;
            default:
                LPC_GPIO2->IE &= ~(pins);
        }
    }
    else {
        LPC_GPIO2->IE &= ~(pins);
    }
    
    // check if there are any interrupts enabled interrupts, or disable if there are none
    if(LPC_GPIO2->IE & 0xfff) {
        IRQClear(EINT2_IRQn);
        IRQEnable(EINT2_IRQn);
        IRQPriority(EINT2_IRQn, PORT2_PRIORITY);
    }
    else {
        IRQDisable(EINT2_IRQn);
    }
}

void Port3SetInterrupt(unsigned short pins, unsigned char mode) {
    // turn on and configure interrupt for specified pins if mode isn't OFF, otherwise disable
    if(mode) {
        LPC_GPIO3->IE |= pins;  // enable interrupts
        switch(mode) {
            case FALLING:
                LPC_GPIO3->IS &= ~(pins);   // edge or level sensitive setting
                LPC_GPIO3->IBE &= ~(pins);  // both or single edges setting
                LPC_GPIO3->IEV &= ~(pins);  // rising/falling edge or high/low level setting
                break;
            case RISING:
                LPC_GPIO3->IS &= ~(pins);
                LPC_GPIO3->IBE &= ~(pins);
                LPC_GPIO3->IEV |= pins;
                break;
            case BOTH:
                LPC_GPIO3->IS &= ~(pins);
                LPC_GPIO3->IBE |= pins;
                //LPC_GPIO3->IEV &= ~(pins);    // doesn't matter
                break;
            case LOW:
                LPC_GPIO3->IS |= pins;
                LPC_GPIO3->IBE &= ~(pins);
                LPC_GPIO3->IEV &= ~(pins);
                break;
            case HIGH:
                LPC_GPIO3->IS |= pins;
                LPC_GPIO3->IBE &= ~(pins);
                LPC_GPIO3->IEV |= pins;
                break;
            default:
                LPC_GPIO3->IE &= ~(pins);
        }
    }
    else {
        LPC_GPIO3->IE &= ~(pins);
    }
    
    // check if there are any interrupts enabled interrupts, or disable if there are none
    if(LPC_GPIO3->IE & 0xfff) {
        IRQClear(EINT3_IRQn);
        IRQEnable(EINT3_IRQn);
        IRQPriority(EINT3_IRQn, PORT3_PRIORITY);
    }
    else {
        IRQDisable(EINT3_IRQn);
    }
}

// *** Port nterrupt functions
void PIOINT0_IRQHandler(void) {
    if(Port0Interrupt) Port0Interrupt(LPC_GPIO0->MIS);  // Port-wide interrupt, passing the pins that interrupted
    if(LPC_GPIO0->MIS & 0x001 && Port0Pin0Interrupt) Port0Pin0Interrupt();  // Pin-specific interrupt
    if(LPC_GPIO0->MIS & 0x002 && Port0Pin1Interrupt) Port0Pin1Interrupt();
    if(LPC_GPIO0->MIS & 0x004 && Port0Pin2Interrupt) Port0Pin2Interrupt();
    if(LPC_GPIO0->MIS & 0x008 && Port0Pin3Interrupt) Port0Pin3Interrupt();
    if(LPC_GPIO0->MIS & 0x010 && Port0Pin4Interrupt) Port0Pin4Interrupt();
    if(LPC_GPIO0->MIS & 0x020 && Port0Pin5Interrupt) Port0Pin5Interrupt();
    if(LPC_GPIO0->MIS & 0x040 && Port0Pin6Interrupt) Port0Pin6Interrupt();
    if(LPC_GPIO0->MIS & 0x080 && Port0Pin7Interrupt) Port0Pin7Interrupt();
    if(LPC_GPIO0->MIS & 0x100 && Port0Pin8Interrupt) Port0Pin8Interrupt();
    if(LPC_GPIO0->MIS & 0x200 && Port0Pin9Interrupt) Port0Pin9Interrupt();
    if(LPC_GPIO0->MIS & 0x400 && Port0Pin10Interrupt) Port0Pin10Interrupt();
    if(LPC_GPIO0->MIS & 0x800 && Port0Pin11Interrupt) Port0Pin11Interrupt();
    LPC_GPIO0->IC = LPC_GPIO0->RIS; // Clear the interrupt
}

void PIOINT1_IRQHandler(void) {
    if(Port1Interrupt) Port1Interrupt(LPC_GPIO1->MIS);  // Port-wide interrupt, passing the pins that interrupted
    if(LPC_GPIO1->MIS & 0x001 && Port1Pin0Interrupt) Port1Pin0Interrupt();  // Pin-specific interrupt
    if(LPC_GPIO1->MIS & 0x002 && Port1Pin1Interrupt) Port1Pin1Interrupt();
    if(LPC_GPIO1->MIS & 0x004 && Port1Pin2Interrupt) Port1Pin2Interrupt();
    if(LPC_GPIO1->MIS & 0x008 && Port1Pin3Interrupt) Port1Pin3Interrupt();
    if(LPC_GPIO1->MIS & 0x010 && Port1Pin4Interrupt) Port1Pin4Interrupt();
    if(LPC_GPIO1->MIS & 0x020 && Port1Pin5Interrupt) Port1Pin5Interrupt();
    if(LPC_GPIO1->MIS & 0x040 && Port1Pin6Interrupt) Port1Pin6Interrupt();
    if(LPC_GPIO1->MIS & 0x080 && Port1Pin7Interrupt) Port1Pin7Interrupt();
    if(LPC_GPIO1->MIS & 0x100 && Port1Pin8Interrupt) Port1Pin8Interrupt();
    if(LPC_GPIO1->MIS & 0x200 && Port1Pin9Interrupt) Port1Pin9Interrupt();
    if(LPC_GPIO1->MIS & 0x400 && Port1Pin10Interrupt) Port1Pin10Interrupt();
    if(LPC_GPIO1->MIS & 0x800 && Port1Pin11Interrupt) Port1Pin11Interrupt();
    LPC_GPIO1->IC = LPC_GPIO1->RIS; // Clear the interrupt
}

void PIOINT2_IRQHandler(void) {
    if(Port2Interrupt) Port2Interrupt(LPC_GPIO2->MIS);  // Port-wide interrupt, passing the pins that interrupted
    if(LPC_GPIO2->MIS & 0x001 && Port2Pin0Interrupt) Port2Pin0Interrupt();  // Pin-specific interrupt
    if(LPC_GPIO2->MIS & 0x002 && Port2Pin1Interrupt) Port2Pin1Interrupt();
    if(LPC_GPIO2->MIS & 0x004 && Port2Pin2Interrupt) Port2Pin2Interrupt();
    if(LPC_GPIO2->MIS & 0x008 && Port2Pin3Interrupt) Port2Pin3Interrupt();
    if(LPC_GPIO2->MIS & 0x010 && Port2Pin4Interrupt) Port2Pin4Interrupt();
    if(LPC_GPIO2->MIS & 0x020 && Port2Pin5Interrupt) Port2Pin5Interrupt();
    if(LPC_GPIO2->MIS & 0x040 && Port2Pin6Interrupt) Port2Pin6Interrupt();
    if(LPC_GPIO2->MIS & 0x080 && Port2Pin7Interrupt) Port2Pin7Interrupt();
    if(LPC_GPIO2->MIS & 0x100 && Port2Pin8Interrupt) Port2Pin8Interrupt();
    if(LPC_GPIO2->MIS & 0x200 && Port2Pin9Interrupt) Port2Pin9Interrupt();
    if(LPC_GPIO2->MIS & 0x400 && Port2Pin10Interrupt) Port2Pin10Interrupt();
    if(LPC_GPIO2->MIS & 0x800 && Port2Pin11Interrupt) Port2Pin11Interrupt();
    LPC_GPIO2->IC = LPC_GPIO2->RIS; // Clear the interrupt
}

void PIOINT3_IRQHandler(void) {
    if(Port3Interrupt) Port3Interrupt(LPC_GPIO3->MIS);  // Port-wide interrupt, passing the pins that interrupted
    if(LPC_GPIO3->MIS & 0x001 && Port3Pin0Interrupt) Port3Pin0Interrupt();  // Pin-specific interrupt
    if(LPC_GPIO3->MIS & 0x002 && Port3Pin1Interrupt) Port3Pin1Interrupt();
    if(LPC_GPIO3->MIS & 0x004 && Port3Pin2Interrupt) Port3Pin2Interrupt();
    if(LPC_GPIO3->MIS & 0x008 && Port3Pin3Interrupt) Port3Pin3Interrupt();
    LPC_GPIO3->IC = LPC_GPIO3->RIS; // Clear the interrupt
}
*/

// ****************************************************************************
// *** UART Functions
// ****************************************************************************

#if UART_EN
    void UARTInit(unsigned int baud) {
        unsigned int baudval;
        
        IRQDisable(USART_IRQn);
        
        // Enable the pins
        LPC_SYSCON->SYSAHBCLKCTRL |= (0x1UL <<12);     // Enable clock to IOCON block and UART block
        LPC_IOCON->PIO0_18 = 0x11;                   // Rx
        LPC_IOCON->PIO0_19 = 0x11;                   // Tx
        
        // ...and flow control if enabled
        #if UART_FLOW
            LPC_IOCON->PIO0_7 = 0x11;               // #CTS
            LPC_IOCON->PIO0_17 = 0x11;               // #RTS
            LPC_USART->MCR |= (0x1UL <<6) | (0x1UL <<7);
        #endif
        
        // Set clock settings
        LPC_SYSCON->UARTCLKDIV = 0x01;              // Clock divider at 1
        
        LPC_USART->LCR = 0x80;                       // enable access to divisor (clock) latches
        if(baud == AUTO) {
            // In autobaud mode, wait for ascii 'A' or 'a' to be sent on the UART (don't send anything else, weirdness results
            LPC_USART->ACR = 0x003;                  // start Autobaud
            while((LPC_USART->ACR & 0x001) == 1);    // wait for Autobaud to finish
        }
        else {
            // Here are a massive list of pre-calculated fractional baud rates
            if((LPC_SYSCON->MAINCLKSEL & 0x03) == 0x03) {
                // assume 72MHz operation
                #if UART_USE_FBR
                    switch(baud) {                          // some predefined baud rates with pre-calculated fractional baud rates
                        case 110: LPC_USART->DLM = 0x92; LPC_USART->DLL = 0x7c; LPC_USART->FDR = 0xb1; break;
                        case 4800: LPC_USART->DLM = 0x03; LPC_USART->DLL = 0x6b; LPC_USART->FDR = 0xe1; break;
                        case 9600: LPC_USART->DLM = 0x01; LPC_USART->DLL = 0x77; LPC_USART->FDR = 0x41; break;
                        case 14400: LPC_USART->DLM = 0; LPC_USART->DLL = 0xfa; LPC_USART->FDR = 0x41; break;
                        case 19200: LPC_USART->DLM = 0; LPC_USART->DLL = 0x7d; LPC_USART->FDR = 0x87; break;
                        case 28800: LPC_USART->DLM = 0; LPC_USART->DLL = 0x7d; LPC_USART->FDR = 0x41; break;
                        case 38400: LPC_USART->DLM = 0; LPC_USART->DLL = 0x4a; LPC_USART->FDR = 0xc7; break;
                        case 56000: LPC_USART->DLM = 0; LPC_USART->DLL = 0x4b; LPC_USART->FDR = 0xe1; break;
                        case 57600: LPC_USART->DLM = 0; LPC_USART->DLL = 0x47; LPC_USART->FDR = 0xa1; break;
                        case 115200: LPC_USART->DLM = 0; LPC_USART->DLL = 0x17; LPC_USART->FDR = 0xa7; break;
                        case 128000: LPC_USART->DLM = 0; LPC_USART->DLL = 0x1f; LPC_USART->FDR = 0xf2; break;
                        case 153600: LPC_USART->DLM = 0; LPC_USART->DLL = 0x17; LPC_USART->FDR = 0xb3; break;
                        case 230400: LPC_USART->DLM = 0; LPC_USART->DLL = 0x10; LPC_USART->FDR = 0x92; break;
                        case 256000: LPC_USART->DLM = 0; LPC_USART->DLL = 0x10; LPC_USART->FDR = 0xa1; break;
                        case 460800: LPC_USART->DLM = 0; LPC_USART->DLL = 0x8; LPC_USART->FDR = 0x92; break;
                        case 921600: LPC_USART->DLM = 0; LPC_USART->DLL = 0x4; LPC_USART->FDR = 0x92; break;
                        case 1000000: LPC_USART->DLM = 0; LPC_USART->DLL = 0x4; LPC_USART->FDR = 0x81; break;
                        case 2000000: LPC_USART->DLM = 0; LPC_USART->DLL = 0x2; LPC_USART->FDR = 0x81; break;
                        case 3000000: LPC_USART->DLM = 0; LPC_USART->DLL = 0x1; LPC_USART->FDR = 0x21; break;
                        case 4000000: LPC_USART->DLM = 0; LPC_USART->DLL = 0x1; LPC_USART->FDR = 0x81; break;
                        default:
                            LPC_USART->FDR = 0x10;
                            baudval = 4500000/baud;	                // baud rate
                            if(baudval > 0xffff) baudval = 0xffff;
                            else if(baudval == 0) baudval = 1;
                            
                            LPC_USART->DLM = (baudval >> 8) & 0xff;
                            LPC_USART->DLL = baudval & 0xff; 
                    }
                #else
                    baudval = 4500000/baud;	                // baud rate
                    if(baudval > 0xffff) baudval = 0xffff;
                    else if(baudval == 0) baudval = 1;
                    
                    LPC_USART->DLM = (baudval >> 8) & 0xff;
                    LPC_USART->DLL = baudval & 0xff; 
                #endif
                    
            }
            else {
                // assume 12MHz operation
                #if UART_USE_FBR
                    switch(baud) {                          // some predefined baud rates with pre-calculated fractional baud rates
                        case 110: LPC_USART->DLM = 0x18; LPC_USART->DLL = 0x6a; LPC_USART->FDR = 0xb1; break;
                        case 2400: LPC_USART->DLM = 0; LPC_USART->DLL = 0xfa; LPC_USART->FDR = 0x41; break;
                        case 4800: LPC_USART->DLM = 0; LPC_USART->DLL = 0x7d; LPC_USART->FDR = 0x41; break;
                        case 9600: LPC_USART->DLM = 0; LPC_USART->DLL = 0x47; LPC_USART->FDR = 0xa1; break;
                        case 14400: LPC_USART->DLM = 0; LPC_USART->DLL = 0x1b; LPC_USART->FDR = 0xed; break;
                        case 19200: LPC_USART->DLM = 0; LPC_USART->DLL = 0x17; LPC_USART->FDR = 0xa7; break;
                        case 28800: LPC_USART->DLM = 0; LPC_USART->DLL = 0x17; LPC_USART->FDR = 0xf2; break;
                        case 38400: LPC_USART->DLM = 0; LPC_USART->DLL = 0x10; LPC_USART->FDR = 0x92; break;
                        case 56000: LPC_USART->DLM = 0; LPC_USART->DLL = 0x7; LPC_USART->FDR = 0xcb; break;
                        case 57600: LPC_USART->DLM = 0; LPC_USART->DLL = 0xd; LPC_USART->FDR = 0x10; break;
                        case 115200: LPC_USART->DLM = 0; LPC_USART->DLL = 0x6; LPC_USART->FDR = 0xc1; break;
                        case 128000: LPC_USART->DLM = 0; LPC_USART->DLL = 0x4; LPC_USART->FDR = 0xf7; break;
                        case 153600: LPC_USART->DLM = 0; LPC_USART->DLL = 0x4; LPC_USART->FDR = 0x92; break;
                        case 230400: LPC_USART->DLM = 0; LPC_USART->DLL = 0x3; LPC_USART->FDR = 0xc1; break;
                        case 256000: LPC_USART->DLM = 0; LPC_USART->DLL = 0x2; LPC_USART->FDR = 0xf7; break;
                        case 460800: LPC_USART->DLM = 0; LPC_USART->DLL = 0x1; LPC_USART->FDR = 0x85; break;
                        default:
                            LPC_USART->FDR = 0x10;
                            baudval = 750000/baud;	                // baud rate
                            if(baudval > 0xffff) baudval = 0xffff;
                            else if(baudval == 0) baudval = 1;
                            
                            LPC_USART->DLM = (baudval >> 8) & 0xff;
                            LPC_USART->DLL = baudval & 0xff; 
                    }
                #else
                    baudval = 750000/baud;	                // baud rate
                    if(baudval > 0xffff) baudval = 0xffff;
                    else if(baudval == 0) baudval = 1;
                    
                    LPC_USART->DLM = (baudval >> 8) & 0xff;
                    LPC_USART->DLL = baudval & 0xff; 
                #endif
            }
            
        }
        
        // Data format
        LPC_USART->LCR = ((UART_PARITY & 0x3) << 4) | ((UART_PARITY_EN & 0x1) << 3) | ((UART_STOP_BITS & 0x1) << 2) | (UART_BIT_LENGTH & 0x3);  // disable access to divisor latches, and set data format

        // Set interrupt stuff
        LPC_USART->IER = 0x05;	// Enable UART receive interrupt and line interrupt
        #if UART_USE_OUTBUFFER
            LPC_USART->IER |= 0x02; // Enables the THRE interrupt
            FUNCUARTBufferPush = 0;
            FUNCUARTBufferPop = 0;
            
            // FIFO needs at least two characters to get out of an "initialisation" condition.
            // nothing gets sent because the TXEN is disabled and then the FIFO is cleared afterwards
            LPC_USART->TER = 0;
            LPC_USART->THR = 0x00;
            LPC_USART->THR = 0x00;
        #endif
                
        // Flush any residual data and clear FIFO
        LPC_USART->FCR = 0x06;                       // Reset buffers
        LPC_USART->FCR = (UART_FIFO_LEVEL & 0x3) << 6 | 0x01; // set RX FIFO interrupt levels, and enable FIFOs.
        LPC_USART->SCR = LPC_USART->LSR;
       
        while (LPC_USART->LSR & 0x01)   LPC_USART->SCR = LPC_USART->RBR;  // Dump any data
        
        LPC_USART->TER = 0x80;
        
        IRQClear(USART_IRQn);
        IRQPriority(USART_IRQn, UART_PRIORITY);
        IRQEnable(USART_IRQn);
        
        #if UART_USE_OUTBUFFER
            // this is a workaround for the above mentioned FIFO two character thing
            LPC_USART->TER = 0;
            LPC_USART->THR = 0x00;
            LPC_USART->THR = 0x00;
            LPC_USART->FCR = 0x06; // Reset buffers
            LPC_USART->FCR = (UART_FIFO_LEVEL & 0x3) << 6 | 0x01; // set RX FIFO interrupt levels, and enable FIFOs.
            LPC_USART->TER = 0x80;
        #endif
    }
    
    void UARTStop(void) {
        LPC_USART->IER = 0;
        IRQDisable(USART_IRQn);
        LPC_SYSCON->SYSAHBCLKCTRL &= ~(0x1UL << 12);     // Disable clock to UART block
    }
    
    void UARTWriteByte(unsigned char data) {
        #if UART_USE_OUTBUFFER
            while(UARTBufferWritable() == 0);
            UARTBufferPush(data);
        #else
            //LPC_USART->IER &= ~0x001;                    // Disable receive interrupt while transmitting (causes problems in loop-back mode)
            while ((LPC_USART->LSR & 0x20) == 0);
            LPC_USART->THR = data;
            //LPC_USART->IER |= 0x001;                     // Re-enable receive interrupt
        #endif
    }
    
    void UARTWrite(unsigned char * data, unsigned int length) {
        #if UART_USE_OUTBUFFER
            while(length > 0) {
                while(UARTBufferWritable() == 0);
                UARTBufferPush(data[0]);
                length--;
                data++;
            }
        #else
            unsigned int i;
            //LPC_USART->IER &= ~0x001;                    // Disable receive interrupt while transmitting (causes problems in loop-back mode)
            for(i=0; i<length; i++) {
                while ((LPC_USART->LSR & 0x20) == 0);
                LPC_USART->THR = data[i];
            }
            //LPC_USART->IER |= 0x001;                     // Re-enable receive interrupt
        #endif
    }

    unsigned char UARTReadByte(void) {
        #if UART_MODE
            unsigned char lsr, byte;
            // In interrupt mode, running this function triggers the user-supplied interrupt code and returns zero
            if(UARTInterrupt) {
                while(LPC_USART->LSR & 0x01) {
                    lsr = LPC_USART->LSR;
                    byte = LPC_USART->RBR;
                    if((lsr & 0x04) && UARTParityError) UARTParityError(byte);
                    else UARTInterrupt(byte);
                }
            }
            return 0;
        #else
            // Otherwise grab what data is available
            return LPC_USART->RBR;
        #endif
    }
    
    
    #if UART_USE_OUTBUFFER
        unsigned char FUNCUARTBuffer[UART_BUFFER_SIZE];
        volatile unsigned short FUNCUARTBufferPush, FUNCUARTBufferPop;
        
        unsigned short UARTBufferWritable(void) {
            if(FUNCUARTBufferPush+1 == FUNCUARTBufferPop || (FUNCUARTBufferPush == (UART_BUFFER_SIZE-1) && FUNCUARTBufferPop == 0)) return 0;
            else return 1;
        }
        unsigned short UARTBufferReadable(void) {
            if(FUNCUARTBufferPush == FUNCUARTBufferPop) return 0;
            else return 1;
        }
        unsigned char UARTBufferPop(void) {
            unsigned char retval;
            retval = FUNCUARTBuffer[FUNCUARTBufferPop++];
            if(FUNCUARTBufferPop >= UART_BUFFER_SIZE) FUNCUARTBufferPop = 0;
            return retval;
        }
        void UARTBufferPush(unsigned char data) {
            FUNCUARTBuffer[FUNCUARTBufferPush++] = data;
            LPC_USART->IER |= 0x02;
            if(FUNCUARTBufferPush >= UART_BUFFER_SIZE) FUNCUARTBufferPush = 0;
        }
    #endif
    
    
    void USART_IRQHandler(void) {
        unsigned char byte = 0;
        unsigned char iir, lsr;
        VCOM_DATA_T* pVcom = &g_vCOM;
        unsigned short serial_state = 0;
        
        iir = (LPC_USART->IIR >> 1) & 0x07;    // interrupt identification register
        switch(iir) {
            case 0x3:   // 1 - Receive Line Status (RLS)
                if(VCOM_isbridge) {
                  lsr = LPC_USART->LSR;
                      /* There are errors or break interrupt update serial_state */
                  if (lsr & 0x02) serial_state |= CDC_SERIAL_STATE_OVERRUN;
                  if (lsr & 0x04) serial_state |= CDC_SERIAL_STATE_PARITY;
                  if (lsr & 0x08) serial_state |= CDC_SERIAL_STATE_FRAMING;
                  if (lsr & 0x10) serial_state |= CDC_SERIAL_STATE_BREAK;

                  USBROM->cdc->SendNotification(pVcom->hCdc, CDC_NOTIFICATION_SERIAL_STATE, serial_state);  
                }
                else {
                    lsr = LPC_USART->LSR;
                    if(lsr & 0x1) {
                        while(LPC_USART->LSR & 0x01) {
                            lsr = LPC_USART->LSR;
                            byte = LPC_USART->RBR;
                            if(lsr & 0x04) {
                                if(UARTParityError) UARTParityError(byte);
                                #if WHO_AM_I == I_AM_THALAMUS && RX_EN && RX_TYPE
                                RXUARTParityError();
                                #endif
                            }
                            else {
                                if(UARTInterrupt) UARTInterrupt(byte);
                                #if WHO_AM_I == I_AM_THALAMUS && RX_EN
                                RXUARTInterrupt(byte);
                                #endif
                                #if (WHO_AM_I == I_AM_HYPO || WHO_AM_I == I_AM_HYPX) && UART_EN && SYSTICK_EN && XBEE_EN
                                XBUARTInterrupt(byte);
                                #endif
                            }
                        }
                    }
                    if(lsr & 0x9c) {
                        while(LPC_USART->LSR & 0x01) {
                            byte = LPC_USART->RBR;
                        }
                    }
                }
                break;
            case 0x2:   // 2a - Receive Data Available (RDA)
                if(VCOM_isbridge) {
                    #if CDC_USE_PLED
                        LEDOn(PLED);
                    #endif
                    VCOM_uart_read(pVcom);
                }
                else {
                    while(LPC_USART->LSR & 0x01) {
                        lsr = LPC_USART->LSR;
                        byte = LPC_USART->RBR;
                        if(lsr & 0x04) {
                            if(UARTParityError) UARTParityError(byte);
                            #if WHO_AM_I == I_AM_THALAMUS && RX_EN && RX_TYPE
                            RXUARTParityError();
                            #endif
                        }
                        else {
                            if(UARTInterrupt) UARTInterrupt(byte);
                            #if WHO_AM_I == I_AM_THALAMUS && RX_EN
                            RXUARTInterrupt(byte);
                            #endif
                            #if (WHO_AM_I == I_AM_HYPO || WHO_AM_I == I_AM_HYPX) && UART_EN && SYSTICK_EN && XBEE_EN
                            XBUARTInterrupt(byte);
                            #endif
                        }
                    }
                }
                break;
            case 0x6:   // 2b - Character Time-out indicator (CTI)
                if(VCOM_isbridge) {
                    #if CDC_USE_PLED
                        LEDOn(PLED);
                    #endif
                    VCOM_uart_read(pVcom);
                }
                break;
            
            case 0x1:   // 3 - THRE interrupt
                //lsr = LPC_USART->LSR;
                if(VCOM_isbridge) {
                    if (pVcom->rxlen) {
                        #if CDC_USE_PLED
                            LEDOn(PLED);
                        #endif
                        VCOM_uart_write(pVcom);
                    }
                }
                #if UART_USE_OUTBUFFER
                else {
                    byte = 16;
                    while(byte-- > 0) {
                        if(UARTBufferReadable()) {
                            LPC_USART->THR = UARTBufferPop();
                        }
                        else {
                            LPC_USART->IER &= ~(0x0002); // Disable the THRE interrupt
                            break;
                        }
                    }
                }
                #endif
                break;
            
            case 0x0:   // 4 - Modem interrupt
                break;
        }
    }
#endif

// ****************************************************************************
// *** I2C Functions
// ****************************************************************************

#if I2C_EN
    volatile unsigned char FUNCI2CMasterState, FUNCI2CMasterState2, FUNCI2CSlaveState, FUNCI2CSlaveState2, FUNCI2CSlaveMode;
    volatile unsigned int FUNCI2CRdLength, FUNCI2CWrLength;
    volatile unsigned int FUNCI2CRdIndex, FUNCI2CWrIndex;
    
    unsigned char FUNCI2CBuffer[I2C_DATA_SIZE];

	// ******* Initialisation, set speed (in kHz)
	void I2CInit(unsigned short speed) {
        unsigned int i;
        
        for(i=0; i<I2C_DATA_SIZE; i++) {
            FUNCI2CBuffer[i] = 0;
        }
            
        FUNCI2CSlaveMode = 0;
        if(speed == SLAVE) {
            FUNCI2CSlaveMode = 1;
        }
        
		FUNCI2CMasterState=I2C_IDLE;
        FUNCI2CSlaveState=I2C_IDLE;
		FUNCI2CRdLength=0;
        FUNCI2CWrLength=0;
		FUNCI2CRdIndex=0;
        FUNCI2CWrIndex=0;

		LPC_SYSCON->SYSAHBCLKCTRL |= (0x1UL << 5);	// Enable clock to I2C

		LPC_IOCON->PIO0_4 = 0x01;	// Set up pins PIO0_4 and PIO0_5 for I2C
		LPC_IOCON->PIO0_5 = 0x01;

		#if I2C_FASTMODE_PLUS
			// if speed is greater than 400k, set to Fast-mode Plus
			LPC_IOCON->PIO0_4 |= (0x1UL << 9);
			LPC_IOCON->PIO0_5 |= (0x1UL << 9);
		#endif
		
		LPC_SYSCON->PRESETCTRL |= (0x1UL << 1);	// I2C reset de-asserted
		LPC_I2C->CONCLR = I2C_AA | I2C_SI | I2C_STA | I2C_ENA;	// Clear status fags

        if((LPC_SYSCON->MAINCLKSEL & 0x03) == 0x03) {
            // assume 72MHz operation
            LPC_I2C->SCLL = (36000 / speed) & 0xffff;	// Set speed
            LPC_I2C->SCLH = (36000 / speed) & 0xffff;

        }
        else {
            // assume 12MHz operation
            LPC_I2C->SCLL = (6000 / speed) & 0xffff;	// Set speed
            LPC_I2C->SCLH = (6000 / speed) & 0xffff;
        }
        
		if(FUNCI2CSlaveMode == 1) {
			// set slave mode (I2C_MODE = 1)
			LPC_I2C->ADR0 = I2C_SLAVE_ADR0;	// slave mode addresses
			LPC_I2C->ADR1 = I2C_SLAVE_ADR1;
			LPC_I2C->ADR2 = I2C_SLAVE_ADR2;
			LPC_I2C->ADR3 = I2C_SLAVE_ADR3;
		}

        IRQClear(I2C_IRQn);
        IRQPriority(I2C_IRQn, I2C_PRIORITY);
		IRQEnable(I2C_IRQn);
		LPC_I2C->CONSET = I2C_ENA | I2C_SI;
	}

	void I2CStop(void) {
		IRQDisable(I2C_IRQn);
		LPC_SYSCON->PRESETCTRL &= ~(0x1UL << 1);	// I2C reset asserted
		LPC_SYSCON->SYSAHBCLKCTRL &= ~(0x1UL << 5);	// Disable clock to I2C
	}

	// ****** The I2C Engine, does the I2C stuff
	unsigned int I2CMaster(unsigned char * wrData, unsigned int  wrLength, unsigned char * rdData, unsigned char rdLength) {
		unsigned int timeout = 0, i;
        
		FUNCI2CMasterState = I2C_IDLE;
		FUNCI2CMasterState2 = I2C_IDLE;
		FUNCI2CRdIndex = 0;
		FUNCI2CWrIndex = 0;
		FUNCI2CRdLength = rdLength;
        FUNCI2CWrLength = wrLength;
        
        if(rdLength > 0) wrLength++;
        
        for(i=0;i<wrLength; i++) {
            FUNCI2CBuffer[i] = wrData[i];
        }
        
		LPC_I2C->CONSET = I2C_STA;	// set start condition
        
		while(1) {
			// loop until start condition transmit detected or timeout and send stop
			if (FUNCI2CMasterState == I2C_STARTED) {
				while (1) {
					// once start state is transmitted, loop until NACK state then send stop
					if (FUNCI2CMasterState2 == I2C_NACK){
						LPC_I2C->CONSET = I2C_STO;	// set stop condition
						LPC_I2C->CONCLR = I2C_SI;	// clear interrupt flag
                        
                        timeout = 0;
						while((LPC_I2C->CONSET & I2C_STO) && (timeout++ < I2C_TIMEOUT));	// wait until a stop condition
						break;
					}
				}
				break;	
			}
			if (timeout++ > I2C_TIMEOUT) {
				// timeout, send stop
				LPC_I2C->CONSET = I2C_STO;	// set stop condition
				LPC_I2C->CONCLR = I2C_SI;	// clear interrupt flag

				timeout = 0;
				while((LPC_I2C->CONSET & I2C_STO) && (timeout++ < I2C_TIMEOUT));	// wait until a stop condition
				break;
			}
		}
        for(i=0;i<rdLength; i++) {
            rdData[i] = FUNCI2CBuffer[i];
        }
        return FUNCI2CRdIndex;
	}
	 
	// ****** Interrupt handler - I2C state is implemented using interrupts
	void I2C_IRQHandler(void) {
        unsigned char state;
        state = LPC_I2C->STAT & 0xff;
        
        if(FUNCI2CSlaveMode == 0) {
            switch (state) {
                case 0x08:	// A START condition has been transmitted
                    FUNCI2CWrIndex = 0;
                    LPC_I2C->DAT = FUNCI2CBuffer[FUNCI2CWrIndex++];
                    LPC_I2C->CONCLR = I2C_STA;
                    FUNCI2CMasterState = I2C_STARTED;
            		LPC_I2C->CONCLR = I2C_SI;
                    break;

                case 0x10:	// A Repeated START condition has been transmitted
                    FUNCI2CRdIndex = 0;
                    LPC_I2C->DAT = FUNCI2CBuffer[FUNCI2CWrIndex++];
                    LPC_I2C->CONCLR = I2C_STA;
                    FUNCI2CMasterState = I2C_RESTARTED;
            		LPC_I2C->CONCLR = I2C_SI;
                    break;

                case 0x18:	// SLA+W has been transmitted; ACK has been received
                    if (FUNCI2CMasterState == I2C_STARTED) {
                        LPC_I2C->DAT = FUNCI2CBuffer[FUNCI2CWrIndex++];
                        FUNCI2CMasterState = I2C_ACK;
                    }
            		LPC_I2C->CONCLR = I2C_SI;
                    break;

                case 0x20:	// SLA+W has not been transmitted; NOT ACK has been received
                    FUNCI2CMasterState = I2C_NACK;
                    FUNCI2CMasterState2 = I2C_NACK;
            		LPC_I2C->CONCLR = I2C_SI;
                    break;
                    
                case 0x28:	// Data byte in I2DAT has been transmitted; ACK has been received
                    if (FUNCI2CWrIndex < FUNCI2CWrLength) {   
                        LPC_I2C->DAT = FUNCI2CBuffer[FUNCI2CWrIndex++];
                        FUNCI2CMasterState = I2C_ACK;
                    }
                    else {
                        if (FUNCI2CRdLength > 0) {
                            LPC_I2C->CONSET = I2C_STA;
                            FUNCI2CMasterState = I2C_REPEATED_START;
                        }
                        else {
                            FUNCI2CMasterState = I2C_ACK;
                            FUNCI2CMasterState2 = I2C_NACK; // very very dirty hax, I2CMasterState used for ACK polling in EEPROM while I2CMasterState2 used for end of I2C operation detection!
                            LPC_I2C->CONSET = I2C_STO;
                        }
                    }
            		LPC_I2C->CONCLR = I2C_SI;
                    break;
                    
                case 0x30:	// Data byte in I2DAT has been transmitted; NOT ACK has been received
                    FUNCI2CMasterState = I2C_NACK;
                    FUNCI2CMasterState2 = I2C_NACK;
                    LPC_I2C->CONSET = I2C_STO;
            		LPC_I2C->CONCLR = I2C_SI;
                    break;
                    
                case 0x38:	// Arbitration lost
                    FUNCI2CMasterState = I2C_ERROR;
            		LPC_I2C->CONCLR = I2C_SI;
                    break;

                case 0x40:	// SLA+R has been trnasmitted; ACK has been received
                    if (FUNCI2CRdLength == 1) {
                        LPC_I2C->CONCLR = I2C_AA;
                    }
                    else {
                        LPC_I2C->CONSET = I2C_AA;
                    }
            		LPC_I2C->CONCLR = I2C_SI;
                    break;
                    
                case 0x48:	// SLA+R has not been transmitted; NOT ACK has been received
                    FUNCI2CMasterState = I2C_NACK;
            		LPC_I2C->CONCLR = I2C_SI;
                    break;
                    
                case 0x50:	// Data byte has been receievd; ACK has been returned
                    FUNCI2CBuffer[FUNCI2CRdIndex++] = LPC_I2C->DAT;
                    if (FUNCI2CRdIndex + 1 < FUNCI2CRdLength) {   
                        FUNCI2CMasterState = I2C_ACK;
                        LPC_I2C->CONSET = I2C_AA;
                    }
                    else {
                        FUNCI2CMasterState = I2C_NACK;
                        LPC_I2C->CONCLR = I2C_AA;
                    }
            		LPC_I2C->CONCLR = I2C_SI;
                    break;

                case 0x58:	// Data byte has been received; NOT ACK has been returned
                    FUNCI2CBuffer[FUNCI2CRdIndex++] = LPC_I2C->DAT;
                    FUNCI2CMasterState = I2C_NACK;
                    FUNCI2CMasterState2 = I2C_NACK;	// hax is needed (I2CMasterState changes too quickly to register in I2CEngine()
                    LPC_I2C->CONSET = I2C_STO;
            		LPC_I2C->CONCLR = I2C_SI;
                    break;
                
                default:	
            		LPC_I2C->CONCLR = I2C_SI;
                    break;
            }
        }
        #if I2C_SLAVE_EN
        else if(FUNCI2CSlaveMode == 1) {
            switch (state) {
                case 0x60:  // Own SLA+W has been received; ACK has been returned
                case 0x68:  // Arbitration lost in SLA+R/W as master; Own SLA+W has been received, ACK returned
                    FUNCI2CWrIndex = 0;
                    FUNCI2CRdIndex = 0;
                    LPC_I2C->CONSET = I2C_AA;
            		LPC_I2C->CONCLR = I2C_SI;
                    FUNCI2CSlaveState = I2C_WR_STARTED;
                    FUNCI2CSlaveState2 = I2C_WR_STARTED;
                    break;
                
                case 0x70:  // General call address (0x00) has been received; ACK has been returned
                case 0x78:  // Arbitration lost in SLA+R/W as master; General call address has been received, ACK has been returned
                    FUNCI2CWrIndex = 0;
                    FUNCI2CRdIndex = 0;
                    LPC_I2C->CONSET = I2C_AA;
            		LPC_I2C->CONCLR = I2C_SI;
                    FUNCI2CSlaveState = I2C_GEN_STARTED;
                    FUNCI2CSlaveState2 = I2C_GEN_STARTED;
                    break;
                
                case 0x80:  // Previously addressed with own SLV address; DATA has been received, ACK has been returned
                case 0x90:  // Previously addressed with General Call; DATA byte has been received; ACK has been returned
                    if (FUNCI2CSlaveState == I2C_WR_STARTED) {
                        FUNCI2CBuffer[FUNCI2CWrIndex++] = LPC_I2C->DAT;
                        LPC_I2C->CONSET = I2C_AA;
                    }
                    else {
                        LPC_I2C->CONCLR = I2C_AA;
                    }
            		LPC_I2C->CONCLR = I2C_SI;
                    break;
                    
                
                case 0xA8:  // Own SLA+R has been received; ACK has been returned
                case 0xB0:  // Arbitration lost in SLA+R/W as master; Own SLA+R has been received, ACK has been returned
                    FUNCI2CRdIndex = 0;
                    LPC_I2C->CONSET = I2C_AA;
            		LPC_I2C->CONCLR = I2C_SI;
                    LPC_I2C->DAT = FUNCI2CBuffer[FUNCI2CRdIndex++];
                    FUNCI2CSlaveState = I2C_RD_STARTED;
                    FUNCI2CSlaveState2 = I2C_RD_STARTED;
                    break;
                
                case 0xB8:  // Data byte in I2DAT has been transmitted; ACK has been received
                case 0xC8:  // Data byte in I2DAT has been transmitted; NOT ACK has been received.
                    if (FUNCI2CSlaveState == I2C_RD_STARTED) {
                        LPC_I2C->DAT = FUNCI2CBuffer[FUNCI2CRdIndex++];
                        LPC_I2C->CONSET = I2C_AA;
                    }
                    else {
                        LPC_I2C->CONCLR = I2C_AA;
                    }
            		LPC_I2C->CONCLR = I2C_SI;
                    break;

                case 0xC0:  // Data byte in I2DAT has been transmitted; NOT ACK has been received
                    LPC_I2C->CONSET = I2C_AA;
            		LPC_I2C->CONCLR = I2C_SI;
                    FUNCI2CSlaveState = I2C_NACK;
                break;
                
                case 0xA0:  // A STOP condition or Repeated START condition has been received while still addressed as SLV/REC or SLV/TRX
                    LPC_I2C->CONSET = I2C_AA;
                    LPC_I2C->CONCLR = I2C_SI;
                    FUNCI2CSlaveState = I2C_IDLE;
                    if(I2CInterrupt) I2CInterrupt(FUNCI2CBuffer, FUNCI2CWrIndex);
                    break;
                
                default:
                    LPC_I2C->CONCLR = I2C_SI;
                    LPC_I2C->CONSET = I2C_ENA | I2C_SI;
                    break;
            }
        }
        #endif
	}
    
#endif


// ****************************************************************************
// *** SSP Functions
// ****************************************************************************

#if SSP0_EN
    void SSP0Init(unsigned short speed) {
        unsigned char i;
        unsigned short dummy=dummy;

        LPC_SYSCON->PRESETCTRL &= ~0x1;
        LPC_SYSCON->PRESETCTRL |= 0x1;              // deassert reset on SSP
        
        // Set up pins
        LPC_SYSCON->SYSAHBCLKCTRL |= (0x1UL << 11);
        LPC_IOCON->PIO0_8 = 0x01;
        LPC_IOCON->PIO0_9 = 0x01;
        LPC_IOCON->SWCLK_PIO0_10 = 0x02;
        
        #if SSP0_SSEL == 2
            LPC_IOCON->PIO0_2 = 0x01;	            // enable SSEL if used
        #elif SSP0_SSEL == 1
            Port0Init(PIN2);
            Port0SetOut(PIN2);
            SSP0S0CLR();
        #endif
    
        // SSP config
        LPC_SSP0->CR0 = ((SSP0_CLK_PHA & 0x1) << 6) | ((SSP0_CLK_POL & 0x1) << 6) | ((SSP0_FORMAT & 0x3) << 4) | ((SSP0_SIZE - 1) & 0xf);
        LPC_SYSCON->SSP0CLKDIV = 1;
        
        for(i=0; i<16; i++ ) dummy = LPC_SSP0->DR;  // Clear out FIFO buffer

        // set mode
        if(speed) { // master mode
            SSP0SetSpeed(speed);
            LPC_SSP0->CR1 = 0;                      // Clear CR1 (and set master mode)
            LPC_SSP0->CR1 |= 0x1 << 1;              // Enable SSP
        }
        else { // slave mode
            LPC_IOCON->PIO0_2 = 0x01;	            // enable SSEL always for slave mode
            
            #if SSP0_INT_LEVEL == 0
                LPC_SSP0->IMSC = 0x6;
            #else
                LPC_SSP0->IMSC = 0x4;               // enable interrupt on RX FIFO half-full.
            #endif
            
            LPC_SSP0->CR1 = 0;                      // Clear CR1
            LPC_SSP0->CR1 = 0x4;                    // Select slave mode
            LPC_SSP0->CR1 |= 0x1 << 1;              // Enable SSP
        
            IRQClear(SSP0_IRQn);
            IRQPriority(SSP0_IRQn, SSP0_PRIORITY);
            IRQEnable(SSP0_IRQn);
        }
    }
    
    void SSP0Stop(void) {
        IRQDisable(SSP0_IRQn);
        LPC_SSP0->CR1 &= ~(0x1 << 1);            // Disable SSP
        LPC_SYSCON->PRESETCTRL &= ~0x1;         // Assert SSP reset
    }
    
    void SSP0SetSpeed(unsigned short speed) {
        unsigned int scale;
        if(speed == 0) speed = 1; // avoid divide by zero
        if((LPC_SYSCON->MAINCLKSEL & 0x03) == 0x03) {
            // assume 72MHz operation
            scale = (72000 + (speed/2))/speed; // 72000/speed but hax to round up using integer mathematics
        }
        else {
            // assume 12MHz operation
            scale = (12000 + (speed/2))/speed;
        }
        // work out ssp scale clock
        if(scale > 254) scale = 254;
        else if(scale < 2) scale = 2;
        LPC_SSP0->CPSR = scale & 0xfe;
    }
    
    void SSP0WriteByte(unsigned short data) {
        unsigned short dummy=dummy;
        while ((LPC_SSP0->SR & 0x02) == 0);
        LPC_SSP0->DR = data;
        while ((LPC_SSP0->SR & 0x14) != 0x04);
        dummy = LPC_SSP0->DR;
    }
    
    void SSP0Write(unsigned short * data, unsigned int length) {
        unsigned int i;
        for(i=0; i<length; i++) {
            SSP0WriteByte(data[i]);
        }
    }
    
    unsigned short SSP0ReadByte(void) {
        while ((LPC_SSP0->SR & 0x02) == 0);
        LPC_SSP0->DR = 0xffff;
        while ((LPC_SSP0->SR & 0x14) != 0x04);
        return (LPC_SSP0->DR);
    }
    
    unsigned short SSP0Byte(unsigned short data) {
        while((LPC_SSP0->SR & 0x02) == 0);
        LPC_SSP0->DR = data;
        while ((LPC_SSP0->SR & 0x14) != 0x04);
        return (LPC_SSP0->DR);
    }
    
    void SSP0NextByte(unsigned short data) {
        while((LPC_SSP0->SR & 0x02) == 0);
        LPC_SSP0->DR = data;
    }
    
    void SSP0_IRQHandler(void) {
        while(LPC_SSP0->SR & 0x4) {// Receive FIFO Not Empty
            if(SSP0Interrupt) SSP0Interrupt(LPC_SSP0->DR);
        }
        LPC_SSP0->ICR = 0x3;
    }
#endif


#if SSP1_EN
    void SSP1Init(unsigned short speed) {
        unsigned char i;
        unsigned short dummy=dummy;

        LPC_SYSCON->PRESETCTRL &= ~0x4;
        LPC_SYSCON->PRESETCTRL |= 0x4;             // deassert reset on SSP
        
        // Set up pins
        LPC_SYSCON->SYSAHBCLKCTRL |= (0x1UL << 18);
        LPC_IOCON->PIO0_21 = 0x02;
        LPC_IOCON->PIO0_22 = 0x83;
        LPC_IOCON->PIO1_15 = 0x03;
        
        #if SSP1_SSEL == 2
            LPC_IOCON->PIO1_19 = 0x02;	            // enable SSEL if used
            Port0Init(PIN12);                       // enable pin on 0_12 (SSEL1)
            Port0SetOut(PIN12);
            SSP1S1CLR();
        #elif SSP1_SSEL == 1
            Port1Init(PIN19);
            Port1SetOut(PIN19);
            SSP1S0CLR();
            Port0Init(PIN12);
            Port0SetOut(PIN12);
            SSP1S1CLR();
        #endif
    
        // SSP config
        LPC_SSP1->CR0 = ((SSP1_CLK_PHA & 0x1) << 6) | ((SSP1_CLK_POL & 0x1) << 6) | ((SSP1_FORMAT & 0x3) << 4) | ((SSP1_SIZE - 1) & 0xf);
        LPC_SYSCON->SSP1CLKDIV = 1;
        
        for(i=0; i<16; i++ ) dummy = LPC_SSP1->DR;   // Clear out FIFO buffer

        // set mode
        if(speed) { // master mode
            SSP1SetSpeed(speed);
            LPC_SSP1->CR1 = 0;                       // Clear CR1 (and set master mode)
            LPC_SSP1->CR1 |= 0x1 << 1;               // Enable SSP
        }
        else {
            #if SSP1_INT_LEVEL == 0
                LPC_SSP1->IMSC = 0x6;
            #else
                LPC_SSP1->IMSC = 0x4;                // enable interrupt on RX FIFO half-full.
            #endif
            
            LPC_SSP1->CR1 = 0;                       // Clear CR1
            LPC_SSP1->CR1 = 0x4;                     // Select slave mode
            LPC_SSP1->CR1 |= 0x1 << 1;               // Enable SSP
        
            IRQClear(SSP1_IRQn);
            IRQPriority(SSP1_IRQn, SSP1_PRIORITY);
            IRQEnable(SSP1_IRQn);
        }
    }
    
    void SSP1Stop(void) {
        IRQDisable(SSP1_IRQn);
        LPC_SSP1->CR1 &= ~(0x1 << 1);            // Disable SSP
        LPC_SYSCON->PRESETCTRL &= ~0x4;         // Assert SSP reset
    }
    
    void SSP1SetSpeed(unsigned short speed) {
        unsigned int scale;
        if(speed == 0) speed = 1; // avoid divide by zero
        if((LPC_SYSCON->MAINCLKSEL & 0x03) == 0x03) {
            // assume 72MHz operation
            scale = (72000 + (speed/2))/speed; // 72000/speed but hax to round up using integer mathematics
        }
        else {
            // assume 12MHz operation
            scale = (12000 + (speed/2))/speed;
        }
        // work out ssp scale clock
        if(scale > 254) scale = 254;
        else if(scale < 2) scale = 2;
        LPC_SSP1->CPSR = scale & 0xfe;
    }
    
    void SSP1WriteByte(unsigned short data) {
        unsigned short dummy=dummy;
        while ((LPC_SSP1->SR & 0x02) == 0);
        LPC_SSP1->DR = data;
        //while ((LPC_SSP1->SR & 0x14) != 0x04);
        while ((LPC_SSP1->SR & 0x10));
        dummy = LPC_SSP1->DR;
    }
    
    void SSP1Write(unsigned short * data, unsigned int length) {
        unsigned int i;
        for(i=0; i<length; i++) {
            SSP1WriteByte(data[i]);
        }
    }
    
    unsigned short SSP1ReadByte(void) {
        while ((LPC_SSP1->SR & 0x02) == 0);
        LPC_SSP1->DR = 0xffff;
        while ((LPC_SSP1->SR & 0x14) != 0x04);
        return (LPC_SSP1->DR);
    }
    
    unsigned short SSP1Byte(unsigned short data) {
        while ((LPC_SSP1->SR & 0x02) == 0);
        LPC_SSP1->DR = data;
        while ((LPC_SSP1->SR & 0x14) != 0x04);
        return (LPC_SSP1->DR);
    }

    void SSP1NextByte(unsigned short data) {
        while ((LPC_SSP1->SR & 0x02) == 0);
        LPC_SSP1->DR = data;
    }
    
    void SSP1_IRQHandler(void) {
        while(LPC_SSP1->SR & 0x4) {// Receive FIFO Not Empty
            if(SSP1Interrupt) SSP1Interrupt(LPC_SSP1->DR);
        }
        LPC_SSP1->ICR = 0x3;
    }
#endif

// ****************************************************************************
// *** Timers Functions
// ****************************************************************************

void Timer0Init(unsigned short prescale) {
    LPC_SYSCON->SYSAHBCLKCTRL |= 1<<7;
    LPC_CT16B0->PR = prescale;
    LPC_CT16B0->MCR = 0;
    LPC_CT16B0->CCR = 0;
    LPC_CT16B0->EMR = 0;
    LPC_CT16B0->CTCR = 0;
    LPC_CT16B0->PWMC = 0;
    LPC_CT16B0->TCR = 1;
    IRQClear(CT16B0_IRQn);
	IRQPriority(CT16B0_IRQn, TIMER0_PRIORITY);
    IRQEnable(CT16B0_IRQn);
}
void Timer1Init(unsigned short prescale) {
    LPC_SYSCON->SYSAHBCLKCTRL |= 1<<8;
    LPC_CT16B1->PR = prescale;
    LPC_CT16B1->MCR = 0;
    LPC_CT16B1->CCR = 0;
    LPC_CT16B1->EMR = 0;
    LPC_CT16B1->CTCR = 0;
    LPC_CT16B1->PWMC = 0;
    LPC_CT16B1->TCR = 1;
    IRQClear(CT16B1_IRQn);
	IRQPriority(CT16B1_IRQn, TIMER1_PRIORITY);
    IRQEnable(CT16B1_IRQn);
}
void Timer2Init(unsigned int prescale) {
    LPC_SYSCON->SYSAHBCLKCTRL |= 1<<9;
    LPC_CT32B0->PR = prescale;
    LPC_CT32B0->MCR = 0;
    LPC_CT32B0->CCR = 0;
    LPC_CT32B0->EMR = 0;
    LPC_CT32B0->CTCR = 0;
    LPC_CT32B0->PWMC = 0;
    LPC_CT32B0->TCR = 1;
    IRQClear(CT32B0_IRQn);
	IRQPriority(CT32B0_IRQn, TIMER2_PRIORITY);
    IRQEnable(CT32B0_IRQn);
}
void Timer3Init(unsigned int prescale) {
    LPC_SYSCON->SYSAHBCLKCTRL |= 1<<10;
    LPC_CT32B1->PR = prescale;
    LPC_CT32B1->MCR = 0;
    LPC_CT32B1->CCR = 0;
    LPC_CT32B1->EMR = 0;
    LPC_CT32B1->CTCR = 0;
    LPC_CT32B1->PWMC = 0;
    LPC_CT32B1->TCR = 1;
    IRQClear(CT32B1_IRQn);
	IRQPriority(CT32B1_IRQn, TIMER3_PRIORITY);
    IRQEnable(CT32B1_IRQn);
}

void Timer0Stop(void) {
    LPC_CT16B0->TCR = 0x2;
	IRQDisable(CT16B0_IRQn);
    LPC_SYSCON->SYSAHBCLKCTRL &= ~(0x1UL <<7);
    LPC_CT16B0->TCR = 0x0;
}
void Timer1Stop(void) {
    LPC_CT16B1->TCR = 0x2;
	IRQDisable(CT16B1_IRQn);
    LPC_SYSCON->SYSAHBCLKCTRL &= ~(0x1UL <<8);
    LPC_CT16B1->TCR = 0x0;
}
void Timer2Stop(void) {
    LPC_CT32B0->TCR = 0x2;
	IRQDisable(CT32B0_IRQn);
    LPC_SYSCON->SYSAHBCLKCTRL &= ~(0x1UL <<9);
    LPC_CT32B0->TCR = 0x0;
}
void Timer3Stop(void) {
    LPC_CT32B1->TCR = 0x2;
	IRQDisable(CT32B1_IRQn);
    LPC_SYSCON->SYSAHBCLKCTRL &= ~(0x1UL <<10);
    LPC_CT32B1->TCR = 0x0;
}

void Timer0Match0(unsigned short interval, unsigned char mode) {
    if(mode & INTERRUPT) mode |= 0x1;
    LPC_CT16B0->MR0 = interval;
    LPC_CT16B0->MCR &= ~0x7;
    LPC_CT16B0->EMR &= ~0x30;
    LPC_CT16B0->PWMC &= ~0x1;
    LPC_CT16B0->MCR |= (mode & 0x7);
    if(mode & OUTPUTTOGGLE) {
        LPC_CT16B0->EMR |= (mode & 0x30);
        LPC_IOCON->PIO0_8 = 0x02;
        if(mode & OUTPUTLOW) LPC_CT16B0->EMR |= 1;
        else if(mode & OUTPUTHIGH) LPC_CT16B0->EMR &= ~1;
    }
    if(mode & PWM) LPC_CT16B0->PWMC |= 0x1;
}
void Timer0Match1(unsigned short interval, unsigned char mode) {
    if(mode & INTERRUPT) mode |= 0x1;
    LPC_CT16B0->MR1 = interval;
    LPC_CT16B0->MCR &= ~(0x7 << 3);
    LPC_CT16B0->EMR &= ~(0x30 << 2);
    LPC_CT16B0->PWMC &= ~(0x1 << 1);
    LPC_CT16B0->MCR |= (mode & 0x7) << 3;
    if(mode & OUTPUTTOGGLE) {
        LPC_CT16B0->EMR |= (mode & 0x30) << 2;
        LPC_IOCON->PIO0_9 = 0x02;
        if(mode & OUTPUTLOW) LPC_CT16B0->EMR |= 1<<1;
        else if(mode & OUTPUTHIGH) LPC_CT16B0->EMR &= ~(0x1UL <<1);
    }
    if(mode & PWM) LPC_CT16B0->PWMC |= 0x1 << 1;
}
void Timer0Match2(unsigned short interval, unsigned char mode) {
    if(mode & INTERRUPT) mode |= 0x1;
    LPC_CT16B0->MR2 = interval;
    LPC_CT16B0->MCR &= ~(0x7 << 6);
    LPC_CT16B0->EMR &= ~(0x30 << 4);
    LPC_CT16B0->PWMC &= ~(0x1 << 2);
    LPC_CT16B0->MCR |= (mode & 0x7) << 6;
    if(mode & OUTPUTTOGGLE) {
        LPC_CT16B0->EMR |= (mode & 0x30) << 4;
        #if TMR0M2_PIN == 0
            LPC_IOCON->SWCLK_PIO0_10 = 0x03;
        #else
            LPC_IOCON->PIO1_15 = 0x02;
        #endif
        if(mode & OUTPUTLOW) LPC_CT16B0->EMR |= 1<<2;
        else if(mode & OUTPUTHIGH) LPC_CT16B0->EMR &= ~(0x1UL <<2);
    }
    if(mode & PWM) LPC_CT16B0->PWMC |= 0x1 << 2;
}
void Timer0Match3(unsigned short interval, unsigned char mode) {
    if(mode & INTERRUPT) mode |= 0x1;
    LPC_CT16B0->MR3 = interval;
    LPC_CT16B0->MCR &= ~(0x7 << 9);
    LPC_CT16B0->EMR &= ~(0x30 << 6);
    LPC_CT16B0->PWMC &= ~(0x1 << 3);;
    LPC_CT16B0->MCR |= (mode & 0x7) << 9;
    if(mode & OUTPUTTOGGLE) {
        LPC_CT16B0->EMR |= (mode & 0x30) << 6;
        if(mode & OUTPUTLOW) LPC_CT16B0->EMR |= 1<<3;
        else if(mode & OUTPUTHIGH) LPC_CT16B0->EMR &= ~(0x1UL <<3);
    }
    if(mode & PWM) LPC_CT16B0->PWMC |= 0x1 << 3;
}
void Timer0Capture(unsigned char mode) {
    if(mode) {
        if(mode & COUNTER) {
            LPC_CT16B0->CTCR = mode & 0x3;
            LPC_CT16B0->TC = 0;
        }
        else {
            LPC_CT16B0->CTCR = 0;
            if(mode & (PWRESET | FALLING)) { // captures on falling edge, therefore resets on rising edge
                LPC_CT16B0->CTCR = (0x1 << 4);
            }
            else if(mode & (PWRESET | RISING)) { // captures on rising edge, therefore resets on falling edge
                LPC_CT16B0->CTCR = (0x1 << 4) | (0x1 << 5);
            }
            if(mode & INTERRUPT) mode |= 0x4;
            LPC_CT16B0->CCR = mode & 0x7;
            
        }
    
        LPC_IOCON->PIO0_2 &= ~(0x07);
        LPC_IOCON->PIO0_2 |= 0x02;
    }
}
void Timer1Match0(unsigned short interval, unsigned char mode) {
    if(mode & INTERRUPT) mode |= 0x1;
    LPC_CT16B1->MR0 = interval;
    LPC_CT16B1->MCR &= ~0x7;
    LPC_CT16B1->EMR &= ~0x30;
    LPC_CT16B1->PWMC &= ~0x1;
    LPC_CT16B1->MCR |= (mode & 0x7);
    if(mode & OUTPUTTOGGLE) {
        LPC_CT16B1->EMR |= (mode & 0x30);
        LPC_IOCON->PIO0_21 = 0x01;
        if(mode & OUTPUTLOW) LPC_CT16B1->EMR |= 1;
        else if(mode & OUTPUTHIGH) LPC_CT16B1->EMR &= ~1;
    }
    if(mode & PWM) LPC_CT16B1->PWMC |= 0x1;
}
void Timer1Match1(unsigned short interval, unsigned char mode) {
    if(mode & INTERRUPT) mode |= 0x1;
    LPC_CT16B1->MR1 = interval;
    LPC_CT16B1->MCR &= ~(0x7 << 3);
    LPC_CT16B1->EMR &= ~(0x30 << 2);
    LPC_CT16B1->PWMC &= ~(0x1 << 1);
    LPC_CT16B1->MCR |= (mode & 0x7) << 3;
    if(mode & OUTPUTTOGGLE) {
        LPC_CT16B1->EMR |= (mode & 0x30) << 2;
        LPC_IOCON->PIO0_22 = 0x02;
        if(mode & OUTPUTLOW) LPC_CT16B1->EMR |= 1<<1;
        else if(mode & OUTPUTHIGH) LPC_CT16B1->EMR &= ~(0x1UL <<1);
    }
    if(mode & PWM) LPC_CT16B1->PWMC |= 0x1 << 1;
}
void Timer1Match2(unsigned short interval, unsigned char mode) {
    if(mode & INTERRUPT) mode |= 0x1;
    LPC_CT16B1->MR2 = interval;
    LPC_CT16B1->MCR &= ~(0x7 << 6);
    LPC_CT16B1->EMR &= ~(0x30 << 4);
    LPC_CT16B1->PWMC &= ~(0x1 << 2);
    LPC_CT16B1->MCR |= (mode & 0x7) << 6;
    if(mode & OUTPUTTOGGLE) {
        LPC_CT16B1->EMR |= (mode & 0x30) << 4;
        if(mode & OUTPUTLOW) LPC_CT16B1->EMR |= 1<<2;
        else if(mode & OUTPUTHIGH) LPC_CT16B1->EMR &= ~(0x1UL <<2);
    }
    if(mode & PWM) LPC_CT16B1->PWMC |= 0x1 << 2;
}
void Timer1Match3(unsigned short interval, unsigned char mode) {
    if(mode & INTERRUPT) mode |= 0x1;
    LPC_CT16B1->MR3 = interval;
    LPC_CT16B1->MCR &= ~(0x7 << 9);
    LPC_CT16B1->EMR &= ~(0x30 << 6);
    LPC_CT16B1->PWMC &= ~(0x1 << 3);;
    LPC_CT16B1->MCR |= (mode & 0x7) << 9;
    if(mode & OUTPUTTOGGLE) {
        LPC_CT16B1->EMR |= (mode & 0x30) << 6;
        if(mode & OUTPUTLOW) LPC_CT16B1->EMR |= 1<<3;
        else if(mode & OUTPUTHIGH) LPC_CT16B1->EMR &= ~(0x1UL <<3);
    }
    if(mode & PWM) LPC_CT16B1->PWMC |= 0x1 << 3;
}
void Timer1Capture(unsigned char mode) {
    if(mode) {        
        if(mode & COUNTER) {
            LPC_CT16B1->CTCR = mode & 0x3;
            LPC_CT16B1->TC = 0;
        }
        else {
            LPC_CT16B1->CTCR = 0;
            if(mode & (PWRESET | FALLING)) { // captures on falling edge, therefore resets on rising edge
                LPC_CT16B1->CTCR = (0x1 << 4);
            }
            else if(mode & (PWRESET | RISING)) { // captures on rising edge, therefore resets on falling edge
                LPC_CT16B1->CTCR = (0x1 << 4) | (0x1 << 5);
            }
            if(mode & INTERRUPT) mode |= 0x4;
            LPC_CT16B1->CCR = mode & 0x7;
        }
    
        LPC_IOCON->PIO0_20 &= ~(0x07);
        LPC_IOCON->PIO0_20 |= 0x01;
    }
}
void Timer2Match0(unsigned int interval, unsigned char mode) {
    if(mode & INTERRUPT) mode |= 0x1;
    LPC_CT32B0->MR0 = interval;
    LPC_CT32B0->MCR &= ~0x7;
    LPC_CT32B0->EMR &= ~0x30;
    LPC_CT32B0->PWMC &= ~0x1;
    LPC_CT32B0->MCR |= (mode & 0x7);
    if(mode & OUTPUTTOGGLE) {
        LPC_CT32B0->EMR |= (mode & 0x30);
        LPC_IOCON->PIO0_18 = 0x02;
        if(mode & OUTPUTLOW) LPC_CT32B0->EMR |= 1;
        else if(mode & OUTPUTHIGH) LPC_CT32B0->EMR &= ~1;
    }
    if(mode & PWM) LPC_CT32B0->PWMC |= 0x1;
}
void Timer2Match1(unsigned int interval, unsigned char mode) {
    if(mode & INTERRUPT) mode |= 0x1;
    LPC_CT32B0->MR1 = interval;
    LPC_CT32B0->MCR &= ~(0x7 << 3);
    LPC_CT32B0->EMR &= ~(0x30 << 2);
    LPC_CT32B0->PWMC &= ~(0x1 << 1);
    LPC_CT32B0->MCR |= (mode & 0x7) << 3;
    if(mode & OUTPUTTOGGLE) {
        LPC_CT32B0->EMR |= (mode & 0x30) << 2;
        LPC_IOCON->PIO0_19 = 0x02;
        if(mode & OUTPUTLOW) LPC_CT32B0->EMR |= 1<<1;
        else if(mode & OUTPUTHIGH) LPC_CT32B0->EMR &= ~(0x1UL <<1);
    }
    if(mode & PWM) LPC_CT32B0->PWMC |= 0x1 << 1;
}
void Timer2Match2(unsigned int interval, unsigned char mode) {
    if(mode & INTERRUPT) mode |= 0x1;
    LPC_CT32B0->MR2 = interval;
    LPC_CT32B0->MCR &= ~(0x7 << 6);
    LPC_CT32B0->EMR &= ~(0x30 << 4);
    LPC_CT32B0->PWMC &= ~(0x1 << 2);
    LPC_CT32B0->MCR |= (mode & 0x7) << 6;
    if(mode & OUTPUTTOGGLE) {
        LPC_CT32B0->EMR |= (mode & 0x30) << 4;
        LPC_IOCON->PIO0_1 = 0x02;
        if(mode & OUTPUTLOW) LPC_CT32B0->EMR |= 1<<2;
        else if(mode & OUTPUTHIGH) LPC_CT32B0->EMR &= ~(0x1UL <<2);
    }
    if(mode & PWM) LPC_CT32B0->PWMC |= 0x1 << 2;
}
void Timer2Match3(unsigned int interval, unsigned char mode) {
    if(mode & INTERRUPT) mode |= 0x1;
    LPC_CT32B0->MR3 = interval;
    LPC_CT32B0->MCR &= ~(0x7 << 9);
    LPC_CT32B0->EMR &= ~(0x30 << 6);
    LPC_CT32B0->PWMC &= ~(0x1 << 3);
    LPC_CT32B0->MCR |= (mode & 0x7) << 9;
    if(mode & OUTPUTTOGGLE) {
        LPC_CT32B0->EMR |= (mode & 0x30) << 6;
        LPC_IOCON->TDI_PIO0_11 = 0x03;
        if(mode & OUTPUTLOW) LPC_CT32B0->EMR |= 1<<3;
        else if(mode & OUTPUTHIGH) LPC_CT32B0->EMR &= ~(0x1UL <<3);
    }
    if(mode & PWM) LPC_CT32B0->PWMC |= 0x1 << 3;
}
void Timer2Capture(unsigned char mode) {
    if(mode) {
        if(mode & COUNTER) {
            LPC_CT32B0->CTCR = mode & 0x3;
            LPC_CT32B0->TC = 0;
        }
        else {
            LPC_CT32B0->CTCR = 0;
            if(mode & (PWRESET | FALLING)) { // captures on falling edge, therefore resets on rising edge
                LPC_CT32B0->CTCR = (0x1 << 4);
            }
            else if(mode & (PWRESET | RISING)) { // captures on rising edge, therefore resets on falling edge
                LPC_CT32B0->CTCR = (0x1 << 4) | (0x1 << 5);
            }
            if(mode & INTERRUPT) mode |= 0x4;
            LPC_CT32B0->CCR = mode & 0x7;
        }
    
        LPC_IOCON->PIO0_17 &= ~(0x07);
        LPC_IOCON->PIO0_17 |= 0x02;
    }
}
void Timer3Match0(unsigned int interval, unsigned char mode) {
    if(mode & INTERRUPT) mode |= 0x1;
    LPC_CT32B1->MR0 = interval;
    LPC_CT32B1->MCR &= ~0x7;
    LPC_CT32B1->EMR &= ~0x30;
    LPC_CT32B1->PWMC &= ~0x1;
    LPC_CT32B1->MCR |= (mode & 0x7);
    if(mode & OUTPUTTOGGLE) {
        LPC_CT32B1->EMR |= (mode & 0x30);
        LPC_IOCON->TDO_PIO0_13 = 0x03;
        if(mode & OUTPUTLOW) LPC_CT32B1->EMR |= 1;
        else if(mode & OUTPUTHIGH) LPC_CT32B1->EMR &= ~1;
    }
    if(mode & PWM) LPC_CT32B1->PWMC |= 0x1;
}
void Timer3Match1(unsigned int interval, unsigned char mode) {
    if(mode & INTERRUPT) mode |= 0x1;
    LPC_CT32B1->MR1 = interval;
    LPC_CT32B1->MCR &= ~(0x7 << 3);
    LPC_CT32B1->EMR &= ~(0x30 << 2);
    LPC_CT32B1->PWMC &= ~(0x1 << 1);
    LPC_CT32B1->MCR |= (mode & 0x7) << 3;
    if(mode & OUTPUTTOGGLE) {
        LPC_CT32B1->EMR |= (mode & 0x30) << 2;
        LPC_IOCON->TRST_PIO0_14 = 0x03;
        if(mode & OUTPUTLOW) LPC_CT32B1->EMR |= 1<<1;
        else if(mode & OUTPUTHIGH) LPC_CT32B1->EMR &= ~(0x1UL <<1);
    }
    if(mode & PWM) LPC_CT32B1->PWMC |= 0x1 << 1;
}
void Timer3Match2(unsigned int interval, unsigned char mode) {
    if(mode & INTERRUPT) mode |= 0x1;
    LPC_CT32B1->MR2 = interval;
    LPC_CT32B1->MCR &= ~(0x7 << 6);
    LPC_CT32B1->EMR &= ~(0x30 << 4);
    LPC_CT32B1->PWMC &= ~(0x1 << 2);
    LPC_CT32B1->MCR |= (mode & 0x7) << 6;
    if(mode & OUTPUTTOGGLE) {
        LPC_CT32B1->EMR |= (mode & 0x30) << 4;
        LPC_IOCON->SWDIO_PIO0_15 = 0x03;
        if(mode & OUTPUTLOW) LPC_CT32B1->EMR |= 1<<2;
        else if(mode & OUTPUTHIGH) LPC_CT32B1->EMR &= ~(0x1UL <<2);
    }
    if(mode & PWM) LPC_CT32B1->PWMC |= 0x1 << 2;
}
void Timer3Match3(unsigned int interval, unsigned char mode) {
    if(mode & INTERRUPT) mode |= 0x1;
    LPC_CT32B1->MR3 = interval;
    LPC_CT32B1->MCR &= ~(0x7 << 9);
    LPC_CT32B1->EMR &= ~(0x30 << 6);
    LPC_CT32B1->PWMC &= ~(0x1 << 3);;
    LPC_CT32B1->MCR |= (mode & 0x7) << 9;
    if(mode & OUTPUTTOGGLE) {
        LPC_CT32B1->EMR |= (mode & 0x30) << 6;
        LPC_IOCON->PIO0_16 = 0x02;
        if(mode & OUTPUTLOW) LPC_CT32B1->EMR |= 1<<3;
        else if(mode & OUTPUTHIGH) LPC_CT32B1->EMR &= ~(0x1UL <<3);
    }
    if(mode & PWM) LPC_CT32B1->PWMC |= 0x1 << 3;
}
void Timer3Capture(unsigned char mode) {
    if(mode) {
        if(mode & COUNTER) {
            LPC_CT32B1->CTCR = mode & 0x3;
            LPC_CT32B1->TC = 0;
        }
        else {
            LPC_CT32B1->CTCR = 0;
            if(mode & (PWRESET | FALLING)) { // captures on falling edge, therefore resets on rising edge
                LPC_CT32B1->CTCR = (0x1 << 4);
            }
            else if(mode & (PWRESET | RISING)) { // captures on rising edge, therefore resets on falling edge
                LPC_CT32B1->CTCR = (0x1 << 4) | (0x1 << 5);
            }
            if(mode & INTERRUPT) mode |= 0x4;
            LPC_CT32B1->CCR = mode & 0x7;
        }
    
        LPC_IOCON->TMS_PIO0_12 &= ~(0x07);
        LPC_IOCON->TMS_PIO0_12 |= 0x03;
    }
}
void CT16B0_IRQHandler(void) {
    unsigned int interrupts = LPC_CT16B0->IR & 0x1f;
    if(Timer0Interrupt) Timer0Interrupt(LPC_CT16B0->IR & 0x1f);
    if(interrupts & 0x1 && Timer0Interrupt0) Timer0Interrupt0();
    if(interrupts & 0x2 && Timer0Interrupt1) Timer0Interrupt1();
    if(interrupts & 0x4 && Timer0Interrupt2) Timer0Interrupt2();
    if(interrupts & 0x8 && Timer0Interrupt3) Timer0Interrupt3();
    if(interrupts & 0x10 && Timer0InterruptC) Timer0InterruptC(LPC_CT16B0->CR0);
    LPC_CT16B0->IR = interrupts;
    __NOP(); __NOP();
}
void CT16B1_IRQHandler(void) {
    unsigned int interrupts = LPC_CT16B1->IR & 0x1f;
    if(Timer1Interrupt) Timer1Interrupt(LPC_CT16B1->IR & 0x1f);
    if(interrupts & 0x1 && Timer1Interrupt0) Timer1Interrupt0();
    if(interrupts & 0x2 && Timer1Interrupt1) Timer1Interrupt1();
    if(interrupts & 0x4 && Timer1Interrupt2) Timer1Interrupt2();
    if(interrupts & 0x8 && Timer1Interrupt3) Timer1Interrupt3();
    if(interrupts & 0x10 && Timer1InterruptC) Timer1InterruptC(LPC_CT16B1->CR0);
    LPC_CT16B1->IR = interrupts;
    __NOP(); __NOP();
}
void CT32B0_IRQHandler(void) {
    unsigned int interrupts = LPC_CT32B0->IR & 0x1f;
    if(Timer2Interrupt) Timer2Interrupt(LPC_CT32B0->IR & 0x1f);
    if(interrupts & 0x1 && Timer2Interrupt0) Timer2Interrupt0();
    if(interrupts & 0x2 && Timer2Interrupt1) Timer2Interrupt1();
    if(interrupts & 0x4 && Timer2Interrupt2) Timer2Interrupt2();
    if(interrupts & 0x8 && Timer2Interrupt3) Timer2Interrupt3();
    if(interrupts & 0x10 && Timer2InterruptC) Timer2InterruptC(LPC_CT32B0->CR0);
    LPC_CT32B0->IR = interrupts;
    __NOP(); __NOP();
}
void CT32B1_IRQHandler(void) {

    unsigned int interrupts = LPC_CT32B1->IR & 0x1f;
    if(Timer3Interrupt) Timer3Interrupt(interrupts);
    if(interrupts & 0x1 && Timer3Interrupt0) Timer3Interrupt0();
    if(interrupts & 0x2 && Timer3Interrupt1) Timer3Interrupt1();
    if(interrupts & 0x4 && Timer3Interrupt2) Timer3Interrupt2();
    if(interrupts & 0x8 && Timer3Interrupt3) Timer3Interrupt3();
    if(interrupts & 0x10 && Timer3InterruptC) Timer3InterruptC(LPC_CT32B1->CR0);
    LPC_CT32B1->IR = interrupts;
    __NOP(); __NOP();
}

// ****************************************************************************
// *** ADC Functions
// ****************************************************************************

// *** Initialise the ADC
void ADCInit(unsigned short channels) {
	if(channels) {
		if(channels & CHN0) LPC_IOCON->TDI_PIO0_11 = 0x02;            // Sets AD0 to Analogue mode if specified
		if(channels & CHN1) LPC_IOCON->TMS_PIO0_12 = 0x02;            // Sets AD1 to Analogue mode if specified
		if(channels & CHN2) LPC_IOCON->TDO_PIO0_13 = 0x02;             // Sets AD2 to Analogue mode if specified
		if(channels & CHN3) LPC_IOCON->TRST_PIO0_14 = 0x02;             // Sets AD3 to Analogue mode if specified
		if(channels & CHN4) LPC_IOCON->SWDIO_PIO0_15 = 0x02;            // Sets AD4 to Analogue mode if specified
		if(channels & CHN5) LPC_IOCON->PIO0_16 = 0x01;               // Sets AD5 to Analogue mode if specified
		if(channels & CHN6) LPC_IOCON->PIO0_22 = 0x01;              // Sets AD6 to Analogue mode if specified
		if(channels & CHN7) LPC_IOCON->PIO0_23 = 0x01;              // Sets AD7 to Analogue mode if specified
		
		LPC_SYSCON->SYSAHBCLKCTRL |= (0x1UL <<13);	// Enable clock to the ADC peripheral
		LPC_SYSCON->PDRUNCFG &= ~(0x1UL <<4);		// Enable power to the ADC peripheral
		
        // Set sample rates depending on clock speed, attempt to get as close to the 15.5MHz maximum ADC clock rate (12-bit) or 31MHz (10-bit) as possible
        if((LPC_SYSCON->MAINCLKSEL & 0x03) == 0x03) {
            // assume 72MHz operation
            #if ADC_10BIT
                LPC_ADC->CR = 0x0200;               // Sample at 24MHz clock, at 10bits
            #else
                LPC_ADC->CR = 0x0400;               // Sample at 14.4MHz clock, at 12bits
            #endif
        }
        else {
            // assume 12MHz operation
            LPC_ADC->CR = 0x0000;                   // Sample at 12MHz clock
        }
        
        #if ADC_LPWRMODE
            LPC_ADC->CR |= (0x1UL << 22);
        #else
            LPC_ADC->CR &= ~(0x1UL << 22);
        #endif
        
        #if ADC_10BIT
            LPC_ADC->CR |= (0x1UL << 23);
        #else
            LPC_ADC->CR &= ~(0x1UL << 23);
        #endif
        
        #if ADC_MODE
			// in interrupt mode, set up burst mode and global interrupts
            LPC_ADC->CR |= 0x010000 | channels;						// Burst mode
			LPC_ADC->INTEN = 0x100;									// Enable global interrupt
			
            IRQClear(ADC_IRQn);
            IRQPriority(ADC_IRQn, ADC_PRIORITY);
            IRQEnable(ADC_IRQn);
		#else
            // in on-demand mode, disable burst mode and interrupts
            LPC_ADC->INTEN = 0x000;									// Disable global interrupt
		#endif
	}
}

// *** Stop the ADC
void ADCStop(void) {
	IRQDisable(ADC_IRQn);
	LPC_ADC->CR = 0x000F00;					// Disable burst mode
	LPC_SYSCON->SYSAHBCLKCTRL &= ~(0x1UL <<13);	// Disable clock to the ADC peripheral
	LPC_SYSCON->PDRUNCFG |= (0x1UL <<4);		    // Disable power to the ADC peripheral
}

// *** Read ADC value
unsigned short ADCRead(unsigned char channel) {
	#if ADC_MODE
        // in interrupt mode, select channel and return last data value for that channel
        #if ADC_10BIT
            if(channel & CHN0) return ((LPC_ADC->DR[0] >> 6) & 0x3FF);
            else if(channel & CHN1) return ((LPC_ADC->DR[1] >> 6) & 0x3FF);
            else if(channel & CHN2) return ((LPC_ADC->DR[2] >> 6) & 0x3FF);
            else if(channel & CHN3) return ((LPC_ADC->DR[3] >> 6) & 0x3FF);
            else if(channel & CHN4) return ((LPC_ADC->DR[4] >> 6) & 0x3FF);
            else if(channel & CHN5) return ((LPC_ADC->DR[5] >> 6) & 0x3FF);
            else if(channel & CHN6) return ((LPC_ADC->DR[6] >> 6) & 0x3FF);
            else if(channel & CHN7) return ((LPC_ADC->DR[7] >> 6) & 0x3FF);
            else return 0xffff;
        #else
            if(channel & CHN0) return ((LPC_ADC->DR[0] >> 4) & 0xFFF);
            else if(channel & CHN1) return ((LPC_ADC->DR[1] >> 4) & 0xFFF);
            else if(channel & CHN2) return ((LPC_ADC->DR[2] >> 4) & 0xFFF);
            else if(channel & CHN3) return ((LPC_ADC->DR[3] >> 4) & 0xFFF);
            else if(channel & CHN4) return ((LPC_ADC->DR[4] >> 4) & 0xFFF);
            else if(channel & CHN5) return ((LPC_ADC->DR[5] >> 4) & 0xFFF);
            else if(channel & CHN6) return ((LPC_ADC->DR[6] >> 4) & 0xFFF);
            else if(channel & CHN7) return ((LPC_ADC->DR[7] >> 4) & 0xFFF);
            else return 0xffff;
        #endif
        
	#else
        // in on-demand mode, initialise a reading
        LPC_ADC->CR &= ~0x000000ff;
        LPC_ADC->CR |= 0x1000000 | channel;      // Start now!
        
		while(!(LPC_ADC->GDR & (0x1UL << 31)));			// Wait for ADC to complete
        #if ADC_10BIT
            return ((LPC_ADC->GDR >> 6) & 0x3FF);       // Return the ADC value
        #else
            return ((LPC_ADC->GDR >> 4) & 0xFFF);       // Return the ADC value
        #endif
	#endif
}

// *** ADC interrupt handler
#if ADC_MODE
	void ADC_IRQHandler(void) {
		unsigned int gdr;
		gdr = LPC_ADC->GDR;
        #if ADC_10BIT
            if(ADCInterrupt) ADCInterrupt(((gdr >> 24) & 0x7), ((gdr >> 6) & 0x3FF)); // If user-supplied handler is available, run it, passing data and channel
        #else
            if(ADCInterrupt) ADCInterrupt(((gdr >> 24) & 0x7), ((gdr >> 4) & 0xFFF));
        #endif
    }
#endif

// ****************************************************************************
// *** USB Functions
// ****************************************************************************

USBD_API_T* USBROM;
USBD_HANDLE_T hUsb;

void USBInit(void) {
    // Config USB Clock
    LPC_SYSCON->PDRUNCFG &= ~((1 << 10) | (1 <<  8));   // Enable power to the USB PHY and PLL
    LPC_SYSCON->USBPLLCLKSEL = 0x01;                    // Select PLL as clock source
    LPC_SYSCON->USBPLLCTRL = 0x23;                      // Select PLL divider to 4 (12Mhz - 48MHz)
    while (!(LPC_SYSCON->USBPLLSTAT & 0x01));           // Wait for PLL lock

    LPC_SYSCON->USBCLKSEL = 0x00;                       // Selec USB PLL input for USB Clock
    LPC_SYSCON->USBCLKDIV = 0x01;                       // Set USB clock divider
    
    // Configure USB pins
    LPC_SYSCON->SYSAHBCLKCTRL |= (1 << 14) | (1 << 27);  // Enable clock to USB and USB RAM

    LPC_IOCON->PIO0_3 = 0x01;   // VBUS
    LPC_IOCON->PIO0_6 = 0x01;   // Soft connect
    
    USBROM = (USBD_API_T*)((*(ROM **)(0x1FFF1FF8))->pUSBD);
}

// MSC
void MSCDummy(unsigned int address, unsigned char ** bufferAdr, unsigned int length) { return; }
signed int MSCDummy2(unsigned int address, unsigned char * buffer, unsigned int length) { return 0; }

void MSCRead(unsigned int address, unsigned char ** bufferAdr, unsigned int length) WEAK_ALIAS(MSCDummy);
void MSCWrite(unsigned int address, unsigned char ** bufferAdr, unsigned int length) WEAK_ALIAS(MSCDummy);
signed int MSCVerify(unsigned int address, unsigned char * buffer, unsigned int length) WEAK_ALIAS(MSCDummy2);

void MSCInit(unsigned int deviceCapacity) {
    USBD_API_INIT_PARAM_T usb_param;
    USB_CORE_DESCS_T desc;
    
    USBInit();
    
    desc.device_desc = MSC_DeviceDescriptor;
    desc.string_desc = MSC_StringDescriptor;
    desc.full_speed_desc = MSC_ConfigDescriptor;
    desc.high_speed_desc = MSC_ConfigDescriptor;
    desc.device_qualifier = 0;
    
    memset((void*)&usb_param, 0, sizeof(USBD_API_INIT_PARAM_T));
    usb_param.usb_reg_base = LPC_USB_BASE;
    usb_param.mem_base = 0x20004000;
    usb_param.mem_size = 0x800;
    //usb_param.mem_base = 0x10001000;
    //usb_param.mem_size = 0x1000;
    usb_param.max_num_ep = 2;

    USBROM->hw->Init(&hUsb, &desc, &usb_param);  
    
    USB_INTERFACE_DESCRIPTOR* pIntfDesc = (USB_INTERFACE_DESCRIPTOR *)&MSC_ConfigDescriptor[sizeof(USB_CONFIGURATION_DESCRIPTOR)];
    USBD_MSC_INIT_PARAM_T msc_param;

    memset((void*)&msc_param, 0, sizeof(USBD_MSC_INIT_PARAM_T));
    msc_param.mem_base = usb_param.mem_base;
    msc_param.mem_size = usb_param.mem_size;
    /* mass storage paramas */
    msc_param.InquiryStr = (uint8_t*)"UAirDisk"; 
    /*msc_param.BlockCount = deviceCapacity / MSC_BLOCK_SIZE;
    msc_param.MemorySize = deviceCapacity;*/
    
    #define MSC_MemorySize      ((uint32_t)(8 * 1024)) 
    #define MSC_BlockCount      (MSC_MemorySize / MSC_BLOCK_SIZE)
    msc_param.BlockCount = deviceCapacity / MSC_BLOCK_SIZE;
    msc_param.BlockSize = MSC_BLOCK_SIZE;
    msc_param.MemorySize = deviceCapacity;

    if ((pIntfDesc == 0) ||
    (pIntfDesc->bInterfaceClass != USB_DEVICE_CLASS_STORAGE) ||
    (pIntfDesc->bInterfaceSubClass != MSC_SUBCLASS_SCSI) )
    return;

    msc_param.intf_desc = (uint8_t*)pIntfDesc;
    /* user defined functions */
    msc_param.MSC_Write = (void(*)(uint32_t offset, uint8_t** src, uint32_t length))MSCWrite; 
    msc_param.MSC_Read = (void(*)(uint32_t offset, uint8_t** src, uint32_t length))MSCRead;
    msc_param.MSC_Verify = (ErrorCode_t (*)( uint32_t offset, uint8_t* buf, uint32_t length))MSCVerify;   

    USBROM->msc->init(hUsb, &msc_param);
    /* update memory variables */
    usb_param.mem_base = msc_param.mem_base;
    usb_param.mem_size = msc_param.mem_size;
  

    IRQClear(USB_IRQn);
    IRQPriority(USB_IRQn, USB_PRIORITY);
    IRQEnable(USB_IRQn);
    USBROM->hw->Connect(hUsb, 1);	
}

// HID
unsigned char * report_buffer;

void HIDDummy(unsigned char * buffer) { return; }
unsigned char HIDDummy2(unsigned char * buffer) { return 0; }

void HIDOutReport(unsigned char * buffer) WEAK_ALIAS(HIDDummy);
unsigned char HIDInReport(unsigned char * buffer) WEAK_ALIAS(HIDDummy2);
void HIDOutFeature(unsigned char * buffer) WEAK_ALIAS(HIDDummy);
unsigned char HIDInFeature(unsigned char * buffer) WEAK_ALIAS(HIDDummy2);

void HIDInit(void) {
    USBD_API_INIT_PARAM_T usb_param;
    USB_CORE_DESCS_T desc;
    
    USBInit();
    
    desc.device_desc = HID_DeviceDescriptor;
    desc.string_desc = HID_StringDescriptor;
    desc.full_speed_desc = HID_ConfigDescriptor;
    desc.high_speed_desc = HID_ConfigDescriptor;
    desc.device_qualifier = 0;

    memset((void*)&usb_param, 0, sizeof(USBD_API_INIT_PARAM_T));
    usb_param.usb_reg_base = LPC_USB_BASE;
    usb_param.mem_base = 0x20004000;
    usb_param.mem_size = 0x800;
    usb_param.max_num_ep = 2;
    usb_param.USB_Configure_Event = USB_Configure_Event;  // HID only
    
    USBROM->hw->Init(&hUsb, &desc, &usb_param);  

    USB_INTERFACE_DESCRIPTOR* pIntfDesc = (USB_INTERFACE_DESCRIPTOR *)&HID_ConfigDescriptor[sizeof(USB_CONFIGURATION_DESCRIPTOR)];
    USBD_HID_INIT_PARAM_T hid_param;
    USB_HID_REPORT_T reports_data[1];

    memset((void*)&hid_param, 0, sizeof(USBD_HID_INIT_PARAM_T));
    /* HID paramas */
    hid_param.max_reports = 1;
    /* Init reports_data */
    reports_data[0].len = HID_ReportDescSize;
    reports_data[0].idle_time = 0;
    reports_data[0].desc = (uint8_t *)&HID_ReportDescriptor[0];

    if ((pIntfDesc == 0) || (pIntfDesc->bInterfaceClass != USB_DEVICE_CLASS_HUMAN_INTERFACE)) return;

    hid_param.mem_base = usb_param.mem_base;
    hid_param.mem_size = usb_param.mem_size;
    hid_param.intf_desc = (uint8_t*)pIntfDesc;
    /* user defined functions */
    hid_param.HID_GetReport = HID_GetReport;
    hid_param.HID_SetReport = HID_SetReport;
    hid_param.HID_EpIn_Hdlr  = HID_Ep_Hdlr;
    hid_param.HID_EpOut_Hdlr = HID_Ep_Hdlr;
    hid_param.report_data  = reports_data;

    USBROM->hid->init(hUsb, &hid_param);
    /* allocate USB accessable memory space for report data */
    report_buffer =  (unsigned char*)hid_param.mem_base;
    hid_param.mem_base += 4;
    hid_param.mem_size += 4;
    /* update memory variables */
    usb_param.mem_base = hid_param.mem_base;
    usb_param.mem_size = hid_param.mem_size;

    IRQClear(USB_IRQn);
    IRQPriority(USB_IRQn, USB_PRIORITY);
    IRQEnable(USB_IRQn);
    USBROM->hw->Connect(hUsb, 1);	
}

ErrorCode_t USB_Configure_Event (USBD_HANDLE_T hUsb) {
    USB_CORE_CTRL_T* pCtrl = (USB_CORE_CTRL_T*)hUsb;
    if (pCtrl->config_value) {                   /* Check if USB is configured */
        USBROM->hw->WriteEP(hUsb, HID_EP_IN, report_buffer, 1);
    }
    return LPC_OK;
}

ErrorCode_t HID_GetReport( USBD_HANDLE_T hHid, USB_SETUP_PACKET* pSetup, uint8_t** pBuffer, uint16_t* plength) {
    switch (pSetup->wValue.WB.H) {
        case HID_REPORT_FEATURE:
            *pBuffer = report_buffer;
            if(HIDInFeature(report_buffer)) {
                *plength = HID_FEATURE_BYTES;
            }
            else {
                *plength = 0;
            }
        break;
        case HID_REPORT_INPUT:              /* Not Supported */
        case HID_REPORT_OUTPUT:             /* Not Supported */
            return (ERR_USBD_STALL);
    }
    return (LPC_OK);
}

ErrorCode_t HID_SetReport( USBD_HANDLE_T hHid, USB_SETUP_PACKET* pSetup, uint8_t** pBuffer, uint16_t length) {
    
    if (length == 0) return LPC_OK;

    switch (pSetup->wValue.WB.H) {
        case HID_REPORT_FEATURE:
            HIDOutFeature(*pBuffer);
            break;
        case HID_REPORT_INPUT:              /* Not Supported */
        case HID_REPORT_OUTPUT:             /* Not Supported */
            return (ERR_USBD_STALL);
    }
    return (LPC_OK);
}

ErrorCode_t HID_Ep_Hdlr (USBD_HANDLE_T hUsb, void* data, uint32_t event) {
    USB_HID_CTRL_T* pHidCtrl = (USB_HID_CTRL_T*)data;

    switch (event) {
        case USB_EVT_IN:
            if(HIDInReport(report_buffer)) {
                USBROM->hw->WriteEP(hUsb, pHidCtrl->epin_adr, report_buffer, HID_IN_BYTES);
            }
            else {
                USBROM->hw->WriteEP(hUsb, pHidCtrl->epin_adr, report_buffer, 0);
            }
            break;
        case USB_EVT_OUT:
            USBROM->hw->ReadEP(hUsb, pHidCtrl->epout_adr, report_buffer);
            HIDOutReport(report_buffer);
            break;
    }
    return LPC_OK;
}

// CDC
VCOM_DATA_T g_vCOM;
unsigned char VCOM_isbridge;

void CDCInit(unsigned char bridge) {
    USBD_API_INIT_PARAM_T usb_param;
    USBD_CDC_INIT_PARAM_T cdc_param;
    USB_CORE_DESCS_T desc;
    USBD_HANDLE_T hCdc;
    uint32_t ep_indx;
    
    USBInit();
    
    VCOM_isbridge = bridge;
    
    desc.device_desc = CDC_DeviceDescriptor;
    desc.string_desc = CDC_StringDescriptor;
    desc.full_speed_desc = CDC_ConfigDescriptor;
    desc.high_speed_desc = CDC_ConfigDescriptor;
    desc.device_qualifier = 0;
    
    memset((void*)&usb_param, 0, sizeof(USBD_API_INIT_PARAM_T));
    usb_param.usb_reg_base = LPC_USB_BASE;
    usb_param.mem_base = 0x10001000;
    usb_param.mem_size = 0x800;
    // usb_param.mem_base = 0x20004000;
    // usb_param.mem_size = 0x800;
    usb_param.max_num_ep = 3;
    
    /* init CDC params */
    memset((void*)&cdc_param, 0, sizeof(USBD_CDC_INIT_PARAM_T));

    /* user defined functions */
    cdc_param.SetLineCode = VCOM_SetLineCode; 
    usb_param.USB_SOF_Event = VCOM_sof_event; 
    cdc_param.SendBreak = VCOM_SendBreak;

    /* USB Initialization */
    USBROM->hw->Init(&hUsb, &desc, &usb_param);  

    // init CDC params
    cdc_param.mem_base = usb_param.mem_base;
    cdc_param.mem_size = usb_param.mem_size;
    cdc_param.cif_intf_desc = (uint8_t *)&CDC_ConfigDescriptor[USB_CONFIGUARTION_DESC_SIZE];
    cdc_param.dif_intf_desc = (uint8_t *)&CDC_ConfigDescriptor[USB_CONFIGUARTION_DESC_SIZE + USB_INTERFACE_DESC_SIZE + 0x0013 + USB_ENDPOINT_DESC_SIZE ];

    USBROM->cdc->init(hUsb, &cdc_param, &hCdc);

    /* store USB handle */
    memset((void*)&g_vCOM, 0, sizeof(VCOM_DATA_T));
    g_vCOM.hUsb = hUsb;
    g_vCOM.hCdc = hCdc;
    g_vCOM.send_fn = VCOM_usb_send;

    /* allocate transfer buffers */
    g_vCOM.rxBuf = (uint8_t*)(cdc_param.mem_base + (0 * USB_MAX_BULK_PACKET));
    g_vCOM.txBuf = (uint8_t*)(cdc_param.mem_base + (1 * USB_MAX_BULK_PACKET));
    cdc_param.mem_size -= (4 * USB_MAX_BULK_PACKET);

    /* register endpoint interrupt handler */
    ep_indx = (((USB_CDC_EP_BULK_IN & 0x0F) << 1) + 1);
    USBROM->core->RegisterEpHandler (hUsb, ep_indx, VCOM_bulk_in_hdlr, &g_vCOM);

    /* register endpoint interrupt handler */
    ep_indx = ((USB_CDC_EP_BULK_OUT & 0x0F) << 1);
    USBROM->core->RegisterEpHandler (hUsb, ep_indx, VCOM_bulk_out_hdlr, &g_vCOM);

    /* enable IRQ */
    IRQPriority(USB_IRQn, USB_PRIORITY);
    IRQEnable(USB_IRQn); //  enable USB0 interrrupts 
    g_vCOM.send_fn = VCOM_uart_send;
    
    if(VCOM_isbridge) {
        /* init UART for bridge */
        VCOM_init_bridge(&g_vCOM, 0);
        /* enable IRQ */
        IRQPriority(USART_IRQn, UART_PRIORITY);
        IRQEnable(USART_IRQn); //  enable Uart interrrupt
        
        #if CDC_USE_PLED
            LEDInit(PLED);
        #endif
    }
    /* USB Connect */
    USBROM->hw->Connect(hUsb, 1);
}

void VCOM_usb_send(VCOM_DATA_T* pVcom) {
    /* data received send it back */
    pVcom->txlen -= USBROM->hw->WriteEP (pVcom->hUsb, USB_CDC_EP_BULK_IN, pVcom->txBuf, pVcom->txlen);   
}

void VCOM_init_bridge(VCOM_DATA_T* pVcom, CDC_LINE_CODING* line_coding) {
    uint32_t Fdiv, baud = 9600;
    uint8_t  lcr = 0x3;    	/* 8 bits, no Parity, 1 Stop bit */

    if(line_coding) {
        if(line_coding->bCharFormat) {
            lcr |= (1 << 2);                 /* Number of stop bits */
        }
        if(line_coding->bParityType) { /* Parity bit type */
            lcr |= (1 << 3);
            lcr |=  (((line_coding->bParityType - 1) & 0x3) << 4);
        }
        if(line_coding->bDataBits) {
            lcr |= ((line_coding->bDataBits - 5) & 0x3);
        }
        else {
            lcr |= 0x3;
        }
        baud = line_coding->dwDTERate;
        /* enable SOF after we are connected */
        USBROM->hw->EnableEvent(pVcom->hUsb, 0, USB_EVT_SOF, 1);
    }
    else {
        /* Enable UART clock */
        LPC_SYSCON->SYSAHBCLKCTRL |= (1<<12);
        LPC_SYSCON->UARTCLKDIV = 0x1;     /* divided by 1 */

        LPC_IOCON->PIO0_18 &= ~0x07;    /*  UART I/O config */
        LPC_IOCON->PIO0_18 |= 0x01;     /* UART RXD */
        LPC_IOCON->PIO0_19 &= ~0x07;	
        LPC_IOCON->PIO0_19 |= 0x01;     /* UART TXD */
        
        #if UART_FLOW
            LPC_IOCON->PIO0_7 = 0x11;               // #CTS
            LPC_IOCON->PIO0_17 = 0x11;               // #RTS
            LPC_USART->MCR |= (0x1UL <<6) | (0x1UL <<7);
        #endif
    } 

    Fdiv = ( (72000000/LPC_SYSCON->UARTCLKDIV) / 16 ) / baud ;	/*baud rate */
    LPC_USART->IER = 0;
    LPC_USART->LCR = lcr | 0x80; /* DLAB = 1 */		
    LPC_USART->DLM = Fdiv / 256;							
    LPC_USART->DLL = Fdiv % 256;
    LPC_USART->FDR = 0x10; // reset fractional baud generator
    LPC_USART->LCR = lcr;		/* DLAB = 0 */
    LPC_USART->FCR = 0x07;    /* Enable and reset TX and RX FIFO. 
                                      Rx trigger level 4 chars*/
    LPC_USART->IER = 0x07;	/* Enable UART1 interrupt */
}

void VCOM_uart_write(VCOM_DATA_T* pVcom) {
    uint8_t *pbuf = pVcom->rxBuf;
    uint32_t tx_cnt =  16;
    /* find space in TX fifo */
    tx_cnt = 0xF - (tx_cnt & 0xF);

    if (tx_cnt > (pVcom->rxlen - pVcom->ser_pos)) {
        tx_cnt = (pVcom->rxlen - pVcom->ser_pos);
    }

    while(tx_cnt) {
        if(LPC_USART->LSR & 0x20) {
            LPC_USART->THR = pbuf[pVcom->ser_pos++];
            tx_cnt--;
        }
    }

    /* if done check anything pending */
    if (pVcom->ser_pos == pVcom->rxlen) {
        /* Tx complete free the buffer */
        pVcom->ser_pos = 0;
        pVcom->rxlen = 0;
        if(pVcom->usbrx_pend) {
            pVcom->usbrx_pend = 0;
            VCOM_bulk_out_hdlr(pVcom->hUsb, (void*)pVcom, USB_EVT_OUT);
        }
    }
    return;
}

void VCOM_uart_read(VCOM_DATA_T* pVcom) {
    uint8_t *pbuf;

    pbuf = pVcom->txBuf;

    if( (LPC_USART->LSR & 0x01) && (pVcom->txlen < USB_MAX_BULK_PACKET) ) { 
        pbuf[pVcom->txlen++] = LPC_USART->RBR;
    } 

    if (pVcom->txlen == USB_MAX_BULK_PACKET) {
        VCOM_usb_send(pVcom);
    }
    pVcom->last_ser_rx = pVcom->sof_counter;
}

void CDCWriteByte(unsigned char byte) {
    VCOM_DATA_T* pVcom = &g_vCOM;

    if(pVcom->txlen < USB_MAX_BULK_PACKET) {
        pVcom->txBuf[pVcom->txlen++] = byte;
    }

    if (pVcom->txlen == USB_MAX_BULK_PACKET) {
        VCOM_usb_send(pVcom);
    }
    pVcom->last_ser_rx = pVcom->sof_counter;
}

void VCOM_uart_send(VCOM_DATA_T* pVcom) {
    unsigned int tx_cnt;
    unsigned char *pbuf = pVcom->rxBuf;
    
    if(VCOM_isbridge) {
        /* data received on USB send it to UART */
        #if CDC_USE_PLED
            LEDOn(PLED);
        #endif
        VCOM_uart_write(pVcom);
    }
    else {
        tx_cnt = (pVcom->rxlen - pVcom->ser_pos);
        
        while(tx_cnt) {
            CDCReadByte(pbuf[pVcom->ser_pos++]);
            tx_cnt--;
        }
        
        pVcom->ser_pos = 0;
        pVcom->rxlen = 0;
        if(pVcom->usbrx_pend) {
            pVcom->usbrx_pend = 0;
            VCOM_bulk_out_hdlr(pVcom->hUsb, (void*)pVcom, USB_EVT_OUT);
        }
    }
}


void CDCWrite(unsigned char * byte, unsigned int length) {
    unsigned int i;
    for(i=0; i<length; i++) {
        CDCWriteByte(byte[i]);
    }
}

void CDCDummy(unsigned char byte) { return; }
void CDCReadByte(unsigned char byte) WEAK_ALIAS(CDCDummy);

ErrorCode_t VCOM_SetLineCode (USBD_HANDLE_T hCDC, CDC_LINE_CODING* line_coding) {
    VCOM_DATA_T* pVcom = &g_vCOM;
    //int i;
    /* baud rate change reset buffers */
    pVcom->ser_pos = 0;
    pVcom->rxlen = pVcom->txlen = 0;
    if(VCOM_isbridge) VCOM_init_bridge(pVcom, line_coding);
    else if(line_coding) USBROM->hw->EnableEvent(pVcom->hUsb, 0, USB_EVT_SOF, 1);

    return LPC_OK;
}

ErrorCode_t VCOM_sof_event(USBD_HANDLE_T hUsb) {
    VCOM_DATA_T* pVcom = &g_vCOM;
    uint8_t lcr;
    uint32_t diff = pVcom->sof_counter - pVcom->last_ser_rx;

    pVcom->sof_counter++;

    if (pVcom->break_time) {
        pVcom->break_time--;
        if (pVcom->break_time == 0) {
            if(VCOM_isbridge) {
                lcr = LPC_USART->LCR;
                if (lcr & (1 << 6)) {
                    lcr &= ~(1 << 6);
                    LPC_USART->LCR = lcr;
                }
            }
        }
    }

    if ( pVcom->last_ser_rx && (diff > 5)) {
        VCOM_usb_send(pVcom);
    }

    return LPC_OK;
}


ErrorCode_t VCOM_SendBreak(USBD_HANDLE_T hCDC, uint16_t mstime) {
    VCOM_DATA_T* pVcom = &g_vCOM;
    uint8_t lcr;

    if(VCOM_isbridge) {
        lcr = LPC_USART->LCR;
        if ( mstime) {
            lcr |= (1 << 6);
        } else {
            lcr &= ~(1 << 6);
        }
        
        LPC_USART->LCR = lcr;
    }

    pVcom->break_time = mstime;

    return LPC_OK;
}

ErrorCode_t VCOM_bulk_in_hdlr(USBD_HANDLE_T hUsb, void* data, uint32_t event) {
  return LPC_OK;
}

ErrorCode_t VCOM_bulk_out_hdlr(USBD_HANDLE_T hUsb, void* data, uint32_t event) {
    VCOM_DATA_T* pVcom = (VCOM_DATA_T*) data;

    if(event == USB_EVT_OUT) {
        if (pVcom->rxlen == 0) {
        pVcom->rxlen = USBROM->hw->ReadEP(hUsb, USB_CDC_EP_BULK_OUT, pVcom->rxBuf);
        pVcom->send_fn(pVcom);
        }
        else {
        /* indicate bridge write buffer pending in USB buf */
        pVcom->usbrx_pend = 1;
        }
    }
    return LPC_OK;
}

void USB_IRQHandler(void) {
    USBROM->hw->ISR(hUsb);
}

// ****************************************************************************
// *** LED Functions
// ****************************************************************************

volatile unsigned char FUNCLEDStatus;

unsigned char RSTPoll(void) {
    unsigned char result;
    result = RSTRead();
    Port0SetOut(PIN0);
    LEDWrite(RLED, FUNCLEDStatus & RLED);
    return result;
}

unsigned char PRGPoll(void) {
    unsigned char result;
    result = PRGRead();
    Port0SetOut(PIN1);
    LEDWrite(PLED, FUNCLEDStatus & PLED);
    return result;
}

// ****************************************************************************
// *** UAir Interlink Functions
// ****************************************************************************

#if ILINK_EN & SSP0_EN
    volatile unsigned char FUNCILinkState;
    volatile unsigned short FUNCILinkID, FUNCILinkChecksumA, FUNCILinkChecksumB, FUNCILinkLength, FUNCILinkPacket;
    unsigned short FUNCILinkRxBuffer[ILINK_RXBUFFER_SIZE];
    
    void ILinkInit(unsigned short speed) {
        SSP0Init(speed);
        FUNCILinkState = 0;
        FUNCILinkTxBufferBusy = 0;
    }
    
    void ILinkPoll(unsigned short message) {
        unsigned short tempBufferA, tempBufferB, tempBufferC, tempBufferD;
        
        SSP0S0SEL();
        tempBufferA = SSP0Byte(0xec41);
        SSP0S0CLR();
        SSP0S0SEL();
        tempBufferB = SSP0Byte(0x13be);
        SSP0S0CLR();
        SSP0S0SEL();
        tempBufferC = SSP0Byte(message);
        SSP0S0CLR();
        SSP0S0SEL();
        tempBufferD = SSP0Byte(0);
        SSP0S0CLR();
        
        ILinkProcess(tempBufferA);
        ILinkProcess(tempBufferB);
        ILinkProcess(tempBufferC);
        ILinkProcess(tempBufferD);
    }
    
    void ILinkProcess(unsigned short data) {
        switch(FUNCILinkState) {
            default: // fall through to case 0
            case 0:  // search for 0xec41 sync characters
                if(data == 0xec41) FUNCILinkState = 1;
                break;
            case 1:  // search for 0xec41 sync characters
                if(data == 0x13be) FUNCILinkState = 2;
                else FUNCILinkState = 0;
                break;
            case 2:  // read the ID
                FUNCILinkID = data;
                FUNCILinkChecksumA = data;
                FUNCILinkChecksumB = FUNCILinkChecksumA;
                FUNCILinkState = 3;
                break;
            case 3:  // read the packet length
                FUNCILinkLength = data;
                FUNCILinkChecksumA += data;
                FUNCILinkChecksumB += FUNCILinkChecksumA;
                
                if(FUNCILinkLength >= ILINK_RXBUFFER_SIZE) FUNCILinkState = 0;
                else if(FUNCILinkLength > 0) FUNCILinkState = 4;
                else { // special case for zero-length packet
                    if(ILinkMessageRequest) ILinkMessageRequest(FUNCILinkID);
                    FUNCILinkState = 0;
                }
                
                FUNCILinkPacket = 0;
                break;
            case 4:  // read a byte of payload
                FUNCILinkRxBuffer[FUNCILinkPacket++] = data;
                
                if(FUNCILinkPacket >= FUNCILinkLength) FUNCILinkState = 5;
                
                FUNCILinkChecksumA += data;
                FUNCILinkChecksumB += FUNCILinkChecksumA;
                break;
            case 5:  // check first checksum
                if(data == FUNCILinkChecksumA) FUNCILinkState = 6;
                else {
                    if(ILinkMessageError) ILinkMessageError(FUNCILinkID);
                    FUNCILinkState = 0;
                }
                break;
            case 6:  // check second checksum
                if(data == FUNCILinkChecksumB) {
                    if(ILinkMessage) ILinkMessage(FUNCILinkID, FUNCILinkRxBuffer, FUNCILinkLength);
                }
                else {
                    if(ILinkMessageError) ILinkMessageError(FUNCILinkID);
                }
                FUNCILinkState = 0;
                break;
        }
    }
    
    void ILinkFetchData(void) {
        unsigned char idle = 0;
        unsigned int count = ILINK_MAX_FETCH;
        unsigned short data;
        while(idle < 4 && count-- > 0) {
            if(ILinkReadable()) {
                SSP0S0SEL();
                data = SSP0Byte(ILinkPop());
                SSP0S0CLR();
                
                ILinkProcess(data);
            }
            else {
                SSP0S0SEL();
                data = SSP0Byte(0xffff);
                SSP0S0CLR();

                ILinkProcess(data);
                if(FUNCILinkState == 0) idle++;
            }
        }
    }
    
    
    unsigned char ILinkSendMessage(unsigned short id, unsigned short * buffer, unsigned short length) {
        unsigned short value;
        unsigned int j;
        unsigned short chkA, chkB;
        
        if(FUNCILinkTxBufferBusy == 0) {
            FUNCILinkTxBufferBusy = 1;
            if(ILinkWritable() > length+6) {
                ILinkPush(0xec41);
                ILinkPush(0x13be);
                ILinkPush(id);
                ILinkPush(length);
                chkA = id + length;
                chkB = chkA + id;
                
                for(j=0; j<length; j++) {
                    value = buffer[j];
                    ILinkPush(value);
                    chkA += value;
                    chkB += chkA;
                }
                ILinkPush(chkA);
                ILinkPush(chkB);
                FUNCILinkTxBufferBusy = 0;
                return 1;
            }
			FUNCILinkTxBufferBusy = 0;
        }
        
        return 0;
    }
    
    //#if ILINK_EN == 2
        unsigned short FUNCILinkTxBuffer[ILINK_TXBUFFER_SIZE];
        unsigned int FUNCILinkTxBufferBusy;
        volatile unsigned short FUNCILinkTxBufferPushPtr, FUNCILinkTxBufferPopPtr;

        unsigned short ILinkWritable(void) {
            if(FUNCILinkTxBufferPushPtr < FUNCILinkTxBufferPopPtr) return FUNCILinkTxBufferPopPtr - FUNCILinkTxBufferPushPtr - 1;
            else return ILINK_TXBUFFER_SIZE + FUNCILinkTxBufferPopPtr - FUNCILinkTxBufferPushPtr - 1;
        }
        unsigned short ILinkReadable(void) {
            if(FUNCILinkTxBufferPushPtr < FUNCILinkTxBufferPopPtr) return ILINK_TXBUFFER_SIZE + FUNCILinkTxBufferPushPtr - FUNCILinkTxBufferPopPtr;
            else return FUNCILinkTxBufferPushPtr - FUNCILinkTxBufferPopPtr;
        }
        unsigned short ILinkPop(void) {
            unsigned short retval;
            retval = FUNCILinkTxBuffer[FUNCILinkTxBufferPopPtr++];
            if(FUNCILinkTxBufferPopPtr >= ILINK_TXBUFFER_SIZE) FUNCILinkTxBufferPopPtr = 0;
            return retval;
        }
        void ILinkPush(unsigned short data) {
            FUNCILinkTxBuffer[FUNCILinkTxBufferPushPtr++] = data;
            if(FUNCILinkTxBufferPushPtr >= ILINK_TXBUFFER_SIZE) FUNCILinkTxBufferPushPtr = 0;
        }

        void SSP0Interrupt(unsigned short data) {
            while(ILinkReadable() && (LPC_SSP0->SR & 0x02)) {
                SSP0NextByte(ILinkPop());
            }
            ILinkProcess(data);
        }
    //#endif
#endif


#if WHO_AM_I == I_AM_THALAMUS

    // ****************************************************************************
    // *** ULTRA Functions (Thalamus only)
    // ****************************************************************************
    
    volatile unsigned char FUNCUltraNewData;
    volatile unsigned short FUNCUltraValue;
    volatile unsigned char FUNCUltraOutOfRange;
    volatile unsigned char FUNCUltraUnderRange;
    volatile unsigned char FUNCUltraFastRate;

     
    unsigned char UltraInit(void) {
        Port0SetOut(PIN7);
        Port0Write(PIN7, 0);
        
        Timer1Init(71); // runs at 1us
        Timer1Match0(10, INTERRUPT); // Pulse interrupt
        Timer1Match1(6666, INTERRUPT); // 
        Timer1Match3(12000, RESET | INTERRUPT); // overrun interrupt
        Timer1Capture(FALLING | PWRESET | INTERRUPT); // step6: set capture interrupt for falling edge of echo;
        FUNCUltraFastRate = 0;
        return 1;
    }
    
    unsigned short UltraGetData(void) {
        if(FUNCUltraOutOfRange) return 0;
        else return (FUNCUltraValue * 170)/1000;
    }
	
	
    unsigned short UltraGetRawData(void) {
        if(FUNCUltraOutOfRange) return 0;
        else return FUNCUltraValue;
    }
	
	unsigned short UltraGetNewRawData(void) {
        if(FUNCUltraNewData) {
            FUNCUltraNewData = 0;
			if(FUNCUltraOutOfRange) return 0;
			else return FUNCUltraValue;
        }
        else return 0;
	}
	
    unsigned short UltraGetNewData(void) {
        if(FUNCUltraNewData) {
            FUNCUltraNewData = 0;
			if(FUNCUltraOutOfRange) return 0;
			else return (FUNCUltraValue * 170)/1000;
        }
        else return 0;
    }

    void Timer1InterruptC(unsigned short timerValue) {
        if(FUNCUltraOutOfRange) {
            FUNCUltraOutOfRange = 0;
        }
        else {
			FUNCUltraValue = timerValue;
            FUNCUltraNewData = 1;
            if(FUNCUltraFastRate) {
                if(timerValue < 6500) {
                    FUNCUltraUnderRange = 1;
                }
                else {
                    FUNCUltraUnderRange = 0;
                    Port0Write(PIN7, 1);
                    Timer1Reset();
                    Timer1Go();
                }
            }
        }
    }
    void Timer1Interrupt0(void) {
        Port0Write(PIN7, 0);
    }

    void Timer1Interrupt1(void) {
        if(FUNCUltraUnderRange) {
            FUNCUltraUnderRange = 0;
            Port0Write(PIN7, 1);
            Timer1Reset();
            Timer1Go();
        }
    }

    void Timer1Interrupt3(void) {
        if(Port0Read(PIN20)) { // if signal is high, then Ultrasound timeout/out of range
            FUNCUltraOutOfRange = 1;
        }
        
        FUNCUltraUnderRange = 0;
        Port0Write(PIN7, 1);
    }
    
    // ****************************************************************************
    // *** RX Functions (Thalamus)
    // ****************************************************************************

    volatile unsigned char FUNCRXCount;
    volatile unsigned char FUNCRXLastByte;
    volatile unsigned char FUNCRXChannel;
    volatile unsigned char FUNCRXNewData;
    volatile unsigned char FUNCRXStatus;
    #if RX_TYPE == 0
    unsigned short FUNCRXChanBuffer[7];
    unsigned short FUNCRXChan[7];
    #else
    unsigned short FUNCRXChanBuffer[18];
    unsigned short FUNCRXChan[18];
    #endif

    void RXUARTInit(void) {
        IRQDisable(USART_IRQn);

        // Enable the pins
        LPC_SYSCON->SYSAHBCLKCTRL |= (0x1UL <<12);     // Enable clock to IOCON block and UART block
        #if RX_TYPE == 0
            LPC_IOCON->PIO0_18 = 0x11;                   // Rx
        #else
            LPC_IOCON->PIO0_18 = 0x49;                   // Rx
        #endif

        // Set clock settings
        LPC_SYSCON->UARTCLKDIV = 0x01;              // Clock divider at 1

        LPC_USART->LCR = 0x80;                       // enable access to divisor (clock) latches
        if((LPC_SYSCON->MAINCLKSEL & 0x03) == 0x03) {
            #if RX_TYPE == 0
                LPC_USART->DLM = 0;
                LPC_USART->DLL = 0x17;
                LPC_USART->FDR = 0xa7;
            #else
                LPC_USART->DLM = 0;
                LPC_USART->DLL = 0x15;
                LPC_USART->FDR = 0x78;
            #endif
        }
        else {
            LPC_USART->DLM = 0;
            LPC_USART->DLL = 0x6;
            LPC_USART->FDR = 0xc1;
        }

        // Data format 
        #if RX_TYPE == 0
            LPC_USART->LCR = 0x3;
        #else
            LPC_USART->LCR = 0x1f;  // disable access to divisor latches, and set data format
        #endif
            
        LPC_USART->IER = 0x05;
        LPC_USART->FCR = 0x06;
        LPC_USART->FCR = 0x01;
        LPC_USART->SCR = LPC_USART->LSR;
        while (LPC_USART->LSR & 0x01)   LPC_USART->SCR = LPC_USART->RBR;
        LPC_USART->TER = 0x80;

        IRQClear(USART_IRQn);
        IRQPriority(USART_IRQn, UART_PRIORITY);
        IRQEnable(USART_IRQn);
    }
    
    void RXInit(void) {
        Port0Init(PIN17 | PIN18);
        Port0SetOut(PIN17 | PIN18); // PIN17 is power pin to PNP base, PIN18 is RX pin
        Port0Write(PIN17 | PIN18, 0);
        
        Delay(500);
        RXUARTInit();
        
		WDTInit(5);
    }
    
    void RXBind(void) {
        UARTStop();
        
        Port0Init(PIN17 | PIN18);
        Port0SetOut(PIN17 | PIN18); // PIN17 is power pin to PNP base, PIN18 is RX pin
        
        Port0Write(PIN17, 1); // Power off
        Delay(500);
        Port0Write(PIN18, 1); // RX pin high during power on to go into bind mode        
        Port0Write(PIN17, 0); // Power On
        
        Delay(100); // Delay after Rx startup is needed
        
        // Three low pulses to go into Master bind mode
        Port0Write(PIN18, 0); RXDelay(); Port0Write(PIN18, 1); RXDelay();
        Port0Write(PIN18, 0); RXDelay(); Port0Write(PIN18, 1); RXDelay();
        Port0Write(PIN18, 0); RXDelay(); Port0Write(PIN18, 1); RXDelay();

        FUNCRXCount = 0;
        FUNCRXNewData = 0;
        
        RXUARTInit();
    }

    void RXDelay(void) {
        volatile unsigned int i;
        for(i=0; i<600; i++);
    }
	
	void RXWDTInterrupt(void) {
		FUNCRXCount = 0;
	}
    
    void RXUARTParityError(void) {
        FUNCRXStatus = 0;
    }
    
    unsigned char RXProcess(unsigned char RXByte) {
        #if RX_TYPE == 0
            switch(FUNCRXCount) {
                case 0:
                    if(RXByte == 0x03) FUNCRXCount++;
                    break;
                case 1:
                    //if(RXByte == 0x01) FUNCRXCount++;   // Not clear what this sync byte does, we've gotten 0x01 on DX6e and DX5e, and 0x29 and 0xa2 for DX4e
                    //else FUNCRXCount = 0;
                    FUNCRXCount++;
                    break;
                default:
                    if(FUNCRXCount % 2 == 0) {   // even bytes
                        FUNCRXLastByte = RXByte;
                        FUNCRXChannel = (RXByte >> 2) & 0x7;
                    }
                    else {  // odd bytes
                        FUNCRXChanBuffer[FUNCRXChannel] = (FUNCRXLastByte & 0x3) << 8 | RXByte;
                    }

                    FUNCRXCount++;
                    if(FUNCRXCount >= 16) {
                        FUNCRXCount = 0;
                        FUNCRXNewData = 1;
                        FUNCRXChan[0] = FUNCRXChanBuffer[0];
                        FUNCRXChan[1] = FUNCRXChanBuffer[1];
                        FUNCRXChan[2] = FUNCRXChanBuffer[2];
                        FUNCRXChan[3] = FUNCRXChanBuffer[3];
                        FUNCRXChan[4] = FUNCRXChanBuffer[4];
                        FUNCRXChan[5] = FUNCRXChanBuffer[5];
                        FUNCRXChan[6] = FUNCRXChanBuffer[6];
                        return 1;
                    }
            }
            return 0;
        #else
            switch(FUNCRXCount) {
                case 0:
                    if(RXByte == 0x0f) {
                        FUNCRXStatus = 1;
                        FUNCRXCount++;
                    }
                    break;
                case 1:
                    FUNCRXChanBuffer[0] = RXByte;
                    FUNCRXCount++;
                    break;
                case 2:
                    FUNCRXChanBuffer[0] |= RXByte << 8;
                    FUNCRXChanBuffer[1] = RXByte >> 3;
                    FUNCRXCount++;
                    break;
                case 3:
                    FUNCRXChanBuffer[1] |= RXByte << 5;
                    FUNCRXChanBuffer[2] = RXByte >> 6;
                    FUNCRXCount++;
                    break;
                case 4:
                    FUNCRXChanBuffer[2] |= RXByte << 2;
                    FUNCRXCount++;
                    break;
                case 5:
                    FUNCRXChanBuffer[2] |= RXByte << 10;
                    FUNCRXChanBuffer[3] = RXByte >> 1;
                    FUNCRXCount++;
                    break;
                case 6:
                    FUNCRXChanBuffer[3] |= RXByte << 7;
                    FUNCRXChanBuffer[4] = RXByte >> 4;
                    FUNCRXCount++;
                    break;
                case 7:
                    FUNCRXChanBuffer[4] |= RXByte << 4;
                    FUNCRXChanBuffer[5] = RXByte >> 7;
                    FUNCRXCount++;
                    break;
                case 8:
                    FUNCRXChanBuffer[5] |= RXByte << 1;
                    FUNCRXCount++;
                    break;
                case 9:
                    FUNCRXChanBuffer[5] |= RXByte << 9;
                    FUNCRXChanBuffer[6] = RXByte >> 2;
                    FUNCRXCount++;
                    break;
                case 10:
                    FUNCRXChanBuffer[6] |= RXByte << 6;
                    FUNCRXChanBuffer[7] = RXByte >> 5;
                    FUNCRXCount++;
                    break;
                case 11:
                    FUNCRXChanBuffer[7] |= RXByte << 3;
                    FUNCRXCount++;
                    break;
                case 12:
                    FUNCRXChanBuffer[8] = RXByte;
                    FUNCRXCount++;
                    break;
                case 13:
                    FUNCRXChanBuffer[8] |= RXByte << 8;
                    FUNCRXChanBuffer[9] = RXByte >> 3;
                    FUNCRXCount++;
                    break;
                case 14:
                    FUNCRXChanBuffer[9] |= RXByte << 5;
                    FUNCRXChanBuffer[10] = RXByte >> 6;
                    FUNCRXCount++;
                    break;
                case 15:
                    FUNCRXChanBuffer[10] |= RXByte << 2;
                    FUNCRXCount++;
                    break;
                case 16:
                    FUNCRXChanBuffer[10] |= RXByte << 10;
                    FUNCRXChanBuffer[11] = RXByte >> 1;
                    FUNCRXCount++;
                    break;
                case 17:
                    FUNCRXChanBuffer[11] |= RXByte << 7;
                    FUNCRXChanBuffer[12] = RXByte >> 4;
                    FUNCRXCount++;
                    break;
                case 18:
                    FUNCRXChanBuffer[12] |= RXByte << 4;
                    FUNCRXChanBuffer[13] = RXByte >> 7;
                    FUNCRXCount++;
                    break;
                case 19:
                    FUNCRXChanBuffer[13] |= RXByte << 1;
                    FUNCRXCount++;
                    break;
                case 20:
                    FUNCRXChanBuffer[13] |= RXByte << 9;
                    FUNCRXChanBuffer[14] = RXByte >> 2;
                    FUNCRXCount++;
                    break;
                case 21:
                    FUNCRXChanBuffer[14] |= RXByte << 6;
                    FUNCRXChanBuffer[15] = RXByte >> 5;
                    FUNCRXCount++;
                    break;
                case 22:
                    FUNCRXChanBuffer[15] |= RXByte << 3;
                    FUNCRXCount++;
                    break;
                case 23:
                    FUNCRXChanBuffer[16] = RXByte;
                    FUNCRXChanBuffer[17] = RXByte >> 1;
                    if(!(RXByte & 0x0C)) {
                        FUNCRXStatus = 2;
                    }
                    FUNCRXCount++;
                    break;
                case 24:
                    if(RXByte == 0x00 && FUNCRXStatus == 2) {
                        unsigned int i;
                        FUNCRXNewData = 1;
                        for(i=0; i<16; i++) FUNCRXChan[i] = FUNCRXChanBuffer[i] & 0x7ff;
                        FUNCRXChan[16] = FUNCRXChanBuffer[16] & 0x1;
                        FUNCRXChan[17] = FUNCRXChanBuffer[17] & 0x1;
                    }
                    FUNCRXCount = 0;
                    break;
                default:
                    FUNCRXCount = 0;
                    break;
            }
            return 0;
        #endif
    }

    unsigned char RXGetData(unsigned short * RXChannels) {
        unsigned int i;
        
        if(FUNCRXNewData) {
            #if RX_TYPE == 0
            for(i=0; i<7; i++) RXChannels[i] = FUNCRXChan[i];
            #else
            for(i=0; i<16; i++) RXChannels[i] = FUNCRXChan[i];
            RXChannels[16] = FUNCRXChan[16];
            RXChannels[17] = FUNCRXChan[17];
            #endif
            FUNCRXNewData = 0;
            return 1;
        }
        else return 0;
    }

    void RXUARTInterrupt(unsigned char UARTData){
        #if RX_TYPE == 0
            RXProcess(UARTData);
        #else
            unsigned int out;
            __asm volatile ("rev %0, %1" : "=r" (out) : "r" (UARTData) );
            __asm volatile ("rbit %0, %1" : "=r" (out) : "r" (out) );
            RXProcess(UARTData & 0xff);
        #endif
		WDTFeed();
    }


    // ****************************************************************************
    // *** PWM Ouput Functions (Thalamus)
    // ****************************************************************************

    volatile unsigned char FUNCPWMPostscale;
	volatile unsigned short FUNCPWMN_duty;
	volatile unsigned short FUNCPWME_duty;
	volatile unsigned short FUNCPWMS_duty;
	volatile unsigned short FUNCPWMW_duty;
	volatile unsigned short FUNCPWMX_duty;
	volatile unsigned short FUNCPWMY_duty;
	
    #if PWM_FILTERS_ON == 1
        volatile unsigned int FUNCPWMN_fil;
        volatile unsigned int FUNCPWME_fil;
        volatile unsigned int FUNCPWMS_fil;
        volatile unsigned int FUNCPWMW_fil;
        volatile unsigned int FUNCPWMX_fil;
        volatile unsigned int FUNCPWMY_fil;
    #endif

    void PWMInit(unsigned char channels) {
        Timer2Init(71); // set to 1uS
        Timer3Init(71); // set to 1uS

        Timer2Match0(1000000/PWM_NESWFREQ, INTERRUPT);
        
        #if PWM_FILTERS_ON == 1
            FUNCPWMN_fil = PWM_DEFAULT_N;
            FUNCPWME_fil = PWM_DEFAULT_E;
            FUNCPWMS_fil = PWM_DEFAULT_S;
            FUNCPWMW_fil = PWM_DEFAULT_W;
            FUNCPWMX_fil = PWM_DEFAULT_X;
            FUNCPWMW_fil = PWM_DEFAULT_Y;
        #endif
		
		FUNCPWMN_duty = PWM_DEFAULT_N;
		FUNCPWME_duty = PWM_DEFAULT_E;
		FUNCPWMS_duty = PWM_DEFAULT_S;
		FUNCPWMW_duty = PWM_DEFAULT_W;
		FUNCPWMX_duty = PWM_DEFAULT_X;
		FUNCPWMY_duty = PWM_DEFAULT_Y;
        
        if(channels & PWM_S) {  Timer3Match3(PWM_DEFAULT_S, OUTPUTLOW);  }
        if(channels & PWM_E) {  Timer3Match1(PWM_DEFAULT_E, OUTPUTLOW);  }
        if(channels & PWM_N) {  Timer3Match0(PWM_DEFAULT_N, OUTPUTLOW);  }
        if(channels & PWM_W) {  Timer3Match2(PWM_DEFAULT_W, OUTPUTLOW);  }
        if(channels & PWM_X) {  Timer2Match3(PWM_DEFAULT_X, OUTPUTLOW);  }
        if(channels & PWM_Y) {  Timer2Match1(PWM_DEFAULT_Y, OUTPUTLOW);  }
        
        FUNCPWMPostscale = 0;
    }

    void Timer2Interrupt0() {
		Timer3SetMatch3(FUNCPWMS_duty);
		Timer3SetMatch1(FUNCPWME_duty);
		Timer3SetMatch0(FUNCPWMN_duty);
		Timer3SetMatch2(FUNCPWMW_duty);
		
        if(FUNCPWMPostscale++ > PWM_NESWFREQ/PWM_XYFREQ) {
			Timer2SetMatch3(FUNCPWMX_duty);
			Timer2SetMatch1(FUNCPWMY_duty);
			
            Timer2SetStatus(MATCH1 | MATCH3);
            FUNCPWMPostscale = 0;
        }
        Timer3SetStatus(MATCH0 | MATCH1 | MATCH2 | MATCH3);
		
        Timer2Reset();
        Timer2Go();
        Timer3Reset();
        Timer3Go();
    }

    // ****************************************************************************
    // *** IMU Functions (Thalamus)
    // ****************************************************************************
	volatile unsigned short FUNCBaro_C1, FUNCBaro_C2, FUNCBaro_C3,FUNCBaro_C4, FUNCBaro_C5, FUNCBaro_C6;
	volatile signed long long FUNCBaro_sensitivity, FUNCBaro_offset;
	volatile unsigned char FUNCBaro_type;
    void SensorInit(void) {
        unsigned char I2CBuffer[5];
        
        I2CInit(400);
       
	   
        // *** Barometer
		// detect which barometer is available
		// NOTE: seems to be that you need to reset the MEAS baro early on otherwise it just doesn't work.
		FUNCBaro_type = 0;
		
		I2CBuffer[0] = BARO_LPS_ADDR;
        I2CBuffer[1] = 0x0f;     // WHOAMI register
		I2CBuffer[2] = BARO_LPS_ADDR | 0x1;
        if(I2CMaster(I2CBuffer, 2, I2CBuffer, 1)) {
			if(I2CBuffer[0] == 0xbb) {
				I2CBuffer[0] = BARO_LPS_ADDR;
				I2CBuffer[1] = 0x10;     // Pressure resolution mode
				I2CBuffer[2] = ((BARO_LPS_TEMP_AVERAGING & 0x7) << 4) | (BARO_LPS_PRES_AVERAGING & 0xf); // oversampling settings
				I2CMaster(I2CBuffer, 3, 0, 0);

				I2CBuffer[0] = BARO_LPS_ADDR;
				I2CBuffer[1] = 0x20;
				I2CBuffer[2] = ((BARO_LPS_RATE & 0x7) << 4) | 0x04; // Output data rate, and block data update
				I2CMaster(I2CBuffer, 3, 0, 0);
				
				I2CBuffer[0] = BARO_LPS_ADDR;
				I2CBuffer[1] = 0x20 + 0x80;    // Ctrl register start location
				I2CBuffer[2] = 0x80 | ((BARO_LPS_RATE & 0x7) << 4) | 0x04; // Power on on separate operation
				I2CMaster(I2CBuffer, 3, 0, 0);
				
				FUNCBaro_type = 1;
			}
		}
		
		I2CBuffer[0] = BARO_MS_ADDR;
		I2CBuffer[1] = 0x1e;     // factory reset
		I2CMaster(I2CBuffer, 2, 0, 0);
		
		Delay(3);
		
		I2CBuffer[0] = BARO_MS_ADDR;
		I2CBuffer[1] = 0xa0;     // PROM byte
		I2CBuffer[2] = BARO_MS_ADDR | 1;
		if(I2CMaster(I2CBuffer, 2, I2CBuffer, 2)) {
			unsigned int n_rem = 0;
			n_rem = BaroCrc4(n_rem, I2CBuffer[0]);
			n_rem = BaroCrc4(n_rem, I2CBuffer[1]);
			
			I2CBuffer[0] = BARO_MS_ADDR;
			I2CBuffer[1] = 0xa2;     // PROM bye
			I2CBuffer[2] = BARO_MS_ADDR | 1;
			I2CMaster(I2CBuffer, 2, I2CBuffer, 2);
			FUNCBaro_C1 = (I2CBuffer[0] << 8) | I2CBuffer[1];
			n_rem = BaroCrc4(n_rem, I2CBuffer[0]);
			n_rem = BaroCrc4(n_rem, I2CBuffer[1]);
			
			I2CBuffer[0] = BARO_MS_ADDR;
			I2CBuffer[1] = 0xa4;     // PROM bye
			I2CBuffer[2] = BARO_MS_ADDR | 1;
			I2CMaster(I2CBuffer, 2, I2CBuffer, 2);
			FUNCBaro_C2 = (I2CBuffer[0] << 8) | I2CBuffer[1];
			n_rem = BaroCrc4(n_rem, I2CBuffer[0]);
			n_rem = BaroCrc4(n_rem, I2CBuffer[1]);
			
			I2CBuffer[0] = BARO_MS_ADDR;
			I2CBuffer[1] = 0xa6;     // PROM bye
			I2CBuffer[2] = BARO_MS_ADDR | 1;
			I2CMaster(I2CBuffer, 2, I2CBuffer, 2);
			FUNCBaro_C3 = (I2CBuffer[0] << 8) | I2CBuffer[1];
			n_rem = BaroCrc4(n_rem, I2CBuffer[0]);
			n_rem = BaroCrc4(n_rem, I2CBuffer[1]);
			
			I2CBuffer[0] = BARO_MS_ADDR;
			I2CBuffer[1] = 0xa8;     // PROM bye
			I2CBuffer[2] = BARO_MS_ADDR | 1;
			I2CMaster(I2CBuffer, 2, I2CBuffer, 2);
			FUNCBaro_C4 = (I2CBuffer[0] << 8) | I2CBuffer[1];
			n_rem = BaroCrc4(n_rem, I2CBuffer[0]);
			n_rem = BaroCrc4(n_rem, I2CBuffer[1]);
			
			I2CBuffer[0] = BARO_MS_ADDR;
			I2CBuffer[1] = 0xaa;     // PROM bye
			I2CBuffer[2] = BARO_MS_ADDR | 1;
			I2CMaster(I2CBuffer, 2, I2CBuffer, 2);
			FUNCBaro_C5 = (I2CBuffer[0] << 8) | I2CBuffer[1];
			n_rem = BaroCrc4(n_rem, I2CBuffer[0]);
			n_rem = BaroCrc4(n_rem, I2CBuffer[1]);
			
			I2CBuffer[0] = BARO_MS_ADDR;
			I2CBuffer[1] = 0xac;     // PROM bye
			I2CBuffer[2] = BARO_MS_ADDR | 1;
			I2CMaster(I2CBuffer, 2, I2CBuffer, 2);
			FUNCBaro_C6 = (I2CBuffer[0] << 8) | I2CBuffer[1];
			n_rem = BaroCrc4(n_rem, I2CBuffer[0]);
			n_rem = BaroCrc4(n_rem, I2CBuffer[1]);
			
			I2CBuffer[0] = BARO_MS_ADDR;
			I2CBuffer[1] = 0xae;     // PROM bye
			I2CBuffer[2] = BARO_MS_ADDR | 1;
			I2CMaster(I2CBuffer, 2, I2CBuffer, 2);
			n_rem = BaroCrc4(n_rem, I2CBuffer[0]);
			n_rem = BaroCrc4(n_rem, 0);
			
			n_rem = (0x000F & (n_rem >> 12)); // // final 4-bit reminder is CRC code
			if(n_rem == (I2CBuffer[1] & 0x0f)) {
				FUNCBaro_type = 2;
			}
		}
		
		
		 // *** Accelerometer
        I2CBuffer[0] = ACCEL_ADDR;
        I2CBuffer[1] = 0x20 + 0x80; // Control Register CTRL_REG1_A
        I2CBuffer[2] = ((ACCEL_RATE & 0xf) << 4) | ((ACCEL_LOW_POWER & 0x1) << 3) | 0x7; // XYZ axis enable
        I2CMaster(I2CBuffer, 3, 0, 0);
      
        I2CBuffer[0] = ACCEL_ADDR;
        I2CBuffer[1] = 0x23 + 0x80; // Control Register CTRL_REG4_A
        I2CBuffer[2] = 0x80 | ((ACCEL_RANGE & 0x3) << 4) | (((~ACCEL_LOW_POWER) & 0x1) << 3);
        I2CMaster(I2CBuffer, 3, 0, 0);
        
		#if ACCEL_FIFO_EN
			I2CBuffer[0] = ACCEL_ADDR;
			I2CBuffer[1] = 0x24 + 0x80; // Control Register CTRL_REG5_A
			I2CBuffer[2] = 0x40; // FIFO enable
			I2CMaster(I2CBuffer, 3, 0, 0);
			
			I2CBuffer[0] = ACCEL_ADDR;
			I2CBuffer[1] = 0x2e + 0x80; // Control Register FIFO_CTRL_REG_A
			I2CBuffer[2] = 0x40; // FIFO mode
			I2CMaster(I2CBuffer, 3, 0, 0);
		#endif
        
        // *** Gyroscope
        I2CBuffer[0] = GYRO_ADDR;
        I2CBuffer[1] = 0x20 + 0x80; // Control Register CTRL_REG1_G
        I2CBuffer[2] = ((GYRO_RATE & 0x3) << 6) | ((GYRO_BANDWIDTH & 0x3) << 4) | 0xf; // XYZ axis enable
        I2CMaster(I2CBuffer, 3, 0, 0);

        I2CBuffer[0] = GYRO_ADDR;
        I2CBuffer[1] = 0x23 + 0x80; // Control Register CTRL_REG4_G
        I2CBuffer[2] = 0x80 | ((GYRO_RANGE & 0x3) << 4); // Set dynamic range
        I2CMaster(I2CBuffer, 3, 0, 0);
        
        I2CBuffer[0] = GYRO_ADDR;
        I2CBuffer[1] = 0x24 + 0x80; // Control Register CTRL_REG5_G
        I2CBuffer[2] = ((GYRO_LPF & 0x1) << 1) ; // Set the secondary low pass filter
        I2CMaster(I2CBuffer, 3, 0, 0);
        
        // *** Magneto
        I2CBuffer[0] = MAGNETO_ADDR;
        I2CBuffer[1] = 0x00;    // Config address start location
        I2CBuffer[2] = ((MAGNETO_AVERAGING & 0x3) << 5) | ((MAGNETO_RATE & 0x7) << 2) | (MAGNETO_BIAS & 0x3); // Configuration Register A
        I2CBuffer[3] = (MAGNETO_GAIN & 0x7) << 5; // Configuration Register B
        I2CBuffer[4] = MAGNETO_MODE & 0x3; // Mode Register
        I2CMaster(I2CBuffer, 5, 0, 0);
		
    }

	unsigned int BaroCrc4(unsigned int n_rem, unsigned char byte) {
		unsigned char i;
		n_rem ^= (unsigned short) byte;
		for (i=0; i<8; i++){
			if (n_rem & 0x8000) {
				n_rem = (n_rem << 1) ^ 0x3000;
			}
			else {
				n_rem = (n_rem << 1);
			}
		}
		return n_rem;
	}
	
    unsigned char GetAccel(signed short * data) {
        unsigned char I2CBuffer[6];
        unsigned char * ptr; // mess with pointers to shoehorn chars into signed short array
        ptr = (unsigned char *) data;
        
        I2CBuffer[0] = ACCEL_ADDR;
        I2CBuffer[1] = 0x28 + 0x80;    // Data register start
        I2CBuffer[2] = ACCEL_ADDR | 1;
        if(I2CMaster(I2CBuffer, 2, I2CBuffer, 6)){
            ptr[0] = I2CBuffer[0]; // Thalamus X LSB 
            ptr[1] = I2CBuffer[1]; // Thalamus X MSB
            
            ptr[2] = I2CBuffer[4]; // Thalamus Y LSB
            ptr[3] = I2CBuffer[5]; // Thalamus Y MSB
            
            ptr[4] = I2CBuffer[2]; // Thalamus Z LSB
            ptr[5] = I2CBuffer[3]; // Thalamus Z MSB
            
            data[0] = -data[0];
			data[1] = -data[1];
			
            return 1;
        }
        else return 0;
    }

    unsigned char GetGyro(signed short * data) {
        unsigned char I2CBuffer[8];
        unsigned char * ptr; // mess with pointers to shoehorn chars into signed short array
        char temp;
        ptr = (unsigned char *) data;
        
        I2CBuffer[0] = GYRO_ADDR;
        I2CBuffer[1] = 0x26 + 0x80;    // Data register start
        I2CBuffer[2] = GYRO_ADDR | 1;
        if(I2CMaster(I2CBuffer, 2, I2CBuffer, 8)) {
            ptr[0] = I2CBuffer[2]; // Thalamus X LSB 
            ptr[1] = I2CBuffer[3]; // Thalamus X MSB
            
            ptr[2] = I2CBuffer[6]; // Thalamus Y LSB
            ptr[3] = I2CBuffer[7]; // Thalamus Y MSB
            
            ptr[4] = I2CBuffer[4]; // Thalamus Z LSB
            ptr[5] = I2CBuffer[5]; // Thalamus Z MSB
            data[2] = -data[2];
            
            temp = I2CBuffer[0]; // Temperature
            data[3] = (signed short) temp;
            
            return 1;
        }
        else return 0;
    }
    
    unsigned char GetMagneto(signed short * data) {
        unsigned char I2CBuffer[6];
        unsigned char * ptr; // mess with pointers to shoehorn chars into signed short array
        ptr = (unsigned char *) data;

        #if MAGNETO_MODE
            unsigned char tries, length= 0;
            
            I2CBuffer[0] = MAGNETO_ADDR;
            I2CBuffer[1] = 0x02;    // Mode Register
            I2CBuffer[2] = 0x1;     // Set to start Single-Measurement Mode
            I2CMaster(I2CBuffer, 5, 0, 0);
            
            tries = 0;
            while(tries++ < MAGNETO_TRIES_MAX) {
                I2CBuffer[0] = MAGNETO_ADDR;
                I2CBuffer[1] = 0x09;	    // Contains the RDY flag
                I2CBuffer[2] = MAGNETO_ADDR | 1;
                if(I2CMaster(I2CBuffer, 2, I2CBuffer, 1)) {
                    if(I2CBuffer[0] & 0x01) {
                        length = 1;
                        break;
                    }
                    else Delay(MAGNETO_TRIES_DELAY);
                }
                else {
                    break;
                }
            }

            if(length > 0) {
        #endif
        
        I2CBuffer[0] = MAGNETO_ADDR;
        I2CBuffer[1] = 0x03;    // data Register start
        I2CBuffer[2] = MAGNETO_ADDR | 1;
        if(I2CMaster(I2CBuffer, 2, I2CBuffer, 6)) {
            
            ptr[0] = I2CBuffer[1]; // Seraphim X LSB 
            ptr[1] = I2CBuffer[0]; // Seraphim X MSB
            
            ptr[2] = I2CBuffer[3]; // Seraphim Y LSB
            ptr[3] = I2CBuffer[2]; // Seraphim Y MSB
            
            ptr[4] = I2CBuffer[5]; // Seraphim Z LSB
            ptr[5] = I2CBuffer[4]; // Seraphim Z MSB
            data[2] = -data[2];
            
            return 1;
        }
        else return 0;
        
        #if MAGNETO_MODE        
            }
            return 0;
        #endif
    }
    
	void TrigBaro(void) {
		unsigned char I2CBuffer[5];
		
		if(FUNCBaro_type == 2) {
			I2CBuffer[0] = BARO_MS_ADDR;
			I2CBuffer[1] = 0x40 | ((BARO_MS_OSR & 0x7) << 1);     // trigger D1
			I2CMaster(I2CBuffer, 2, 0, 0);
		}
	}

	void TrigBaroTemp(void) {
		unsigned char I2CBuffer[5];
		
		if(FUNCBaro_type == 2) {
			I2CBuffer[0] = BARO_MS_ADDR;
			I2CBuffer[1] = 0x50 | ((BARO_MS_OSR & 0x7) << 1);     // trigger D2
			I2CMaster(I2CBuffer, 2, 0, 0);
		}
	}
	
    unsigned int GetBaro(void) {
        unsigned char I2CBuffer[5];
		
		if(FUNCBaro_type == 1) {
			I2CBuffer[0] = BARO_LPS_ADDR;
			I2CBuffer[1] = 0x28 + 0x80;    // Temperature data
			I2CBuffer[2] = BARO_LPS_ADDR | 1;
			if(I2CMaster(I2CBuffer, 2, I2CBuffer, 3)) {
				return (unsigned int) ((I2CBuffer[2] << 16) | (I2CBuffer[1] << 8) | I2CBuffer[0]);
			}
			else return 0;
		}
		else if(FUNCBaro_type == 2) {
			I2CBuffer[0] = BARO_MS_ADDR;
			I2CBuffer[1] = 0x00;    // ADC data
			I2CBuffer[2] = BARO_MS_ADDR | 1;
			if(I2CMaster(I2CBuffer, 2, I2CBuffer, 3)) {
				return (unsigned int) ((I2CBuffer[0] << 16) | (I2CBuffer[1] << 8) | I2CBuffer[2]);
			}
			else return 0;
		}
		else return 0;
    }
    
    float GetBaroPressure(void) { // in Pa
        unsigned int reading;
		
		if(FUNCBaro_type == 1) {
			reading = GetBaro();
			if(((reading & 0x800000) == 1) || (reading == 0)) return 0; // bit numer 24 is the sign bit, shouldn't be 1 (signifying negative value) because we're not using delta pressure
			else return (float)reading/40.96f;
		}
		else if(FUNCBaro_type == 2) {
			reading = GetBaro();
			if(reading == 0) return 0;
			else {
				float P = ((((signed long long)reading * FUNCBaro_sensitivity ) >> 21) - FUNCBaro_offset) / (float) (1 << 15);
				return P;
			}
		}
		
		return 0;
    }
    
    float Pressure2Alt(float pressure) { // in m
        return ((float)101325-pressure) * 0.0832546913138f; // max error around 25m, linearise around sea level
    }
    
    float GetBaroTemp(void) { // in degrees C
        unsigned char I2CBuffer[5];
        signed short reading;
        
		if(FUNCBaro_type == 1) {
			I2CBuffer[0] = BARO_LPS_ADDR;
			I2CBuffer[1] = 0x2B + 0x80;    // Temperature data
			I2CBuffer[2] = BARO_LPS_ADDR | 1;
			I2CMaster(I2CBuffer, 2, I2CBuffer, 2);
			
			reading = (signed short) ((I2CBuffer[1] << 8) | I2CBuffer[0]);
			
			return (float)42.5 + (float)reading/(float)480;
		}
		else if(FUNCBaro_type == 2) {
			I2CBuffer[0] = BARO_MS_ADDR;
			I2CBuffer[1] = 0x00;    // ADC data
			I2CBuffer[2] = BARO_MS_ADDR | 1;
			if(I2CMaster(I2CBuffer, 2, I2CBuffer, 3)) {
				unsigned int D2 = ((I2CBuffer[0] << 16) | (I2CBuffer[1] << 8) | I2CBuffer[2]);
				signed long long dT = D2 - ((unsigned long long)FUNCBaro_C5 << 8);
				float T = (2000 + (((unsigned long long)dT * FUNCBaro_C6) / (float)(1 << 23))) / 100;
				
				FUNCBaro_offset = ((unsigned int)FUNCBaro_C2 << 16) + ((dT * (FUNCBaro_C4) >> 7));
				FUNCBaro_sensitivity = ((unsigned int)FUNCBaro_C1 << 15) + ((dT * (FUNCBaro_C3) >> 8));
			 /*
				if(T < 20) {
					signed int TEMP = 2000 + (signed long)dT * (signed long)FUNCBaro_C6 / (signed long)(1 << 23);
					float T2 = (TEMP - 2000) * (TEMP - 2000);
					signed long OFF2  = (5 * T2) / 2;
					signed long SENS2 = (5 * T2) / 4;
			 
					if(T < -15) { // if temperature lower than -15 Celsius
						T2 = (TEMP + 1500) * (TEMP + 1500);
						OFF2  += 7 * T2;
						SENS2 += 11 * T2 / 2;
					} 
					FUNCBaro_offset -= OFF2;
					FUNCBaro_sensitivity -= SENS2;
					T = (float)TEMP / 100; 
				}*/

				return T;
			}
			else return 0;
		}
		
		return 0;
    }
    
    void SensorSleep(void) {
        // Magneto sleep
        unsigned char I2CBuffer[3];

        I2CBuffer[0] = ACCEL_ADDR;
        I2CBuffer[1] = 0x20; // Control Register CTRL_REG1_A
        I2CBuffer[2] = ((ACCEL_RATE & 0xf) << 4) | ((ACCEL_LOW_POWER & 0x1) << 3) | 0x0; // XYZ axis disable
        I2CMaster(I2CBuffer, 3, 0, 0);
        
        I2CBuffer[0] = GYRO_ADDR;
        I2CBuffer[1] = 0x20; // Control Register CTRL_REG1_G
        I2CBuffer[2] = ((GYRO_RATE & 0x3) << 6) | ((GYRO_BANDWIDTH & 0x3) << 4) | 0x8; // XYZ axis disable
        I2CMaster(I2CBuffer, 3, 0, 0);
        
        I2CBuffer[0] = MAGNETO_ADDR;
        I2CBuffer[1] = 0x02;    // Mode Register
        I2CBuffer[2] = 0x02;    // Idle mode
        I2CMaster(I2CBuffer, 3, 0, 0);
        
		if(FUNCBaro_type == 1) {
			I2CBuffer[0] = BARO_LPS_ADDR;
			I2CBuffer[1] = 0x20;    // CTRL_REG1
			I2CBuffer[2] = 0x00;    // Power down
			I2CMaster(I2CBuffer, 3, 0, 0);
		}
    }

#endif

#if WHO_AM_I == I_AM_HYPO

    // ****************************************************************************
    // *** PWM Ouput Functions (Hypo)
	// TODO: fix PWM a la Thalamus
    // ****************************************************************************

    volatile unsigned char FUNCPWMPostscale;
	
    #if PWM_FILTERS_ON == 1
        volatile unsigned int FUNCPWMA_fil;
        volatile unsigned int FUNCPWMB_fil;
        volatile unsigned int FUNCPWMC_fil;
        volatile unsigned int FUNCPWMD_fil;
        volatile unsigned int FUNCPWMS_fil;
    #endif

    void PWMInit(unsigned char channels) {
        Timer2Init(71); // set to 1uS
        Timer3Init(71); // set to 1uS

        Timer2Match0(1000000/PWM_ABCDFREQ, INTERRUPT);
        
        #if PWM_FILTERS_ON == 1
            FUNCPWMA_fil = PWM_DEFAULT_A;
            FUNCPWMB_fil = PWM_DEFAULT_B;
            FUNCPWMC_fil = PWM_DEFAULT_C;
            FUNCPWMD_fil = PWM_DEFAULT_D;
            FUNCPWMS_fil = PWM_DEFAULT_S;
        #endif
        
        if(channels & PWM_A) {  Timer3Match3(PWM_DEFAULT_A, OUTPUTLOW);  }
        if(channels & PWM_B) {  Timer3Match1(PWM_DEFAULT_B, OUTPUTLOW);  }
        if(channels & PWM_C) {  Timer3Match0(PWM_DEFAULT_C, OUTPUTLOW);  }
        if(channels & PWM_D) {  Timer3Match2(PWM_DEFAULT_D, OUTPUTLOW);  }
        if(channels & PWM_S) {  Timer2Match3(PWM_DEFAULT_S, OUTPUTLOW);  }
        
        
        FUNCPWMPostscale = 0;
    }

    void Timer2Interrupt0() {
        Timer2Reset();
        Timer2Go();
        Timer3Reset();
        Timer3Go();
        
        if(FUNCPWMPostscale++ > PWM_ABCDFREQ/PWM_SFREQ) {
            Timer2SetStatus(MATCH1 | MATCH3);
            FUNCPWMPostscale = 0;
        }
        Timer3SetStatus(MATCH0 | MATCH1 | MATCH2 | MATCH3);
    }

    // ****************************************************************************
    // *** GPS Functions (Hypo)
    // ****************************************************************************

    #if GPS_EN
        void GPSInit(void) {
            unsigned char UBX_CFG_RST[13] = { GPS_ADDR, 0xb5, 0x62, 0x06, 0x04, 0x04, 0x00, 0xff, 0xff, 0x09, 0x00, 0x15, 0x6f}; // CFG-PM2 Power management config: set wake up
            unsigned char UBX_CFG_PRT[29] = { GPS_ADDR, 0xb5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 0x84, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0xa0, 0x96 }; // CFG-PRT Port config: sets DDC slave address to 0x84 (0x42 in 7 bit), sets UBX protocol only.
            unsigned char UBX_CFG_TPS[41] = { GPS_ADDR, 0xb5, 0x62, 0x06, 0x31, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x32, 0x00, 0x00, 0x00, 0xc0, 0xc6, 0x2d, 0x00, 0x20, 0xa1, 0x07, 0x00, 0xa0, 0x86, 0x01, 0x00, 0xa0, 0x86, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf7, 0x00, 0x00, 0x00, 0x49, 0xEC }; // CFG-TPS Timepulse config: three second pulses when not locked, half pulse when locked
            
            #if GPS_5HZ_RATE
                unsigned char UBX_CFG_RATE[15] = { GPS_ADDR, 0xb5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xc8, 0x00, 0x01, 0x00, 0x01, 0x00, 0xde, 0x6a }; // CFG-RATE Navigation/Measurement Rate Settings: set to 5Hz rate (max supported by NEO-6Q)
            #else
                unsigned char UBX_CFG_RATE[15] = { GPS_ADDR, 0xb5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xe8, 0x03, 0x01, 0x00, 0x01, 0x00, 0x01, 0x39 }; // CFG-RATE Navigation/Measurement Rate Settings: set to 1Hz rate
            #endif
            #if GPS_AIRBORNE 
                unsigned char UBX_CFG_NAV5[45] = {GPS_ADDR, 0xb5, 0x62, 0x06, 0x24, 0x24, 0x00, 0x09, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x5E, 0x11 }; // CFG-NAV5 Navigation engine settings: set the dynamic model to Airborne<1g and dead reckoning time to 1s
            #endif
            
            I2CInit(100);
            Delay(100);
            I2CMaster(UBX_CFG_RST, 13, 0, 0);   // Send the reset configs
            I2CMaster(UBX_CFG_PRT, 29, 0, 0);   // Send the port configs
            I2CMaster(UBX_CFG_TPS, 41, 0, 0);   // Send the Timepulse settings (otherwise the LED don't flash)
            I2CMaster(UBX_CFG_RATE, 15, 0, 0);  // Send the Rate settings
            
            #if GPS_AIRBORNE
                I2CMaster(UBX_CFG_NAV5, 45, 0, 0);   // Send the Nav settings (to put in Airborne<1g dynamic model)
            #endif
            
            #if GPS_METHOD == 1
                FUNCGPSState = 0;
            #endif
        }
        
        void GPSHotstart(void) {
            unsigned char UBX_CFG_RST[13] = { GPS_ADDR, 0xb5, 0x62, 0x06, 0x04, 0x04, 0x00, 0x00, 0x00, 0x09, 0x00, 0x17, 0x76}; // CFG-PM2 Power management config: set wake up
            I2CMaster(UBX_CFG_RST, 13, 0, 0);   // Send the reset configs
        }
        
        void GPSSleep(void) {
            unsigned char UBX_CFG_RST[13] = {GPS_ADDR, 0xb5, 0x62, 0x06, 0x04, 0x04, 0x00, 0x00, 0x00, 0x08, 0x00, 0x16, 0x74}; // CFG-PM2 Power management config: set shutdown
            I2CMaster(UBX_CFG_RST, 13, 0, 0);   // Send the power management configs
        }
        
        void GPSPoll(unsigned short id) {
            unsigned char id1, id2;
            unsigned char UBX_POLL[9] = {GPS_ADDR, 0xb5, 0x62, 0, 0, 0x00, 0x00, 0, 0};

            id1 = id >> 8;
            id2 = id & 0xff;
            
            UBX_POLL[3] = id1;            // ID
            UBX_POLL[4] = id2;            // ID
            UBX_POLL[7] = id1 + id2;      // checksum A
            UBX_POLL[8] = id1*4 + id2*3;  // checksum B
            
            I2CMaster(UBX_POLL, 9, 0, 0);
        }

        #if GPS_METHOD == 0
            unsigned short GPSGetMessage(unsigned short id, unsigned char * buffer) {
                unsigned char data[GPS_BUFFER_SIZE], character, tries, state, id1=0, id2=0, chkA=0, chkB=0;
                unsigned short length, request, payloadlength=0, i, packetcounter=0;
                
                tries = GPS_TRIES_MAX;
                
                while(tries > 0) {
                    data[0] = GPS_ADDR;
                    data[1] = 0xfd;	    // Contains the number of valid bytes
                    data[2] = GPS_ADDR | 1;
                    if(I2CMaster(data, 2, data, 2)) length = data[1] + (data[0] << 8);
                    else length = 0;
                    
                    state = 0;
                    
                    if(length > 0) {
                        if(length > GPS_BUFFER_SIZE) { // prevent overflow
                            request = GPS_BUFFER_SIZE;
                            length -= GPS_BUFFER_SIZE;
                        }
                        else {
                            request = length;
                            length = 0;
                        }
                        
                        data[0] = GPS_ADDR;
                        data[1] = 0xff;	    // Contains the message stream
                        data[2] = GPS_ADDR | 1;
                        if(I2CMaster(data, 2, data, request)) {
                            for(i=0; i<request; i++) {
                                character = data[i];
                                switch(state) {
                                default: // fall through to case 0
                                case 0:  // search for 0xB5 first start header
                                    if(character == 0xB5) {state = 1; }
                                    break;
                                case 1:  // search for 0x62 second start header
                                    if(character == 0x62) {state = 2;}
                                    else state = 0;
                                    break;
                                case 2:  // read the first ID
                                    id1 = character;
                                    chkA = character;
                                    chkB = chkA;
                                    state = 3;
                                    break;
                                case 3:  // read the second ID
                                    id2 = character;
                                    chkA += character;
                                    chkB += chkA;
                                    state = 4;
                                    break;
                                case 4:  // read the first byte of length
                                    payloadlength = character;
                                    chkA += character;
                                    chkB += chkA;
                                    state = 5;
                                    break;
                                case 5:  // read the second byte of length
                                    payloadlength += character << 8;
                                    chkA += character;
                                    chkB += chkA;
                                    
                                    if(((id1 << 8) | id2) != id) { // check for the ID we want
                                        state = 0;
                                        if(i+payloadlength+2 > request) { // don't bother processing the packet if it's not the right message (dont' exceed requested data size)
                                            i = request;
                                        }
                                        else {
                                            i += payloadlength + 2;
                                        }
                                    }
                                    else {
                                        if(payloadlength > 0) state = 6; // zero-length messages go direct to checksum
                                        else state = 7;
                                    }
                                    
                                    packetcounter = 0;
                                    break;
                                case 6:  // read a byte of payload
                                    buffer[packetcounter] = character;
                                    packetcounter++;
                                    
                                    if(packetcounter >= payloadlength) state = 7;
                                    
                                    chkA += character;
                                    chkB += chkA;
                                    break;
                                case 7:  // check first checksum
                                    if(character == chkA) state = 8;
                                    else state = 0;
                                    break;
                                case 8:  // check second checksum
                                    if(character == chkB) {
                                        // data valid
                                        return payloadlength;
                                    }
                                    else {
                                        // data invalid
                                        return 0;
                                    }
                                    state = 0;
                                    break;
                                }
                            
                            }
                        }
                    }
                    else {
                        Delay(GPS_TRIES_DELAY);
                        tries--;
                    }
                }
                return 0;
            }
            
            void GPSClearBuffer(void) {
                unsigned char data[GPS_BUFFER_SIZE];
                unsigned short length, request;
                
                data[0] = GPS_ADDR;
                data[1] = 0xfd;	    // Contains the number of valid bytes
                data[2] = GPS_ADDR | 1;
                if(I2CMaster(data, 2, data, 2)) length = data[1] + (data[0] << 8);
                else length = 0;
                
                while(length > 0) {
                    if(length > GPS_BUFFER_SIZE) { // prevent overflow
                        request = GPS_BUFFER_SIZE;
                        length -= GPS_BUFFER_SIZE;
                    }
                    else {
                        request = length;
                        length = 0;
                    }
                    
                    data[0] = GPS_ADDR;
                    data[1] = 0xff;	    // Contains the message stream
                    data[2] = GPS_ADDR | 1;
                    I2CMaster(data, 2, data, request);
                }
            }
            
            unsigned char GPSGetLocation(signed int * location) {
                unsigned char payload[28];
                unsigned char i;
                unsigned char * ptr; // mess with pointers to shoehorn chars into int array
                ptr = (unsigned char *) location;
                
                GPSPoll(ID_NAV_POSLLH);
                if(GPSGetMessage(ID_NAV_POSLLH, payload)) {
                    GPSClearBuffer();
                    for(i=0; i<8; i++) {
                        ptr[i] = payload[i+4];
                    }
                    for(; i<12; i++) {
                        ptr[i] = payload[i+8];
                    }
                    return 1;
                }
                return 0;
            }

            unsigned char GPSGetFix(void) {
                unsigned char payload[16];
                
                GPSPoll(ID_NAV_STATUS);
                if(GPSGetMessage(ID_NAV_STATUS, payload)) {
                    GPSClearBuffer();
                    
                    if(payload[5] & 0x01) {   // first bit of this byte is "gpsFixOk"
                        return payload[4];    // this contains gpsFix information
                    }
                }
                return 0;
            }
        #endif
        
        #if GPS_METHOD == 1
            volatile unsigned char FUNCGPSState, FUNCGPSChecksumA, FUNCGPSChecksumB, FUNCGPSID1, FUNCGPSID2;
            volatile unsigned short FUNCGPSLength, FUNCGPSPacket, FUNCGPSID;
            unsigned char FUNCGPSBuffer[GPS_BUFFER_SIZE];
        
            void GPSSetRate(unsigned short id, unsigned char rate) {
                unsigned char id1, id2;
                unsigned char UBX_POLL[17] = {GPS_ADDR, 0xb5, 0x62, 0x06, 0x01, 0x08, 0x00, 0, 0, 0, 0x00, 0x00, 0x00, 0x00, 0x00, 0, 0 };

                id1 = id >> 8;
                id2 = id & 0xff;
                
                UBX_POLL[7] = id1;              // ID
                UBX_POLL[8] = id2;              // ID
                UBX_POLL[9] = rate;             // rate
                
                UBX_POLL[15] = 15 + id1 + id2 + rate;           // checksum A
                UBX_POLL[16] = 163 + id1*8 + id2*7 + rate*6;    // checksum B
                
                I2CMaster(UBX_POLL, 17, 0, 0);
            }
            
            void GPSFetchData(void) {
                unsigned char data[GPS_BUFFER_SIZE], character;
                unsigned short length=0, request, i;
                
                data[0] = GPS_ADDR;
                data[1] = 0xfd;	    // Contains the number of valid bytes
                data[2] = GPS_ADDR | 1;
                if(I2CMaster(data, 2, data, 2)) length = data[1] + (data[0] << 8);
                else length = 0;
                
                while(length > 0) {
                    if(length > I2C_DATA_SIZE) { // prevent overflow
                        request = I2C_DATA_SIZE;
                        length -= I2C_DATA_SIZE;
                    }
                    else {
                        request = length;
                        length = 0;
                    }
                    
                    data[0] = GPS_ADDR;
                    data[1] = 0xff;	    // Contains the message stream
                    data[2] = GPS_ADDR | 1;
                    if(I2CMaster(data, 2, data, request)) {
                        
                        for(i=0; i<request; i++) {
                            character = data[i];
                            switch(FUNCGPSState) {
                            default: // fall through to case 0
                            case 0:  // search for 0xB5 first start header
                                if(character == 0xB5) FUNCGPSState = 1;
                                break;
                            case 1:  // search for 0x62 second start header
                                if(character == 0x62) FUNCGPSState = 2;
                                else FUNCGPSState = 0;
                                break;
                            case 2:  // read the first ID
                                FUNCGPSID1 = character;
                                FUNCGPSChecksumA = character;
                                FUNCGPSChecksumB = FUNCGPSChecksumA;
                                FUNCGPSState = 3;
                                break;
                            case 3:  // read the second ID
                                FUNCGPSID2 = character;
                                FUNCGPSChecksumA += character;
                                FUNCGPSChecksumB += FUNCGPSChecksumA;
                                FUNCGPSState = 4;
                                break;
                            case 4:  // read the first byte of length
                                FUNCGPSLength = character;
                                FUNCGPSChecksumA += character;
                                FUNCGPSChecksumB += FUNCGPSChecksumA;
                                FUNCGPSState = 5;
                                break;
                            case 5:  // read the second byte of length
                                FUNCGPSLength += character << 8;
                                FUNCGPSChecksumA += character;
                                FUNCGPSChecksumB += FUNCGPSChecksumA;
                                
                                if(FUNCGPSLength > 0) FUNCGPSState = 6;
                                else FUNCGPSState = 7;
                                
                                FUNCGPSPacket = 0;
                                break;
                            case 6:  // read a byte of payload
                                if(FUNCGPSPacket < GPS_BUFFER_SIZE) {
                                    FUNCGPSBuffer[FUNCGPSPacket] = character;
                                }
                                FUNCGPSPacket++;
                                
                                if(FUNCGPSPacket >= FUNCGPSLength) FUNCGPSState = 7;
                                
                                FUNCGPSChecksumA += character;
                                FUNCGPSChecksumB += FUNCGPSChecksumA;
                                break;
                            case 7:  // check first checksum
                                if(character == FUNCGPSChecksumA) FUNCGPSState = 8;
                                else FUNCGPSState = 0;
                                break;
                            case 8:  // check second checksum
                                if(character == FUNCGPSChecksumB) {
                                    // data valid
                                    FUNCGPSID = (FUNCGPSID1 << 8) | FUNCGPSID2;
                                    if(GPSMessage) GPSMessage(FUNCGPSID, FUNCGPSBuffer, FUNCGPSLength);
                                }
                                FUNCGPSState = 0;
                                break;
                            }
                        }
                    }
                }
            }
        #endif
        
    #endif
    
    
    // ****************************************************************************
    // *** Flash Functions
    // ****************************************************************************
    
    #if SSP1_EN
    #if FLASH_EN
        volatile unsigned int FUNCFlashCR0Reset, FUNCFlashCPSRReset;
        
        volatile unsigned int FUNCFlashCurrentSector;
        unsigned char FUNCFlashSectorBuffer[4096];
        volatile unsigned char FUNCFlashBufferChanged;
        
        void FlashInit(void) {
            SSP1Init(12000); // Init SPI port at 12MHz
            
            Port0Init(PIN23);   // Init CS for flash chip
            Port0SetOut(PIN23);
            
            FlashStart();
            
            FlashCLR();
            
            // This flash chip boots up with write protection enabled everywhere, we need to disable it
            FlashSEL();
            SSP1WriteByte(0x50); // Enable Write to status register
            FlashCLR();
            
            FlashSEL();
            SSP1WriteByte(0x01); // Write to Write Status Register
            SSP1WriteByte(0x00); // - disables all protection
            FlashCLR();
            
            FlashEnd();
            
            FUNCFlashCurrentSector = 0xffffffff;
            FUNCFlashBufferChanged = 0;
        }
        
        void FlashStart(void) {
            FUNCFlashCR0Reset = LPC_SSP1->CR0; // store old config
            FUNCFlashCPSRReset = LPC_SSP1->CPSR; // store old clock speed
            
            LPC_SSP1->CR0 = 7;  // Switch to Phase = 0, Polarity = 0, Format = 0, Frame = 8-bit;
            LPC_SSP1->CPSR = 6;  // run at highest speed possible (last reliable test was at 12Mhz)
        }
        
        void FlashEnd(void) {
            LPC_SSP1->CR0 = FUNCFlashCR0Reset;
            LPC_SSP1->CPSR = FUNCFlashCPSRReset;
        }
        
        void FlashWait(void) {
            FlashSEL();
            SSP1WriteByte(0x05); // Read status register
            while(SSP1ReadByte() & 0x1); // check busy status
            FlashCLR();
        }
        
        void FlashWrEn(void) {
            FlashSEL();
            SSP1WriteByte(0x06); // enables write access
            FlashCLR();
        }
        
        unsigned int FlashGetID(void) {
            unsigned char ManID, Type, Capacity;
            
            FlashStart();
            
            FlashSEL();
            SSP1WriteByte(0x9f); // grabs JEDEC ID
            
            ManID = SSP1ReadByte();
            Type = SSP1ReadByte();
            Capacity = SSP1ReadByte();
            FlashCLR();
            
            FlashEnd();
            
            return (ManID << 16) | (Type << 8) | Capacity;
        }
        
        unsigned char FlashGetMan(void) {
            return FlashGetID() >> 16;
        }
        
        unsigned char FlashGetType(void) {
            return FlashGetID() >> 8;
        }
        
        unsigned char FlashGetCapacity(void) {
            // TODO: translate emory capacity codes
            return FlashGetID() >> 0;
        }
        
        void FlashEraseChip(void) {
            FlashStart();
            FlashWait();    // Wait for busy
            FlashWrEn();    // Enable write
            
            // Erase the whole chip
            FlashSEL();
            SSP1WriteByte(0x60);
            FlashCLR();
            
            FlashEnd();
        }
        
        void FlashErase4k(unsigned int address) {
            FlashStart();
            FlashWait();    // Wait for busy
            FlashWrEn();    // Enable write
            
            // Erase 4k sector beginning at address & 0xfffff000
            FlashSEL();
            SSP1WriteByte(0x20);
            SSP1WriteByte(address >> 16);
            SSP1WriteByte(address >> 8);
            SSP1WriteByte(address >> 0);
            FlashCLR();
            
            FlashEnd();
        }
        
        void FlashErase32k(unsigned int address) {
            FlashStart();
            FlashWait();    // Wait for busy
            FlashWrEn();    // Enable write
            
            // Erase 32k sector beginning at address & 0xffff8000
            FlashSEL();
            SSP1WriteByte(0x52);
            SSP1WriteByte(address >> 16);
            SSP1WriteByte(address >> 8);
            SSP1WriteByte(address >> 0);
            FlashCLR();
            
            FlashEnd();
        }
        
        void FlashErase64k(unsigned int address) {
            FlashStart();
            FlashWait();    // Wait for busy
            FlashWrEn();    // Enable write
            
            // Erase 64k sector beginning at address & 0xffff0000
            FlashSEL();
            SSP1WriteByte(0xd8);
            SSP1WriteByte(address >> 16);
            SSP1WriteByte(address >> 8);
            SSP1WriteByte(address >> 0);
            FlashCLR();
            
            FlashEnd();
        }
        
        void FlashRawWriteByte(unsigned int address, unsigned char data) {
            FlashStart();
            FlashWait();    // Wait for busy
            FlashWrEn();    // Enable write
            
            // Send byte
            FlashSEL();
            SSP1WriteByte(0x02);
            SSP1WriteByte(address >> 16);
            SSP1WriteByte(address >> 8);
            SSP1WriteByte(address >> 0);
            SSP1WriteByte(data);
            FlashCLR();
            
            FlashEnd();
        }
        
        unsigned char FlashRawReadByte(unsigned int address) {
            unsigned char data;
            
            FlashStart();
            FlashWait();    // Wait for busy
            
            // Read byte
            FlashSEL();
            SSP1WriteByte(0x03);
            SSP1WriteByte(address >> 16);
            SSP1WriteByte(address >> 8);
            SSP1WriteByte(address >> 0);
            //SSP1WriteByte(0); // dummy byte if using fast read 0x0b, do not use if using regular read 0x03
            data = SSP1ReadByte();
            FlashCLR();
            
            FlashEnd();
            
            return data;
        }
        
        unsigned char FlashVerify(unsigned int address, unsigned char * data, unsigned int length) {
            unsigned int i;
            
            FlashStart();
            FlashWait();    // Wait for busy
            
            // Read byte
            FlashSEL();
            SSP1WriteByte(0x03);
            SSP1WriteByte(address >> 16);
            SSP1WriteByte(address >> 8);
            SSP1WriteByte(address >> 0);
            //SSP1WriteByte(0); // dummy byte if using fast read 0x0b, do not use if using regular read 0x03
            for(i=0; i<length; i++) {
                if(SSP1ReadByte() != data[i]) {
                    FlashCLR();
                    FlashEnd();
                    return 0;
                }
            }
            FlashCLR();
            FlashEnd();
            
            return 1;
        }
        
        void FlashRawWrite(unsigned int address, unsigned char * data, unsigned int length) {
            unsigned char odd;
            
            // because the flash chip's auto increment write feature writes two bytes at a time,
            // if the address is an odd number, we need to do a single byte write
            if((address & 0x1) && length > 0) { // check if address is odd number
                FlashRawWriteByte(address, data[0]);
                address++;
                data++;
                length--;
            }
            
            // now that the address is even, check if the length is odd, if it is then there's
            // one more byte at the end that needs to be written using a single byte write
            if(length & 0x1) {
                odd = 1;
                length--; // make length even again
            }
            else odd = 0;
            
            if(length > 0) {
                FlashStart();
                FlashWait();    // Wait for busy
                FlashWrEn();    // Enable write

                // Start auto address increment programming
                FlashSEL();
                SSP1WriteByte(0xad);
                SSP1WriteByte(address >> 16);
                SSP1WriteByte(address >> 8);
                SSP1WriteByte(address >> 0);
                
                SSP1WriteByte(data[0]);
                SSP1WriteByte(data[1]);
                FlashCLR();
                address+=2;
                data+=2;
                length-=2;
                
                // write even length of bytes, ensure that if length is odd, the last byte is not written
                while(length) {
                    // Poll for Busy
                    // TODO: replace with hardware end of write detection
                    FlashWait();
                    
                    // Write bytes
                    FlashSEL();
                    SSP1WriteByte(0xad);
                    SSP1WriteByte(data[0]);
                    SSP1WriteByte(data[1]);
                    FlashCLR();
                    
                    address+=2;
                    data+=2;
                    length-=2;
                }
                
                // Poll for Busy
                FlashWait();
                
                // Send Write disable to end auto address increment programming
                FlashSEL();
                SSP1WriteByte(0x04);
                FlashCLR();
                
                FlashEnd();
            }
            
            // if there was one more byte left over to write, deal with this using FlashWriteByte
            if(odd) { // check if length is odd number
                FlashRawWriteByte(address, data[0]);
            }
            
        }
        
        void FlashRawRead(unsigned int address, unsigned char * data, unsigned int length) {
            unsigned int i;
            
            if(length > 0) {
                FlashStart();
                FlashWait();    // Wait for busy
                
                // Read byte
                FlashSEL();
                SSP1WriteByte(0x03);
                SSP1WriteByte(address >> 16);
                SSP1WriteByte(address >> 8);
                SSP1WriteByte(address >> 0);
                //SSP1WriteByte(0); // dummy byte if using fast read 0x0b, do not use if using regular read 0x03
                for(i=0; i<length; i++) {
                    data[i] = SSP1ReadByte();
                }
                FlashCLR();
                
                FlashEnd();
            }
        }
        
        void FlashBufferSector(unsigned int address) {
            FlashFlushBuffer(); // FUNCFlashBufferChanged = 0 is in here
            if((address & 0xfffff000) != FUNCFlashCurrentSector) {
                FUNCFlashCurrentSector = address & 0xfffff000;
                FlashRawRead(FUNCFlashCurrentSector, FUNCFlashSectorBuffer, 4096);
            }        
        }
        
        void FlashFlushBuffer(void) {
            if(FUNCFlashCurrentSector != 0xffffffff) {
                if(FUNCFlashBufferChanged) { // only need to write if buffer was changed
                    FlashErase4k(FUNCFlashCurrentSector);
                    FlashRawWrite(FUNCFlashCurrentSector, FUNCFlashSectorBuffer, 4096);
                    FUNCFlashBufferChanged = 0; // after flushing, there is now no difference between the buffer and the flash
                }
            }
        }
        
        void FlashWriteByte(unsigned int address, unsigned char data) {
            // check conditions under which we'd need to buffer a new sector (if there is an invalid buffer, or buffer contains wrong sector)
            if((FUNCFlashCurrentSector == 0xffffffff) || ((address & 0xfffff000) != FUNCFlashCurrentSector)) {
                FlashBufferSector(address);
            }
            
            FUNCFlashBufferChanged = 1;
            FUNCFlashSectorBuffer[address & 0xfff] = data; // save the byte to the buffer
        }
        
        void FlashWrite(unsigned int address, unsigned char * data, unsigned int length) {
            unsigned int writeLength, i;
            while(length) {
                // check conditions under which we'd need to buffer a new sector (if there is an invalid buffer, or buffer contains wrong sector)
                if((FUNCFlashCurrentSector == 0xffffffff) || ((address & 0xfffff000) != FUNCFlashCurrentSector)) {
                    FlashBufferSector(address);
                }
                
                writeLength = (0x1000 - (address & 0xfff));     // this is the writable bytes in this sector starting from the address
                
                for(i=0; i<length && i<writeLength; i++) { // write into buffer
                    FUNCFlashSectorBuffer[(address & 0xfff) + i] = data[i];
                }
                
                FUNCFlashBufferChanged = 1;
                address += i;
                length -= i;
                data += i;
                
            }
        }
        
        unsigned char FlashReadByte(unsigned int address) {
            // see if there's a valid sector in buffer, and that the sector is the one we want
            if((FUNCFlashCurrentSector != 0xffffffff) && ((address & 0xfffff000) == FUNCFlashCurrentSector)) {
                return FUNCFlashSectorBuffer[address & 0xfff]; // return the byte from the buffer
            }
            else {
                return FlashRawReadByte(address); // otherwise get it from flash
            }
        }
        
        void FlashRead(unsigned int address, unsigned char * data, unsigned int length) {
            unsigned int readLength, i;
            
            while(length) {
                // see if there's a valid sector in buffer, and that the sector is the one we want
                if((FUNCFlashCurrentSector != 0xffffffff) && ((address & 0xfffff000) == FUNCFlashCurrentSector)) {
                    readLength = (0x1000 - (address & 0xfff));     // this is the readable bytes in this sector starting from the address
                    for(i=0; i<length && i<readLength; i++) { // read it all out
                        data[i] = FUNCFlashSectorBuffer[(address & 0xfff) + i];
                    }
                    address += i;
                    length -= i;
                    data += i;
                }
                else {
                    FlashRawRead(address, data, length); // otherwise get it from flash
                    break;
                }
            }
        }
        
        
    #endif
    #endif
#endif

#if WHO_AM_I == I_AM_HYPO || WHO_AM_I == I_AM_HYPX
        
    // ****************************************************************************
    // *** XBee Functions
    // ****************************************************************************
    
    #if UART_EN && SYSTICK_EN && XBEE_EN
        unsigned char FUNCXBeetBuf[TBUF_LEN];
        volatile unsigned char FUNCXBeetBufCount;
        
        volatile unsigned int FUNCXBeeState;
        volatile unsigned short FUNCXBeeLength, FUNCXBeeID, FUNCXBeePacket;
        volatile unsigned char FUNCXBeeChecksum;
        unsigned char FUNCXBeeBuffer[XBEE_BUFFER_SIZE];
        
        unsigned char XBeeSendATCommand(void) {
            xbee_at_command.frameID = Random() | 0x1;
            XBeeSendFrame(ID_XBEE_ATCOMMAND, (unsigned char *)&xbee_at_command, sizeof(xbee_at_command)-2-16+xbee_at_command.varLen);

            // check status
            xbee_at_response.isNew = 0;
            FUNCTimeout = 1000;
            while(FUNCTimeout > 0 && xbee_at_response.isNew == 0);
            if(xbee_at_response.frameID == xbee_at_command.frameID && xbee_at_response.commandStatus == 0) {
                return 1;
            }
            return 0;
        }
        
        unsigned char XBeeWriteBroadcast(unsigned char * buffer, unsigned short length) {
            unsigned int i;
            for(i=0; i<length; i++) {
                xbee_transmit_request.RFData[i] = buffer[i];
            }
            xbee_transmit_request.varLen = length;
            xbee_transmit_request.destinationAddress = 0xffff000000000000ULL; //broadcast (big-endian)
            xbee_transmit_request.networkAddress = 0xfeff;
            XBeeSendPacket();
            return 1;
        }
        
        unsigned char XBeeWriteCoordinator(unsigned char * buffer, unsigned short length) {
            unsigned int i;
            for(i=0; i<length; i++) {
                xbee_transmit_request.RFData[i] = buffer[i];
            }
            xbee_transmit_request.varLen = length;
            xbee_transmit_request.destinationAddress = 0x0000000000000000ULL; //to coordinator (big-endian)
            xbee_transmit_request.networkAddress = 0xfeff;
            XBeeSendPacket();
            return 1;
        }
        
        unsigned char XBeeSendPacket(void) {
            xbee_transmit_request.frameID = Random() | 0x01;
            xbee_transmit_request.broadcastRadius = 0;
            xbee_transmit_request.options = 0x01; // disable ACK;

            XBeeSendFrame(ID_XBEE_TRANSMITREQUEST, (unsigned char *)&xbee_transmit_request, sizeof(xbee_transmit_request)-2-255+xbee_transmit_request.varLen);

            return 1;
        }

        void XBeeAllowJoin() {
            XBeeStopJoin();
            
            // set NJ
            xbee_at_command.frameID = Random() | 0x1;
            xbee_at_command.ATCommand1 = 'N';
            xbee_at_command.ATCommand2 = 'J';
            xbee_at_command.parameterValue[0] = XBEE_JOINPERIOD;
            xbee_at_command.varLen = 1;
            XBeeSendATCommand();
        }
        
        void XBeeStopJoin() {
            // set NJ
            xbee_at_command.ATCommand1 = 'N';
            xbee_at_command.ATCommand2 = 'J';
            xbee_at_command.parameterValue[0] = 0x00;
            xbee_at_command.varLen = 1;
            XBeeSendATCommand();
        }

        
        void XBeeJoin() {
            xbee_at_response.isNew = 0;
            
            // NR to network reset
            xbee_at_command.ATCommand1 = 'N';
            xbee_at_command.ATCommand2 = 'R';
            xbee_at_command.varLen = 0;
            if(!XBeeSendATCommand()) XBeeCommFail();

            // NJ0 to prevent others joining
            XBeeStopJoin();
            
            // PL# to set power level
            xbee_at_command.ATCommand1 = 'P';
            xbee_at_command.ATCommand2 = 'L';
            xbee_at_command.parameterValue[0] = XBEE_POWER_LEVEL;
            xbee_at_command.varLen = 1;
            if(!XBeeSendATCommand()) XBeeCommFail();
            
            // JN1 to enable join notification
            xbee_at_command.ATCommand1 = 'J';
            xbee_at_command.ATCommand2 = 'N';
            xbee_at_command.parameterValue[0] = 1;
            xbee_at_command.varLen = 1;
            if(!XBeeSendATCommand()) XBeeCommFail();
            
            // CE0 to disable coordinator mode
            xbee_at_command.ATCommand1 = 'C';
            xbee_at_command.ATCommand2 = 'E';
            xbee_at_command.parameterValue[0] = 0;
            xbee_at_command.varLen = 1;
            if(!XBeeSendATCommand()) XBeeCommFail();
            
            // ID0 to set PAN ID to 0 (autofind network)
            xbee_at_command.ATCommand1 = 'I';
            xbee_at_command.ATCommand2 = 'D';
            xbee_at_command.parameterValue[0] = 0;
            xbee_at_command.varLen = 1;
            if(!XBeeSendATCommand()) XBeeCommFail();
            
            // EE1 for encryption enable
            xbee_at_command.ATCommand1 = 'E';
            xbee_at_command.ATCommand2 = 'E';
            xbee_at_command.parameterValue[0] = 1;
            xbee_at_command.varLen = 1;
            if(!XBeeSendATCommand()) XBeeCommFail();
            
            // D6 to enable RTS flow control
            xbee_at_command.ATCommand1 = 'D';
            xbee_at_command.ATCommand2 = '6';
            xbee_at_command.parameterValue[0] = 1;
            xbee_at_command.varLen = 1;
            if(!XBeeSendATCommand()) XBeeCommFail();
            
            // KY### to set link key
            xbee_at_command.ATCommand1 = 'K';
            xbee_at_command.ATCommand2 = 'Y';
            xbee_at_command.parameterValue[0] = 'U';
            xbee_at_command.parameterValue[1] = 'N';
            xbee_at_command.parameterValue[2] = 'I';
            xbee_at_command.parameterValue[3] = 'V';
            xbee_at_command.parameterValue[4] = 'E';
            xbee_at_command.parameterValue[5] = 'R';
            xbee_at_command.parameterValue[6] = 'S';
            xbee_at_command.parameterValue[7] = 'A';
            xbee_at_command.parameterValue[8] = 'L';
            xbee_at_command.parameterValue[9] = '_';
            xbee_at_command.parameterValue[10] = 'A';
            xbee_at_command.parameterValue[11] = 'I';
            xbee_at_command.parameterValue[12] = 'R';
            xbee_at_command.parameterValue[13] = '1';
            xbee_at_command.parameterValue[14] = '2';
            xbee_at_command.parameterValue[15] = '3';
            xbee_at_command.varLen = 16;
            if(!XBeeSendATCommand()) XBeeCommFail();
            
            // AC to apply
            xbee_at_command.ATCommand1 = 'A';
            xbee_at_command.ATCommand2 = 'C';
            xbee_at_command.varLen = 0;
            if(!XBeeSendATCommand()) XBeeCommFail();
            
            while(1) {
                Delay(100);
                
                // AI0 to get association information
                xbee_at_command.frameID = Random() | 0x1;
                xbee_at_command.ATCommand1 = 'A';
                xbee_at_command.ATCommand2 = 'I';
                xbee_at_command.varLen = 0;
                //if(!XBeeSendATCommand()) XBeeCommFail();
                if(!XBeeSendATCommand()) Delay(1000);
                if(xbee_at_response.commandData[0] == 0) break;
            }
            
            // WR to write changes
            xbee_at_command.ATCommand1 = 'W';
            xbee_at_command.ATCommand2 = 'R';
            xbee_at_command.varLen = 0;
            if(!XBeeSendATCommand()) XBeeCommFail();
        }

        
        void XBeeCoordinatorJoin() {
            xbee_at_response.isNew = 0;
            
            // NR to network reset
            xbee_at_command.ATCommand1 = 'N';
            xbee_at_command.ATCommand2 = 'R';
            xbee_at_command.varLen = 0;
            if(!XBeeSendATCommand()) XBeeCommFail();
            
            // set Powerlevel
            xbee_at_command.ATCommand1 = 'P';
            xbee_at_command.ATCommand2 = 'L';
            xbee_at_command.parameterValue[0] = XBEE_POWER_LEVEL;
            xbee_at_command.varLen = 1;
            if(!XBeeSendATCommand()) XBeeCommFail();
            
            // set CE mode
            xbee_at_command.ATCommand1 = 'C';
            xbee_at_command.ATCommand2 = 'E';
            xbee_at_command.parameterValue[0] = 1;
            xbee_at_command.varLen = 1;
            if(!XBeeSendATCommand()) XBeeCommFail();
            
            // set PAN ID
            xbee_at_command.ATCommand1 = 'I';
            xbee_at_command.ATCommand2 = 'D';
            xbee_at_command.parameterValue[0] = 0;
            xbee_at_command.varLen = 1;
            if(!XBeeSendATCommand()) XBeeCommFail();
            
            // encryption enable
            xbee_at_command.ATCommand1 = 'E';
            xbee_at_command.ATCommand2 = 'E';
            xbee_at_command.parameterValue[0] = 1;
            xbee_at_command.varLen = 1;
            if(!XBeeSendATCommand()) XBeeCommFail();

            // encryption options
            xbee_at_command.ATCommand1 = 'E';
            xbee_at_command.ATCommand2 = 'O';
            xbee_at_command.parameterValue[0] = 0x02;
            xbee_at_command.varLen = 1;
            if(!XBeeSendATCommand()) XBeeCommFail();
            
            // set network key to zero
            xbee_at_command.ATCommand1 = 'N';
            xbee_at_command.ATCommand2 = 'K';
            xbee_at_command.parameterValue[0] = 0;
            xbee_at_command.varLen = 1;
            if(!XBeeSendATCommand()) XBeeCommFail();
            
            // set link key
            xbee_at_command.ATCommand1 = 'K';
            xbee_at_command.ATCommand2 = 'Y';
            xbee_at_command.parameterValue[0] = 'U';
            xbee_at_command.parameterValue[1] = 'N';
            xbee_at_command.parameterValue[2] = 'I';
            xbee_at_command.parameterValue[3] = 'V';
            xbee_at_command.parameterValue[4] = 'E';
            xbee_at_command.parameterValue[5] = 'R';
            xbee_at_command.parameterValue[6] = 'S';
            xbee_at_command.parameterValue[7] = 'A';
            xbee_at_command.parameterValue[8] = 'L';
            xbee_at_command.parameterValue[9] = '_';
            xbee_at_command.parameterValue[10] = 'A';
            xbee_at_command.parameterValue[11] = 'I';
            xbee_at_command.parameterValue[12] = 'R';
            xbee_at_command.parameterValue[13] = '1';
            xbee_at_command.parameterValue[14] = '2';
            xbee_at_command.parameterValue[15] = '3';
            xbee_at_command.varLen = 16;
            if(!XBeeSendATCommand()) XBeeCommFail();
            
            // set NJ
            xbee_at_command.ATCommand1 = 'N';
            xbee_at_command.ATCommand2 = 'J';
            xbee_at_command.parameterValue[0] = 0x00;
            xbee_at_command.varLen = 1;
            if(!XBeeSendATCommand()) XBeeCommFail();
            
            // WR to write changes
            xbee_at_command.ATCommand1 = 'W';
            xbee_at_command.ATCommand2 = 'R';
            xbee_at_command.varLen = 0;
            if(!XBeeSendATCommand()) XBeeCommFail();
            
            while(1) {
                Delay(100);
                
                // AI0 to get association information
                xbee_at_command.frameID = Random() | 0x1;
                xbee_at_command.ATCommand1 = 'A';
                xbee_at_command.ATCommand2 = 'I';
                xbee_at_command.varLen = 0;
                if(XBeeSendATCommand()) break;
            }
            
            XBeeAllowJoin();
        }

        void XBeeSendFrame(unsigned char id, unsigned char * buffer, unsigned short length) {
            unsigned int i;
            unsigned char chksum;
            
            UARTWriteByte(0x7E);            // Start byte
            UARTWriteByte(length >> 8);     // length MSB
            UARTWriteByte(length & 0xff);   // length LSB
            UARTWriteByte(id);
            chksum = id;
            
            for(i=0; i<length-1; i++) {
                UARTWriteByte(buffer[i]);
                chksum += buffer[i];
            }
            UARTWriteByte(0xff - chksum);
        }

        void XBeeSetDefaults(void) {
            unsigned char changes = 0;
            // Try to enter AT mode at 115200 baud
            FUNCXBeetBufCount = 0;
            FUNCTimeout = 1000;
            UARTWrite((unsigned char *)"+++", 3);
            while(FUNCTimeout > 0 && FUNCXBeetBufCount < 3); // wait for 1 second or until 3 characters received: "OK\r"a
            
            if(XBeetBufCompare((unsigned char *)"OK\r", 3) == 0) {
                // if timed out or didn't receive OK, maybe we're at the wrong baud?
                // Try to enter AT mode at 9600 baud
                UARTInit(9600);
                Delay(100);
                
                FUNCXBeetBufCount = 0;
                FUNCTimeout = 1000;
                UARTWrite((unsigned char *)"+++", 3);
                while(FUNCTimeout > 0 && FUNCXBeetBufCount < 3); // wait for 1 second or until 3 characters received: "OK\r"a
                
                if(XBeetBufCompare((unsigned char *)"OK\r", 3) == 0) {
                    // If timed out or did not receive OK, don't know what to do, fail.
                    XBeeCommFail();
                }
                
                // assuming successfully entered AT mode at 9600 baud, set baud rate to 115200 for next time
                FUNCXBeetBufCount = 0;
                FUNCTimeout = 1000;
                UARTWrite((unsigned char *)"ATBD7\r", 6);
                while(FUNCTimeout > 0 && FUNCXBeetBufCount < 3); // wait for 1 second or until 3 characters received: "OK\r"a
                if(XBeetBufCompare((unsigned char *)"OK\r", 3) == 0) {
                    // If timed out or didn't receive OK, don't know what to do, fail.
                    XBeeCommFail();
                }
                changes+=10;
            }
            
            // assuming successfully entered AT mode, procede to check settings
            // Check API mode
            FUNCXBeetBufCount = 0;
            FUNCTimeout = 1000;
            UARTWrite((unsigned char *)"ATAP\r", 5);
            while(FUNCTimeout > 0 && FUNCXBeetBufCount < 2); // wait for 1 second or until 1 characters received
            if(XBeetBufCompare((unsigned char *)"1\r", 2) == 0) {
                // if API mode was not already on, set it
                FUNCXBeetBufCount = 0;
                FUNCTimeout = 1000;
                UARTWrite((unsigned char *)"ATAP1\r", 6);
                while(FUNCTimeout > 0 && FUNCXBeetBufCount < 3); // wait for 1 second or until 3 characters received: "OK\r"a
                if(XBeetBufCompare((unsigned char *)"OK\r", 3) == 0) {
                    // If timed out or didn't receive OK, don't know what to do, fail.
                    XBeeCommFail();
                }
                changes++;
            }
            
            // If any changes, write them
            if(changes > 0) {
                FUNCXBeetBufCount = 0;
                FUNCTimeout = 3000;
                UARTWrite((unsigned char *)"ATWR\r", 5);
                while(FUNCTimeout < 0 && FUNCXBeetBufCount < 3); // wait for 4 second or until 3 characters received: "OK\r"a
                if(XBeetBufCompare((unsigned char *)"OK\r", 3) == 0) {
                    // If timed out or didn't receive OK, don't know what to do, fail.
                    //XBeeCommFail();
                    // For some reason this isn't always detected, disable this catch for now
                }
            }
            
            // Close ATCN mode
            UARTWrite((unsigned char *)"ATCN\r", 5);
            
            if(changes > 10) {
                // this signifies that the UART baud was changed, switch baud back up
                Delay(100); // either a delay or wait for UART FIFO to clear
                UARTInit(115200);
            }
        }

        
        void XBeeFactoryReset(void) {
        
            //Xbee can be factory reset by restarting XBee with TX held low (BREAK signal), afterwards it'll be on 9600 baud
            UARTInit(9600);
            Port0Init(PIN19);
            Port0SetOut(PIN19);
            Port0Write(PIN19, 0);
            
            XBeeReset();
            Delay(50);
            
            FUNCXBeetBufCount = 0;
            FUNCTimeout = 1000;
            XBeeRelease();
            while(FUNCTimeout > 0 && FUNCXBeetBufCount < 3); // wait for 1 second or until 3 characters received: "OK\r"a

            Delay(100);
            if(XBeetBufCompare((unsigned char *)"OK\r", 3) == 0) return;
            
            UARTInit(9600);
            
            // issue restore defaults command
            FUNCXBeetBufCount = 0;
            FUNCTimeout = 1000;
            UARTWrite((unsigned char *)"ATRE\r", 5);
            while(FUNCTimeout > 0 && FUNCXBeetBufCount < 3); // wait for 1 second or until 3 characters received: "OK\r"a
            
            if(XBeetBufCompare((unsigned char *)"OK\r", 3) == 0) return;
            
            // issue save settings command
            FUNCXBeetBufCount = 0;
            FUNCTimeout = 3000;
            UARTWrite((unsigned char *)"ATWR\r", 5);
            while(FUNCTimeout > 0 && FUNCXBeetBufCount < 3); // wait for 3 second or until 3 characters received: "OK\r"a
            
            if(XBeetBufCompare((unsigned char *)"OK\r", 3) == 0) return;
            
            // now restart
            FUNCXBeetBufCount = 0;
            FUNCTimeout = 1000;
            UARTWrite((unsigned char *)"ATFR\r", 5);
            while(FUNCTimeout > 0 && FUNCXBeetBufCount < 3); // wait for 1 second or until 3 characters received: "OK\r"a
            
            if(XBeetBufCompare((unsigned char *)"OK\r", 3) == 0) return;
        
        }
        
        void XBeeCommFail() {
            while(1);
        }

        unsigned int XBeetBufCompare(unsigned char * compare, unsigned int length) {
            unsigned int i;
            if(FUNCXBeetBufCount < length) return 0;
            for(i=0; i<length; i++) {
                if(compare[i] != FUNCXBeetBuf[FUNCXBeetBufCount-length+i]) return 0;
            }
            return 1;
        }

        void XBeeInit(void) {
            FUNCXBeeState = 0;
            UARTInit(115200);
            FUNCXBeetBufCount = 0;
            
            Port0Init(PIN20 | PIN17);
            Port0SetOut(PIN20 | PIN17);
            
            XBeeReset();
            Delay(50);
            
            XBeeRelease();
            XBeeAllow();
            
            // Start timeout and wait to see if XBee is already configured (should receive modem_status API frame)
            FUNCTimeout = 1000;
            xbee_modem_status.isNew = 0;
            while(FUNCTimeout > 0 && xbee_modem_status.isNew == 0);
            
            if(xbee_modem_status.isNew == 0) {
                XBeeSetDefaults();
            }
        }
        
        
        void XBUARTInterrupt(unsigned char byte) {
            // insert into buffer
            if(FUNCXBeetBufCount < TBUF_LEN) FUNCXBeetBuf[FUNCXBeetBufCount++] = byte;
            
            switch(FUNCXBeeState) {
                default: // fall through to 0
                case 0: // find start character 0x7e
                    if(byte == 0x7e) FUNCXBeeState++;
                    break;
                case 1: // length MSB
                    FUNCXBeeLength = byte << 8;
                    FUNCXBeeState++;
                    break;
                case 2: // length LSB
                    FUNCXBeeLength |= byte;
                    if(FUNCXBeeLength > 1) FUNCXBeeState++;
                    else FUNCXBeeState = 0;
                    FUNCXBeePacket = 0;
                    FUNCXBeeChecksum = 0;
                    break;
                case 3: // cmd frame
                    FUNCXBeeID = byte;
                    FUNCXBeeLength--; // length value includes ID, we need length in terms of payload size
                    FUNCXBeeState++;
                    FUNCXBeeChecksum += byte;
                    break;
                case 4: // cmd data
                    if(FUNCXBeePacket < XBEE_BUFFER_SIZE) {
                        FUNCXBeeBuffer[FUNCXBeePacket] = byte;
                    }
                    FUNCXBeePacket++;
                    
                    if(FUNCXBeePacket >= FUNCXBeeLength) FUNCXBeeState++;
                    
                    FUNCXBeeChecksum += byte;
                    break;
                case 5: // checksum
                    if(0xff - FUNCXBeeChecksum == byte) {
                        if(XBeeMessage) XBeeMessage(FUNCXBeeID, FUNCXBeeBuffer, FUNCXBeeLength);
                    }
                    else {
                    }
                    FUNCXBeeState = 0;
                    break;
                case 0xff: // bypass mode
                    if(XBeeMessage) XBeeMessage(0, &byte, 1);
                    break;
            }
        }
    #endif
    
#endif
    
    
#ifdef __cplusplus
}
#endif 