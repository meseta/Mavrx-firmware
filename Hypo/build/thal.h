// ****************************************************************************
// *** Copyright (c) 2011, Universal Air Ltd. All rights reserved.
// *** Source and binaries are released under the BSD 3-Clause license
// *** See readme_forebrain.txt files for the text of the license
// ****************************************************************************
 
#ifndef __THAL_H__
#define __THAL_H__
#include "thal.h"
#include "config.h"
#include "lpc1347.h"

#include "usb/mw_usbd_rom_api.h"

#ifndef WEAK
    #define WEAK __attribute__ ((weak))
#endif

#ifndef WEAK_ALIAS
    #define WEAK_ALIAS(f) __attribute__ ((weak, alias (#f)));
#endif

#ifndef ALIAS
    #define ALIAS(f) __attribute__ ((alias (#f)));
#endif

#ifndef PACKED
    #define PACKED __attribute__ ((packed))
#endif

#ifndef TRUE
    #define TRUE        1
    #define FALSE       0
#endif

#ifndef NULL
    #ifdef __cplusplus              // EC++
        #define NULL          0
    #else
        #define NULL          ((void *) 0)
    #endif
#endif

#if defined (__cplusplus)
extern "C" {
#endif


// ****************************************************************************
// *** Misc Functions
// ****************************************************************************

// *** In-application programming functions

typedef void (*FUNCIAP)(unsigned int *, unsigned int *);

void Reprogram(void);
unsigned int ReadPID(void);
void ReadUID(unsigned int * uid);
unsigned int ReadUIDHash(void);

// *** EEPROM functions (a subset of IAP
void EEPROMWriteByte(unsigned int address, unsigned char data);
unsigned char EEPROMReadByte(unsigned int address);
void EEPROMWrite(unsigned int address, unsigned char * data, unsigned int length);
void EEPROMRead(unsigned int address, unsigned char * data, unsigned int length);

// *** Reset functions
void Reset(void);
static inline void ResetInit(void) { LPC_IOCON->RESET_PIO0_0 = 0x90; }

// *** Byte handling functions
/*static inline unsigned char LowByte(unsigned short value)   { return (value & 0xff); }
static inline unsigned char HighByte(unsigned short value)  { return (value >> 8) & 0xff; }
static inline unsigned char FirstByte(unsigned int value)   { return (value & 0xff); }
static inline unsigned char SecondByte(unsigned int value)  { return (value >> 8) & 0xff; }
static inline unsigned char ThirdByte(unsigned int value)   { return (value >> 16) & 0xff; }
static inline unsigned char FourthByte(unsigned int value)  { return (value >> 24) & 0xff; }
static inline unsigned short MakeShort(unsigned char byte2, unsigned char byte1)  { return ((byte2 << 8) & byte1); }
static inline unsigned int MakeInt(unsigned char byte4, unsigned char byte3, unsigned char byte2, unsigned char byte1) { return ((byte4 << 24) | (byte3 << 16) | (byte2 << 8) | byte1); }
*/

// *** Fast trig approximation functions
float finvSqrt(float x);
float fatan2(float y, float x);
float fasin(float x);
float fsin(float x);
float fcos(float x);

// *** Random number functions
#if RAND_MERSENNE
    void RandomSeed(unsigned int seed);
    unsigned int Random(void);
#else
    extern volatile unsigned int FUNCRandomNumber;
    static inline unsigned int Random(void) { FUNCRandomNumber = FUNCRandomNumber * RAND_A + RAND_C; return FUNCRandomNumber; };
    static inline void RandomSeed(unsigned int seed) { FUNCRandomNumber = seed; };
#endif


typedef	struct _PWRD {
    void (*set_pll)(unsigned int * cmd, unsigned int * resp);
    void (*set_power)(unsigned int * cmd, unsigned int * resp);
}  PWRD;

typedef	struct _ROM {
    const USBD_API_T * pUSBD;
    const unsigned p_clib;
    const unsigned p_cand;
    const PWRD * pPWRD;
    const unsigned p_dev1;
    const unsigned p_dev2;
    const unsigned p_dev3;
    const unsigned p_dev4; 
}  ROM;

// ****************************************************************************
// *** Clock Functions
// ****************************************************************************

// *** Clock mode functions
#define XTAL    0
#define IRC72   1
#define IRC12   2
void ClockModeXTAL(void);
void ClockModeIRC72(void);
void ClockModeIRC12(void);

// *** Delay functions
void Delay(unsigned int milliseconds);
void WaitDelay(unsigned int milliseconds);

// *** System tick timer functions
#if SYSTICK_EN
    void SysTickInit(void);
    void SysTickStop(void);
    extern volatile unsigned int FUNCSysTicks, FUNCTimeout;

    extern WEAK void SysTickInterrupt(void);
    extern WEAK void SDTick(void);
    void SysTickDelay(unsigned int milliseconds);
    void SysTickUDelay(unsigned int microseconds);
#endif


// *** Watchdog timer functions
#define INTERRUPT   0x40
void WDTInit(unsigned int milliseconds);
void WDTStop(void);
static inline void WDTFeed(void) {    LPC_WWDT->FEED = 0xAA; LPC_WWDT->FEED = 0x55;    }
extern WEAK void WDTInterrupt(void);

/*
// *** Clockout functions
//#define OFF        0  // this defined as such elsewhere
#define IRCOSC      1
#define SYSOSC      2
#define WDTOSC      3
#define MAINCLK     4
#define USBSOF      5
void ClockOut(unsigned char mode, unsigned char divider);
*/

void RITInit(unsigned long long value);
void RITInitms(unsigned int value);

static inline void RITMask(unsigned long long mask) { LPC_RITIMER->MASK = mask & 0xffffffff; LPC_RITIMER->MASK_H = mask >> 32; }
static inline void RITStop(void) { LPC_RITIMER->CTRL &= ~0x8; }
static inline void RITGo(void) { LPC_RITIMER->CTRL |= 0x8; }
static inline void RITReset(void) { LPC_RITIMER->COUNTER=0; LPC_RITIMER->COUNTER_H=0; }
static inline unsigned long long RITValue(void) { return ((unsigned long long) LPC_RITIMER->COUNTER_H << 32) | LPC_RITIMER->COUNTER; }

extern WEAK void RITInterrupt(void);

/*
// ****************************************************************************
// *** Power mode Functions
// ****************************************************************************

// *** Sleep mode
void Sleep(void);

// *** Deep sleep
#define PORT0PIN0   0x0000000000000001ULL
#define PORT0PIN1   0x0000000000000002ULL
#define PORT0PIN2   0x0000000000000004ULL
#define PORT0PIN3   0x0000000000000008ULL
#define PORT0PIN4   0x0000000000000010ULL
#define PORT0PIN5   0x0000000000000020ULL
#define PORT0PIN6   0x0000000000000040ULL
#define PORT0PIN7   0x0000000000000080ULL
#define PORT0PIN8   0x0000000000000100ULL
#define PORT0PIN9   0x0000000000000200ULL
#define PORT0PIN10  0x0000000000000400ULL
#define PORT0PIN11  0x0000000000000800ULL
#define PORT1PIN0   0x0000000000001000ULL
#define PORT1PIN1   0x0000000000002000ULL
#define PORT1PIN2   0x0000000000004000ULL
#define PORT1PIN3   0x0000000000008000ULL
#define PORT1PIN4   0x0000000000010000ULL
#define PORT1PIN5   0x0000000000020000ULL
#define PORT1PIN6   0x0000000000040000ULL
#define PORT1PIN7   0x0000000000080000ULL
#define PORT1PIN8   0x0000000000100000ULL
#define PORT1PIN9   0x0000000000200000ULL
#define PORT1PIN10  0x0000000000400000ULL
#define PORT1PIN11  0x0000000000800000ULL
#define PORT2PIN0   0x0000000001000000ULL
#define PORT2PIN1   0x0000000002000000ULL
#define PORT2PIN2   0x0000000004000000ULL
#define PORT2PIN3   0x0000000008000000ULL
#define PORT2PIN4   0x0000000010000000ULL
#define PORT2PIN5   0x0000000020000000ULL
#define PORT2PIN6   0x0000000040000000ULL
#define PORT2PIN7   0x0000000080000000ULL
#define PORT2PIN8   0x0000000100000000ULL
#define PORT2PIN9   0x0000000200000000ULL
#define PORT2PIN10  0x0000000400000000ULL
#define PORT2PIN11  0x0000000800000000ULL
#define PORT3PIN0   0x0000001000000000ULL
#define PORT3PIN1   0x0000002000000000ULL
#define PORT3PIN2   0x0000004000000000ULL
#define PORT3PIN3   0x0000008000000000ULL

void DeepSleep(unsigned long long startpins, unsigned long long startdirection, unsigned int timer);

// *** Deep power down mode
void PowerDown(void);
static inline void WriteBootData0(unsigned int data) { LPC_PMU->GPREG0 = data; }
static inline void WriteBootData1(unsigned int data) { LPC_PMU->GPREG1 = data; }
static inline void WriteBootData2(unsigned int data) { LPC_PMU->GPREG2 = data; }
static inline void WriteBootData3(unsigned int data) { LPC_PMU->GPREG3 = data; }
static inline void WriteBootData4(unsigned int data) { LPC_PMU->GPREG4 &= ~(0xfffff << 11); LPC_PMU->GPREG4 |= ((data & 0xfffff) << 11); }
static inline unsigned int ReadBootData0(void) { return LPC_PMU->GPREG0; }
static inline unsigned int ReadBootData1(void) { return LPC_PMU->GPREG1; }
static inline unsigned int ReadBootData2(void) { return LPC_PMU->GPREG2; }
static inline unsigned int ReadBootData3(void) { return LPC_PMU->GPREG3; }
static inline unsigned int ReadBootData4(void) { return (LPC_PMU->GPREG4 >> 11) & 0xfffff; }

// *** Brownout detect
#define BOD0    0 // 1.69V assert 1.84V deassert
#define BOD1    1 // 2.29V assert 2.44V deassert
#define BOD2    2 // 2.59V assert 2.74V deassert
#define BOD3    3 // 2.87V assert 2.98V deassert

void BODInit(unsigned char interruptlevel);
void BODStop(void);
extern WEAK void BODInterrupt(void);

// *** Reset status
#define POWERON     0x1
#define EXTERNAL    0x2
#define WATCHDOG    0x4
#define BROWNOUT    0x8
#define SOFTWARE    0x10
#define POWEREDDOWN 0x20
unsigned char ResetStatus(void);
*/

// ****************************************************************************
// *** GPIO Functions
// ****************************************************************************

// *** Pin number definitions
#define PIN0        (0x1UL << 0)
#define PIN1        (0x1UL << 1)
#define PIN2        (0x1UL << 2)
#define PIN3        (0x1UL << 3)
#define PIN4        (0x1UL << 4)
#define PIN5        (0x1UL << 5)
#define PIN6        (0x1UL << 6)
#define PIN7        (0x1UL << 7)
#define PIN8        (0x1UL << 8)
#define PIN9        (0x1UL << 9)
#define PIN10       (0x1UL << 10)
#define PIN11       (0x1UL << 11)
#define PIN12       (0x1UL << 12)
#define PIN13       (0x1UL << 13)
#define PIN14       (0x1UL << 14)
#define PIN15       (0x1UL << 15)
#define PIN16       (0x1UL << 16)
#define PIN17       (0x1UL << 17)
#define PIN18       (0x1UL << 18)
#define PIN19       (0x1UL << 19)
#define PIN20       (0x1UL << 20)
#define PIN21       (0x1UL << 21)
#define PIN22       (0x1UL << 22)
#define PIN23       (0x1UL << 23)
#define PIN24       (0x1UL << 24)
#define PIN25       (0x1UL << 25)
#define PIN26       (0x1UL << 26)
#define PIN27       (0x1UL << 27)
#define PIN28       (0x1UL << 28)
#define PIN29       (0x1UL << 29)
#define PIN30       (0x1UL << 30)
#define PIN31       (0x1UL << 31)
#define ALL         0xffffffff

// *** Initialise pins as digital IO mode (note: default state is digital input)
void Port0Init(unsigned int pins);
void Port1Init(unsigned int pins);


// *** Sets pins as inputs or outputs
static inline void Port0SetIn(unsigned int pins) { LPC_GPIO->DIR[0] &= ~pins;    }
static inline void Port1SetIn(unsigned int pins) { LPC_GPIO->DIR[1] &= ~pins;    }
static inline void Port0SetOut(unsigned int pins) { LPC_GPIO->DIR[0] |= pins;    }
static inline void Port1SetOut(unsigned int pins) { LPC_GPIO->DIR[1] |= pins;    }

// *** Write high or low
static inline void Port0High(unsigned int pins) { LPC_GPIO->SET[0] = pins;    }
static inline void Port1High(unsigned int pins) { LPC_GPIO->SET[1] = pins;    }
static inline void Port0Low(unsigned int pins) { LPC_GPIO->CLR[0] = pins;    }
static inline void Port1Low(unsigned int pins) { LPC_GPIO->CLR[1] = pins;    }

// *** Writes digital high or low to pins that are in Output mode
static inline void Port0Write(unsigned int pins, unsigned char value) {
    if(value) LPC_GPIO->SET[0] = pins;
    else LPC_GPIO->CLR[0] = pins;
}
static inline void Port1Write(unsigned int pins, unsigned char value) {
    if(value) LPC_GPIO->SET[1] = pins;
    else LPC_GPIO->CLR[1] = pins;
}

// *** Writes digital high or low to pins that are in Output mode
static inline void Port0WriteMasked(unsigned int pins, unsigned int value) {
    LPC_GPIO->MASK[0] = ~pins;
    LPC_GPIO->MPIN[0] = value;
}
static inline void Port1WriteMasked(unsigned int pins, unsigned int value) {
    LPC_GPIO->MASK[1] = ~pins;
    LPC_GPIO->MPIN[1] = value;
}


// *** Toggle the status of a pin
static inline void Port0Toggle(unsigned int pins) {    LPC_GPIO->NOT[0] = pins;    }
static inline void Port1Toggle(unsigned int pins) {    LPC_GPIO->NOT[0] = pins;    }

// *** Returns the digital value on a pin that are in Input mode
static inline unsigned char Port0Read(unsigned int pins) {
    if(LPC_GPIO->PIN[0] & pins) return 1;
    else return 0;
}
static inline unsigned char Port1Read(unsigned int pins) {
    if(LPC_GPIO->PIN[1] & pins) return 1;
    else return 0;
}

// *** Returns the data on a whole port
static inline unsigned short Port0Data(void) { return LPC_GPIO->PIN[0]; }
static inline unsigned short Port1Data(void) { return LPC_GPIO->PIN[1]; }

// *** Returns the data on a whole port, but with masked pins
static inline unsigned int Port0DataMasked(unsigned int pins) {
    LPC_GPIO->MASK[0] = ~pins;
    return LPC_GPIO->MPIN[0];
}
static inline unsigned int Port1DataMasked(unsigned int pins) {
    LPC_GPIO->MASK[1] = ~pins;
    return LPC_GPIO->MPIN[1];
}

// *** Configure hysteresis on a pin (note: default stat is hysteresis disabled)
#define ON            1// this defined as such elsewhere
#define OFF            0// this defined as such elsewhere
void Port0Hysteresis(unsigned int pins, unsigned char value);
void Port1Hysteresis(unsigned int pins, unsigned char value);

// *** Configure hysteresis on a pin (note: default stat is hysteresis disabled)
#define INVERT            1
#define NORMAL            0
void Port0Invert(unsigned int pins, unsigned char value);
void Port1Invert(unsigned int pins, unsigned char value);


// *** Configure pull-up/-down resistors on a pin (note: default state is pull-up enabled)
#define PULLUP      0x2
#define PULLDOWN    0x1
#define REPEATER    0x3

void Port0Pull(unsigned int pins, unsigned char value);
void Port1Pull(unsigned int pins, unsigned char value);


// *** Configure interrupts on a pin (note: default state is interrupts disabled)
//#define OFF       0  // this defined as such elsewhere
#define RISING      0x1
#define FALLING     0x2
#define BOTH        0x3
#define LOW         0x10
#define HIGH        0x20
/*void Port0SetInterrupt(unsigned short pins, unsigned char mode);
void Port1SetInterrupt(unsigned short pins, unsigned char mode);

// *** Function prototypes for user-supplied port-wide interrupt functions
extern WEAK void Port0Interrupt(unsigned short GPIOPins);
extern WEAK void Port1Interrupt(unsigned short GPIOPins);
extern WEAK void Port2Interrupt(unsigned short GPIOPins);
extern WEAK void Port3Interrupt(unsigned short GPIOPins);

static inline unsigned short Port0GetInterrupt(void) { return LPC_GPIO0->MIS; }
static inline unsigned short Port1GetInterrupt(void) { return LPC_GPIO1->MIS; }
static inline unsigned short Port2GetInterrupt(void) { return LPC_GPIO2->MIS; }
static inline unsigned short Port3GetInterrupt(void) { return LPC_GPIO3->MIS; }

// *** Function prototypes for user-supplied pin-specific interrupt functions
extern WEAK void Port0Pin0Interrupt(void);
extern WEAK void Port0Pin1Interrupt(void);
extern WEAK void Port0Pin2Interrupt(void);
extern WEAK void Port0Pin3Interrupt(void);
extern WEAK void Port0Pin4Interrupt(void);
extern WEAK void Port0Pin5Interrupt(void);
extern WEAK void Port0Pin6Interrupt(void);
extern WEAK void Port0Pin7Interrupt(void);
extern WEAK void Port0Pin8Interrupt(void);
extern WEAK void Port0Pin9Interrupt(void);
extern WEAK void Port0Pin10Interrupt(void);
extern WEAK void Port0Pin11Interrupt(void);
extern WEAK void Port1Pin0Interrupt(void);
extern WEAK void Port1Pin1Interrupt(void);
extern WEAK void Port1Pin2Interrupt(void);
extern WEAK void Port1Pin3Interrupt(void);
extern WEAK void Port1Pin4Interrupt(void);
extern WEAK void Port1Pin5Interrupt(void);
extern WEAK void Port1Pin6Interrupt(void);
extern WEAK void Port1Pin7Interrupt(void);
extern WEAK void Port1Pin8Interrupt(void);
extern WEAK void Port1Pin9Interrupt(void);
extern WEAK void Port1Pin10Interrupt(void);
extern WEAK void Port1Pin11Interrupt(void);
extern WEAK void Port2Pin0Interrupt(void);
extern WEAK void Port2Pin1Interrupt(void);
extern WEAK void Port2Pin2Interrupt(void);
extern WEAK void Port2Pin3Interrupt(void);
extern WEAK void Port2Pin4Interrupt(void);
extern WEAK void Port2Pin5Interrupt(void);
extern WEAK void Port2Pin6Interrupt(void);
extern WEAK void Port2Pin7Interrupt(void);
extern WEAK void Port2Pin8Interrupt(void);
extern WEAK void Port2Pin9Interrupt(void);
extern WEAK void Port2Pin10Interrupt(void);
extern WEAK void Port2Pin11Interrupt(void);
extern WEAK void Port3Pin0Interrupt(void);
extern WEAK void Port3Pin1Interrupt(void);
extern WEAK void Port3Pin2Interrupt(void);
extern WEAK void Port3Pin3Interrupt(void);
*/



// ****************************************************************************
// *** UART Functions
// ****************************************************************************

#if UART_EN
    // *** UART start, stop functions functions
    #define AUTO                0
    
    void UARTInit(unsigned int baud);
    void UARTStop(void);
    void UARTFlush(void);
    
    // *** UART Write functions
    void UARTWrite(unsigned char * data, unsigned int length);
    void UARTWriteByte(unsigned char data);
    
    // *** UART Read functions
    static inline unsigned char UARTAvailable(void) { return LPC_USART->LSR & 0x1; }
    static inline unsigned char UARTOverrun(void) { return (LPC_USART->LSR >> 1) & 0x1; }
    unsigned char UARTReadByte(void);
    
    extern WEAK void UARTInterrupt(unsigned char UARTData);
    extern WEAK void UARTParityError(unsigned char UARTData);
    
    #if UART_USE_OUTBUFFER
        extern unsigned char FUNCUARTBuffer[UART_BUFFER_SIZE];
        extern volatile unsigned short FUNCUARTBufferPush, FUNCUARTBufferPop;
        
        unsigned short UARTBufferWritable(void);
        unsigned short UARTBufferReadable(void);
        unsigned char UARTBufferPop(void);
        void UARTBufferPush(unsigned char data);
    #endif
#endif 


// ****************************************************************************
// *** I2C Functions
// ****************************************************************************

#if I2C_EN

    // *** Some software state machine definitions
    #define I2C_IDLE            0
    #define I2C_STARTED         1
    #define I2C_RESTARTED       2
    #define I2C_REPEATED_START  3
    #define I2C_ACK             4
    #define I2C_NACK            5
    #define I2C_NACK_END        6
    #define I2C_WR_STARTED      7
    #define I2C_RD_STARTED      8
    #define I2C_GEN_STARTED     9
    #define I2C_ERROR           255

    #define I2C_AA              0x04
    #define I2C_SI              0x08
    #define I2C_STO             0x10
    #define I2C_STA             0x20
    #define I2C_ENA             0x40
    
    #define SLAVE               0

    // *** I2C global datas
    extern volatile unsigned char FUNCI2CMasterState, FUNCI2CMasterState2, FUNCI2CSlaveState, FUNCI2CSlaveState2, FUNCI2CMode;
    extern volatile unsigned int FUNCI2CRdLength, FUNCI2CWrLength;
    extern volatile unsigned int FUNCI2CRdIndex, FUNCI2CWrIndex;
    extern unsigned char FUNCI2CBuffer[I2C_DATA_SIZE];
    
    // *** I2C functions
    void I2CInit(unsigned short speed);
    void I2CStop(void);
    unsigned int I2CMaster(unsigned char * wrData, unsigned int  wrLength, unsigned char * rdData, unsigned char rdLength);

    // *** I2C user-provided interrupts (for slave mode only)
    extern WEAK void I2CInterrupt(unsigned char * I2CData, unsigned int  I2CWriteLength);
    

#endif


// ****************************************************************************
// *** SSP Functions
// ****************************************************************************

#if SSP0_EN
    // *** SSP Initialise and stop
    #ifndef SLAVE
        #define SLAVE               0
    #endif
    void SSP0Init(unsigned short speed);
    void SSP0Stop(void);
    
    // ** Change SSP speed (used for example for reading from SD cards)
    void SSP0SetSpeed(unsigned short speed);
    
    // *** Read and write functions
    void SSP0WriteByte(unsigned short data);
    unsigned short SSP0ReadByte(void);
    void SSP0Write(unsigned short * data, unsigned int i);
    unsigned short SSP0Byte(unsigned short data);
    void SSP0NextByte(unsigned short data);
    
    // *** SSP SSEL functions
    static inline void SSP0Wait(void) { while (LPC_SSP0->SR & 0x10); }
    static inline void SSP0S0SEL(void) { Port0Write(PIN2, 0); }
    static inline void SSP0S0CLR(void) { SSP0Wait(); Port0Write(PIN2, 1); }
    
    // *** SSP user-provided interrupts (for slave mode only)
    extern WEAK void SSP0Interrupt(unsigned short SSP0Data);
#endif

#if SSP1_EN
    #ifndef SLAVE
        #define SLAVE               0
    #endif
    // *** SSP Initialise and stop
    void SSP1Init(unsigned short speed);
    void SSP1Stop(void);
    
    // ** Change SSP speed (used for example for reading from SD cards)
    void SSP1SetSpeed(unsigned short speed);
    
    // *** Read and write functions
    void SSP1WriteByte(unsigned short data);
    unsigned short SSP1ReadByte(void);
    void SSP1Write(unsigned short * data, unsigned int i);
    unsigned short SSP1Byte(unsigned short data);
    void SSP1NextByte(unsigned short data);
    
    // *** SSP SSEL functions
    static inline void SSP1Wait(void) { while (LPC_SSP1->SR & 0x10); }
    static inline void SSP1S0SEL(void) { Port1Write(PIN19, 0); }
    static inline void SSP1S0CLR(void) { SSP1Wait(); Port1Write(PIN19, 1); }
    static inline void SSP1S1SEL(void) { Port0Write(PIN12, 0); }
    static inline void SSP1S1CLR(void) { SSP1Wait(); Port0Write(PIN12, 1); }
    
    // *** SSP user-provided interrupts (for slave mode only)
    extern WEAK void SSP1Interrupt(unsigned short SSP1Data);
#endif


// ****************************************************************************
// *** Timers Functions
// ****************************************************************************

// *** Initialisations and stops
void Timer0Init(unsigned short prescale);
void Timer1Init(unsigned short prescale);
void Timer2Init(unsigned int prescale);
void Timer3Init(unsigned int prescale);

void Timer0Stop(void);
void Timer1Stop(void);
void Timer2Stop(void);
void Timer3Stop(void);

// *** Timer controls
static inline void Timer0Go(void) { LPC_CT16B0->TCR = 1; }
static inline void Timer1Go(void) { LPC_CT16B1->TCR = 1; }
static inline void Timer2Go(void) { LPC_CT32B0->TCR = 1; }
static inline void Timer3Go(void) { LPC_CT32B1->TCR = 1; }
static inline void Timer0Pause(void) { LPC_CT16B0->TCR = 0; }
static inline void Timer1Pause(void) { LPC_CT16B1->TCR = 0; }
static inline void Timer2Pause(void) { LPC_CT32B0->TCR = 0; }
static inline void Timer3Pause(void) { LPC_CT32B1->TCR = 0; }
static inline void Timer0Reset(void) { LPC_CT16B0->TCR = 0; LPC_CT16B0->TC = 0; }
static inline void Timer1Reset(void) { LPC_CT16B1->TCR = 0; LPC_CT16B1->TC = 0; }
static inline void Timer2Reset(void) { LPC_CT32B0->TCR = 0; LPC_CT32B0->TC = 0; }
static inline void Timer3Reset(void) { LPC_CT32B1->TCR = 0; LPC_CT32B1->TC = 0; }

// *** Timer values
static inline unsigned short Timer0Read(void) { return LPC_CT16B0->TC; }
static inline unsigned short Timer1Read(void) { return LPC_CT16B1->TC; }
static inline unsigned int Timer2Read(void) { return LPC_CT32B0->TC; }
static inline unsigned int Timer3Read(void) { return LPC_CT32B1->TC; }
static inline void Timer0Write(unsigned short value) { LPC_CT16B0->TC = value; }
static inline void Timer1Write(unsigned short value) { LPC_CT16B1->TC = value; }
static inline void Timer2Write(unsigned int value) { LPC_CT32B0->TC = value; }
static inline void Timer3Write(unsigned int value) { LPC_CT32B1->TC = value; }
static inline unsigned short Timer0Prescaler(void) { return LPC_CT16B0->PC; }
static inline unsigned short Timer1Prescaler(void) { return LPC_CT16B1->PC; }
static inline unsigned int Timer2Prescaler(void) { return LPC_CT32B0->PC; }
static inline unsigned int Timer3Prescaler(void) { return LPC_CT32B1->PC; }

static inline void Timer0DisableInterrupt(void) { IRQDisable(CT16B0_IRQn); }
static inline void Timer1DisableInterrupt(void) { IRQDisable(CT16B1_IRQn); }
static inline void Timer2DisableInterrupt(void) { IRQDisable(CT32B0_IRQn); }
static inline void Timer3DisableInterrupt(void) { IRQDisable(CT32B1_IRQn); }
static inline void Timer0EnableInterrupt(void) { IRQEnable(CT16B0_IRQn); }
static inline void Timer1EnableInterrupt(void) { IRQEnable(CT16B1_IRQn); }
static inline void Timer2EnableInterrupt(void) { IRQEnable(CT32B0_IRQn); }
static inline void Timer3EnableInterrupt(void) { IRQEnable(CT32B1_IRQn); }

// *** Timer match control
//#define OFF       0       // already defined elsewhere
//#define INTERRUPT   0x40  // already defined elsewhere
#define RESET       0x2   // already defined elsewhere
#define STOP        0x4
#define PWM         0x8
#define OUTPUTOFF   0       // this one doesn't need to be specified
#define OUTPUTLOW   0x10
#define OUTPUTHIGH  0x20
#define OUTPUTTOGGLE 0x30

void Timer0Match0(unsigned short interval, unsigned char mode);
void Timer0Match1(unsigned short interval, unsigned char mode);
void Timer0Match2(unsigned short interval, unsigned char mode);
void Timer0Match3(unsigned short interval, unsigned char mode);
void Timer1Match0(unsigned short interval, unsigned char mode);
void Timer1Match1(unsigned short interval, unsigned char mode);
void Timer1Match2(unsigned short interval, unsigned char mode);
void Timer1Match3(unsigned short interval, unsigned char mode);
void Timer2Match0(unsigned int interval, unsigned char mode);
void Timer2Match1(unsigned int interval, unsigned char mode);
void Timer2Match2(unsigned int interval, unsigned char mode);
void Timer2Match3(unsigned int interval, unsigned char mode);
void Timer3Match0(unsigned int interval, unsigned char mode);
void Timer3Match1(unsigned int interval, unsigned char mode);
void Timer3Match2(unsigned int interval, unsigned char mode);
void Timer3Match3(unsigned int interval, unsigned char mode);

static inline void Timer0SetMatch0(unsigned short interval) {   LPC_CT16B0->MR0 = interval; }
static inline void Timer0SetMatch1(unsigned short interval) {   LPC_CT16B0->MR1 = interval; }
static inline void Timer0SetMatch2(unsigned short interval) {   LPC_CT16B0->MR2 = interval; }
static inline void Timer0SetMatch3(unsigned short interval) {   LPC_CT16B0->MR3 = interval; }
static inline void Timer1SetMatch0(unsigned short interval) {   LPC_CT16B1->MR0 = interval; }
static inline void Timer1SetMatch1(unsigned short interval) {   LPC_CT16B1->MR1 = interval; }
static inline void Timer1SetMatch2(unsigned short interval) {   LPC_CT16B1->MR2 = interval; }
static inline void Timer1SetMatch3(unsigned short interval) {   LPC_CT16B1->MR3 = interval; }
static inline void Timer2SetMatch0(unsigned int interval) {     LPC_CT32B0->MR0 = interval; }
static inline void Timer2SetMatch1(unsigned int interval) {     LPC_CT32B0->MR1 = interval; }
static inline void Timer2SetMatch2(unsigned int interval) {     LPC_CT32B0->MR2 = interval; }
static inline void Timer2SetMatch3(unsigned int interval) {     LPC_CT32B0->MR3 = interval; }
static inline void Timer3SetMatch0(unsigned int interval) {     LPC_CT32B1->MR0 = interval; }
static inline void Timer3SetMatch1(unsigned int interval) {     LPC_CT32B1->MR1 = interval; }
static inline void Timer3SetMatch2(unsigned int interval) {     LPC_CT32B1->MR2 = interval; }
static inline void Timer3SetMatch3(unsigned int interval) {     LPC_CT32B1->MR3 = interval; }

// *** Timer match status
#define MATCH0      0x1
#define MATCH1      0x2
#define MATCH2      0x4
#define MATCH3      0x8

static inline unsigned char Timer0Status(void) { return LPC_CT16B0->EMR & 0xf; }
static inline unsigned char Timer1Status(void) { return LPC_CT16B1->EMR & 0xf; }
static inline unsigned char Timer2Status(void) { return LPC_CT32B0->EMR & 0xf; }
static inline unsigned char Timer3Status(void) { return LPC_CT32B1->EMR & 0xf; }

static inline void Timer0SetStatus(unsigned char status) { LPC_CT16B0->EMR &= 0xfff0; LPC_CT16B0->EMR |= status & 0xf; }
static inline void Timer1SetStatus(unsigned char status) { LPC_CT16B1->EMR &= 0xfff0; LPC_CT16B1->EMR |= status & 0xf; }
static inline void Timer2SetStatus(unsigned char status) { LPC_CT32B0->EMR &= 0xfff0; LPC_CT32B0->EMR |= status & 0xf; }
static inline void Timer3SetStatus(unsigned char status) { LPC_CT32B1->EMR &= 0xfff0; LPC_CT32B1->EMR |= status & 0xf; }

// *** Timer capture control
// #define OFF         0       // already defined elsewhere
//#define RISING      0x1     // already defined elsewhere
//#define FALLING     0x2     // already defined elsewhere
// #define BOTH        0x3     // already defined elsewhere
// #define INTERRUPT   0x40    // already defined elsewhere
#define COUNTER   0x10
#define PWRESET 0x80

void Timer0Capture(unsigned char mode);
void Timer1Capture(unsigned char mode);
void Timer2Capture(unsigned char mode);
void Timer3Capture(unsigned char mode);

static inline unsigned short Timer0CaptureValue(void) { return LPC_CT16B0->CR0; }
static inline unsigned short Timer1CaptureValue(void) { return LPC_CT16B1->CR0; }
static inline unsigned int Timer2CaptureValue(void) { return LPC_CT32B0->CR0; }
static inline unsigned int Timer3CaptureValue(void) { return LPC_CT32B1->CR0; }

// *** Timer interrupts
//#define MATCH0      0x1 // defined as such elsewhere
//#define MATCH1      0x2 // defined as such elsewhere
//#define MATCH2      0x4 // defined as such elsewhere
//#define MATCH3      0x8 // defined as such elsewhere
#define CAPTURE     0x10
extern WEAK void Timer0Interrupt(unsigned char TimerMatch);
extern WEAK void Timer0Interrupt0(void);
extern WEAK void Timer0Interrupt1(void);
extern WEAK void Timer0Interrupt2(void);
extern WEAK void Timer0Interrupt3(void);
extern WEAK void Timer0InterruptC(unsigned short TimerValue);
extern WEAK void Timer1Interrupt(unsigned char TimerMatch);
extern WEAK void Timer1Interrupt0(void);
extern WEAK void Timer1Interrupt1(void);
extern WEAK void Timer1Interrupt2(void);
extern WEAK void Timer1Interrupt3(void);
extern WEAK void Timer1InterruptC(unsigned short TimerValue);
extern WEAK void Timer2Interrupt(unsigned char TimerMatch);
extern WEAK void Timer2Interrupt0(void);
extern WEAK void Timer2Interrupt1(void);
extern WEAK void Timer2Interrupt2(void);
extern WEAK void Timer2Interrupt3(void);
extern WEAK void Timer2InterruptC(unsigned int TimerValue);
extern WEAK void Timer3Interrupt(unsigned char TimerMatch);
extern WEAK void Timer3Interrupt0(void);
extern WEAK void Timer3Interrupt1(void);
extern WEAK void Timer3Interrupt2(void);
extern WEAK void Timer3Interrupt3(void);
extern WEAK void Timer3InterruptC(unsigned int TimerValue);

// ****************************************************************************
// *** ADC Functions
// ****************************************************************************

// *** ADC channel number definitions
#define CHN0        (0x1UL << 0)
#define CHN1        (0x1UL << 1)
#define CHN2        (0x1UL << 2)
#define CHN3        (0x1UL << 3)
#define CHN4        (0x1UL << 4)
#define CHN5        (0x1UL << 5)
#define CHN6        (0x1UL << 6)
#define CHN7        (0x1UL << 7)

// *** Initialise or stop ADC
void ADCInit(unsigned short channels); 
void ADCStop(void);

// *** Read ADC value
unsigned short ADCRead(unsigned char channel);
static inline void ADCTrigger(unsigned char channel) {
    LPC_ADC->CR &= ~0x000000ff;
    LPC_ADC->CR |= 0x1000000 | channel;      // Start now!
}

static inline unsigned short ADCGet(void) {
    return ((LPC_ADC->GDR >> 4) & 0xFFF); 
}
// *** Prototype for user-supplied ADC interrupt function (only used when ADC is set to interrupt mode)
extern WEAK void ADCInterrupt(unsigned char ADCChannel, unsigned short ADCValue);

// ****************************************************************************
// *** USB Functions
// ****************************************************************************

extern USBD_API_T* USBROM;
extern USBD_HANDLE_T hUsb;

void USBInit(void);

// MSC
#define KBYTE   1024
#define MBYTE   1048576
#define GBYTE   1073741824
    
extern unsigned char MSC_DeviceDescriptor[];
extern unsigned char MSC_StringDescriptor[];
extern unsigned char MSC_ConfigDescriptor[];

void MSCInit(unsigned int deviceCapacity);

void MSCRead(unsigned int offset, unsigned char ** buffer, unsigned int length);
void MSCWrite(unsigned int offset, unsigned char ** buffer, unsigned int length);
signed int MSCVerify(unsigned int offset, unsigned char * source, unsigned int length);

// HID
extern unsigned char HID_DeviceDescriptor[];
extern unsigned char HID_StringDescriptor[];
extern unsigned char HID_ConfigDescriptor[];

void HIDInit(void);

extern uint8_t* report_buffer;
extern ErrorCode_t USB_Configure_Event(USBD_HANDLE_T hUsb);
ErrorCode_t HID_GetReport( USBD_HANDLE_T hHid, USB_SETUP_PACKET* pSetup, uint8_t** pBuffer, uint16_t* plength);
ErrorCode_t HID_SetReport( USBD_HANDLE_T hHid, USB_SETUP_PACKET* pSetup, uint8_t** pBuffer, uint16_t length);
ErrorCode_t HID_Ep_Hdlr (USBD_HANDLE_T hUsb, void* data, uint32_t event) ;

void HIDOutReport(unsigned char * buffer);
unsigned char HIDInReport(unsigned char * buffer);
void HIDOutFeature(unsigned char * buffer);
unsigned char HIDInFeature(unsigned char * buffer);


// CDC
#define BRIDGED  1
#define VIRTUAL 0

extern unsigned char VCOM_isbridge;

extern unsigned char CDC_DeviceDescriptor[];
extern unsigned char CDC_StringDescriptor[];
extern unsigned char CDC_ConfigDescriptor[];
void CDCInit(unsigned char bridge);

void CDCReadByte(unsigned char byte);
void CDCWriteByte(unsigned char byte);
void CDCWrite(unsigned char * byte, unsigned int length);

struct VCOM_DATA;
typedef void (*VCOM_SEND_T) (struct VCOM_DATA* pVcom);

typedef struct VCOM_DATA {
  USBD_HANDLE_T hUsb;
  USBD_HANDLE_T hCdc;
  uint8_t* rxBuf;
  uint8_t* txBuf;
  volatile uint8_t ser_pos;
  volatile uint16_t rxlen;
  volatile uint16_t txlen;
  VCOM_SEND_T send_fn;
  volatile uint32_t sof_counter;
  volatile uint32_t last_ser_rx;
  volatile uint16_t break_time;
  volatile uint16_t usbrx_pend;
} VCOM_DATA_T;

extern VCOM_DATA_T g_vCOM;
void VCOM_usb_send(VCOM_DATA_T* pVcom);
void VCOM_init_bridge(VCOM_DATA_T* pVcom, CDC_LINE_CODING* line_coding);
void VCOM_uart_write(VCOM_DATA_T* pVcom);
void VCOM_uart_read(VCOM_DATA_T* pVcom);
void VCOM_uart_send(VCOM_DATA_T* pVcom);
void VCOM_virtual_send(VCOM_DATA_T* pVcom);
ErrorCode_t VCOM_SetLineCode (USBD_HANDLE_T hCDC, CDC_LINE_CODING* line_coding);
ErrorCode_t VCOM_sof_event(USBD_HANDLE_T hUsb);
ErrorCode_t VCOM_SendBreak (USBD_HANDLE_T hCDC, uint16_t mstime);
ErrorCode_t VCOM_bulk_in_hdlr(USBD_HANDLE_T hUsb, void* data, uint32_t event) ;
ErrorCode_t VCOM_bulk_out_hdlr(USBD_HANDLE_T hUsb, void* data, uint32_t event) ;

// ****************************************************************************
// *** LED Functions
// ****************************************************************************

#define RLED    (0x1UL << 0)
#define PLED    (0x1UL << 1)
#define ULED    (0x1UL << 6)
#define VLED    (0x1UL << 3)
#define ALLLED  (RLED | PLED | ULED | VLED)

extern volatile unsigned char FUNCLEDStatus;

static inline void LEDOff(unsigned int pins) { Port0High(pins); FUNCLEDStatus &= ~pins;}
static inline void LEDOn(unsigned int pins) { Port0Low(pins); FUNCLEDStatus |= pins;}
static inline void LEDInit(unsigned int pins) { Port0Init(pins); Port0SetOut(pins); LEDOff(pins);}
static inline void LEDWrite(unsigned int pins, unsigned char value) { if(value) LEDOn(pins); else LEDOff(pins); }
static inline void LEDToggle(unsigned int pins) { Port0Toggle(pins); FUNCLEDStatus ^= pins;}

static inline void RSTInit(void) { Port0Init(PIN0); Port0SetIn(PIN0); }
static inline void PRGInit(void) { Port0Init(PIN1); Port0SetIn(PIN1); }
static inline unsigned char RSTRead(void) { Port0SetIn(PIN0); __NOP(); __NOP(); __NOP(); return Port0Read(PIN0); }
static inline unsigned char PRGRead(void) { Port0SetIn(PIN1); __NOP(); __NOP(); __NOP(); return Port0Read(PIN1); }

unsigned char RSTPoll(void);
unsigned char PRGPoll(void);

static inline void RSTReset(void) { ResetInit(); }


// ****************************************************************************
// *** UAir Interlink Functions
// ****************************************************************************

#if ILINK_EN && SSP0_EN
    #define ID_ILINK_IDENTIFY   0x0000
    #define ID_ILINK_CLEARBUF   0x0003
    #define ID_ILINK_THALCTRL   0x0100
    #define ID_ILINK_THALSTAT   0x0101
    #define ID_ILINK_THALPARAM  0x0102
    #define ID_ILINK_THALPAREQ  0x0103
    #define ID_ILINK_INPUTS0    0x4000
    #define ID_ILINK_OUTPUTS0   0x4100
    #define ID_ILINK_RAWIMU     0x4200
    #define ID_ILINK_SCALEDIMU  0x4201
    #define ID_ILINK_PAYLDCTRL  0x7c00
    #define ID_ILINK_POSITION   0x7d00
    #define ID_ILINK_ALTITUDE   0x7e00
    #define ID_ILINK_ATTITUDE   0x7f00
    #define ID_ILINK_ATDEMAND   0x7f01
    #define ID_ILINK_MODEMAND   0x7f02
    #define ID_ILINK_GPSFLY     0x7f03
    #define ID_ILINK_DEBUG      0x00ff
    
    typedef struct ilink_debug_struct {
        float debug0;
        float debug1;
        float debug2;
        float debug3;
        float debug4;
        float debug5;
        float debug6;
        float debug7;
        unsigned short isNew;
    } PACKED ilink_debug_t;

    typedef struct ilink_gpsfly_struct {
        float northDemand;
        float eastDemand;
        float headingDemand;
        float altitude;
        float altitudeDemand;
        float vAcc;
        float velD;
        unsigned short isNew;
    } PACKED ilink_gpsfly_t;

    typedef struct ilink_identify_struct {  // Contains the ID of the craft
        unsigned short deviceID;            // ID: Thalamus is 1 for example
        unsigned int firmVersion;           // Firmware version: check that this matches
        unsigned short isNew;
    } PACKED ilink_identify_t;
    
    typedef struct ilink_thalctrl_struct {
        unsigned short command;
        unsigned int data;
        unsigned short isNew;
    } PACKED ilink_thalctrl_t;
    
    typedef struct ilink_thalstat_struct {  // Flight status
        unsigned short flightMode;          // Bitfield, bit 4 is GPS mode, bit 3 is altitude control, bit 2 is yaw control, bit 1 is anglerate control, bit 0 is stabilization
        unsigned short sensorStatus;        // Bitfield, bit 6 is Baro status, bit5 is magneto status, bit 4 is Gyro status, bit 3 is Accel status, bit 2-0 flight status (corresponds to MAV_STATE)    
        unsigned short battVoltage;         // Battery voltage
        unsigned short isNew;
    } PACKED ilink_thalstat_t; 

    typedef struct ilink_imu_struct {       // IMU data
        signed short xAcc;
        signed short yAcc;
        signed short zAcc;
        signed short xGyro;
        signed short yGyro;
        signed short zGyro;
        signed short xMag;
        signed short yMag;
        signed short zMag;
        unsigned short isNew;
    } PACKED ilink_imu_t;
    
    typedef struct ilink_attitude_struct {  // Attitude data
        float roll;
        float pitch;
        float yaw;
        float rollRate;
        float pitchRate;
        float yawRate;
        unsigned short isNew;
    } PACKED ilink_attitude_t;
    
	typedef struct ilink_position_struct {  // Position data
        double craftX;
        double craftY;
        double craftZ;
        double targetX;
        double targetY;
        double targetZ;
        unsigned short state;
        unsigned short isNew;
    } PACKED ilink_position_t;
    
    typedef struct ilink_payldctrl_struct { // Payload/camera control
        float camRoll;
        float camPitch;
        float camYaw;
        unsigned short controlMask;
        unsigned short isNew;
    } PACKED ilink_payldctrl_t;
    
    typedef struct ilink_atdemand_struct {  // Attitude data
        float roll;
        float pitch;
        float yaw;
        float thrust;
        unsigned short isNew;
    } PACKED ilink_atdemand_t;
    
    typedef struct ilink_thalparam_struct { // Parameters
        unsigned short paramID;
        float paramValue;
        unsigned short paramCount;
        char paramName[16];
        unsigned short isNew;
    } PACKED ilink_thalparam_t;
    
    typedef struct ilink_thalpareq_struct { // Parameter request
        unsigned short reqType;             // Request type, 0 is get all, 1 is get One, 2 is save all, 3 is reload all
        unsigned short paramID;             // Parameter to request, set to 0xffff to fetch by name
        char paramName[16];                 // Parameter name to request    
        unsigned short isNew;
    } PACKED ilink_thalpareq_t;
    
    typedef struct ilink_iochan_struct {    // Input/output channel data
        unsigned short channel[6];
        unsigned short isNew;
    } PACKED ilink_iochan_t;
    
    typedef struct ilink_altitude_struct {  // Altitude data
        float ultra;                      // E.g. ultrasound
        float baro;                      // E.g. barometer
        float filtered;                      // Change in altitude
        unsigned short isNew;
    } PACKED ilink_altitude_t;
    
    extern volatile unsigned char FUNCILinkState;
    extern volatile unsigned short FUNCILinkID, FUNCILinkChecksumA, FUNCILinkChecksumB, FUNCILinkLength, FUNCILinkPacket;
    extern unsigned short FUNCILinkRxBuffer[ILINK_RXBUFFER_SIZE];
    
    void ILinkInit(unsigned short speed);
    void ILinkPoll(unsigned short message);
    void ILinkProcess(unsigned short data);
    void ILinkFetchData(void);
    unsigned char ILinkSendMessage(unsigned short id, unsigned short * buffer, unsigned short length);
    
    extern WEAK void ILinkMessage(unsigned short id, unsigned short * buffer, unsigned short length);
    extern WEAK void ILinkMessageRequest(unsigned short id);
    extern WEAK void ILinkMessageError(unsigned short id);
    
    extern unsigned int FUNCILinkTxBufferBusy;
    extern unsigned short FUNCILinkTxBuffer[ILINK_TXBUFFER_SIZE];
    extern volatile unsigned short FUNCILinkTxBufferPushPtr, FUNCILinkTxBufferPopPtr;

    unsigned short ILinkWritable(void);
    unsigned short ILinkReadable(void);
    unsigned short ILinkPop(void);
    void ILinkPush(unsigned short data);
#endif


#if WHO_AM_I == I_AM_THALAMUS

    // ****************************************************************************
    // *** ULTRA Functions (Thalamus only)
    // ****************************************************************************
    
    extern volatile unsigned char FUNCUltraNewData;
    extern volatile unsigned short FUNCUltraValueMM;
    extern volatile unsigned char FUNCUltraOutOfRange;
    extern volatile unsigned char FUNCUltraUnderRange;
    extern volatile unsigned char FUNCUltraFastRate;
    
    unsigned char UltraInit(void);
    static inline void UltraSlow(void) { FUNCUltraFastRate = 0; }
    static inline void UltraFast(void) { FUNCUltraFastRate = 1; }
    unsigned short UltraGetData(void);
    unsigned short UltraGetNewData(void);
    unsigned short UltraGetRawData(void);
    unsigned short UltraGetNewRawData(void);

    // ****************************************************************************
    // *** RX Functions (Thalamus only)
    // ****************************************************************************
    #if RX_EN
        void RXUARTInit(void);
        void RXInit(void);
        void RXBind(void);
        void RXDelay(void);
        
        static inline void RXPowerDown(void) { Port0Write(PIN17, 1); }
        static inline void RXPowerUp(void) { Port0Write(PIN17, 0); }

        #define RX_THRO 0
        #define RX_AILE 1
        #define RX_ELEV 2
        #define RX_RUDD 3
        #define RX_GEAR 4
        #define RX_AUX1 4
        #define RX_FLAP 5
        #define RX_AUX2 6

        extern volatile unsigned char FUNCRXCount;
        extern volatile unsigned char FUNCRXLastByte;
        extern volatile unsigned char FUNCRXChannel;
        extern volatile unsigned char FUNCRXNewData;
        
        
        extern volatile unsigned char FUNCRXStatus;
        
        #if RX_TYPE == 0
        extern unsigned short FUNCRXChanBuffer[7];
        extern unsigned short FUNCRXChan[7];
        #else
        extern unsigned short FUNCRXChanBuffer[18];
        extern unsigned short FUNCRXChan[18];
        extern void RXUARTParityError(void);
        #endif
        

        unsigned char RXProcess(unsigned char RXByte);
        unsigned char RXGetData(unsigned short * RXChannels);
        extern void RXUARTInterrupt(unsigned char UARTData);
        extern void RXWDTInterrupt(void);
    #endif

    // ****************************************************************************
    // *** PWM Ouput Functions
    // ****************************************************************************

    #define PWM_N       (1 << 0)
    #define PWM_E       (1 << 1)
    #define PWM_S       (1 << 2)
    #define PWM_W       (1 << 3)
    #define PWM_X       (1 << 4)
    #define PWM_Y       (1 << 5)
    #define PWM_NESW    0x0f
    #define PWM_ALL     0x3f


    extern volatile unsigned char FUNCPWMPostscale;
	extern volatile unsigned short FUNCPWMN_duty;
	extern volatile unsigned short FUNCPWME_duty;
	extern volatile unsigned short FUNCPWMS_duty;
	extern volatile unsigned short FUNCPWMW_duty;
	extern volatile unsigned short FUNCPWMX_duty;
	extern volatile unsigned short FUNCPWMY_duty;

    void PWMInit(unsigned char channels);

    #if PWM_FILTERS_ON == 0
        static inline void PWMSetN(unsigned int value) {  FUNCPWMN_duty = value;   	}
        static inline void PWMSetE(unsigned int value) {  FUNCPWME_duty = value;    }
        static inline void PWMSetS(unsigned int value) {  FUNCPWMS_duty = value;    }
        static inline void PWMSetW(unsigned int value) {  FUNCPWMW_duty = value;    }
        static inline void PWMSetX(unsigned int value) {  FUNCPWMX_duty = value;    }
        static inline void PWMSetY(unsigned int value) {  FUNCPWMY_duty = value;    }
    #else 
        extern volatile unsigned int FUNCPWMN_fil;
        extern volatile unsigned int FUNCPWME_fil;
        extern volatile unsigned int FUNCPWMS_fil;
        extern volatile unsigned int FUNCPWMW_fil;
        extern volatile unsigned int FUNCPWMX_fil;
        extern volatile unsigned int FUNCPWMY_fil;
        static inline void PWMSetN(unsigned int value) {  FUNCPWMN_fil *= PWM_NESW_FILTER; FUNCPWMN_fil += value * (1-PWM_NESW_FILTER); FUNCPWMN_duty = FUNCPWMN_fil;   }
        static inline void PWMSetE(unsigned int value) {  FUNCPWME_fil *= PWM_NESW_FILTER; FUNCPWME_fil += value * (1-PWM_NESW_FILTER); FUNCPWME_duty = FUNCPWME_fil;   }
        static inline void PWMSetS(unsigned int value) {  FUNCPWMS_fil *= PWM_NESW_FILTER; FUNCPWMS_fil += value * (1-PWM_NESW_FILTER); FUNCPWMS_duty = FUNCPWMS_fil;   }
        static inline void PWMSetW(unsigned int value) {  FUNCPWMW_fil *= PWM_NESW_FILTER; FUNCPWMW_fil += value * (1-PWM_NESW_FILTER); FUNCPWMW_duty = FUNCPWMW_fil;   }
        static inline void PWMSetX(unsigned int value) {  FUNCPWMX_fil *= PWM_XY_FILTER; FUNCPWMX_fil += value * (1-PWM_XY_FILTER); FUNCPWMX_duty = FUNCPWMX_fil;   }
        static inline void PWMSetY(unsigned int value) {  FUNCPWMY_fil *= PWM_XY_FILTER; FUNCPWMY_fil += value * (1-PWM_XY_FILTER); FUNCPWMY_duty = FUNCPWMY_fil;   }
    #endif

    static inline void PWMSetNESW(unsigned int valueN, unsigned int valueE, unsigned int valueS, unsigned int valueW) {
        PWMSetN(valueN);
        PWMSetE(valueE);
        PWMSetS(valueS);
        PWMSetW(valueW);
    }

    static inline void PWMSetNESWXY(unsigned int valueN, unsigned int valueE, unsigned int valueS, unsigned int valueW, unsigned int valueX, unsigned int valueY) {
        PWMSetN(valueN);
        PWMSetE(valueE);
        PWMSetS(valueS);
        PWMSetW(valueW);
        PWMSetX(valueX);
        PWMSetY(valueY);
    }

    // ****************************************************************************
    // *** IMU Functions
    // ****************************************************************************

    #define ACCEL_ADDR	0x30
    #define GYRO_ADDR	0xd0
    #define MAGNETO_ADDR	0x3c
    #define BARO_LPS_ADDR	0xb8
	#define BARO_MS_ADDR 0xee
	
	extern volatile unsigned char FUNCBaro_type;
	extern volatile unsigned short FUNCBaro_C1, FUNCBaro_C2, FUNCBaro_C3,FUNCBaro_C4, FUNCBaro_C5, FUNCBaro_C6;
	extern volatile signed long long FUNCBaro_sensitivity, FUNCBaro_offset;
	unsigned int BaroCrc4(unsigned int n_rem, unsigned char byte);
	
    void SensorInit(void);
    unsigned char GetAccel(signed short * data);
    unsigned char GetGyro(signed short * data);
    unsigned char GetMagneto(signed short * data);
    unsigned int GetBaro(void);
	void TrigBaroTemp(void);
	void TrigBaro(void);
    float GetBaroPressure(void);
    float Pressure2Alt(float pressure);
    float GetBaroTemp(void);
    void SensorSleep(void);

#endif

#if WHO_AM_I == I_AM_HYPO

    // ****************************************************************************
    // *** PWM Ouput Functions
    // ****************************************************************************

    #define PWM_A   (1 << 0)
    #define PWM_B   (1 << 1)
    #define PWM_C   (1 << 2)
    #define PWM_D   (1 << 3)
    #define PWM_S   (1 << 4)
    #define PWM_ALL 0x1f


    extern volatile unsigned char FUNCPWMPostscale;

    void PWMInit(unsigned char channels);

    #if PWM_FILTERS_ON == 0
        static inline void PWMSetA(unsigned int value) {  Timer3SetMatch3(value);   }
        static inline void PWMSetB(unsigned int value) {  Timer3SetMatch2(value);   }
        static inline void PWMSetC(unsigned int value) {  Timer3SetMatch1(value);   }
        static inline void PWMSetD(unsigned int value) {  Timer3SetMatch0(value);   }
        static inline void PWMSetS(unsigned int value) {  Timer2SetMatch3(value);   }
    #else 
        extern volatile unsigned int FUNCPWMA_fil;
        extern volatile unsigned int FUNCPWMB_fil;
        extern volatile unsigned int FUNCPWMC_fil;
        extern volatile unsigned int FUNCPWMD_fil;
        extern volatile unsigned int FUNCPWMS_fil;
        static inline void PWMSetA(unsigned int value) {  FUNCPWMA_fil *= PWM_ABCD_FILTER; FUNCPWMA_fil += value * (1-PWM_ABCD_FILTER); Timer3SetMatch3(FUNCPWMA_fil);   }
        static inline void PWMSetB(unsigned int value) {  FUNCPWMB_fil *= PWM_ABCD_FILTER; FUNCPWMB_fil += value * (1-PWM_ABCD_FILTER); Timer3SetMatch1(FUNCPWMB_fil);   }
        static inline void PWMSetC(unsigned int value) {  FUNCPWMC_fil *= PWM_ABCD_FILTER; FUNCPWMC_fil += value * (1-PWM_ABCD_FILTER); Timer3SetMatch1(FUNCPWMC_fil);   }
        static inline void PWMSetD(unsigned int value) {  FUNCPWMD_fil *= PWM_ABCD_FILTER; FUNCPWMD_fil += value * (1-PWM_ABCD_FILTER); Timer3SetMatch0(FUNCPWMD_fil);   }
        static inline void PWMSetS(unsigned int value) {  FUNCPWMS_fil *= PWM_S_FILTER; FUNCPWMS_fil += value * (1-PWM_S_FILTER); Timer2SetMatch3(FUNCPWMS_fil);   }
    #endif

    static inline void PWMSetABCD(unsigned int valueA, unsigned int valueB, unsigned int valueC, unsigned int valueD) {
        PWMSetA(valueA);
        PWMSetB(valueB);
        PWMSetC(valueC);
        PWMSetD(valueD);
    }

    // ****************************************************************************
    // *** GPS Functions
    // ****************************************************************************
    #if GPS_EN
        #define GPS_ADDR	    0x84
        
        #define ID_NAV_POSECEF  0x0101
        #define ID_NAV_POSLLH   0x0102
        #define ID_NAV_STATUS   0x0103
        #define ID_NAV_SOL      0x0106
        #define ID_NAV_VELNED   0x0112
        #define ID_NAV_TIMEUTC  0x0121

        void GPSInit(void);
        void GPSHotstart(void);
        void GPSSleep(void);
        
        void GPSPoll(unsigned short id);
        
        #if GPS_METHOD == 0
            unsigned short GPSGetMessage(unsigned short id, unsigned char * buffer);
            unsigned char GPSGetFix(void);
            unsigned char GPSGetLocation(signed int * location);
            void GPSClearBuffer(void);
        #endif
        
        #if GPS_METHOD == 1
            extern volatile unsigned char FUNCGPSState, FUNCGPSChecksumA, FUNCGPSChecksumB, FUNCGPSID1, FUNCGPSID2;
            extern volatile unsigned short FUNCGPSLength, FUNCGPSPacket, FUNCGPSID;
            extern unsigned char FUNCGPSBuffer[GPS_BUFFER_SIZE];
            
            typedef struct gps_nav_posecef_struct{
                unsigned int iTOW;
                signed int ecefX;
                signed int ecefY;
                signed int ecefZ;
                unsigned int pAcc;
                unsigned char isNew;
            } PACKED gps_nav_posecef_t;
            
            typedef struct gps_nav_posllh_struct {
                unsigned int iTOW;
                signed int lon;
                signed int lat;
                signed int height;
                signed int hMSL;
                unsigned int hAcc;
                unsigned int vAcc;
                unsigned char isNew;
            } PACKED gps_nav_posllh_t;
            
            typedef struct gps_nav_status_struct {
                unsigned int iTOW;
                unsigned char gpsFix;
                unsigned char flags;
                unsigned char fixStat;
                unsigned char flags2;
                unsigned int ttff;
                unsigned int msss;
                unsigned char isNew;
            } PACKED gps_nav_status_t;
            
            typedef struct gps_nav_sol_struct {
                unsigned int iTOW;
                signed int fTOW;
                signed short week;
                unsigned char gpsFix;
                unsigned char flags;
                signed int ecefX;
                signed int ecefY;
                signed int ecefZ;
                unsigned int pAcc;
                signed int ecefVX;
                signed int ecefVY;
                signed int ecefVZ;
                unsigned int sAcc;
                unsigned short pDOP;
                unsigned char resl;
                unsigned char numSV;
                unsigned int res2;
                unsigned char isNew;
            } PACKED gps_nav_sol_t;
        
            typedef struct gps_nav_velned_struct{
                unsigned int iTOW;
                signed int velN;
                signed int velE;
                signed int velD;
                unsigned int speed;
                unsigned int gSpeed;
                signed int heading;
                unsigned int sAcc;
                unsigned int cAcc;
                unsigned char isNew;
            } PACKED gps_nav_velned_t;
            
            typedef struct gps_nav_timeutc_struct {
                unsigned int iTOW;
                unsigned int tAcc;
                signed int nano;
                unsigned short year;
                unsigned char month;
                unsigned char day;
                unsigned char hour;
                unsigned char min;
                unsigned char sec;
                unsigned char valid;
                unsigned char isNew;
            } PACKED gps_nav_timeutc_t;
                
            void GPSSetRate(unsigned short it, unsigned char rate);
            void GPSFetchData(void);
            extern WEAK void GPSMessage(unsigned short id, unsigned char * buffer, unsigned short length);
        #endif
    #endif
        
    // ****************************************************************************
    // *** Flash Functions
    // ****************************************************************************
    
    #if SSP1_EN & FLASH_EN
        static inline void FlashSEL(void) { Port0Write(PIN23, 0); }
        static inline void FlashCLR(void) { SSP1Wait(); Port0Write(PIN23, 1); }
    
        extern volatile unsigned int FUNCFlashCR0Reset, FUNCFlashCPSRReset;
        
        void FlashInit(void);
        void FlashStart(void);
        void FlashEnd(void);
        void FlashWait(void);
        void FlashWrEn(void);
        void FlashEraseChip(void);
        void FlashErase4k(unsigned int address);
        void FlashErase32k(unsigned int address);
        void FlashErase64k(unsigned int address);
        
        unsigned int FlashGetID(void);
        unsigned char FlashGetMan(void);
        unsigned char FlashGetType(void);
        unsigned char FlashGetCapacity(void);
        
        void FlashRawWriteByte(unsigned int address, unsigned char data);
        void FlashRawWrite(unsigned int address, unsigned char * data, unsigned int length);
        unsigned char FlashRawReadByte(unsigned int address);
        void FlashRawRead(unsigned int address, unsigned char * data, unsigned int length);
        unsigned char FlashVerify(unsigned int address, unsigned char * data, unsigned int length);
        
        // buffered versions of the above
        extern volatile unsigned int FUNCFlashCurrentSector;
        extern unsigned char FUNCFlashSectorBuffer[4096];
        extern volatile unsigned char FUNCFlashSectorChanged;
        void FlashBufferSector(unsigned int address);
        void FlashFlushBuffer(void);
        void FlashWriteByte(unsigned int address, unsigned char data);
        void FlashWrite(unsigned int address, unsigned char * data, unsigned int length);
        unsigned char FlashReadByte(unsigned int address);
        void FlashRead(unsigned int address, unsigned char * data, unsigned int length);
        
    #endif
#endif
    
#if WHO_AM_I == I_AM_HYPO || WHO_AM_I == I_AM_HYPX        
    // ****************************************************************************
    // *** XBee Functions
    // ****************************************************************************
    
    #if UART_EN && SYSTICK_EN && XBEE_EN
    
        #define ID_XBEE_ATCOMMAND   0x08
        #define ID_XBEE_ATCOMMANDQ  0x09
        #define ID_XBEE_ATRESPONSE  0x88
        #define ID_XBEE_TRANSMITSTATUS 0x8b
        #define ID_XBEE_MODEMSTATUS 0x8A
        #define ID_XBEE_RECEIVEPACKET   0x90
        #define ID_XBEE_TRANSMITREQUEST 0x10
        #define IX_XBEE_NODEIDENTIFICATIONINDICATOR 0x95
        
        #define TBUF_LEN    10
    
        typedef struct xbee_modem_status_struct {
            unsigned char status;
            volatile unsigned char isNew;
            unsigned short varLen;
        } PACKED xbee_modem_status_t;

        typedef struct xbee_at_command_struct {
            unsigned char frameID;
            unsigned char ATCommand1;
            unsigned char ATCommand2;
            unsigned char parameterValue[16];
            volatile unsigned char isNew;
            unsigned short varLen;
        } PACKED xbee_at_command_t;

        typedef struct xbee_at_response_struct {
            unsigned char frameID;
            unsigned char ATCommand1;
            unsigned char ATCommand2;
            unsigned char commandStatus;
            unsigned char commandData[8];
            volatile unsigned char isNew;
            unsigned short varLen;
        } PACKED xbee_at_response_t;
        
        typedef struct xbee_transmit_status_struct {
            unsigned char frameID;
            unsigned short networkAddress;
            unsigned char transmitRetryCount;
            unsigned char deliveryStatus;
            unsigned char discoveryStatus;
            volatile unsigned char isNew;
            unsigned short varLen;
        } PACKED xbee_transmit_status_t;
        
        typedef struct xbee_receive_packet_struct {
            unsigned long long sourceAddress;
            unsigned short networkAddress;
            unsigned char receiveOptions;
            unsigned char RFData[255];
            volatile unsigned char isNew;
            unsigned short varLen;
        } PACKED xbee_receive_packet_t;

        typedef struct xbee_transmit_request_struct {
            unsigned char frameID;
            unsigned long long destinationAddress;
            unsigned short networkAddress;
            unsigned char broadcastRadius;
            unsigned char options;
            unsigned char RFData[255];
            volatile unsigned char isNew;
            unsigned short varLen;
        } PACKED xbee_transmit_request_t;
        
        typedef struct xbee_node_identification_indicator_struct {
            unsigned long long senderSourceAddress;
            unsigned short senderNetworkAddress;
            unsigned char receiveOptions;
            unsigned short remoteNetworkAddress;
            unsigned long long remoteSourceAddress;
            unsigned char NIString;
            unsigned char null;
            unsigned short parentNetworkAddress;
            unsigned char deviceType;
            unsigned char sourceEvent;
            unsigned short digiProfileID;
            unsigned short digiManufacturerID;
            volatile unsigned char isNew;
            unsigned short varLen;
        } PACKED xbee_node_identification_indicator_t;
        
        extern xbee_modem_status_t xbee_modem_status;
        extern xbee_at_command_t xbee_at_command;
        extern xbee_at_response_t xbee_at_response;
        extern xbee_transmit_status_t xbee_transmit_status;
        extern xbee_receive_packet_t xbee_receive_packet;
        extern xbee_transmit_request_t xbee_transmit_request;
        extern xbee_node_identification_indicator_t xbee_node_identification_indicator;
        
        extern unsigned char FUNCXBeetBuf[TBUF_LEN];
        extern volatile unsigned char FUNCXBeetBufCount;
        unsigned int XBeetBufCompare(unsigned char * compare, unsigned int length);
        
        extern volatile unsigned int FUNCXBeeState;
        extern volatile unsigned short FUNCXBeeLength, FUNCXBeeID, FUNCXBeePacket;
        extern volatile unsigned char FUNCXBeeChecksum;
        extern unsigned char FUNCXBeeBuffer[XBEE_BUFFER_SIZE];
    
        extern WEAK void XBeeMessage(unsigned char id, unsigned char * buffer, unsigned short length);
        void XBeeInit(void);
        
        void XBeeCommFail(void);
        void XBeeSetDefaults(void);
        void XBeeFactoryReset(void);
        void XBeeCoordinatorJoin(void);
        void XBeeAllowJoin(void);
        void XBeeStopJoin(void);
        void XBeeJoin(void);
        void XBeeSendFrame(unsigned char id, unsigned char * buffer, unsigned short length);
        unsigned char XBeeSendATCommand(void);
        unsigned char XBeeSendPacket(void);
        unsigned char XBeeWriteBroadcast(unsigned char * buffer, unsigned short length);
        unsigned char XBeeWriteCoordinator(unsigned char * buffer, unsigned short length);
        
        static inline void XBeeReset(void) { Port0Init(PIN20); Port0SetOut(PIN20); Port0Write(PIN20, 0); }
        static inline void XBeeRelease(void) { Port0Init(PIN20); Port0SetOut(PIN20); Port0Write(PIN20, 1); }
        static inline void XBeeInhibit(void) { Port0Write(PIN17, 1); IRQDisable(USART_IRQn); }
        static inline void XBeeAllow(void) { Port0Write(PIN17, 0); IRQEnable(USART_IRQn); }
        static inline void XBeeStartBypass(void) { FUNCXBeeState = 0xff;}
        static inline void XBeeStopBypass(void) { FUNCXBeeState = 0;}
        void XBeeInit(void);
        void XBUARTInterrupt(unsigned char UARTData);
        
    #endif
    
#endif

#ifdef __cplusplus
}
#endif 

#endif