// ****************************************************************************
// *** Copyright (c) 2011, Universal Air Ltd. All rights reserved.
// *** Source and binaries are released under the BSD 3-Clause license
// *** See readme_forebrain.txt files for the text of the license
// ****************************************************************************

#include "LPC1347.h"
#include "thal.h"

#ifndef WEAK
    #define WEAK_ALIAS(f) __attribute__ ((weak, alias (#f)))
    #define WEAK __attribute__ ((weak))
    #define ALIAS(f) __attribute__ ((alias (#f)));
#endif

    
// ****************************************************************************
// *** Interrupt Handlers
// ****************************************************************************

#if defined (__cplusplus)
extern "C" {
#endif

extern WEAK int main(void);
extern WEAK void setup(void);
extern WEAK void loop(void);

extern void SystemInit(void);

__attribute__ ((section(".after_vectors"))) void GeneralFault(void) {
	unsigned int i;
    LPC_IOCON->PIO0_1 = 0x50;
    LPC_IOCON->PIO0_3 = 0x50;
    LPC_GPIO->DIR[0] |= 0x000a;
    
    LPC_GPIO->MASK[0] = ~0x000a;
    LPC_GPIO->MPIN[0] = 0x0002;
    
    while(1) {
        LPC_GPIO->NOT[0] = 0x000a;
        for(i=0; i<500000; i++) {__NOP();}
    }
}

__attribute__ ((section(".after_vectors"))) void EmptyFunction(void) {
	return;
}

void NMI_Handler(void)          WEAK_ALIAS(GeneralFault);
void HardFault_Handler(void)    WEAK_ALIAS(GeneralFault);
void MemManage_Handler(void)    WEAK_ALIAS(GeneralFault);
void BusFault_Handler(void)     WEAK_ALIAS(GeneralFault);
void UsageFault_Handler(void)   WEAK_ALIAS(GeneralFault);
void SVCall_Handler(void)       WEAK_ALIAS(GeneralFault);
void DebugMon_Handler(void)     WEAK_ALIAS(GeneralFault);
void PendSV_Handler(void)       WEAK_ALIAS(GeneralFault);

void PIN_INT0_IRQHandler(void)  WEAK_ALIAS(EmptyFunction);
void PIN_INT1_IRQHandler(void)  WEAK_ALIAS(EmptyFunction);
void PIN_INT2_IRQHandler(void)  WEAK_ALIAS(EmptyFunction);
void PIN_INT3_IRQHandler(void)  WEAK_ALIAS(EmptyFunction);
void PIN_INT4_IRQHandler(void)  WEAK_ALIAS(EmptyFunction);
void PIN_INT5_IRQHandler(void)  WEAK_ALIAS(EmptyFunction);
void PIN_INT6_IRQHandler(void)  WEAK_ALIAS(EmptyFunction);
void PIN_INT7_IRQHandler(void)  WEAK_ALIAS(EmptyFunction);
void GINT0_IRQHandler(void)     WEAK_ALIAS(EmptyFunction);
void GINT1_IRQHandler(void)     WEAK_ALIAS(EmptyFunction);
void RIT_IRQHandler(void)       WEAK_ALIAS(EmptyFunction);
void SSP1_IRQHandler(void)      WEAK_ALIAS(EmptyFunction);
void I2C_IRQHandler(void)       WEAK_ALIAS(EmptyFunction);
void CT16B0_IRQHandler(void)    WEAK_ALIAS(EmptyFunction);
void CT16B1_IRQHandler(void)    WEAK_ALIAS(EmptyFunction);
void CT32B0_IRQHandler(void)    WEAK_ALIAS(EmptyFunction);
void CT32B1_IRQHandler(void)    WEAK_ALIAS(EmptyFunction);
void SSP0_IRQHandler(void)      WEAK_ALIAS(EmptyFunction);
void USART_IRQHandler(void)     WEAK_ALIAS(EmptyFunction);
void USB_IRQHandler(void)       WEAK_ALIAS(EmptyFunction);
void USB_FIQHandler(void)       WEAK_ALIAS(EmptyFunction);
void ADC_IRQHandler(void)       WEAK_ALIAS(EmptyFunction);
void WDT_IRQHandler(void)       WEAK_ALIAS(EmptyFunction);
void BOD_IRQHandler(void)       WEAK_ALIAS(EmptyFunction);
void FMC_IRQHandler(void)       WEAK_ALIAS(EmptyFunction);
void OSCFAIL_IRQHandler(void)   WEAK_ALIAS(EmptyFunction);
void PVTCIRCUIT_IRQHandler(void)    WEAK_ALIAS(EmptyFunction);
void USBWakeup_IRQHandler(void) WEAK_ALIAS(EmptyFunction);
void SysTick_Handler(void)      WEAK_ALIAS(EmptyFunction);
void IntDefaultHandler(void)    WEAK_ALIAS(EmptyFunction);

extern void __stack_top(void);

#if defined (__cplusplus)
} // extern "C"
#endif

// ****************************************************************************
// *** Startup functions
// ****************************************************************************

__attribute__ ((section(".after_vectors")))
void data_init(unsigned int romstart, unsigned int start, unsigned int len) {
	unsigned int *pulDest = (unsigned int*) start;
	unsigned int *pulSrc = (unsigned int*) romstart;
	unsigned int loop;
	for (loop = 0; loop < len; loop = loop + 4)
		*pulDest++ = *pulSrc++;
}

__attribute__ ((section(".after_vectors")))
void bss_init(unsigned int start, unsigned int len) {
	unsigned int *pulDest = (unsigned int*) start;
	unsigned int loop;
	for (loop = 0; loop < len; loop = loop + 4)
		*pulDest++ = 0;
}

extern unsigned int __data_section_table;
extern unsigned int __data_section_table_end;
extern unsigned int __bss_section_table;
extern unsigned int __bss_section_table_end;

__attribute__ ((section(".after_vectors")))
void Reset_Handler(void) {
    // Copy the data sections from flash to SRAM.
	unsigned int LoadAddr, ExeAddr, SectionLen;
	unsigned int *SectionTableAddr;

	// Load base address of Global Section Table
	SectionTableAddr = &__data_section_table;

    // Copy the data sections from flash to SRAM.
	while (SectionTableAddr < &__data_section_table_end) {
		LoadAddr = *SectionTableAddr++;
		ExeAddr = *SectionTableAddr++;
		SectionLen = *SectionTableAddr++;
		data_init(LoadAddr, ExeAddr, SectionLen);
	}
	// At this point, SectionTableAddr = &__bss_section_table;
	// Zero fill the bss segment
	while (SectionTableAddr < &__bss_section_table_end) {
		ExeAddr = *SectionTableAddr++;
		SectionLen = *SectionTableAddr++;
		bss_init(ExeAddr, SectionLen);
	}

    #if STARTUP_DELAY
        volatile unsigned int i;
        for(i=0; i<STARTUP_DELAY; i++);
    #endif
    
    // Set clock mode, DEFAULT_CLOCK is defined in config.h, and the default behaviour
    // is to set the clock to 72MHz from the external crystal.  Using defines here to
    // reduce code space
    #if DEFAULT_CLOCK == XTAL
        ClockModeXTAL();
    #elif DEFAULT_CLOCK == IRC72
        ClockModeIRC72();
    #elif DEFAULT_CLOCK == IRC12
        ClockModeIRC12();
    #endif
    
    LPC_SYSCON->SYSAHBCLKDIV  = 1;
    LPC_SYSCON->SYSAHBCLKCTRL |= (1 << 6) | (1 << 16);

    // Set all pins to digital inputs (except P0[0] which is the reset button)
	#if PORT_STARTUP_INIT
        Port0Init(ALL & ~PIN0);	
        Port1Init(ALL);
    #endif

    // Initialise and start the system tick timer if allowed by the SYSTICK_EN
    // definition in config.h, if the system tick timer is running, then the Delay()
    // function will use it, otherwise Delay() will use a fixed loop which is not
    // accurate when there are interrupts running, as any interrupt would stop the
    // loop and cuase the delay to be longer than expected
	#if SYSTICK_EN && SYSTICK_STARTUP
		SysTickInit();
	#endif

    // Run the user-supplied setup() function if it exists
	if(setup) {
		setup();
	}
    
    // Run the user-supplied main() function if it exists
	if(main) {
		main();
	}
    
    // Loop the user-supplied setup() function if it exists
	if(loop) {
		while(1) loop();
	}
    
    // Do nothing (except handle interrupts)
	while(1);
}


// ****************************************************************************
// *** Vector Table
// ****************************************************************************

__attribute__ ((section(".vectors"), used))
const void *vectors[] = {
    &__stack_top,
    Reset_Handler,
    NMI_Handler,
    HardFault_Handler,
    MemManage_Handler,
    BusFault_Handler,
    UsageFault_Handler,
    0,0,0,0,
    SVCall_Handler,
    DebugMon_Handler,
    0,
    PendSV_Handler,
    SysTick_Handler,
    PIN_INT0_IRQHandler,
    PIN_INT1_IRQHandler,
    PIN_INT2_IRQHandler,
    PIN_INT3_IRQHandler,
    PIN_INT4_IRQHandler,
    PIN_INT5_IRQHandler,
    PIN_INT6_IRQHandler,
    PIN_INT7_IRQHandler,
    GINT0_IRQHandler,
    GINT1_IRQHandler,
    0,0,
    RIT_IRQHandler,
    0,
    SSP1_IRQHandler,
    I2C_IRQHandler,
    CT16B0_IRQHandler,
    CT16B1_IRQHandler,
    CT32B0_IRQHandler,
    CT32B1_IRQHandler,
    SSP0_IRQHandler,
    USART_IRQHandler,
    USB_IRQHandler,
    USB_FIQHandler,
    ADC_IRQHandler,
    WDT_IRQHandler,
    BOD_IRQHandler,
    FMC_IRQHandler,
    OSCFAIL_IRQHandler,
    PVTCIRCUIT_IRQHandler,
    USBWakeup_IRQHandler,
    0,
};