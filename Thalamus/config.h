// ****************************************************************************
// *** Copyright (c) 2011, Universal Air Ltd. All rights reserved.
// *** Source and binaries are released under the BSD 3-Clause license
// *** See readme files for the text of the license
// ****************************************************************************
 
#ifndef __CONFIG_H__
#define __CONFIG_H__

#define I_AM_NOTHING        0
#define I_AM_THALAMUS       1
#define I_AM_HYPO           2
#define I_AM_HYPX           3

// ****************************************************************************
// *** BOARD IDENTITY
// ****************************************************************************

#define WHO_AM_I            I_AM_THALAMUS

// ****************************************************************************
// *** Misc Functions
// ****************************************************************************

#define IAP_EN              1           // Set to 1 to enable the IAP functions like Reprogram() (set to 0 to disable and save some RAM)

#define RAND_MERSENNE       0           // Set to 1 to use Mersenne Twister as pseudorandom number generator, otherwise use Linear Congruential    

#define MERSENNE_N          624         // Size of the vector for Mersenne Twister random number generator
#define MERSENNE_M          397         // Coefficient for the Mersenne Twister random number generator

#define RAND_A              16644525    // Coefficient for the linear congruential random number generator
#define RAND_C              32767       // Coefficient for the linear congruential random number generator
 
#define CRP                 0xffffffff  // Code read protection settings
                                        // 0xffffffff = No CRP
                                        // 0x4e697370 = No ISP
                                        // 0x12345678 = CRP1
                                        // 0x87654321 = CRP2
                                        // ********** = CRP3  WARNING! CRP3 STOPS EXTERNAL ACCESS TO PROGRAMMING MODE (user code must provide Reprogram() functionality to update flash)
                                        //                      This is so dangerous, we're not giving you the value, it is up to you to seek this value out from the
                                        //                      LPC1343 manuals and datasheets.



// ****************************************************************************
// *** Clock Functions
// ****************************************************************************
#define DEFAULT_CLOCK       0           // Default start-up/wake-up clock mode (0=72MHz crystal, 1=72MHz IRC, 2=12MHz IRC)
 
#define SYSTICK_EN          1           // Set to 1 to enable SysTick (set to 0 to disable and save some RAM)
#define SYSTICK_STARTUP     1           // Set to 1 to enable SysTick timer during startup
#define SYSTICK_US          1000        // SysTick period in microseconds (maximum: 233016us at 72MHz, 1398101us at 12MHz)
#define SYSTICK_PRIORITY    1           // SysTick interrupt priority

#define WDT_PRIORITY        1           // WDT interrupt priority
#define WDT_MODE            2           // WDT mode (0=reset on WDT timeout, 1=repeated interrupt on WDT timeout, 2=interrupt and then restart WDT)
#define WDT_CLK             1           // WDT clock (0=WDT OSC, 1=IRC)

#define RIT_RESET           1           // Repetitive Interrupt Timer reset (0=don't reset, 1=reset on match)
#define RIT_PRIORITY        4           // Repetitive Interrupt Timer priority (note to self: this needs to be <3 in some circumstances, can't work out why...)

// ****************************************************************************
// *** Power mode Config
// ****************************************************************************
/*
#define AUTOSLEEP           0           // Set to 1 to sleep on end of interrupt function
#define WAKEUP_HYS          0           // Set 1 to enable hysteresis on the WAKEUP pin
*/

#define STARTUP_DELAY       0           // Startup delay (in terms of empty loop

// ****************************************************************************
// *** GPIO Config
// ****************************************************************************
#define PORT_STARTUP_INIT     1         // Whether to initialise all ports during startup

/*
#define PORT0_PRIORITY        2         // Port 0 pins interrupt priority
#define PORT1_PRIORITY        2         // Port 1 pins interrupt priority

*/
// ****************************************************************************
// *** USB HID and MSC functions
// ****************************************************************************
#define USB_EN              1           // Set to 1 to enable USB ROM functions

#define USB_VENDOR_ID       0x1FC9      // The default NXP Vendor ID is 0x1FC9
#define USB_MSC_ID          0x0114      // Product ID for MSC device
#define USB_CDC_ID          0x0232      // Product ID for CDC device
#define USB_HID_ID          0x0003      // Product ID for HID device

#define HID_IN_BYTES        64          // The number of bytes for the HID input report (device to computer)
#define HID_OUT_BYTES       2          // The number of bytes for the HID output report (computer to device)
#define HID_FEATURE_BYTES   5           // The number of bytes for the HID feature report
#define HID_RATE            100          // HID report rate (ms)

#define MSC_BLOCK_SIZE      512         // The block size (bytes) of the MSC device

#define CDC_USE_PLED        0           // Use PLED as a flashing status indicator

#define USB_PRIORITY        2           // USB interrupt priority


// ****************************************************************************
// *** UART Config
// ****************************************************************************
#define UART_EN             1           // Set to 1 to enable UART (set to 0 to disable and save some RAM)
#define UART_FLOW           0           // Set to 1 to enable UART auto-flow control (CTS and RTS)

#define UART_BIT_LENGTH     3           // Bits per UART "word" (3=8-bit, 2=7-bit, 1=6-bit, 0=5-bit)
#define UART_STOP_BITS      0           // Stop bits (0=1 stop bit, 1=2 stop bits (or 1.5 stop bits if 5-bit data mode))
#define UART_PARITY_EN      0           // Parity (0=disable, 1=enable)
#define UART_PARITY         0           // Parity type (0=Odd parity, 1=Even parity, 2=force mark parity, 3=force space parity)
#define UART_FIFO_LEVEL     0           // Interrupt trigger level for buffer (0=Trigger every byte, 1=trigger if below 4 bytes, 2=trigger if below 8 bytes, 3=trigger if below 14 bytes)

#define UART_USE_FBR        1           // Set to 1 to use pre-defined fractional baud rates
#define UART_PRIORITY       2           // UART interrupt priority

#define UART_USE_OUTBUFFER  0           // Set to 1 to enable UART output buffer
#define UART_BUFFER_SIZE    128          // Size of the UART output buffer

// ****************************************************************************
// *** I2C Config
// ****************************************************************************
#define I2C_EN              1           // Set to 1 to enable I2C (set to 0 to disable and save some RAM)
#define I2C_SLAVE_EN        0           // Set to 1 to enable slave mode (set to 0 to save some RAM)
#define I2C_DATA_SIZE       68          // Size of I2C Slave buffer
#define I2C_TIMEOUT         0xff     // Timeout for I2C

#define I2C_FASTMODE_PLUS   0           // Set to 1 for > 400kHz operation

#define I2C_SLAVE_ADR0      0xe8        // Slave address 0, only available in slave mode
#define I2C_SLAVE_ADR1      0x00        // Slave address 1, only available in slave mode
#define I2C_SLAVE_ADR2      0x00        // Slave address 2, only available in slave mode
#define I2C_SLAVE_ADR3      0x00        // Slave address 3, only available in slave mode

#define I2C_PRIORITY        3           // I2C interrupt priority


// ****************************************************************************
// *** SSP/SPI Config
// ****************************************************************************
#define SSP0_EN             1           // Set to 1 to enable SSP0

#define SSP0_SIZE           16          // Transfer size in bits (valid values are 4-bit to 16-bit)
#define SSP0_FORMAT         0           // Frame format (0=SPI, 1=TI, 2=Microware)
#define SSP0_CLK_POL        0           // Clock polarity (0=CLK low between frames, 1=CLK high between frames)
#define SSP0_CLK_PHA        0           // Clock phase (0=Capture data on transition away from inter-frame state, 1=Capture data on transition to inter-frame state)

#define SSP0_SSEL           1           // Use SSEL pin (2=automatically use SSEL, 1=manually use SSEL, 0=don't use)

#define SSP0_INT_LEVEL      0           // Interrupt level (0=interrupt whenever data received, 1=interrupt when FIFO half-full (4 frames)
#define SSP0_PRIORITY       2           // SSP interrupt priority


#define SSP1_EN             1           // Set to 1 to enable SSP0

#define SSP1_SIZE           16          // Transfer size in bits (valid values are 4-bit to 16-bit)
#define SSP1_FORMAT         0           // Frame format (0=SPI, 1=TI, 2=Microware)
#define SSP1_CLK_POL        0           // Clock polarity (0=CLK low between frames, 1=CLK high between frames)
#define SSP1_CLK_PHA        0           // Clock phase (0=Capture data on transition away from inter-frame state, 1=Capture data on transition to inter-frame state)

#define SSP1_SSEL           1           // Use SSEL pin (2=automatically use SSEL, 1=manually use SSEL, 0=don't use)

#define SSP1_INT_LEVEL      0           // Interrupt level (0=interrupt whenever data received, 1=interrupt when FIFO half-full (4 frames)
#define SSP1_PRIORITY       2           // SSP interrupt priority

// ****************************************************************************
// *** Timer Config
// ****************************************************************************
#define TIMER0_PRIORITY     4           // Timer priority
#define TMR0M2_PIN          0           // Select the pin for Timer 0 Match 2 (0=P0[10], 1=P1[15])    
#define TIMER1_PRIORITY     0           // Timer priority
#define TIMER2_PRIORITY     0           // Timer priority
#define TIMER3_PRIORITY     0           // Timer priority

// ****************************************************************************
// *** ADC Config
// ****************************************************************************
#define ADC_MODE            0           // ADC mode (0=on-demand, 1=interrupt)
#define ADC_PRIORITY        1           // ADC interrupt priority
#define ADC_10BIT           0           // Set to enable 10 bit mode
#define ADC_LPWRMODE        0           // Set to enable low power mode (turns ADC off between samples)    

// ****************************************************************************
// *** ILink Config
// ****************************************************************************
#define ILINK_EN            1              // Enable the Interlink functions (0=off, 1=on)
#define ILINK_BUFFER_SIZE   256          // ILink buffer size
#define ILINK_MAX_FETCH     256          // Maximum characters to fetch at once

#if WHO_AM_I == I_AM_THALAMUS
    // ****************************************************************************
    // *** RX Functions (Thalamus only)
    // ****************************************************************************
    #define RX_EN           1              // Enable RX library functions
    #define RX_TYPE         0               // 0: Spektrum satellite decode
                                            // 1: Futaba S.Bus decode
    
    // ****************************************************************************
    // *** PWM Functions (Thalamus only)
    // ****************************************************************************

    #define PWM_DEFAULT_N   1000            // Default PWM outputs (in microseconds), 
    #define PWM_DEFAULT_E   1000            //  use low values (i.e. 1000) for ESCs
    #define PWM_DEFAULT_S   1000
    #define PWM_DEFAULT_W   1000
    #define PWM_DEFAULT_X   1500
    #define PWM_DEFAULT_Y   1500

    #define PWM_FILTERS_ON  0               // PWM output filters (filter is SPR)

    #define PWM_NESW_FILTER 0               // Filter coefficients for SPR
    #define PWM_XY_FILTER   0               // value is between 0 and 1, lower is less filterig

    #define PWM_NESWFREQ    400             // PWM frequency (maximum 450)
    #define PWM_XYFREQ      100             // This value needs to be a fraction of PWM_NESWFREQ

    // ****************************************************************************
    // *** IMU Functions (Thalamus only)
    // ****************************************************************************

    #define ACCEL_RANGE         1           // Set the dynamic range: 0=+/- 2g, 1= +/- 4g, 2=+/-8g, 3=+/-16g
    #define ACCEL_RATE          7           // Set the data rate: 0=off, 1=1Hz, 2=10Hz, 3=25Hz, 4=50Hz, 5=100Hz, 6=200Hz, 7=400Hz, 8=1.620kHz (low power mode ONLY), 9=1.344kHz (normal)/5.376kHz (low power mode)
    #define ACCEL_LOW_POWER     0           // Set to enable low power mode

    #define GYRO_RANGE          2           // Set dynamic range: 0=250dps, 1=500dps, 2=2000dps, 3=2000dps
    #define GYRO_RATE           2           // Set the data rate: 0=100Hz, 1=200Hz, 2=400Hz, 3=800Hz
    #define GYRO_BANDWIDTH      2           // Sets the bandwidth
                                                //      For Gyro rate 0 (100Hz): 0=12.5Hz, 1=25Hz, 2=25Hz, 3=25Hz
                                                //      For Gyro rate 1 (200Hz): 0=12.5Hz, 1=25Hz, 2=50Hz, 3=70Hz
                                                //      For Gyro rate 2 (400Hz): 0=20Hz, 1=25Hz, 2=50Hz, 3=110Hz
                                                //      For Gyro rate 3 (800Hz): 0=30Hz, 1=35Hz, 2=30Hz, 3=110Hz
    #define GYRO_LPF            1           // Set to enable the low pass filter
    
    #define MAGNETO_MODE        0           // Set to 0 for continuous mode, 1 for single-measurement mode
    #define MAGNETO_AVERAGING   3           // Set to 0 for no averaging, 1 for two-sample, 2 for four-sample, and 3 for eight-sample averaging
    #define MAGNETO_RATE        6           // Continuous mode sample rate, 0=0.75Hz, 1=1.5Hz, 2=3Hz, 3=7.5Hz, 4=15Hz, 5=30Hz, 6=75Hz
    #define MAGNETO_BIAS        0           // Sets bias mode for all axes, set to 0 for no bias, 1 for positive bias, 2 for negative bias
    #define MAGNETO_GAIN        1           // Sets gain for certain sensor ranges.  0=+/-0.88Ga, 1=+/-1.3Ga, 2=+/-1.9Ga, 3=+/-2.5Ga, 4=+/-4.0Ga, 5=+/-4.7Ga, 6=+/-5.6Ga, 7=+/-8.1Ga
    #define MAGNETO_TRIES_MAX   5           // When in single-measurement mode, how many tries to read data from the stream before giving up?
    #define MAGNETO_TRIES_DELAY 1           // Millisecond delay between trying to read data from the magneto

    #define BARO_PRES_AVERAGING 9           // Pressure oversampling internal averages: 0=1, 1=2, 2=4, 3=8, 4=16, 5=32, 6=64, 7=128, 8=256, 9=384, 10=512 (512/128 not available on ODR=25/25Hz)
    #define BARO_TEMP_AVERAGING 4           // Temperature oversampling internal averages: 0=1, 1=2, 2=4, 3=8, 4=16, 5=32, 6=64, 7=128 (512/128 not available on ODR=25Hz)
    #define BARO_RATE           7           // Sets the output data rate (pressure/temperature): 0=one shot, 1=1/1Hz, 2=7/1Hz, 3=12.5/1Hz, 4=25/1Hz,  5=7/7Hz, 6=12.5/12.5Hz, 7=25Hz/25Hz

#endif

#if WHO_AM_I == I_AM_HYPO

    // ****************************************************************************
    // *** PWM Functions (Hypo only)
    // ****************************************************************************

    #define PWM_DEFAULT_A   1500           // Default PWM outputs (in microseconds), 
    #define PWM_DEFAULT_B   1500           //  use low values (i.e. 1000) for ESCs
    #define PWM_DEFAULT_C   1500
    #define PWM_DEFAULT_D   1500
    #define PWM_DEFAULT_S   1500

    #define PWM_FILTERS_ON  1              // PWM output filters (filter is SPR)
    #define PWM_ABCD_FILTER 0              // Filter coefficients for SPR
    #define PWM_S_FILTER    0              // value is between 0 and 1, lower is less filterig

    #define PWM_ABCDFREQ    50             // PWM frequency (maximum 450)
    #define PWM_SFREQ       50             // This value needs to be a fraction of PWM_ABCDFREQ

    // ****************************************************************************
    // *** GPS Functions (Hypo only)
    // ****************************************************************************

    #define GPS_EN              1           // Set to 1 to enable GPS code (set to 0 to save some RAM)
    
    #define GPS_5HZ_RATE        1           // Set to 1 to allow up to 5Hz position rate (set to 0 for the default 0Hz)
    #define GPS_AIRBORNE        0           // Set to 1 to allow GPSInit() function to set the dynamic platform model to "Airborne < 1g", and GPSFix() will only return if there is a 3D fix
    
    #define GPS_METHOD          1           // Method, use 0 for the slow but simple polling method, or 1 for the faster but more complex periodic method    
    
    #define GPS_TRIES_MAX       15          // When polling, how many tries to read data from the stream before giving up?
    #define GPS_TRIES_DELAY     100         // When polling, millisecond delay between trying to read data from the stream
    
    #define GPS_BUFFER_SIZE     64          // Size of the GPS buffer, determines the largest packet that can be stored
    
    
    // ****************************************************************************
    // *** Flash Functions (Hypo only)
    // ****************************************************************************
    
    #define FLASH_EN            0           // Set to 1 to enable flash, NOTE: flash operation requires the use of 4kb of RAM for sector buffer!
    
#endif

#if WHO_AM_I == I_AM_HYPX
    #define XBEE_EN             1           // Set to 1 to enable the XBee code

    #define XBEE_POWER_LEVEL    0           // Set transmit power: 4=18dBm/63mW, 3=16dBm/40mW, 2=14dBm/25mW, 1=12dBm/16mW, 0=0dBm/1mW
    #define XBEE_BUFFER_SIZE    128         // XBee buffer size
    #define XBEE_JOINPERIOD     60          // Number of seconds to allow bind
#endif


#endif // __CONFIG_H__