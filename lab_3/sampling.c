/**
 * sampling.c
 *
 *  Created on: Aug 12, 2012, modified 11/23/2020
 *      Author: Gene Bogdanov
 *              Adam Yang
 *              Prudence Lam
 *              Sirut Buasai
 *
 * For ADC Sampling and ISR
 */

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "Crystalfontz128x128_ST7735.h"
#include "buttons.h"
#include "inc/hw_memmap.h"
#include "sysctl_pll.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "driverlib/pin_map.h"
#include "sampling.h"
#include "kiss_fft.h"
#include "_kiss_fft_guts.h"


#include "driverlib/adc.h"
#include "inc/tm4c1294ncpdt.h"

/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>
#include <xdc/cfg/global.h>

/* From Lab 3 */
#include "driverlib/udma.h"

#define ADC_BUFFER_SIZE 2048                             // size must be a power of 2
#define ADC_OFFSET 2048
#define ADC_BUFFER_WRAP(i) ((i) & (ADC_BUFFER_SIZE - 1)) // index wrapping macro
#define VIN_RANGE 3.3                                    // total V_in range
#define PIXELS_PER_DIV 20                                // LCD pixels per voltage division
#define ADC_BITS 12                                      // number of bits in ADC

#define PI 3.14159265358979f
#define NFFT 1024 // FFT length
#define KISS_FFT_CFG_SIZE (sizeof(struct kiss_fft_state)+sizeof(kiss_fft_cpx)*(NFFT-1))


extern uint32_t gSystemClock; // [Hz] system clock frequency

#define PWM_FREQUENCY 20000 // PWM frequency = 20 kHz
#define MAX_BUTTON_PRESS 10 // For the FIFO
#define FIFO_SIZE 10        // Maximum items in FIFO
#define MAILBOX_TIMEOUT 15  // in system tick periods

//volatile int32_t gADCBufferIndex = ADC_BUFFER_SIZE - 1;  // latest sample index
volatile uint16_t gADCBuffer[ADC_BUFFER_SIZE];           // circular buffer
volatile uint32_t gADCErrors = 0;                        // number of missed ADC deadlines
//extern volatile int32_t gADCBufferIndex;                // latest sample index
volatile int32_t bIndex;
extern volatile uint16_t gADCBuffer[ADC_BUFFER_SIZE];   // circular buffer
volatile uint16_t sample[LCD_HORIZONTAL_MAX];           // waveform buffer
volatile uint16_t fft_sample[NFFT];                     // fft buffer
volatile uint16_t displayBuffer[LCD_HORIZONTAL_MAX];    // display buffer

// from Lab 3
volatile bool gDMAPrimary = true; // is DMA occurring in the primary channel?

/** Initialize ADC hardware
 */
void ADCInit(void)
{
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
        GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_0); // GPIO setup for analog input AIN3
        SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0); // initialize ADC peripherals
        SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC1);

        // ADC clock
        uint32_t pll_frequency = SysCtlFrequencyGet(CRYSTAL_FREQUENCY);
        uint32_t pll_divisor = (pll_frequency - 1) / (16 * ADC_SAMPLING_RATE) + 1; //round up
        ADCClockConfigSet(ADC0_BASE, ADC_CLOCK_SRC_PLL | ADC_CLOCK_RATE_FULL, pll_divisor);
        ADCClockConfigSet(ADC1_BASE, ADC_CLOCK_SRC_PLL | ADC_CLOCK_RATE_FULL, pll_divisor);
        ADCSequenceDisable(ADC1_BASE, 0);  // choose ADC1 sequence 0; disable before configuring
        ADCSequenceConfigure(ADC1_BASE, 0, ADC_TRIGGER_ALWAYS, 0);  // specify the "Always" trigger
        ADCSequenceStepConfigure(ADC1_BASE, 0, 0, ADC_CTL_CH3 | ADC_CTL_END | ADC_CTL_IE);
                                           // in the 0th step, sample channel 3 (AIN3)
                                           // enable interrupt, and make it the end of sequence

        ADCSequenceEnable(ADC1_BASE, 0);   // enable the sequence. it is now sampling
//        ADCIntEnable(ADC1_BASE, 0);        // enable sequence 0 interrupt in the ADC1 peripheral

        ADCSequenceDMAEnable(ADC1_BASE, 0); // enable DMA for ADC1 sequence 0
        ADCIntEnableEx(ADC1_BASE, ADC_INT_DMA_SS0); // enable ADC1 sequence 0 DMA interrupt


}

/* Search for rising edge trigger using trigger search algorithm
 * @return Integer index of rising edge trigger
 */
int RisingTrigger(void)
{
    bIndex = getADCBufferIndex();
    // Step 1
//    int x = gADCBufferIndex - LCD_HORIZONTAL_MAX / 2; // Half screen width; don't use a magic number
    int x = bIndex - LCD_HORIZONTAL_MAX / 2; // Half screen width; don't use a magic number

    // Step 2
    int x_stop = x - ADC_BUFFER_SIZE/2; // Half the ADC buffer
    for (; x > x_stop; x--) {
        if (    gADCBuffer[ADC_BUFFER_WRAP(x)] >= ADC_OFFSET &&
                gADCBuffer[ADC_BUFFER_WRAP(x-1)]< ADC_OFFSET) // Older sample
            break;
    }
    // Step 3
    if (x == x_stop) // for loop ran to the end
        x = bIndex - LCD_HORIZONTAL_MAX / 2; // reset x back to how it was initialized
    return x;
}


/* Search for rising edge trigger using trigger search algorithm
 * @return Integer index of falling edge trigger
 */
int FallingTrigger(void)   // search for falling edge trigger
{
    bIndex = getADCBufferIndex();
    // Step 1
//    int x = gADCBufferIndex - LCD_HORIZONTAL_MAX / 2 /* half screen width; don't use a magic number */;
    int x = bIndex - LCD_HORIZONTAL_MAX / 2 /* half screen width; don't use a magic number */;

    // Step 2
    int x_stop = x - ADC_BUFFER_SIZE/2; // Half the ADC buffer
    for (; x > x_stop; x--) {
        if (    gADCBuffer[ADC_BUFFER_WRAP(x)] < ADC_OFFSET &&
                gADCBuffer[ADC_BUFFER_WRAP(x-1)] >= ADC_OFFSET) // Older sample
            break;
    }
    // Step 3
    if (x == x_stop) // for loop ran to the end
        x = bIndex - LCD_HORIZONTAL_MAX / 2; // reset x back to how it was initialized
    return x;
}

// From Lab 3
int32_t getADCBufferIndex(void)
{
    int32_t index;
    IArg gateKey = GateTask_enter(gateTask0);
    if (gDMAPrimary) {  // DMA is currently in the primary channel
        index = ADC_BUFFER_SIZE/2 - 1 -
                uDMAChannelSizeGet(UDMA_SEC_CHANNEL_ADC10 | UDMA_PRI_SELECT);
        GateTask_leave(gateTask0, gateKey);
    }
    else {              // DMA is currently in the alternate channel
        index = ADC_BUFFER_SIZE - 1 -
                uDMAChannelSizeGet(UDMA_SEC_CHANNEL_ADC10 | UDMA_ALT_SELECT);
        GateTask_leave(gateTask0, gateKey);
    }
    return index;
}



void ADC_ISR(void) // DMA (From Lab 3)
{
    ADCIntClearEx(ADC1_BASE, ADC_INT_DMA_SS0); // clear the ADC1 sequence 0 DMA interrupt flag


    // Check the primary DMA channel for end of transfer, and restart if needed.
    if (uDMAChannelModeGet(UDMA_SEC_CHANNEL_ADC10 | UDMA_PRI_SELECT) == UDMA_MODE_STOP) {
        uDMAChannelTransferSet(UDMA_SEC_CHANNEL_ADC10 | UDMA_PRI_SELECT,
                               UDMA_MODE_PINGPONG, (void*)&ADC1_SSFIFO0_R,
                               (void*)&gADCBuffer[0], ADC_BUFFER_SIZE/2); // restart the primary channel (same as setup)
        gDMAPrimary = false; // DMA is currently occurring in the alternate buffer
    }

    // Check the alternate DMA channel for end of transfer, and restart if needed.
    // Also set the gDMAPrimary global.
    if(uDMAChannelModeGet(UDMA_SEC_CHANNEL_ADC10 | UDMA_ALT_SELECT) == UDMA_MODE_STOP) {
        uDMAChannelTransferSet(UDMA_SEC_CHANNEL_ADC10 | UDMA_ALT_SELECT,
                               UDMA_MODE_PINGPONG, (void*)&ADC1_SSFIFO0_R,
                               (void*)&gADCBuffer[ADC_BUFFER_SIZE/2], ADC_BUFFER_SIZE/2); // restart alternate channel
        gDMAPrimary = true;
    }


    // The DMA channel may be disabled if the CPU is paused by the debugger.
    if (!uDMAChannelIsEnabled(UDMA_SEC_CHANNEL_ADC10)) {
        uDMAChannelEnable(UDMA_SEC_CHANNEL_ADC10); // re-enable the DMA channel
    }
}



/* From Lab 2 */
/** ISR for ADC Sampling
 */
//void ADC_ISR(void)
//{
//    ADC1_ISC_R = ADC_ISC_IN0;           // clear ADC1 sequence0 interrupt flag in the ADCISC register
//    if (ADC1_OSTAT_R & ADC_OSTAT_OV0) { // check for ADC FIFO overflow
//        gADCErrors++;                   // count errors
//        ADC1_OSTAT_R = ADC_OSTAT_OV0;   // clear overflow condition
//    }
//    gADCBuffer[
//               gADCBufferIndex = ADC_BUFFER_WRAP(gADCBufferIndex + 1)
//               ] = ADC1_SSFIFO0_R;               // read sample from the ADC1 sequence 0 FIFO
//}
