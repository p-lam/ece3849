/**
 * sampling.c
 *
 *  Created on: Aug 12, 2012, modified 11/23/2020
 *      Author: Gene Bogdanov
 *              Adam Yang
 *              Prudence Lam
 *
 * For ADC Sampling and ISR
 */

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "inc/hw_memmap.h"
#include "buttons.h"
#include "sysctl_pll.h"
#include "driverlib/adc.h"
#include "inc/tm4c1294ncpdt.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "driverlib/pwm.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"
#include "Crystalfontz128x128_ST7735.h"
#include "sampling.h"
#include <math.h>
#include "kiss_fft.h"
#include "_kiss_fft_guts.h"

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

volatile int32_t gADCBufferIndex = ADC_BUFFER_SIZE - 1;  // latest sample index
volatile uint16_t gADCBuffer[ADC_BUFFER_SIZE];           // circular buffer
volatile uint32_t gADCErrors = 0;                        // number of missed ADC deadlines
extern volatile int32_t gADCBufferIndex;                // latest sample index
extern volatile uint16_t gADCBuffer[ADC_BUFFER_SIZE];   // circular buffer
volatile uint16_t sample[LCD_HORIZONTAL_MAX];           // waveform buffer
volatile uint16_t fft_sample[NFFT];                     // fft buffer
volatile uint16_t displayBuffer[LCD_HORIZONTAL_MAX];    // display buffer


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
        ADCIntEnable(ADC1_BASE, 0);        // enable sequence 0 interrupt in the ADC1 peripheral

}

/* Search for rising edge trigger using trigger search algorithm
 * @return Integer index of rising edge trigger
 */
int RisingTrigger(void)
{
    // Step 1
    int x = gADCBufferIndex - LCD_HORIZONTAL_MAX / 2; // Half screen width; don�t use a magic number

    // Step 2
    int x_stop = x - ADC_BUFFER_SIZE/2; // Half the ADC buffer
    for (; x > x_stop; x--) {
        if (    gADCBuffer[ADC_BUFFER_WRAP(x)] >= ADC_OFFSET &&
                gADCBuffer[ADC_BUFFER_WRAP(x-1)]< ADC_OFFSET) // Older sample
            break;
    }
    // Step 3
    if (x == x_stop) // for loop ran to the end
        x = gADCBufferIndex - LCD_HORIZONTAL_MAX / 2; // reset x back to how it was initialized
    return x;
}


/* Search for rising edge trigger using trigger search algorithm
 * @return Integer index of falling edge trigger
 */
int FallingTrigger(void)   // search for falling edge trigger
{
    // Step 1
    int x = gADCBufferIndex - LCD_HORIZONTAL_MAX / 2 /* half screen width; don�t use a magic number */;
    // Step 2
    int x_stop = x - ADC_BUFFER_SIZE/2; // Half the ADC buffer
    for (; x > x_stop; x--) {
        if (    gADCBuffer[ADC_BUFFER_WRAP(x)] < ADC_OFFSET &&
                gADCBuffer[ADC_BUFFER_WRAP(x-1)] >= ADC_OFFSET) // Older sample
            break;
    }
    // Step 3
    if (x == x_stop) // for loop ran to the end
        x = gADCBufferIndex - LCD_HORIZONTAL_MAX / 2; // reset x back to how it was initialized
    return x;
}


/** ISR for ADC Sampling
 */
void ADC_ISR(void)
{
    ADC1_ISC_R = ADC_ISC_IN0;           // clear ADC1 sequence0 interrupt flag in the ADCISC register
    if (ADC1_OSTAT_R & ADC_OSTAT_OV0) { // check for ADC FIFO overflow
        gADCErrors++;                   // count errors
        ADC1_OSTAT_R = ADC_OSTAT_OV0;   // clear overflow condition
    }
    gADCBuffer[
               gADCBufferIndex = ADC_BUFFER_WRAP(gADCBufferIndex + 1)
               ] = ADC1_SSFIFO0_R;               // read sample from the ADC1 sequence 0 FIFO
}
