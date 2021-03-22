/**
 * main.c
 *
 * ECE 3849 Lab1: Digital Oscilloscope
 *      Created on: 11/3/2020
 *      Authors: Adam Yang
 *               Prudence Lam
 *
 * Implements a digital oscilloscope on the LCD using the EK-TM4C1294XL LaunchPad
 * with BOOSTXL-EDUMKII BoosterPack.
 *
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include "driverlib/fpu.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "driverlib/timer.h"
#include "Crystalfontz128x128_ST7735.h"
#include "buttons.h"
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "driverlib/pin_map.h"
#include "sampling.h"

#define PWM_FREQUENCY 20000 // PWM frequency = 20 kHz
#define MAX_BUTTON_PRESS 10 // For the FIFO
#define FIFO_SIZE 10        // Maximum items in FIFO

// Global functions
void SignalInit(void);
uint32_t cpu_load_count(void);

// Global variables
float fScale;                                           // Scaling factor
float cpu_load = 0;                                     // CPU Load
float load_count, cpu_unload;                           // For calculating CPU utilization
uint32_t gSystemClock;                                  // [Hz] system clock frequency
volatile uint32_t gTime = 8345;                         // time in hundredths of a second
extern volatile int32_t gADCBufferIndex;                // latest sample index
extern volatile uint16_t gADCBuffer[ADC_BUFFER_SIZE];   // circular buffer
extern volatile uint32_t gADCErrors;

// Voltage scales
const char * const gVoltageScaleStr[] = {"100 mV","200 mV","500 mV","1 V","2V"};


// Main
int main(void)
{
    IntMasterDisable();

    // Enable the Floating Point Unit, and permit ISRs to use it
    FPUEnable();
    FPULazyStackingEnable();

    // Initialize the system clock to 120 MHz
    gSystemClock = SysCtlClockFreqSet(SYSCTL_XTAL_25MHZ |
                                      SYSCTL_OSC_MAIN |
                                      SYSCTL_USE_PLL |
                                      SYSCTL_CFG_VCO_480, 120000000);

    // Initialize the LCD display driver
    Crystalfontz128x128_Init();
    Crystalfontz128x128_SetOrientation(LCD_ORIENTATION_UP); // set screen orientation

    tContext sContext;
    GrContextInit(&sContext, &g_sCrystalfontz128x128); // Initialize the grlib graphics context
    GrContextFontSet(&sContext, &g_sFontFixed6x8);     // Select font

    // Initialize peripherals
    ButtonInit();
    ADCInit();
    SignalInit();

    // Local variables
    uint16_t sample[LCD_HORIZONTAL_MAX];  // Sample on LCD
    int y, y_prev;                        // Pixel y-coordinates
    int i;                                // Counter
    char buttons;                         // Button state
    float fVoltsPerDiv[] = {0.1, 0.2, 0.5, 1, 2};
    int vpd = 4;                          // Volt per division
    int trig_slope = 1;                   // Read trigger state
    int trigger;                          // Trigger state
    char load_str[50];                    // LCD load display

    cpu_unload = cpu_load_count();

    // Full-screen rectangle
    tRectangle rectFullScreen = {0, 0, GrContextDpyWidthGet(&sContext)-1, GrContextDpyHeightGet(&sContext)-1};

    IntMasterEnable();  // Enable interrupts

    while (true) {
        // Get CPU load
        load_count = cpu_load_count();
        cpu_load = 1.0 - (float)load_count/cpu_unload;

        // FIFO for ADC
        while (fifo_get(&buttons))
            switch(buttons)
            {
            case 'a':
                vpd = ++vpd > 4 ? 4 : vpd++;
                break;
            case 'b':
                vpd = --vpd <= 0 ? 0 : vpd--;
                break;
            case 'e':
                trig_slope = !trig_slope;
                break;
            }

        // Fill screen with black
        GrContextForegroundSet(&sContext, ClrBlack);
        GrRectFill(&sContext, &rectFullScreen);

        // Draw grid
        GrContextForegroundSet(&sContext, ClrBlue);
        for(i = -3; i < 4; i++) {
            GrLineDrawH(&sContext, 0, LCD_HORIZONTAL_MAX - 1, LCD_VERTICAL_MAX/2 + i * PIXELS_PER_DIV);
            GrLineDrawV(&sContext, LCD_VERTICAL_MAX/2 + i * PIXELS_PER_DIV, 0, LCD_HORIZONTAL_MAX - 1);

        }

        // Draw wave
        GrContextForegroundSet(&sContext, ClrYellow);

        // Change trigger state depending on user input
        trigger = trig_slope ? RisingTrigger() : FallingTrigger();

        fScale = (VIN_RANGE * PIXELS_PER_DIV) / ((1 << ADC_BITS) * fVoltsPerDiv[vpd]);

        for (i = 0; i < LCD_HORIZONTAL_MAX - 1; i++)
        {
            // Copy waveform into local buffer
             sample[i] = gADCBuffer[ADC_BUFFER_WRAP(trigger - LCD_HORIZONTAL_MAX / 2 + i)];

            // draw lines
            y = LCD_VERTICAL_MAX / 2 - (int)roundf(fScale * ((int)sample[i] - ADC_OFFSET));
            GrLineDraw(&sContext, i, y_prev, i + 1, y);
            y_prev = y;
        }


        // Draw scales
        GrContextForegroundSet(&sContext, ClrWhite); //white text

        if(trig_slope)
        {
            // Draw rising trigger
            GrLineDraw(&sContext, 105, 10, 115, 10);
            GrLineDraw(&sContext, 115, 10, 115, 0);
            GrLineDraw(&sContext, 115, 0, 125, 0);
            GrLineDraw(&sContext, 112, 6, 115, 2);
            GrLineDraw(&sContext, 115, 2, 118, 6);
        }
        else
        {
            // Draw falling trigger
            GrLineDraw(&sContext, 105, 10, 115, 10);
            GrLineDraw(&sContext, 115, 10, 115, 0);
            GrLineDraw(&sContext, 115, 0, 125, 0);
            GrLineDraw(&sContext, 112, 3, 115, 7);
            GrLineDraw(&sContext, 115, 7, 118, 3);
        }

        // Constant time scale, 20 us
        GrStringDraw(&sContext, "20 us", -1, 4, 0, false);

        // Voltage scale
        GrStringDraw(&sContext, gVoltageScaleStr[vpd], -1, 50, 0, false);

        // CPU load
        snprintf(load_str, sizeof(load_str), "CPU load = %.1f%%", cpu_load*100);
        GrStringDraw(&sContext, load_str, -1, 0, 120, false);

        GrFlush(&sContext); //Flush to display
    }
}


/**Hardware initialization
 * Produces 20 kHz PWM square wave with 40% duty cycle on PF2 and PF3
 */
void SignalInit(void)
{
    // configure M0PWM2, at GPIO PF2, which is BoosterPack 1 header C1 (2nd from right) pin 2
    // configure M0PWM3, at GPIO PF3, which is BoosterPack 1 header C1 (2nd from right) pin 3
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_2 | GPIO_PIN_3); // PF2 = M0PWM2, PF3 = M0PWM3
    GPIOPinConfigure(GPIO_PF2_M0PWM2);
    GPIOPinConfigure(GPIO_PF3_M0PWM3);
    GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_2 | GPIO_PIN_3, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);

    // configure the PWM0 peripheral, gen 1, outputs 2 and 3
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    PWMClockSet(PWM0_BASE, PWM_SYSCLK_DIV_1); // use system clock without division
    PWMGenConfigure(PWM0_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, roundf((float)gSystemClock/PWM_FREQUENCY));
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, roundf((float)gSystemClock/PWM_FREQUENCY*0.4f)); // 40% duty cycle
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3, roundf((float)gSystemClock/PWM_FREQUENCY*0.4f));
    PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT | PWM_OUT_3_BIT, true);
    PWMGenEnable(PWM0_BASE, PWM_GEN_1);
}

/** Count CPU load
 *  @return CPU load as a 32-bit unsigned integer
 */
uint32_t cpu_load_count(void)
{
    uint32_t i = 0;
    TimerIntClear(TIMER3_BASE, TIMER_TIMA_TIMEOUT);
    TimerEnable(TIMER3_BASE,TIMER_A);
    while (!(TimerIntStatus(TIMER3_BASE, false) & TIMER_TIMA_TIMEOUT)) {
        i++;
    }
    return i;
}
