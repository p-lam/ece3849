/** ECE 3849 Lab2 Oscilloscope, Spectrum Analyzer with RTOS
 *
 * Adam Yang
 * Prudence Lam    11/23/2017
 */

/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>
#include <xdc/cfg/global.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>

/* From Lab 1 */
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <math.h>
#include "driverlib/interrupt.h"
#include "driverlib/fpu.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "Crystalfontz128x128_ST7735.h"
#include "buttons.h"
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "driverlib/pin_map.h"
#include "sampling.h"
#include "kiss_fft.h"
#include "_kiss_fft_guts.h"

#define PI 3.14159265358979f
#define NFFT 1024           // FFT length
#define KISS_FFT_CFG_SIZE (sizeof(struct kiss_fft_state)+sizeof(kiss_fft_cpx)*(NFFT-1))
#define PWM_FREQUENCY 20000 // PWM frequency = 20 kHz
#define MAX_BUTTON_PRESS 10 // For the FIFO
#define FIFO_SIZE 10        // Maximum items in FIFO
#define NFFT 1024           // Length of FFT
#define MAILBOX_TIMEOUT 15  // in system tick periods

/* Global functions */
void SignalInit(void);
uint32_t cpu_load_count(void);

/* Global variables */
float fScale;                                           // scaling factor
float cpu_load = 0;                                     // CPU Load
float load_count, cpu_unload;                           // for calculating CPU utilization
uint32_t gSystemClock = 120000000;                      // [Hz] system clock frequency                                // [Hz] system clock frequency
volatile uint32_t gTime = 8345;                         // time in hundredths of a second
extern volatile int32_t gADCBufferIndex;                // latest sample index
extern volatile uint16_t gADCBuffer[ADC_BUFFER_SIZE];   // circular buffer
volatile uint16_t sample[LCD_HORIZONTAL_MAX];           // waveform buffer
volatile uint16_t fft_sample[NFFT];                     // fft buffer
volatile int displayBuffer[LCD_HORIZONTAL_MAX];         // display buffer
extern volatile uint32_t gADCErrors;
char mbValue = 0;
char oscillo_mode = 1;
static char kiss_fft_cfg_buffer[KISS_FFT_CFG_SIZE];     // Kiss FFT config memory
size_t buffer_size = KISS_FFT_CFG_SIZE;
kiss_fft_cfg cfg;                                       // Kiss FFT config
static kiss_fft_cpx in[NFFT], out[NFFT];                // complex waveform and spectrum buffers
float out_db[NFFT];                                     // db spectrum buffer

//from Lab 1
int vpd = 4;
int trig_slope = 1;
float fVoltsPerDiv[] = {0.1, 0.2, 0.5, 1.0, 2.0};
const char * const gVoltageScaleStr[] = {"100 mV","200 mV","500 mV","1 V","2V"};

/* Main */
int main(void)
{
    IntMasterDisable();

    Crystalfontz128x128_Init();
    Crystalfontz128x128_SetOrientation(LCD_ORIENTATION_UP); // set screen orientation

    tContext sContext;
    GrContextInit(&sContext, &g_sCrystalfontz128x128);      // Initialize the grlib graphics context
    GrContextFontSet(&sContext, &g_sFontFixed6x8);          // Select font

    // hardware initialization goes here
    SignalInit();
    ButtonInit();
    ADCInit();

    /* Start BIOS */
    BIOS_start();

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

void clock0Task() {
    Semaphore_post(semaphoreButton);
}

/** High Priority Task for button sensing
 * A Mailbox is used for shared data protection.
 */
void buttonTask() {
    char buttonRead;
    while(1) {
        Semaphore_pend(semaphoreButton, BIOS_WAIT_FOREVER);

        // read hardware button state
        uint32_t gpio_buttons =
                GPIOPinRead(GPIO_PORTJ_BASE, 0xff) & (GPIO_PIN_1 | GPIO_PIN_0); // EK-TM4C1294XL buttons in positions 0 and 1

        // read Joystick Select state
        gpio_buttons |= GPIOPinRead(GPIO_PORTD_BASE,0xff) & (GPIO_PIN_4);   // Joystick Select in position 4

        // read Boosterpack buttom SW1
        gpio_buttons |= (GPIOPinRead(GPIO_PORTH_BASE,0xff) & (GPIO_PIN_1)) << 1;   // BP Button SW1 in position 2

        // read Boosterpack button SW2
        gpio_buttons |= (GPIOPinRead(GPIO_PORTK_BASE,0xff) & (GPIO_PIN_6)) >> 3;   // BP Button SW2 in position 3

        gpio_buttons = ~gpio_buttons;

        uint32_t old_buttons = gButtons;              // save previous button state
        ButtonDebounce(gpio_buttons);                 // Run the button debouncer. The result is in gButtons.
        ButtonReadJoystick();                         // Convert joystick state to button presses. The result is in gButtons.
        uint32_t presses = ~old_buttons & gButtons;   // detect button presses (transitions from not pressed to pressed)
        presses |= ButtonAutoRepeat();                // autorepeat presses if a button is held long enough


        if (presses & 1) { // EK-TM4C1294XL button SW1 is pressed
            buttonRead = 'a';
            Mailbox_post(mailbox0, &buttonRead, BIOS_NO_WAIT);
        }

        if(presses & 2) {  // EK-TM$C129XL button SW2 is pressed
            buttonRead = 'b';
            Mailbox_post(mailbox0, &buttonRead, BIOS_NO_WAIT);
        }

        if(presses & 4) {   // BoosterPack button SW1 is pressed
            buttonRead = 'c';
            Mailbox_post(mailbox0, &buttonRead, BIOS_NO_WAIT);
        }

        if(presses & 8) {  // BoosterPack button SW2 is pressed
            buttonRead = 'd';
            Mailbox_post(mailbox0, &buttonRead, BIOS_NO_WAIT);
        }
    }
}

/** Processes button state at a lower priority than buttonTask()
 * and changes the corresponding voltage scales, system mode, or trigger edge. 
 */
void userInputTask() {
    char buttonRead;

    while(1) {
        if(Mailbox_pend(mailbox0, &buttonRead, BIOS_WAIT_FOREVER)) {
            switch(buttonRead) {
            case 'a':   // increase the voltage scale
                vpd = vpd >= 4 ? 4 : vpd++;
                break;
            case 'b':   // decrease the voltage scale
                vpd = vpd <= 0 ? 0 : vpd--;
                break;
            case 'c':   // toggle oscilloscope/spectrum mode
                oscillo_mode = !oscillo_mode;
                break;
            case 'd':   // toggle rising/falling edge
                trig_slope = !trig_slope;
                break;
            }
            Semaphore_post(semaphoreDisplay);
        }
    }
}

/** Highest priority task to search for ADC buffer trigger,
 * copy waveform into buffer, signal Processing Task, then block
 */
void waveformTask(UArg arg1, UArg arg2) {
    IntMasterEnable();
    int i;
    int trigger;

    while(1) {
        Semaphore_pend(semaphoreWaveform, BIOS_WAIT_FOREVER);
        trigger = trig_slope ? RisingTrigger() : FallingTrigger();

        if(oscillo_mode) {  // oscilloscope mode
            for (i = 0; i < LCD_HORIZONTAL_MAX - 1; i++) {
                sample[i] = gADCBuffer[ADC_BUFFER_WRAP(trigger - LCD_HORIZONTAL_MAX / 2 + i)];
            }
            Semaphore_post(semaphoreProcessing);
        }
        else {  // spectrum mode
            for (i = 0; i < NFFT; i++) {
                fft_sample[i] = gADCBuffer[ADC_BUFFER_WRAP(gADCBufferIndex - i)];
            }
            Semaphore_post(semaphoreProcessing);
        }

    }
}

/** Low priority task to scale the values from waveformTask().
 * Pends to both display and waveform semaphores for data display and 
 * capturing respectively. 
 *
 * For Spectrum mode, a buffer of size 1024 was allcoated for the FFT. The 
 * lowest 128 frequency bins were converted to their magnitudes in dB
 * and scaled to 1d dB/pixel.
 */
void processingTask(UArg arg1, UArg arg2) {

    int i;
    cfg = kiss_fft_alloc(NFFT, 0, kiss_fft_cfg_buffer, &buffer_size); // init Kiss FFT
    while(1) {
        Semaphore_pend(semaphoreProcessing, BIOS_WAIT_FOREVER);

        if(oscillo_mode) {  // oscilloscope mode
            fScale = (VIN_RANGE * PIXELS_PER_DIV) / ((1 << ADC_BITS) * fVoltsPerDiv[vpd]);
            for (i = 0; i < LCD_HORIZONTAL_MAX - 1; i++) {
                // Copy waveform into another buffer
                displayBuffer[i] = LCD_VERTICAL_MAX / 2 - (int)roundf(fScale * ((int)sample[i] - ADC_OFFSET));
            }

            Semaphore_post(semaphoreDisplay);
            Semaphore_post(semaphoreWaveform);
        }
        else {  // spectrum mode
            for (i = 0; i < NFFT; i++) { // generate an input waveform
                in[i].r = fft_sample[i]; // real part of waveform
                in[i].i = 0;             // imaginary part of waveform
            }

            // compute FFT
            kiss_fft(cfg, in, out); 

            // convert first 128 bins of out[] to dB for display
            for(i = 0; i < LCD_HORIZONTAL_MAX - 1; i++) {
                out_db[i] = 160 - ((10.0f) * log10f((out[i].r * out[i].r) + (out[i].i * out[i].i)));
            }
            Semaphore_post(semaphoreDisplay);
            Semaphore_post(semaphoreWaveform);
        }
    }


}

/** Lowest priority task to display the waveform onto the LCD display.
 * Once signaled, draws a single frame. 
 */
void displayTask(UArg arg1, UArg arg2) {
    tContext sContext;
    GrContextInit(&sContext, &g_sCrystalfontz128x128); // Initialize the grlib graphics context
    GrContextFontSet(&sContext, &g_sFontFixed6x8);     // Select font
    int i;
    int y, y_prev;

    tRectangle rectFullScreen = {0, 0, GrContextDpyWidthGet(&sContext)-1, GrContextDpyHeightGet(&sContext)-1};

    while (1) {
        Semaphore_pend(semaphoreDisplay, BIOS_WAIT_FOREVER);

        if(oscillo_mode) {  // oscilloscope mode


            // Fill screen with black
            GrContextForegroundSet(&sContext, ClrBlack);
            GrRectFill(&sContext, &rectFullScreen);

            // Draw grid
            GrContextForegroundSet(&sContext, ClrBlue);
            for (i = -3; i < 4; i++) {
                GrLineDrawH(&sContext, 0, LCD_HORIZONTAL_MAX - 1, LCD_VERTICAL_MAX/2 + i * PIXELS_PER_DIV);
                GrLineDrawV(&sContext, LCD_VERTICAL_MAX/2 + i * PIXELS_PER_DIV, 0, LCD_HORIZONTAL_MAX - 1);
            }

            // Draw wave
            GrContextForegroundSet(&sContext, ClrYellow);

            for (i = 0; i < LCD_HORIZONTAL_MAX - 1; i++)
            {
                // draw lines
                y = displayBuffer[i];
                GrLineDraw(&sContext, i, y_prev, i + 1, y);
                y_prev = y;
            }

            // Draw scales
            GrContextForegroundSet(&sContext, ClrWhite); //white text

            if(trig_slope) // Draw rising trigger
            {
                GrLineDraw(&sContext, 105, 10, 115, 10);
                GrLineDraw(&sContext, 115, 10, 115, 0);
                GrLineDraw(&sContext, 115, 0, 125, 0);
                GrLineDraw(&sContext, 112, 6, 115, 2);
                GrLineDraw(&sContext, 115, 2, 118, 6);
            }
            else // Draw falling trigger
            {
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


            GrFlush(&sContext); // Flush to display
        }
        else {  // Spectrum mode

            // Fill screen with black
            GrContextForegroundSet(&sContext, ClrBlack);
            GrRectFill(&sContext, &rectFullScreen);

            // Draw grid
            GrContextForegroundSet(&sContext, ClrBlue);
            for (i = 0; i < 7; i++) {
                GrLineDrawV(&sContext, i * PIXELS_PER_DIV, 0, LCD_HORIZONTAL_MAX - 1);
            }
            for (i = -3; i < 4; i++) {
                GrLineDrawH(&sContext, 0, LCD_HORIZONTAL_MAX - 1, LCD_VERTICAL_MAX/2 + i * PIXELS_PER_DIV);
            }

            // Draw wave
            GrContextForegroundSet(&sContext, ClrYellow);

            for(i = 0; i < LCD_HORIZONTAL_MAX - 1; i++) {
                y = out_db[i];

                GrLineDraw(&sContext, i, y_prev, i + 1, y);
                y_prev = y;
            }

            // Draw frequency scale
            GrStringDraw(&sContext, "20 kHz", -1, 4, 0, false);

            // Draw decibel scale
            GrStringDraw(&sContext, "20 dBV", -1, 50, 0, false);

            GrFlush(&sContext); // Flush to display
        }
    }

}