/*
 * sampling.h
 *
 *  Created on: Nov 4, 2020
 *      Author: Prudence Lam
 *              Adam Yang
 */

#ifndef SAMPLING_H_
#define SAMPLING_H_

#include <stdint.h>

#define ADC_BUFFER_SIZE 2048                             // size must be a power of 2
#define ADC_OFFSET 2048
#define ADC_BUFFER_WRAP(i) ((i) & (ADC_BUFFER_SIZE - 1)) // index wrapping macro
#define VIN_RANGE 3.3                                    // total V_in range
#define PIXELS_PER_DIV 20                                // LCD pixels per voltage division
#define ADC_BITS 12                                      // number of bits in ADC

extern volatile int32_t gADCBufferIndex;  // latest sample index
extern volatile uint16_t gADCBuffer[ADC_BUFFER_SIZE];           // circular buffer
extern volatile uint32_t gADCErrors;                        // number of missed ADC deadlines

void ADCInit(void);
int RisingTrigger(void);
int FallingTrigger(void);
void ADC_ISR(void);




#endif /* SAMPLING_H_ */
