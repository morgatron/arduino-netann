/*
 * handler.cpp
 *
 *  Created on: Mar 8, 2015
 *      Author: morgan
 */
#include "handler.h"
#include "netAnn.h"
#include "Arduino.h"

extern NetworkAnalyser netAnn;

// Analog output config:
const unsigned int ADC_CHANNEL_NUM=0;

volatile int GLB_imax_on_amp =0;

// Circular buffer, power of two.
#define SAMPLE_BUFSIZE 0x010
#define SAMPLE_BUFMASK 0x00F

//Volatiles: these can be changed in an interupt routine.
//Debugging:
volatile int samples [SAMPLE_BUFSIZE] ; //
//volatile int samples2 [BUFSIZE] ;
volatile unsigned int sptr =0;

//probably don't need to volatile

const int CH0_MARK= (0 << 12);
const int CH1_MARK= (1 << 12);
const int TAG_MASK = (1 << 28);
#ifdef __cplusplus
extern "C"
{
#endif
	inline void dac_write_0 (int val) // convenience function to write to DAC0
	{
		DACC->DACC_CDR = (val | CH0_MARK);
	}
	inline void dac_write_1 (int val) // DAC1
	{
		DACC->DACC_CDR = (val+2048 | CH1_MARK);
	}
	inline void dac_write_both(int val0, int val1){
		//The DAC data register is 32 bit, which can hold two 12-bit values to be written. We use the tag bits (the 13th+ bits of each 16-bit division) to flag that the first value is destined for DAC0 and the second is destined for DAC 1. In practice we only set a single bit for the DAC1 channel, as the TAG bits for DAC0 are already 0
		//val0+=2047;
		//val1+=2047;
		int val = ( (val0 +2048) | ((val1 +2048) << 16) );
		val+=134154239; //add's 2048 to both
		DACC->DACC_CDR = val | TAG_MASK;
	}
	//unsigned int j = 0;

	/* ADC_Handler: Where most of the work is done. This function is called every time the ADC finishes a conversion. Thus this supplies the 'clock' that everything works with (although of course the conversion itself is triggered by a counter-timer). Each time it reads, we will read the value from the ADC, do the appropriate 'heterodyning', and write out the next wave-form value to the two DAC channels.
	*/
	void ADC_Handler (void)
	{
        //Serial.println("EOC");
		if (ADC->ADC_ISR & ADC_ISR_EOC1){   // ensure there was an End-of-Conversion on the correct channel, and we read the ISR reg (to reset it, if I remember?). Can probably get rid of this later for a little extra speed, since we shouldn't be here if the EOC wasn't triggered.
			// Don't record data if we've finished recording at this frequency and are waiting for
			// data to be sent etc
			int toOutput=netAnn.on_interrupt(*(ADC->ADC_CDR+1));
			DACC->DACC_CDR=toOutput;
			/*
#ifdef FLAG_TWO_OUTPUTS
			dac_write_both(toOutput, toOutput);
#else
			dac_write_1(toOutput);
#endif
*/
		}
	}

#ifdef __cplusplus
}
#endif


