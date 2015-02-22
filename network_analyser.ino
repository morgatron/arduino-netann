/* Network analyser
Sketch to turn the arduino into a simple network analyser

Todos
* Make sure to integrate over full cycles only
    * Either put cycle counter in adc loop, or try to do something cleverer with saving samples in a circular buffer
* Change number of points used for the wave-form on the fly
    * Add a command setNsenor
* Don't allow frequencies that are too high be attempted (to avoid crashed)
* Compensate for ADC/DAC frequency response 
* Allow low frequencies to be used by increasing number of points OR setting DAC auto-refresh flag`
    * Probably when requesting frequencies below 10 kHz

Features to add:
* Calibration
    * Of the output vs frequency.
    * Of the internal resistance
    (* Of the ADC/DAC voltages)
    (* Of clock)
* LCR meter
    * Given a known resistor (or potentially just the internal resistance), calculate the value of an 
    *    unknown component connected in parallel or in series
* More advanced component anslysis
    * Perform LCR metering with more complex circuits
    * Optimised detection of changes based on simple models
 */
#include <PID_v1.h>
#include <SerialCommand.h>
#include "network_analyser.h"
#include "comms.h"

// GLOBALS----------------------------------------
//Wave-forms----------
float GLB_sens_freq=0;
int GLB_ref_wv_fm[N_REF_WV];
int GLB_sens_wv_fm[Nbase];
int GLB_read_quad0_wv_fm[Nbase];
int GLB_read_quad90_wv_fm[Nbase];
float GLB_sens_phase=0.0; //Radians
int GLB_sens_amplitude=1000; // DAC units
float GLB_read_phase_lag=0*PI; //Rads

float GLB_integ_time=0.1;
float GLB_freq_step=0.1;
float GLB_start_freq=10; //kHz
float GLB_stop_freq=50; //kHz

//Flags to change behaviour
bool GLB_send_data=true;



volatile int GLB_imax_on_amp =0;
SerialCommand sCmd; 

namespace HW{
	void dac_setup();
	void adc_setup();
	void clock_setup(unsigned int NCyc, unsigned int NCycUp);
}
/*write_wave_form
	 Make a down-sampled sine wave based on lookup. Uses a pre-allocated array ("GLB_ref_wv_fm") containing values for a full revolution of sine with amplitude between -2^31 and 2^31. The number of points in this array is an integer multiple of the number of points in the DAC wave-form, so that down sampling can be done simply by skipping an integer multiple of points, and amplitude scaling is simply an integer division of the reference amplitude 
 */
void write_wave_form(int wv_fm[], unsigned int Npts, unsigned int Ncyc, int amplitude, float phase){
	while(phase > TWOPI){ //Phase should be within 0, 2pi already, but lets make sure
		phase-=TWOPI;
	}
	while(phase < 0){
		phase+= TWOPI;
	}
	unsigned int N_ref_step= N_REF_WV/Nbase*Ncyc; 
	int div_fact = IMAX/amplitude;
	int phase_ref_offs= phase*N_REF_WV_ON_2PI; // /2/PI*N_REF_WV;
	int k_ref=phase_ref_offs;

	for(int k=0; k<Nbase; k++){
		wv_fm[k]=GLB_ref_wv_fm[k_ref]/div_fact;
		k_ref+=N_ref_step;//N_REF_ON_N_WV;
		if(k_ref>= N_REF_WV){
			k_ref-=N_REF_WV;
		}
	}
}

// Should make sure the signal is rotated correctly (i.e. delays are taken into account) before using this.
float calcR(float sigR, float sigI, float Rref, float sig0){
	float sR=sigR/sig0;
	float sI=sigI/sig0;
	float num=Rref*(-sI*sI -sR*sR + sR);
	float denom= sI*sI + sR*sR - 2.*sR + 1.;
	return num/denom;
}
float calcWC(float sigR, float sigI, float Rref, float sig0){
	float sR=sigR/sig0;
	float sI=sigI/sig0;
	float num=Rref*-sI;
	float denom= sI*sI + sR*sR - 2.*sR + 1.;
	return denom/num;
}

void setup()
{

	//Register the serial communcations functions
	connectCommands();
	Serial.begin (115200) ;
	pinMode(13, OUTPUT);

	///Make the reference wave form from which all other wave-forms will be calculated
	for(int k=0; k<N_REF_WV; k++){
		GLB_ref_wv_fm[k]=(int)IMAX*sin(TWOPI*(float)k/N_REF_WV);
		//Serial.print("k= "); Serial.print(k); Serial.print(", GLB_ref_wv_fm="); Serial.println(GLB_ref_wv_fm[k]);
	}

	//Make the other wave-forms
	write_wave_form(GLB_sens_wv_fm, N_SENS_WV, NCYC_SENS, GLB_sens_amplitude, GLB_sens_phase);
	write_wave_form(GLB_read_quad0_wv_fm, N_SENS_WV, NCYC_SENS, 200, GLB_sens_phase-GLB_read_phase_lag);
	write_wave_form(GLB_read_quad90_wv_fm, N_SENS_WV, NCYC_SENS, 200, GLB_sens_phase-GLB_read_phase_lag+PI_ON_2);


	//=======================================================================
	//--------------HARDWARE------------------------------------------------------------------
	HW::adc_setup();         // setup ADC
	HW::clock_setup(900, 440); //NCyc:90, NCycUp:40
	HW::dac_setup();        // sets-up up DAC to be auto-triggered by the clock
}

namespace HW{
	void setup_pio_TIOA0 ()  // Configure Ard pin 2 as output from TC0 channel A (copy of trigger event)
	{
		PIOB->PIO_PDR = PIO_PB25B_TIOA0 ;  // disable PIO control
		PIOB->PIO_IDR = PIO_PB25B_TIOA0 ;   // disable PIO interrupts
		PIOB->PIO_ABSR |= PIO_PB25B_TIOA0 ;  // switch to B peripheral
	}
	void clock_setup(unsigned int NCyc, unsigned int NCycUp){
		//Set up a clock to do essentially just pwm
		pmc_enable_periph_clk (TC_INTERFACE_ID + 0*3+0) ;  // clock the counter-timer channel 0 (TC0)
		TcChannel * t = &(TC0->TC_CHANNEL)[0] ;    // t is a pointer to TC0 registers for its channel 0
		t->TC_CCR = TC_CCR_CLKDIS ;  // disable internal clocking while we play with it's registers
		t->TC_IDR = 0xFFFFFFFF ;     // disable all interrupts
		t->TC_SR ;                   // read int status reg to clear the pending event
		t->TC_CMR = TC_CMR_TCCLKS_TIMER_CLOCK1 |   // use TCLK1. I think the clock divisor is 2, so it should be 42MHz
			TC_CMR_WAVE |                  // waveform mode
			TC_CMR_WAVSEL_UP_RC |          // We'll use count-up PWM using RC as threshold
			TC_CMR_EEVT_XC0 |     // Set external events from XC0 (this setup TIOB as output)
			TC_CMR_ACPA_CLEAR | TC_CMR_ACPC_CLEAR |
			TC_CMR_BCPB_CLEAR | TC_CMR_BCPC_CLEAR ;

		t->TC_RC =  NCyc ;     // counter resets on RC, so this sets period of PWM (each cycle is ~23 ns)
		t->TC_RA =  NCycUp ;     // Cycles to stay up. Make it roughly square wave
		t->TC_CMR = (t->TC_CMR & 0xFFF0FFFF) | TC_CMR_ACPA_CLEAR | TC_CMR_ACPC_SET ;  // set clear and set from RA and RC compares
		t->TC_CCR = TC_CCR_CLKEN | TC_CCR_SWTRG ;  // re-enable local clocking and switch to hardware trigger source.
		setup_pio_TIOA0 () ;  // Makes ard pin 2 an output channel for our setup TC0
	}



	void dac_setup ()
	{
		pmc_enable_periph_clk (DACC_INTERFACE_ID) ; // start clocking DAC
		DACC->DACC_CR = DACC_CR_SWRST ;  // reset DAC
		DACC->DACC_MR =  // Set DAC mode-register
			//DACC_MR_TRGEN_EN | DACC_MR_TRGSEL (1) |  // trigger 1 = TIO output of TC0
			DACC_MR_REFRESH (0x0F) |       // Refresh shouldn't be needed if we're writing at high speed
			DACC_MR_TAG | // We use TAG and word mode to output two samples to the 2 channels at once (sort of)
			DACC_MR_WORD;
		DACC->DACC_IDR = 0xFFFFFFFF ; // no interrupts
		DACC->DACC_CHER = DACC_CHER_CH0 | DACC_CHER_CH1 ; // enable ch0 and ch1
	}

	//For writing to the DAC sample buffer (writing to the right data bits and tag bits)
	void adc_setup ()
	{
		NVIC_EnableIRQ (ADC_IRQn) ;   // enable ADC interrupt vector
		ADC->ADC_IDR = 0xFFFFFFFF ;   // disable interrupts
		ADC->ADC_IER = 0x01 ;         // enable AD1 End-Of-Conv interrupt (Arduino pin A6)
		ADC->ADC_CHDR = 0xFFFF ;      // disable all channels
		ADC->ADC_CHER = 0x01 ;        // enable just A6
		ADC->ADC_CGR = 0x15555555 ;   // All gains set to x1
		//ADC->ADC_COR = 0x00010000 ;   // All offsets off, ch0 in differential mode (thus all) (not used anymore)
		ADC->ADC_COR = 0x00000000 ;   // All offsets off
		ADC->ADC_MR = (ADC->ADC_MR & 0xFFFFFFF0) | (1 << 1) | ADC_MR_TRGEN ;  // 1 = trig source TIO from TC0
	}
} //namespace HW

void set_sample_rate(float rate){ // Rate should be in kHz
	unsigned int Ncyc=42000./rate; //assuming 42 MHz ADC clock
	unsigned int NcycUp=Ncyc/2.;
	HW::clock_setup(Ncyc, NcycUp);
}
void set_sens_freq(float freq){
	float rate=freq*N_SENS_WV/NCYC_SENS;
	if(rate <430){
		set_sample_rate(rate);
		GLB_sens_freq=freq;
		if(rate<10){
		// NEED TO SEE IF THIS WORKS!
		DACC->DACC_MR |=DACC_MR_REFRESH (0xFF); // Set DAC mode-register
			//DACC_MR_TRGEN_EN | DACC_MR_TRGSEL (1) |  // trigger 1 = TIO output of TC0
			}
		else{
			DACC->DACC_MR &=DACC_MR_REFRESH (0x00); // Set DAC mode-register
		}
	}
}

// Circular buffer, power of two.
#define BUFSIZE 0x010
#define BUFMASK 0x00F

//Volatiles: these can be changed in an interupt routine.
//Debugging:
volatile int samples [BUFSIZE] ; //
//volatile int samples2 [BUFSIZE] ;
volatile unsigned int sptr =0;

//probably don't need to volatile
volatile int ACQ_cycle_count=0;
volatile int ACQ_cycles_to_integrate=10;
volatile int ACQ_i_sens = 0;
volatile int ACQ_sig0_cum=0;
volatile int ACQ_sig90_cum=0;
volatile unsigned int ACQ_sample_count=0;
const int AI_ZERO=2047; //until we measure it more accurately.

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
		DACC->DACC_CDR = (val | CH1_MARK);
	}
	inline void dac_write_both(int val0, int val1){
		//The DAC data register is 32 bit, which can hold two 12-bit values to be written. We use the tag bits (the 13th+ bits of each 16-bit division) to flag that the first value is destined for DAC0 and the second is destined for DAC 1. In practice we only set a single bit for the DAC1 channel, as the TAG bits for DAC0 are already 0
		val0+=2047;
		val1+=2047;
		int val = val0 | val1 << 16;
		DACC->DACC_CDR = val | TAG_MASK; 
	}
	//unsigned int j = 0;

	/* ADC_Handler: Where most of the work is done. This function is called every time the ADC finishes a conversion. Thus this supplies the 'clock' that everything works with (although of course the conversion itself is triggered by a counter-timer). Each time it reads, we will read the value from the ADC, do the appropriate 'heterodyning', and write out the next wave-form value to the two DAC channels.
	*/
	void ADC_Handler (void) 
	{
        //Serial.println("EOC");
		if (ADC->ADC_ISR & ADC_ISR_EOC0)   // ensure there was an End-of-Conversion on the correct channel, and we read the ISR reg (to reset it, if I remember?). Can probably get rid of this later for a little extra speed, since we shouldn't be here if the EOC wasn't triggered.
		{
            if (ACQ_cycle_count <= ACQ_cycles_to_integrate){
                int val = *(ADC->ADC_CDR+0) - AI_ZERO ;    // get conversion result
                int val0 = val*GLB_read_quad0_wv_fm[i_sens];
                int val90 = val*GLB_read_quad90_wv_fm[i_sens];
                ACQ_sig0_cum += val0; //Maybe this is ok?
                ACQ_sig90_cum += val90; //Maybe this is ok?
                ACQ_sample_count++; //increment number of samples since last read

				ACQ_i_sens++;
				if (ACQ_i_sens>=Nbase){
					ACQ_i_sens=0;
					ACQ_cycle_count++;
				}
			else{
				ACQ_i_sens=0;
			}
			// Save data in a circular buffer so we can inspect it for debugging purposes
			//samples2[sptr]= val90;
			//samples2[0]= 0;
			//sptr = (sptr+1) & BUFMASK ;      // move pointer
			//samples[sptr] = val;           // stick it in circular buffer.

			//Load the DAC write register with the next values to write
			dac_write_both(0, GLB_sens_wv_fm[ACQ_i_sens]);

			// Below: effectively "if i > N, i =0". Only works if N is a power of 2.

			//Actually seems we have plenty of cycles spare to do this:
					
	}

#ifdef __cplusplus
}
#endif

void loop() 
{
	sCmd.readSerial();
	static int t_last=0;
	static int samples_last=0;

	if(ACQ_sample_count-samples_last > 100000 && 0){ // Debugging

                //ADC->ADC_IDR = 0xFFFFFFFF ;   // disable interrupts
	
		int t_elapsed=micros()-t_last;
		Serial.print(ACQ_sample_count-samples_last);Serial.print(" samples in "); Serial.print(t_elapsed); Serial.println(" us");
		Serial.print("p: Sample_rate = "); Serial.println((ACQ_sample_count-samples_last)/((float)t_elapsed), 8);

		t_last=micros();
		samples_last=ACQ_sample_count;

		Serial.println("samples:");
		for(int k=0; k<30; k++){
			//Serial.print(samples[k]); Serial.print(" "); Serial.println(samples2[k]);
		}
		Serial.println("");
		Serial.println("Sens wv_fm:");
		for(int k=0; k<N_SENS_WV; k++){
			Serial.print(GLB_sens_wv_fm[k]); Serial.print(" ");
		}
		Serial.println("");
		Serial.println("read quad0 wv_fm:");
		for(int k=0; k<N_SENS_WV; k++){
			Serial.print(GLB_read_quad0_wv_fm[k]); Serial.print(" ");
		}
		Serial.println("");
		Serial.println("read quad90 wv_fm:");
		for(int k=0; k<N_SENS_WV; k++){
			Serial.print(GLB_read_quad90_wv_fm[k]); Serial.print(" ");
		}
		Serial.println("");
		Serial.println("Ref wv_fm:");
		for(int k=0; k<N_REF_WV; k++){
			Serial.print(GLB_ref_wv_fm[k]); Serial.print(" ");
		}
		Serial.println("");
	}

	static float sig0 =0;
	static float sig90 =0;
	static float sigR =0;
	static float freq=10; //kHz
	const int N_ave=1;
	int t_now=micros();
	
	// Average data for reporting to the computer/controlling
	static int t_last_sig =0;
	if (abs(t_now-t_last_sig) > GLB_integ_time){
	//if (ACQ_sample_count >10000 && 1){
		//Serial.println("sub start");
		int cur_samples=ACQ_sample_count;
		int sig90_cum_temp=sig90_cum; //make tempoaries (as the actual numbers could theoretically get updated while we're here)
		sig90_cum=0;
		int sig0_cum_temp=sig0_cum;
		sig0_cum=0;
		ACQ_sample_count=0;

		sig0 = (float)sig0_cum_temp/cur_samples/(N_ave+1);
		sig90 = (float)sig90_cum_temp/cur_samples/(N_ave+1);
		//sigR = sqrt(sig0*sig0 + sig90*sig90);

	}

	if (ACQ_cycle_count <= ACQ_cycles_to_integrate){
		int sig0_cum_temp=sig0_cum; //make tempoaries (as the actual numbers could theoretically get updated while we're here)
		int sig90_cum_temp=sig90_cum; 
		sig0_cum=0;
		sig90_cum=0;

		sig0 = (float)sig0_cum_temp/ACQ_sample_count;
		sig90 = (float)sig90_cum_temp/ACQ_sample_count;
		freq+=GLB_freq_step;
		if (freq > GLB_stop_freq)
			freq=GLB_start_freq;
		set_sens_freq(freq);
		ACQ_cycles_to_integrate=(int)(GLB_integ_time*freq/1000. + 0.5); // +0.5 is to give rounding behaviour
		
		ACQ_cycle_count=0;
		ACQ_sample_count=0;

		//Send off the data
        if (GLB_send_data){
            sendData(freq, sig0, sig90);
        }

	}
	int t_elapsed=t_now-t_last;
	/*
	if(t_elapsed>200000 && 0){

		//Serial.print("Sig X, Y, R: "); Serial.print(sig0); Serial.print(", "); Serial.print(sig90); Serial.print(", "); Serial.println(sigR);
		//sig0 = (N_ave*sig0 + (float)sig0_cum/cur_samples)/(N_ave+1);
		//sig90 = (N_ave*sig90 + (float)sig90_cum/cur_samples)/(N_ave+1);
		//sig90_cum=0;
		//sig0_cum=0;
		//Serial.print(cur_samples);Serial.print(" samples in "); Serial.print(t_elapsed); Serial.println(" us");
		//Serial.print("Sample_rate = "); Serial.println((cur_samples)/((float)t_elapsed), 8);

		t_last=t_now;
	}
	*/
}
