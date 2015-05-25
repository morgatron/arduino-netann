

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
#include "netAnn.h"
#include "comms.h"
#include "handler.h"

int floorDiv(int num, int denom){
	int res=num/denom;
	if( (num<0 && denom>=0) || (num>0 && denom<=0)){
		res-=1;
	}
	return res;
}
SerialCommand sCmd;

float calcR(float sigR, float sigI, float Rref, float sig0) {
	float sR = sigR / sig0;
	float sI = sigI / sig0;
	float num = Rref * (-sI * sI - sR * sR + sR);
	float denom = sI * sI + sR * sR - 2. * sR + 1.;
	return num / denom;
}
float calcWC(float sigR, float sigI, float Rref, float sig0) {
	float sR = sigR / sig0;
	float sI = sigI / sig0;
	float denom = Rref * -sI;
	float num = sI * sI + sR * sR - 2. * sR + 1.;
	return num/denom;
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
			DACC_MR_REFRESH (0x08) |       // Refresh shouldn't be needed if we're writing at high speed
			DACC_MR_TAG | // We use TAG and word mode to output two samples to the 2 channels at once (sort of)
			DACC_MR_WORD;
		DACC->DACC_IDR = 0xFFFFFFFF ; // no interrupts
#ifdef FLAG_TWO_OUTPUTS
		DACC->DACC_CHER = DACC_CHER_CH0 | DACC_CHER_CH1 ; // enable ch0 and ch1
#else
		DACC->DACC_CHER = DACC_CHER_CH1; // enable ch0 and ch1

#endif
	}
	//For writing to the DAC sample buffer (writing to the right data bits and tag bits)
	void adc_setup ()
	{
		NVIC_EnableIRQ (ADC_IRQn) ;   // enable ADC interrupt vector
		ADC->ADC_IDR = 0xFFFFFFFF ;   // disable interrupts
		ADC->ADC_IER =  1 << 1;         // enable AD1 (2^(7-PIN_NUM)) End-Of-Conv interrupt (Arduino pin A6)
		ADC->ADC_CHDR = 0xFFFF ;      // disable all channels
		ADC->ADC_CHER = 1 << 1 ;        // enable just A6. 2^(7-PIN_NUM)
		ADC->ADC_CGR = 0x15555555 ;   // All gains set to x1
		//ADC->ADC_COR = 0x00010000 ;   // All offsets off, ch0 in differential mode (thus all) (not used anymore)
		ADC->ADC_COR = 0x00000000;
		//ADC->ADC_COR = ADC->ADC_COR | ((1 << 0) << 16 );
		ADC->ADC_MR = (ADC->ADC_MR & 0xFFFFFFF0) | (1 << 1) | ADC_MR_TRGEN ;  // 1 = trig source TIO from TC0
	}
} //namespace HW


NetworkAnalyser::NetworkAnalyser(): b_send_data (true),
										start_freq (1.0),
										stop_freq (2.0),
										freq_step (0.1),
										_sens_phase (0),
										_sens_amplitude_dig (200),
										current_target_freq (0.1),
										_current_freq(current_target_freq),
										integration_time(100000),
										_sample_period(1000),
										_read_lag_pts(0),
										cal_per_volt(1365.),
										bSingleFreq(false),
										bScanning(false),
										ACQ_cycles_to_integrate(10),
										ACQ_cycle_count(0),
										bRepeatPointsInWaveform(false),
										cur_N_SENS_WV(2),
										cur_N_SENS_WV2(2)
{

	writeReferenceWave();
}

// Stuff to run when callibrating for frequency response (and maybe internal resistance)
// Proposed interface: call with the DAC connected straight to the ADC.
// This function should trigger a frequency scan which will be used to callibrate 3 things: 
// the DAC and ADC against each other, (float calDACvsADC)
// The lag between reading and writing (float writeReadLag)
// Ideally also internal resistance, capacitance -> frequency response
// We'll start with just the first two
void NetworkAnalyser::calibrate(){
	float f2=20.;
	QuadratureSample q1=freqSampleAndWait(0.01, 100000), q2=freqSampleAndWait(f2, 100000);
	PRINT_VAR(this->getSensAmplitudeDig());
	PRINT_VAR(q1.R());
	this->calDACvsADC=q1.R()/this->getSensAmplitudeDig();
	this->calWriteReadLag=q2.phi()/TWOPI/ f2;
	PRINT_VAR(calWriteReadLag);
	PRINT_VAR(calDACvsADC);
	return;
}

void NetworkAnalyser::startFreqSample(float target_freq){
	float current_actual_freq=_setCurrentFreq(target_freq); //Set the new frequency to scan
	//this->do_first_sample();
	//delay(1);
	ACQ_cycles_to_integrate=(int)(integration_time*current_actual_freq/1000. + 0.5); // +0.5 is to give rounding behaviour
	sig0_long=0;
	sig90_long=0;
	ACQ_cycle_count=0;
	ACQ_sample_count=0;
}
bool NetworkAnalyser::freqSampleDone(){
	bool bDone=ACQ_cycle_count >= ACQ_cycles_to_integrate;
	if(!bDone){ //check for rollover
		const unsigned int ROLL_OVER=100000000;
		//int comp_val= //(unsigned int)abs(ACQ_sig0_cum);
		int sig0_cum_temp;
		int sig90_cum_temp;
		sig0_cum_temp= ACQ_sig0_cum; // make tempoaries (as the actual numbers could theoretically get
		sig90_cum_temp= ACQ_sig90_cum;// updated while we're here)
		if(  (abs(sig0_cum_temp)>ROLL_OVER) | (abs(sig90_cum_temp)>ROLL_OVER ) ){
			Serial.print("Rollover. Old 0 vals: ");
			Serial.print(ACQ_sig0_cum);
			Serial.print(", ");
			Serial.println(ACQ_sig90_cum);

			sig0_long+=  sig0_cum_temp;
			sig90_long+= sig90_cum_temp;
			ACQ_sig0_cum-= sig0_cum_temp;
			ACQ_sig90_cum-=sig90_cum_temp;
		}
	}
	return bDone;
}

QuadratureSample NetworkAnalyser::calibrateSample(QuadratureSample q, float freq){
	int sensAmp=getSensAmplitudeDig();
	q.sig0/=calDACvsADC*sensAmp;
	q.sig90/=calDACvsADC*sensAmp;
	q=q.rotated(-freq*calWriteReadLag*TWOPI);
	return q;
}

QuadratureSample NetworkAnalyser::sample(){
	int sig0_cum_temp;
	int sig90_cum_temp;

	sig0_cum_temp=ACQ_sig0_cum; // make temporaries (as the actual numbers could theoretically get
	sig90_cum_temp=ACQ_sig90_cum;// updated while we're here)
	int sample_count_temp=ACQ_sample_count;

	ACQ_sig0_cum=0; //Reset the accumulated signal to zero
	ACQ_sig90_cum=0;
    ACQ_cycle_count=0;
    ACQ_sample_count=0;
    sig0_cum_temp=0;
    sig90_cum_temp=0;

	float sig0 = ((float)(sig0_cum_temp + sig0_long)/sample_count_temp);
	float sig90 = ((float)(sig90_cum_temp + sig90_long)/sample_count_temp);

	sig0_long=0;
	sig90_long=0;

	return QuadratureSample(sig0, sig90); // Should maybe return a pair
}
QuadratureSample NetworkAnalyser::freqSampleResult(){
	while(!freqSampleDone()){
		delay(5);
	}
    return sample();
}

QuadratureSample NetworkAnalyser::freqSampleAndWait(float freq, float integration_time){
	if (integration_time==-1){
		integration_time=this->integration_time;
	}
	float old_integration_time =this->integration_time;
	this->integration_time=integration_time;
	startFreqSample(freq);
	QuadratureSample result=freqSampleResult();
	this->integration_time=old_integration_time;
	return result;
}

void NetworkAnalyser::readSerial(){
	sCmd.readSerial();
}

void NetworkAnalyser::scanUpdate(){
	readSerial();
	if (freqSampleDone()){
		QuadratureSample result= freqSampleResult();
        if (this->b_send_data){
            sendData(this->getCurrentFreq(), result.sig0, result.sig90);
        }
		if(bScanning){
			current_target_freq+=freq_step;
			if (current_target_freq > stop_freq){
				current_target_freq=start_freq;
			};
			startFreqSample(current_target_freq);
		}
	}
}

/*write_wave_form
	 Make a down-sampled sine wave based on lookup. Uses a pre-allocated array ("ref_wv_fm")
	 containing values for a full revolution of sine with amplitude between -2^31 and 2^31.
	 The number of points in this array is an integer multiple of the number of points in the
	 DAC wave-form, so that down sampling can be done simply by skipping an integer multiple
	 of points, and amplitude scaling is simply an integer division of the reference amplitude.

	 TODO:
	 This should probably really only take a phase, not a 'shift_points'
 */
const int CH0_MARK= (0 << 12);
const int CH1_MARK= (1 << 12);
const int TAG_MASK = (1 << 28);
void NetworkAnalyser::writeWaveForm(int wv_fm[], unsigned int Npts, unsigned int Ncyc, int amplitude, float phase, int nReps, int shift_pts){
	if(0){
		Serial.println("writeWaveForm:");
		PRINT_VAR(Npts);
		PRINT_VAR(Ncyc);
		PRINT_VAR(amplitude);
		PRINT_VAR(phase);
		PRINT_VAR(nReps);
		PRINT_VAR(shift_pts);
	}
	while(phase > TWOPI){ //Phase should be within 0, 2pi already, but lets make sure
		phase-=TWOPI;
	}
	while(phase < 0){
		phase+= TWOPI;
	}
	int N_ref_step= N_REF_WV*nReps*Ncyc/Npts;
	int div_fact = IMAX/amplitude;
	int phase_ref_offs= phase*N_REF_WV_ON_2PI; // /2/PI*N_REF_WV;
	int k_ref=phase_ref_offs;

	if(k_ref< 0){
		k_ref+=N_REF_WV;
	}
	//Serial.println("Starting loop");
	//PRINT_VAR(k);
	for(int k=0+shift_pts; k<(int)Npts+shift_pts; k++){
		//Serial.println("looping");
		if(k_ref>= N_REF_WV){
			k_ref-=N_REF_WV;
		}
		//deal with shift_pts
		int k3= k-shift_pts;
		if(k3>= Npts){
			k3-=Npts;
			}
		else if(k3<0){
			k3+=Npts;
			}
		wv_fm[k3]=this->ref_wv_fm[k_ref]/div_fact;

		unsigned int k2=floorDiv(k,nReps);
		//PRINT_VAR(k);
		//PRINT_VAR(k3);
		//PRINT_VAR(k2);
		if((k2+1)*nReps==k+1){
			k_ref+=N_ref_step;//N_REF_ON_N_WV;
		}
		//PRINT_VAR(k_ref);
	}
}

void NetworkAnalyser::do_first_sample(){
 #if FLAG_TWO_OUTPUTS
	dac_write_both(this->sens_wv_fm[cur_N_SENS_WV-1], this->sens_wv_fm2[0]);
#else
	dac_write_1(this->sens_wv_fm[cur_N_SENS_WV-1]);
#endif
	delayMicroseconds(2*_sample_period);
}

int NetworkAnalyser::on_interrupt(int input) {
	 // Check if it's a new cycle
	 static int out_A=0;
	 int i_sens=ACQ_i_sens/(nPointRepetitions);
	 if (i_sens>=cur_N_SENS_WV){
	 	this->ACQ_i_sens=0;
	 	this->ACQ_cycle_count++;
	 	i_sens=0;
	 }
	 //int i_sens2=ACQ_i_sens/(nPointRepetitions2);
	 if (ACQ_cycle_count < ACQ_cycles_to_integrate || bSingleFreq){


		int val = input - AI_ZERO;    // get conversion result
		//int val = *(ADC->ADC_CDR+1) - AI_ZERO;    // get conversion result
		int val0 = val*read_quad0_wv_fm[i_sens];
		int val90 = val*read_quad90_wv_fm[i_sens];
		ACQ_sig0_cum += val0;
		ACQ_sig90_cum += val90;
		ACQ_sample_count++; //increment number of samples since last read

		ACQ_i_sens++;
		out_A=this->sens_wv_fm[i_sens];
	}
	else{
		this->ACQ_i_sens=0; //maybe unecessary?
	}

 #if FLAG_TWO_OUTPUTS
	 int i_sens2=ACQ_i_sens2/(nPointRepetitions2);
	 if (i_sens2>=cur_N_SENS_WV2){
	 	this->ACQ_i_sens2=0;
	 	this->ACQ_cycle_count2++;
	 	i_sens2=0;
	 }
	 if (ACQ_cycle_count2 <= ACQ_cycles_to_integrate2 || bSingleFreq){
	 	ACQ_i_sens2++;
		ACQ_sample_count2++; //increment number of samples since last read
	 }
	else{
		this->ACQ_i_sens2=0; //maybe unecessary?
	}
	int out_B=this->sens_wv_fm2[i_sens2];
	return ( (out_A +2048) | ((out_B +2048) << 16) );
#else
	return (out_A+2048 | CH1_MARK);
#endif
	// Save data in a circular buffer so we can inspect it for debugging purposes
	//samples2[sptr]= val90;
	//samples2[0]= 0;
	//sptr = (sptr+1) & BUFMASK ;      // move pointer
	//samples[sptr] = val;           // stick it in circular buffer.

	//Load the DAC write register with the next values to write
	//Serial.print("i_sens:");
	//Serial.println(i_sens);
	//Serial.print("val:"); Serial.println(this->sens_wv_fm[i_sens]);


//	return this->sens_wv_fm[i_sens];
	//dac_write_both(0, this->sens_wv_fm[ACQ_i_sens]);
}

void NetworkAnalyser::writeReferenceWave(){
	for(int k=0; k<N_REF_WV; k++){
			ref_wv_fm[k]=(int)IMAX*cos(TWOPI*(float)k/N_REF_WV);
			//Serial.print("k= "); Serial.print(k); Serial.print(", GLB_ref_wv_fm="); Serial.println(GLB_ref_wv_fm[k]);
		}
}

//To be run in the setup loop
void NetworkAnalyser::setup(){

	//Register the serial communications functions
	connectCommands();

	///Make the reference wave form from which all other wave-forms will be calculated
	//This stuff could probably just be done in the constructor
	//=======================================================================
	//--------------HARDWARE------------------------------------------------------------------
	HW::adc_setup();         // setup ADC
	setSamplePeriod(10000);
	HW::dac_setup();        // sets-up up DAC to be auto-triggered by the clock
	_setCurrentFreq(this->_current_freq); //Assign a start frequency
	updateSensWvfm();
#if FLAG_TWO_OUTPUTS
	updateSensWvfm2();
#endif
	updateReadWvfms();
	return;
}

void NetworkAnalyser::startScan(float startFreq, float stopFreq, int Nsteps, float integTime){
	start_freq=startFreq;
	stop_freq=stopFreq;
	integration_time=integTime;
	freq_step=(stopFreq-startFreq)/Nsteps;
	startFreqSample(startFreq);
	this->continueScanning();
	scanUpdate();
	//while(ACQ_cycle_count>0){ // Wait until an interrupt has happened and everything has reset
	//	delayMicroseconds(10);
	//}
	return;
}



void NetworkAnalyser::stopScan(){
	bScanning=false;
	return;
}
void NetworkAnalyser::continueScanning() {
	bScanning=true;
}
void NetworkAnalyser::toggleDataSending(){
	b_send_data=false;
	return;
}

unsigned int NetworkAnalyser::setSamplePeriod(unsigned int period){
	//unsigned int NcycUp=period/2.;
	this->_sample_period=period;
	HW::clock_setup(period, period/2);
	return this->_sample_period;
}
unsigned int NetworkAnalyser::getSamplePeriod(){
	return this->_sample_period;
}

float NetworkAnalyser::_setSampleRate(float rate){ // Rate should be in kHz
	unsigned int period=CLOCK_RATE/rate; //assuming 42 MHz ADC clock
	PRINT_VAR(period);
	setSamplePeriod(period);
	return CLOCK_RATE/period;
}
float NetworkAnalyser::_getSampleRate(){
	return 42000/getSamplePeriod();
}

float NetworkAnalyser::_setCurrentFreq(float freq){
	//this->stopScan();
	//float sample_period=getSamplePeriod();
	//MIN_SAMPLE_PERIOD;
	unsigned int pointsPerCycle=(int)(1.0/freq/MIN_SAMPLE_PERIOD)/nSampsPerCycle*nSampsPerCycle;
	//delay(500);
	//return freq;

	int pointReps= pointsPerCycle/this->nSampsPerCycle;
	int pointReps2= pointsPerCycle/this->nSampsPerCycle2;
	this->bRepeatPointsInWaveform=pointReps<= N_SENS_WV/2 | pointReps <= N_SENS_WV/2;

	this->nPointRepetitions=pointReps;
	this->nPointRepetitions2=pointReps2;
	if (bRepeatPointsInWaveform){
		this->cur_N_SENS_WV=pointsPerCycle;
		this->cur_N_SENS_WV2=pointsPerCycle;
	}
	else{
		this->cur_N_SENS_WV=nSampsPerCycle;
		this->cur_N_SENS_WV2=nSampsPerCycle2;
	}

	/*
	Serial.print("setCurrentFreq:"); Serial.println(freq);
	PRINT_VAR(pointsPerCycle);
	PRINT_VAR(bRepeatPointsInWaveform);
	PRINT_VAR(nPointRepetitions);
	PRINT_VAR(cur_N_SENS_WV);
	Serial.println();
	*/
	//cur_N_SENS_WV=2;
	//cur_N_SENS_WV2=2;
	//delay(500);
	//return freq;
	updateSensWvfm();
	updateReadWvfms();
	Serial.println("freq set");
	//Serial.println("wvforms updates");
#if FLAG_TWO_OUTPUTS
	updateSensWvfm2();
#endif


	float rate= (pointsPerCycle * MIN_SAMPLE_PERIOD) * freq * 1/MIN_SAMPLE_PERIOD;
	if(rate <=1/MIN_SAMPLE_PERIOD){
		rate=_setSampleRate(rate);
		_current_freq=rate/pointsPerCycle;
		/*
		PRINT_VAR(_current_freq);
		//Serial.print("current_freq: "); Serial.println(_current_freq);
		PRINT_VAR(rate);
		*/

		//ACQ_cycles_to_integrate=integration_time*rate/N_SENS_WV*NCYC_SENS;
		/*
		if(rate<10){
		// NEED TO SEE IF THIS WORKS!
		DACC->DACC_MR |=DACC_MR_REFRESH (0xFF); // Set DAC mode-register
			//DACC_MR_TRGEN_EN | DACC_MR_TRGSEL (1) |  // trigger 1 = TIO output of TC0
			}
		else{
			DACC->DACC_MR &=DACC_MR_REFRESH (0x00); // Set DAC mode-register
		}
		*/
	}
	else{
		Serial.print("Trying to set sample rate as "); Serial.print(rate); Serial.println(", which is too high");
	}
	return _current_freq;
}

unsigned int NetworkAnalyser::setSensAmplitudeDig(unsigned int amp){
	this->_sens_amplitude_dig=amp;
	updateSensWvfm();
	return this->_sens_amplitude_dig;
}

unsigned int NetworkAnalyser::setSensAmplitudeDig2(unsigned int amp){
	this->_sens_amplitude_dig2=amp;
	updateSensWvfm2();
	return this->_sens_amplitude_dig2;
}
int NetworkAnalyser::_setReadLagPts(int pts) {
	this->_read_lag_pts=pts;
	updateReadWvfms();
	return this->_read_lag_pts;
}

int NetworkAnalyser::_getReadLagPts() {
	return this->_read_lag_pts;
}

unsigned int NetworkAnalyser::getSensAmplitudeDig2(){
	return this->_sens_amplitude_dig2;
}
unsigned int NetworkAnalyser::getSensAmplitudeDig(){
	return this->_sens_amplitude_dig;
}

float NetworkAnalyser::setSensAmplitudeV(float ampV){
	unsigned int newAmp=ampV*cal_per_volt;
	setSensAmplitudeDig(newAmp);
	return this->_sens_amplitude_dig/cal_per_volt;
}
float NetworkAnalyser::setSensAmplitudeV2(float ampV){
	unsigned int newAmp=ampV*cal_per_volt;
	setSensAmplitudeDig2(newAmp);
	return this->_sens_amplitude_dig2/cal_per_volt;
}
int NetworkAnalyser::getSensAmplitudeV(){
	return this->_sens_amplitude_dig/cal_per_volt;
}
int NetworkAnalyser::getSensAmplitudeV2(){
	return this->_sens_amplitude_dig2/cal_per_volt;
}

float NetworkAnalyser::_setSensPhase(float new_phase){

	this->_sens_phase=new_phase;
	updateSensWvfm();
	updateReadWvfms();
	return this->_sens_phase;
}
float NetworkAnalyser::_getSensPhase() {
	return this->_sens_phase;
}

void NetworkAnalyser::updateSensWvfm(){
	Serial.println("updateSensWvfm");
	this->writeWaveForm(this->sens_wv_fm, cur_N_SENS_WV, nCycles, this->_sens_amplitude_dig,
			this->_sens_phase, 
			this->bRepeatPointsInWaveform ? this->nPointRepetitions : 1 );
}
#if FLAG_TWO_OUTPUTS
void NetworkAnalyser::updateSensWvfm2(){
	int nPointReps=this->bRepeatPointsInWaveform ? this->nPointRepetitions2 : 1;
	this->writeWaveForm(this->sens_wv_fm2, cur_N_SENS_WV2, nCycles, this->_sens_amplitude_dig2,
			this->_sens_phase2, nPointReps );
}
#endif
void NetworkAnalyser::updateReadWvfms(){
	//Serial.println("updateReadWvfms");

	int nPointReps=this->bRepeatPointsInWaveform ? this->nPointRepetitions2 : 1;
	this->writeWaveForm(read_quad0_wv_fm, cur_N_SENS_WV, nCycles, READ_WVFM_AMP, this->_sens_phase, nPointReps, _read_lag_pts+1);
	this->writeWaveForm(read_quad90_wv_fm, cur_N_SENS_WV, nCycles, READ_WVFM_AMP, this->_sens_phase+ PI_ON_2, nPointReps, _read_lag_pts+1);
	//printQuadWvmfs();
}

void NetworkAnalyser::printQuadWvmfs(){
	//PRINT_VAR(cur_N_SENS_WV);
	Serial.print("read_wvfm_0, N:"); Serial.println(cur_N_SENS_WV);
	for (int k=0; k<this->cur_N_SENS_WV; k++){
		Serial.print(this->read_quad0_wv_fm[k]);
		Serial.print(" ");
	}
	Serial.println();
	Serial.print("read_wvfm_90, N:"); Serial.println(cur_N_SENS_WV);
	for (int k=0; k<this->cur_N_SENS_WV; k++){
		Serial.print(this->read_quad90_wv_fm[k]);
		Serial.print(" ");
	}
	Serial.println();

	return;
}
void NetworkAnalyser::printSensWvfm(){
	//PRINT_VAR(cur_N_SENS_WV);
	Serial.print("sens_wvfm_1, N:"); Serial.println(cur_N_SENS_WV);
	for (int k=0; k<this->cur_N_SENS_WV; k++){
		Serial.print(this->sens_wv_fm[k]);
		Serial.print(" ");
	}
	Serial.println();
#if FLAG_TWO_OUTPUTS
	Serial.println("sens_wvfm_2");
	for (int k=0; k<this->cur_N_SENS_WV2; k++){
		Serial.print(this->sens_wv_fm2[k]);
		Serial.print(" ");
	}
	Serial.println();
#endif

	return;
}
void NetworkAnalyser::printAcqParams(){
	PRINT_VAR(ACQ_cycle_count);
	PRINT_VAR(ACQ_cycles_to_integrate);
	PRINT_VAR(ACQ_i_sens);
	PRINT_VAR(ACQ_sig0_cum);
	PRINT_VAR(ACQ_sig90_cum);
	PRINT_VAR(ACQ_sample_count);
}


