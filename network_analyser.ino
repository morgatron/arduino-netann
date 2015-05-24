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

NetworkAnalyser netAnn;
float Rref=50000;

// Should make sure the signal is rotated correctly (i.e. delays are taken into account) before using this.


void setup() {
	//Register the serial communcations functions
	Serial.begin(115200);
	//connectCommands();

	float startFreq=1., stopFreq = 10., integTime=10000; //kHz, kHz, uS
	int Nsteps=20;
	netAnn.setup();
	//netAnn.startScan(startFreq, stopFreq, Nsteps, integTime);
	//netAnn.stopScan();
	netAnn.calibrate();
	//pinMode(13, OUTPUT);
}

void loop() {
	//sCmd.readSerial();

	Serial.println("loop");
	netAnn.calibrate();
	static float sampFreq=10.5;
	sampFreq+=5;
	if (sampFreq>20){
		sampFreq-=20;
	}
	netAnn._setCurrentFreq(sampFreq);

	//delay(500);
	//sampFreq+=0.01;
	
	//netAnn.calibrate();
	QuadratureSample q=netAnn.freqSampleAndWait(sampFreq, 10000);
	QuadratureSample calQ=netAnn.calibrateSample(q, sampFreq);
	Serial.print("calphi:");
	Serial.println(calQ.phi());
	Serial.print("phi:");
	Serial.println(q.phi());
	Serial.print(q.sig0); Serial.println(q.sig90);
	Serial.println("caled:");
	Serial.print(calQ.sig0); Serial.println(calQ.sig90);
		//Serial.print("sig0NR: ");
		//Serial.println(calcR(calQ.sig0, calQ.sig90, Rref, netAnn.getSensAmplitudeDig()));
		//Serial.print(q.sig0); Serial.println(q.sig90);
		//Serial.print("sig90NR: ");
		//Serial.println(calcWC(q.sig0, q.sig90, Rref, netAnn.getSensAmplitudeDig())/sampFreq);
		//q=q.rotated(sampFreq*netAnn.calWriteReadLag*TWOPI);
	//	Serial.println(calcR(q.sig0, q.sig90, Rref, netAnn.getSensAmplitudeDig()));
	//	Serial.print("sig90: ");
	//	Serial.println(calcWC(q.sig0, q.sig90, Rref, netAnn.getSensAmplitudeDig())/sampFreq);
	//	netAnn.printSensWvfm();
	//}
	//netAnn.printQuadWvmfs();
	
	//netAnn.scanUpdate();
	//netAnn.printSensWvfm();
	//netAnn.printAcqParams();

	/*
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
	 const int N_ave=1;
	 int t_now=micros();

	 // Average data for reporting to the computer/controlling
	 static int t_last_sig =0;

	 if (ACQ_cycle_count >= ACQ_cycles_to_integrate){
	 static float freq=0; //kHz
	 int sig0_cum_temp=ACQ_sig0_cum; //make tempoaries (as the actual numbers could theoretically get updated while we're here)
	 int sig90_cum_temp=ACQ_sig90_cum;
	 ACQ_sig0_cum=0;
	 ACQ_sig90_cum=0;

	 sig0 = (float)sig0_cum_temp/ACQ_sample_count;
	 sig90 = (float)sig90_cum_temp/ACQ_sample_count;
	 freq+=GLB_freq_step;
	 if (freq > GLB_stop_freq)
	 freq=GLB_start_freq;
	 _setSingleFreq(freq);
	 ACQ_cycles_to_integrate=(int)(GLB_integ_time*freq/1000. + 0.5); // +0.5 is to give rounding behaviour
	 Serial.print("NEw ACQ_CYCLES IS ");
	 Serial.println(ACQ_cycles_to_integrate);

	 ACQ_cycle_count=0;
	 ACQ_sample_count=0;

	 //Send off the data
	 if (GLB_send_data){
	 sendData(freq, sig0, sig90);
	 }

	 }
	 int t_elapsed=t_now-t_last;

	 if(t_elapsed>200000 && 0){

	 //Serial.print("Sig X, Y, R: "); Serial.print(sig0); Serial.print(", "); Serial.print(sig90); Serial.print(", "); Serial.println(sigR);
	 //sig0 = (N_ave*sig0 + (float)sig0_cum/cur_samples)/(N_ave+1);
	 //sig90 = (N_ave*sig90 + (float)sig90_cum/cur_samples)/(N_ave+1);
	 //sig90_cum=0;
	 //sig0_cum=0;
	 //Serial.print(cur_samples);Serial.print(" samples in "); Serial.print(t_elapsed); Serial.println(" us");
	 //Serial.print("Sample_rate = "); Serial.println((cur_samples)/((float)t_elapsed), 8);

	 Serial.print("Still going... ACQ_cycle_count is ");
	 Serial.println(ACQ_cycle_count);
	 t_last=t_now;
	 }
	 */

}
