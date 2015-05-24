#ifndef NETWORK_ANALYSER_H
#define NETWORK_ANALYSER_H
#define FLAG_TWO_OUTPUTS 0

//#include <cmath>
#include <math.h>
#include "Arduino.h"

//#define PRINT(X)  Serial.print("X:");Serial.println(X);
#define PRINT_VAR(x) do { Serial.print(#x ": "); Serial.println(x); } while (0)
//#define PRINT_VAR(x)
//#define PRINT_VAR(x) (Serial.print(#x ": "), Serial.println(x))
/* Iteration options:
 *  1. 2 seperate wave-forms (1 sens, 1 heat), 4 different idxs
 *      - Minimises memory use
 *      - More cycles incremeting 4 pointers
 *  2. 4 wave-forms, 2 idxs
 *      - more memory use
 *      - less time incrementing pointers
 *      - still need to apply the index
 *  3. 4 wave-forms, 4 pointers
 *      - Increment the pointer itself
 *      - Maybe saves time on the look-up part
 *      - However still need to check for being out-of-range +
 *          looping back
 *      - Probably doesn't save any time over all at present?
 *      - May be slightly more efficient if each wfm has a different
 *          size
 */

//GLOBAL CONSTANTS
const float CLOCK_RATE=42000.;

const int N_SENS_WV=256;
const int Nbase=256;

const int N_REF_WV=512;//N_HEATER_WV*8;
const int NCYC_SENS=1;
const float PI_ON_2 = M_PI/2.0;
const float TWOPI= 2*M_PI;
const float N_REF_WV_ON_2PI = N_REF_WV/(2*M_PI);
const int IMAX=2147483647;
const int READ_WVFM_AMP=200;
const float MIN_SAMPLE_PERIOD=1.0/300.0; //ms
//


/* NetworkAnalsyer class used to write a frequency to a DAC pin and measure the response on an ADC pin
 *
 * The I/O is done 'in the background' by an interrupt which is triggered by one of the on-board clocks.
 * This is meant to be used by declaring a NetworkAnalyser object, 
 */
float calcR(float sigR, float sigI, float Rref, float sig0);
float calcWC(float sigR, float sigI, float Rref, float sig0);

class QuadratureSample{
public:
	QuadratureSample(float s0, float s90){sig0=s0; sig90=s90;};
	float sig0;
	float sig90;

	float R(){return sqrt(sig0*sig0 + sig90*sig90);}
	float phi(){ return atan2(sig90,sig0); }
	QuadratureSample rotated(float phi){return QuadratureSample( cos(phi)*sig0 - sin(phi)*sig90,
												                 sin(phi)*sig0 + cos(phi)*sig90 ); }
};


class NetworkAnalyser {
public:
	NetworkAnalyser();
	void startScan(float startFreq, float stopFreq, int Nsteps, float integTime);
	void stopScan();
	void continueScanning();
	bool isScanning();
	void scanUpdate();
	void calibrate();
	QuadratureSample calibrateSample(QuadratureSample q, float freq);


	void toggleDataSending();
	void setup();
	int on_interrupt(int input);

	float setSensAmplitudeV(float amp); // Amplitude in Volts
	float setSensAmplitudeV2(float amp); // Amplitude in Volts
	int getSensAmplitudeV();
	int getSensAmplitudeV2();

	const int* getRefWvFm() const {
		return ref_wv_fm;
	}

	const int* getReadQuad0WvFm() const {
		return read_quad0_wv_fm;
	}

	const int* getReadQuad90WvFm() const {
		return read_quad90_wv_fm;
	}

	const int* getSensWvFm() const {
		return sens_wv_fm;
	}
#if FLAG_TWO_OUTPUTS
	const int* getSensWvFm2() const {
			return sens_wv_fm2;
		}
#endif

	//Waveform writing
	void updateSensWvfm();
#if FLAG_TWO_OUTPUTS
	void updateSensWvfm2();
#endif
	void updateReadWvfms();


	//Privaet-ish stuff
	float getCurrentFreq();
	float getCurrentFreq2();
	unsigned int getSamplePeriod();
	unsigned int setSamplePeriod(unsigned int newPeriod);
	float _setCurrentFreq(float freq);
	int _setReadLagPts(int pts);
	int _getReadLagPts();
	float _setSensPhase(float rads);
	float _getSensPhase();
	float _setSampleRate(float rate);
	float _getSampleRate();
	unsigned int setSensAmplitudeDig(unsigned int amp);
	unsigned int setSensAmplitudeDig2(unsigned int amp);
	unsigned int getSensAmplitudeDig();
	unsigned int getSensAmplitudeDig2();
	void do_first_sample();


	bool freqSampleDone();
	void startFreqSample(float target_freq);
	QuadratureSample freqSampleResult();
	QuadratureSample freqSampleAndWait(float freq, float integration_time=-1);

	void readSerial();
	// Changing these WILL do things.
	float start_freq; //kHz
	float stop_freq; //kHz
	float freq_step;
	float cal_per_volt; //DAC val vs volts
	float integration_time;
	bool b_send_data;
	float current_target_freq;
	int _read_lag_pts;
	float calDACvsADC;
	float calWriteReadLag;

	//Debugging things

	void printQuadWvmfs();
	void printSensWvfm();
	void printAcqParams();
private:
	//Wave forms
	int ref_wv_fm[N_REF_WV];
	int sens_wv_fm[Nbase];
	int read_quad0_wv_fm[Nbase];
	int read_quad90_wv_fm[Nbase];
#if FLAG_TWO_OUTPUTS
	int sens_wv_fm2[Nbase];
#endif

	// Private variables
	// These are merely records, giving what these things were set to last.
	// Changeing them won't do anything
	float _current_freq;
	float _current_freq2;
	unsigned int _sample_period;
	unsigned int _sens_amplitude_dig;
	unsigned int _sens_amplitude_dig2;
	float _sens_phase;
	float _sens_phase2;

	// More generic, hardware related stuff. Could possibly go in a separate module
	void writeWaveForm(int wv_fm[], unsigned int Npts, unsigned int Ncyc, int amplitude, float phase, int nReps=1, int shift_pts=0);
	void writeReferenceWave();

	//Acquisition stuff
	volatile int ACQ_cycle_count=0;
	volatile int ACQ_cycle_count2=0;
	volatile int ACQ_cycles_to_integrate=0;
	volatile int ACQ_cycles_to_integrate2=0;
	volatile int ACQ_i_sens = 0;
	volatile int ACQ_i_sens2 = 0;
	volatile int ACQ_sig0_cum=0;
	volatile int ACQ_sig90_cum=0;
	volatile unsigned int ACQ_sample_count=0;
	volatile unsigned int ACQ_sample_count2=0;
	long sig0_long=0;
	long sig90_long=0;
	const int AI_ZERO=2047; //until we measure it more accurately.

	unsigned int cur_N_SENS_WV=2;
	unsigned int cur_N_SENS_WV2=2;
	int nPointRepetitions=1;
	int nPointRepetitions2=1;
	//int nPointRepetitionsWvFm=1;
	//int nPointRepetitionsWvFm2=1;
	bool bRepeatPointsInWaveform;
	int nCycles=1;
	int nSampsPerCycle=4;
	int nSampsPerCycle2=4;
	//Constants
	//Keep them global for the time being
	//Flags
	bool bScanning;
	bool bSingleFreq;
};

//Hardware stuff
namespace HW{
void dac_setup();
void adc_setup();
void clock_setup(unsigned int NCyc, unsigned int NCycUp);
}

#endif //NETWORK_ANALYSER_H
