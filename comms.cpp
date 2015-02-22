#include <PID_v1.h>
#include "SerialCommand.h"
#include "network_analyser.h"
#include "comms.h"

// EXTERNS----------------------------------------
//Wave-forms----------
extern int GLB_ref_wv_fm[N_REF_WV];
extern int GLB_sens_wv_fm[Nbase];
extern int GLB_read_quad0_wv_fm[Nbase];
extern int GLB_read_quad90_wv_fm[Nbase];
extern float GLB_sens_phase; //Radians
extern int GLB_sens_amplitude; // DAC units
extern float GLB_read_phase_lag; //Rads

extern float GLB_integ_time;
extern float GLB_freq_step;
extern float GLB_start_freq; //kHz
extern float GLB_stop_freq; //kHz
//Flags to change behaviour
extern bool GLB_send_data;



//Needed to share with comms.h
extern volatile int GLB_imax_on_amp;
extern SerialCommand sCmd; 
///----------------------------------------------------------------
//--------------  SERIAL COMMUNICATIONS STUFF
// Functions for talking to the computer
/*Write the time, read values, and output value*/
void sendData(float freq, float sig0, float sig90){
	Serial.print("d: ");
	Serial.print(freq);
	Serial.print(" ");
	Serial.print(sig0);
	Serial.print(" ");
	Serial.println(sig90);
}

void cmdSendDataOn(){
	char *arg;
	arg = sCmd.next();	 
	if(strcmp(arg,"True")==0 || strcmp(arg,"1")==0) {		
		GLB_send_data=true;
	}
	else if ( strcmp(arg,"False")==0 || strcmp(arg,"0")==0) {	//strcmpr(arg, "false")
		GLB_send_data=false;
	}
    Serial.print("p: Data sending is "); Serial.println(GLB_send_data);

}


void cmdGetSensWfm(){
    Serial.print("sens_wfm: ");
    //for(int k=0; k<N_SENS_WV; k++){
    for(int k=0; k<Nbase; k++){
        Serial.print(GLB_sens_wv_fm[k]);
        Serial.print(", ");
    }
    Serial.println("");
}
void cmdGetRefWfm(){
    Serial.print("ref_wfm: ");
    for(int k=0; k<N_REF_WV; k++){
        Serial.print(GLB_ref_wv_fm[k]);
        Serial.print(", ");
    }
    Serial.println("");
}
void cmdTestComs(){
	char *arg;
	arg = sCmd.next();	 
	if (arg != NULL) {		
		Serial.print("echoing- you sent:  ");
		Serial.println(arg);
	}
	else {
		Serial.println("We're communicating.");
	}
}
void cmdSetSensPhase(){
	char *arg;
	arg = sCmd.next();	 
	if (arg != NULL) {		
		int new_phase= atof(arg);
		GLB_sens_phase=new_phase;
		write_wave_form(GLB_sens_wv_fm, N_SENS_WV, 8, GLB_sens_amplitude, GLB_sens_phase);
		Serial.println();
		Serial.print("p: NEW SENSE WAVEFORM PHASE IS: ");Serial.println(GLB_sens_phase);
		Serial.println();
	}
	else {
		Serial.print("p: Sense phase is currently set to: "); Serial.println(GLB_sens_phase);
	}
}

void cmdSetFreqScanParams(){
	char *arg;
	arg = sCmd.next();	 
	if (arg != NULL) {		
		GLB_start_freq= atof(arg);
		Serial.print("new startFreq is: ");
		Serial.println(GLB_start_freq);

		arg = sCmd.next();	 
		if (arg != NULL) {		
			GLB_stop_freq= atof(arg);
			Serial.print("new stopFreq is: ");
			Serial.println(GLB_stop_freq);

			arg = sCmd.next();	 
			if (arg != NULL) {		
				GLB_freq_step= atof(arg);
				Serial.print("new freq_step is: ");
				Serial.println(GLB_freq_step);
			}
		}
	}
	else{
		Serial.println("p: Freq scan params should be startF, stopF, stepF");
	}
}
void cmdSetIntegTime(){
	char *arg;
	arg = sCmd.next();	 
	if (arg != NULL) {		
		GLB_integ_time= atof(arg);
		Serial.print("new integration time is: ");
		Serial.println(GLB_integ_time);
	}
	else {
		Serial.print("p: integration time is "); Serial.println(GLB_integ_time);
	}
}
void cmdGetSensPhase(){
    Serial.print("sens_phase: "); Serial.println(GLB_sens_phase);
}
void cmdSetSensAmplitude(){
	char *arg;
	arg = sCmd.next();	 
	if (arg != NULL) {		
		int new_amp= atof(arg);
		GLB_sens_amplitude=new_amp;
		write_wave_form(GLB_sens_wv_fm, N_SENS_WV, NCYC_SENS, GLB_sens_amplitude, GLB_sens_phase);
		Serial.print("p: NEW SENSE WAVEFORM AMPLITUDE IS: ");Serial.println(GLB_sens_amplitude);
	}
	else {
		Serial.print("p: Ampl is currently set to- "); Serial.println(GLB_sens_amplitude);
	}
}

void cmdGetSensAmplitude(){//sens_amp
    Serial.print("sens_amp: ");Serial.println(GLB_sens_amplitude);
}

void cmdSetSensFreq(){
	char *arg;
	arg = sCmd.next();	 
	if (arg != NULL) {		
		float new_freq= atof(arg);
		Serial.print("new_req is: ");
		Serial.println(new_freq);
		set_sens_freq(new_freq);
	}
	else{
		Serial.println("p: Don't understand new sens frequency");
	}
}
void cmdSetSampleRate(){
	char *arg;
	arg = sCmd.next();	 
	if (arg != NULL) {		
		float new_rate= atof(arg);
		Serial.print("new_rate is: ");
		Serial.println(new_rate);
		set_sample_rate(new_rate);
	}
	else{
		Serial.println("p: Don't understand new sample rate");
	}
}
void cmdSetReadLag(){
	char *arg;
	arg = sCmd.next();	 
	if (arg != NULL) {		
		float new_lag= atof(arg);
		GLB_read_phase_lag=new_lag;
		write_wave_form(GLB_read_quad0_wv_fm, N_SENS_WV, NCYC_SENS, 200, GLB_sens_phase-GLB_read_phase_lag);
		write_wave_form(GLB_read_quad90_wv_fm, N_SENS_WV, NCYC_SENS, 200, GLB_sens_phase-GLB_read_phase_lag+PI_ON_2);
		Serial.print("p: NEW PHASE_LAG IS- ");Serial.println(new_lag);
		//If this takes too long, we could also introduce a new pointer for the read waveform and increment it differently to i_sens
	}
	else {
		Serial.print("p: Phase is currently set to "); Serial.println(GLB_read_phase_lag);
	}
}
void cmdGetReadLag(){ //read_lag
		Serial.print("read_lag: ");Serial.println(GLB_read_phase_lag);
}
void cmdUnrecognised(const char *command){
	Serial.println("p: Don't understand!");
}

void connectCommands(){
	sCmd.addCommand("read_lag", cmdSetReadLag);	// 
	sCmd.addCommand("read_lag?", cmdGetReadLag);	// 
	sCmd.addCommand("scan_params", cmdSetFreqScanParams);	// 
	sCmd.addCommand("integ_time", cmdSetIntegTime);	// 

	sCmd.addCommand("sens_phase", cmdSetSensPhase);	// 
	sCmd.addCommand("sens_phase?", cmdGetSensPhase);	// 
	sCmd.addCommand("sens_freq", cmdSetSensFreq);	// 
	sCmd.addCommand("sample_rate", cmdSetSampleRate);	// 
	sCmd.addCommand("sens_wfm?", cmdGetSensWfm);	// 
	sCmd.addCommand("ref_wfm?", cmdGetRefWfm);	// 
	sCmd.addCommand("sens_amp", cmdSetSensAmplitude);	// 
	sCmd.addCommand("sens_amp?", cmdGetSensAmplitude);	// 
	sCmd.addCommand("test_coms", cmdTestComs);	// 		//
	sCmd.addCommand("send_data_on", cmdSendDataOn);

	sCmd.setDefaultHandler(cmdUnrecognised);  
}
// END SERIAL COMMUNCATIONS STUFF
