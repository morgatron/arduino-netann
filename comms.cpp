#include <PID_v1.h>
#include "SerialCommand.h"
#include "netAnn.h"
#include "comms.h"

// EXTERNS----------------------------------------

// Wave-forms-------------------------------------

extern NetworkAnalyser netAnn;



//Needed to share with comms.h
//extern volatile int GLB_imax_on_amp;
extern SerialCommand sCmd; 
///----------------------------------------------------------------
//--------------  SERIAL COMMUNICATIONS STUFF
// Functions for talking to the computer
/*Write the time, read values, and output value*/
void sendFreqRespData(float freq, float sig0, float sig90){
	sendData(freq, sig0, sig0, "freqResp");
}
void sendData(float x, float sig0, float sig90, char[] wvfm_name="raw_sig"){
	char lhs_str[20]="d|"; 
	strcat(lhs_str, wfm_name); strcat(lhs_str, ':');
	Serial.print( lhs_str );
	Serial.print(x);
	Serial.print(" ");
	Serial.print(sig0);
	Serial.print(" ");
	Serial.println(sig90);
}

void cmdCalibrate(){
	netAnn.calibrate();
	Serial.print("new calDACvsADC: ");
	Serial.println(netAnn.calDACvsADC);
	Serial.print("new calWriteReadLag: ");
	Serial.println(netAnn.calWriteReadLag);
}

void cmdGetR(){
	char *arg;
	arg = sCmd.next();
	float freq=10;
	if(arg!=NULL) {
		freq=atof(arg);
	}

	QuadratureSample q=netAnn.freqSampleAndWait(freq);
	q=q.rotated(-freq*netAnn.calWriteReadLag*TWOPI);
	float R=Serial.println(calcR(q.sig0, q.sig90, 50000, netAnn.getSensAmplitudeDig()));
	PRINT_VAR(R);
}

void cmdSendDataOn(){
	char *arg;
	arg = sCmd.next();	 
	if(strcmp(arg,"True")==0 || strcmp(arg,"1")==0) {		
		netAnn.b_send_data=true;
	}
	else if ( strcmp(arg,"False")==0 || strcmp(arg,"0")==0) {	//strcmpr(arg, "false")
		netAnn.b_send_data=false;
	}
	Serial.print("p: Data sending is "); Serial.println(netAnn.b_send_data);

}


void cmdGetSensWfm(){
	Serial.print("sens_wfm: ");
	//for(int k=0; k<N_SENS_WV; k++){
	for(int k=0; k<Nbase; k++){
		const int * wv_fm=netAnn.getSensWvFm();
		Serial.print(wv_fm[k]);
		Serial.print(", ");
	}
	Serial.println("");
}
void cmdGetRefWfm(){
	Serial.print("ref_wfm: ");
	for(int k=0; k<N_REF_WV; k++){
		const int* wv_fm=netAnn.getRefWvFm();
		Serial.print(wv_fm[k]);
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

void cmdSetFreqScanParams(){
	char *arg;
	arg = sCmd.next();	 
	if (arg != NULL) {		
		netAnn.start_freq= atof(arg);
		Serial.print("new startFreq is: ");
		Serial.println(netAnn.start_freq);

		arg = sCmd.next();	 
		if (arg != NULL) {		
			netAnn.stop_freq= atof(arg);
			Serial.print("new stopFreq is: ");
			Serial.println(netAnn.stop_freq);

			arg = sCmd.next();	 
			if (arg != NULL) {		
				netAnn.freq_step= atof(arg);
				Serial.print("new freq_step is: ");
				Serial.println(netAnn.freq_step);
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
		netAnn.integration_time= atof(arg);
		Serial.print("new integration time is: ");
		Serial.println(netAnn.integration_time);
	}
	else {
		Serial.print("p: integration time is "); Serial.println(netAnn.integration_time);
	}
}
void cmdSetSensPhase() {
	char *arg;
	arg = sCmd.next();
	if (arg != NULL) {
		int new_phase = atof(arg);
		new_phase=netAnn._setSensPhase(new_phase);
		Serial.println();
		Serial.print("p: NEW SENSE WAVEFORM PHASE IS: ");
		Serial.println(new_phase);
		Serial.println();
	} else {
		Serial.print("p: Sense phase is currently set to: ");
		Serial.println(netAnn._getSensPhase());
	}
}

void cmdSetSensAmplitude(){
	char *arg;
	arg = sCmd.next();	 
	if (arg != NULL) {		
		float new_amp= atof(arg);

		new_amp=netAnn.setSensAmplitudeV(new_amp);

		Serial.print("p: NEW SENSE WAVEFORM AMPLITUDE IS: ");Serial.println(new_amp);
	}
	else {
		Serial.print("p: Ampl is currently set to- "); Serial.println(netAnn.getSensAmplitudeV());
	}
}


void cmdSetSingleFreq(){
	char *arg;
	arg = sCmd.next();	 
	if (arg != NULL) {		
		float new_freq= atof(arg);
		Serial.print("new_freq is: ");
		Serial.println(netAnn._setCurrentFreq(new_freq));
		netAnn.stopScan();
	}
	else{
		Serial.println("p: Don't understand new sens frequency");
	}
}
void cmdSetSamplePeriod(){
	char *arg;
	arg = sCmd.next();	 
	if (arg != NULL) {		
		int new_period= atof(arg);
		Serial.print("new_rate is: ");
		Serial.println(new_period);
		netAnn.setSamplePeriod(new_period);
	}
	else{
		Serial.println("p: Don't understand new sample period");
	}
}
void cmdSetReadLag(){
	char *arg;
	arg = sCmd.next();	 
	if (arg != NULL) {		
		float new_lag= atof(arg);
		netAnn._setReadLagPts(new_lag);

		//If this takes too long, we could also introduce a new pointer for the read waveform and increment it differently to i_sens
	}
	else {
		Serial.print("p: Phase is currently set to "); Serial.println(netAnn._getReadLagPts());
	}
}

void cmdUnrecognised(const char *command){
	Serial.println("p: Don't understand!");
}

void connectCommands(){
	sCmd.addCommand("read_lag", cmdSetReadLag);	// 
	sCmd.addCommand("scan_params", cmdSetFreqScanParams);	// 
	sCmd.addCommand("integ_time", cmdSetIntegTime);	// 

	sCmd.addCommand("sens_phase", cmdSetSensPhase);	// 
	sCmd.addCommand("sens_freq", cmdSetSingleFreq);	//
	sCmd.addCommand("sample_rate", cmdSetSamplePeriod);	//
	sCmd.addCommand("sens_wfm?", cmdGetSensWfm);	// 
	sCmd.addCommand("ref_wfm?", cmdGetRefWfm);	// 
	sCmd.addCommand("sens_amp", cmdSetSensAmplitude);	// 
	sCmd.addCommand("test_coms", cmdTestComs);	// 		//
	sCmd.addCommand("send_data_on", cmdSendDataOn);

	sCmd.setDefaultHandler(cmdUnrecognised);  
}
// END SERIAL COMMUNCATIONS STUFF
