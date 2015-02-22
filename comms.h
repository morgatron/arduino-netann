#ifndef COMMS_H
#define COMMS_H

void sendData(float freq, float sig0, float sig90);
void cmdSendDataOn();
void cmdGetSensWfm();
void cmdGetSampleRate();
void cmdGetRefWfm();
void cmdTestComs();
void cmdSetSensPhase();
void cmdSetSensAmplitude();
void cmdGetSensAmplitude();
void cmdSetFreqScanParams();
void cmdSetReadLag();
void cmdGetReadLag();
void cmdUnrecognised(const char *command);
void connectCommands();


#endif //COMMS_H
