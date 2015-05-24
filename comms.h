#ifndef COMMS_H
#define COMMS_H

void sendData(float freq, float sig0, float sig90);
void cmdCalibrate();
void cmdSendDataOn();
void cmdGetSensWfm();
void cmdGetSampleRate();
void cmdGetRefWfm();
void cmdTestComs();
void cmdSetSensPhase();
void cmdSetSensAmplitudeV();
void cmdGetSensAmplitudeV();
void cmdSetFreqScanParams();
void cmdSetReadLag();
void cmdGetReadLag();
void cmdUnrecognised(const char *command);
void connectCommands();


#endif //COMMS_H
