#include "microprocessorinterface.h"
#include <vxWorks.h>

#ifndef FPGA_H
#define FPGA_H

#define FIFOTIMEOUT 1

MicroprocessorInterface * FPGA_Interface();
void FPGA_Boot();
void FPGA_Close();

// Wheel tick functions
int32_t FPGA_GetLeftWheelTicks();
int32_t FPGA_GetRightWheelTicks();

// Motor functions
int32_t FPGA_GetLeftMotorTicks();
int32_t FPGA_GetRightMotorTicks();
float FPGA_GetRightMotorVelocity();
float FPGA_GetLeftMotorVelocity();

// Wheel velocity functions
void FPGA_SetRightWheelVelocity(float MetersPerSecond);
void FPGA_SetLeftWheelVelocity(float MetersPerSecond);

// Yaw functions
int16_t FPGA_GetYawRate();
int16_t FPGA_GetYawSwing();
int16_t FPGA_GetYawTemp();
int16_t FPGA_GetYawRef();

// Voltage functions
int16_t FPGA_Get24VLineRead();
int16_t FPGA_GetCRIOVLineRead();
int16_t FPGA_GetESTOPVLineRead();
int16_t FPGA_Get13VLineRead();
int16_t FPGA_Get5VLineRead();
uint16_t FPGA_GetRCCH1();
uint16_t FPGA_GetRCCH2();
uint16_t FPGA_GetRCCH3();
uint8_t FPGA_GetRCESTOP();

// Motor Status
void FPGA_SetMotorStatus(int value);

// GPS function
int32_t FPGA_GetGPSArrayLength();
int16_t FPGA_GetVersion();

// Estop functions
uint8_t FPGA_GetRCOn();
uint8_t FPGA_GetESTOPTriggered();

// GPS functions
int32_t FPGA_GetGPSArrayData(uint8_t * data,uint32_t length);
uint32_t FPGA_IsGPSNew(void);

// Compass functions
uint32_t FPGA_IsCompassNew(void);
float FPGA_GetCompassHeading(void);

// Sonar Functions
uint32_t FPGA_GetSonarPing_1();
uint32_t FPGA_GetSonarPing_2();
uint32_t FPGA_GetSonarPing_3();
uint32_t FPGA_GetSonarPing_4();
uint32_t FPGA_GetSonarPing_5();

// Yaw rate functions
void FPGA_SetYawRateSlope(int32_t value);
void FPGA_SetYawRateOffset(int32_t value);

// dt function
void FPGA_SetPIDdt(uint16_t dt);

// Max speed function
void FPGA_setMaxPIDSpeed(uint16_t max);

//Gain functions
void FPGA_setLPIDProGain(int16_t gain);
void FPGA_setRPIDProGain(int16_t gain);
void FPGA_setLPIDIntGain(int16_t gain);
void FPGA_setRPIDIntGain(int16_t gain);
void FPGA_setLPIDDerGain(int16_t gain);
void FPGA_setRPIDDerGain(int16_t gain);

// Sabertooth Address
void FPGA_setSabertoothAddress(uint8_t address);


#endif /* FPGA_H */
