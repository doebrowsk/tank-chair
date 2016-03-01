#include <vxWorks.h>
#ifndef MICROPROCESSORINTERFACE_H
#define MICROPROCESSORINTERFACE_H
typedef struct MicroprocessorInterfaceCL {
	void (*boot)();
	void (*close)();

	int32_t (*getLeftWheelTicks)();
	int32_t (*getRightWheelTicks)();
	int32_t (*getLeftMotorTicks)();
	int32_t (*getRightMotorTicks)();

	int16_t (*getYawRate)();
	uint32_t (*getSonarPing_1)();
	uint32_t (*getSonarPing_2)();
	uint32_t (*getSonarPing_3)();
	uint32_t (*getSonarPing_4)();
	uint32_t (*getSonarPing_5)();
	void (*setYawSampleCount)(int32_t);
	void (*setRightWheelVelocity)(float);
	void (*setLeftWheelVelocity)(float);
	void (*setMotorStatus)(int);
	void (*setYawRateReset)(int);

	float(*getLeftMotorVelocity)();
	float(*getRightMotorVelocity)();

	int32_t (*getGPSArrayLength)();
	int32_t (*getGPSArrayData)(uint8_t * ,uint32_t);
	uint32_t (*isGPSNew)();

	void (*setYawRateSlope)(int32_t);
	void (*setYawRateOffset)(int32_t);

	uint32_t (*isCompassNew)();
	float (*getCompassHeading)();

    // diagnostics stuff
	uint8_t (*getESTOPTriggered)();
    uint8_t (*getRCOn)();
    int16_t (*getFPGAVersion)();
    int16_t (*getCRIOVLineRead)();
    int16_t (*get5VLineRead)();
    int16_t (*get13VLineRead)();
    int16_t (*get24VLineRead)();
    int16_t (*getESTOPVLineRead)();
    uint16_t (*getRCCH1)();
    uint16_t (*getRCCH2)();
    uint16_t (*getRCCH3)();
    uint8_t (*getRCESTOP)();
    int16_t (*getYawSwing)();
    int16_t (*getYawTemp)();
    int16_t (*getYawRef)();

	void * extraData;
}MicroprocessorInterface;

const MicroprocessorInterface * MicroprocessorInit();

#endif

