#include "fpga.h"
#include "iniparser.h"
#include "harlielog.h"
#include "NiFpga_mainFPGA.h"
#include "packets.h"
#include <stdio.h>
#include "harliealloc.h"
#include "sysutils.h"
#include "microprocessorinterface.h"
#include <vxWorks.h>
#include "MainCRIO.h"
#include "math.h"
#include "zconf.h"

NiFpga_Session FPGA_Session;
NiFpga_Status FPGA_Status;

float leftWheelConversionConstant;
float rightWheelConversionConstant;

uint16_t max_pid_speed = 0;
int16_t left_pid_pro_gain = 0;
int16_t right_pid_pro_gain = 0;
int16_t left_pid_int_gain = 0;
int16_t right_pid_int_gain = 0;
int16_t left_pid_der_gain = 0;
int16_t right_pid_der_gain = 0;
uint8_t sabertooth_address = 130;

uint32_t FPGA_IsGPSNewPrevious;
uint32_t FPGA_IsCompassNewPrevious;

/**
 * Float to fixed point value of uint16_t
 */
uint16_t floatToFXP16_8(const float f) {
	float temp = f * (float) pow(2, 8);
	temp = ceil(temp);
	uint16_t retval = (uint16_t) temp;
	return retval;
}

/**
 * Float to fixed point value of uint32_t
 */
uint32_t floatToFXP32_16(const float f) {
	float temp = f * (float) pow(2, 16);
	temp = ceil(temp);
	uint32_t retval = (uint32_t) temp;
	return retval;
}

/**
 * Fixed point 16bit to float
 */
float fxp16_8ToFloat(const uint16_t value) {
	float temp = (float) (((float)value)/((float)(pow(2,8))));
	return temp;
}

/**
 * Fixed point 32bit to float
 */
float fxp32_16ToFloat(const uint32_t value) {
	float temp = (float) (((float)value)/((float)(pow(2,16))));
	return temp;
}


/**
 * Initializes a microprocessor struct and initializes all the functions in the struct
 */
MicroprocessorInterface * FPGA_Interface()
{
	MicroprocessorInterface * interface = (MicroprocessorInterface*)hMalloc(sizeof(MicroprocessorInterface));
	interface->boot = &FPGA_Boot;
	interface->close = &FPGA_Close;
	interface->getLeftMotorTicks = &FPGA_GetLeftMotorTicks;
	interface->getRightMotorTicks = &FPGA_GetRightMotorTicks;
	interface->getLeftWheelTicks = &FPGA_GetLeftWheelTicks;
	interface->getRightWheelTicks = &FPGA_GetRightWheelTicks;
	interface->getYawRate = &FPGA_GetYawRate;
	interface->setLeftWheelVelocity = &FPGA_SetLeftWheelVelocity;
	interface->setRightWheelVelocity = &FPGA_SetRightWheelVelocity;
	interface->getRightMotorVelocity = &FPGA_GetRightMotorVelocity;
	interface->getLeftMotorVelocity = &FPGA_GetLeftMotorVelocity;
	interface->getGPSArrayData = &FPGA_GetGPSArrayData;
	interface->getGPSArrayLength = &FPGA_GetGPSArrayLength;
	interface->setMotorStatus=&FPGA_SetMotorStatus;
	interface->getESTOPTriggered = &FPGA_GetESTOPTriggered;
	interface->getRCOn = &FPGA_GetRCOn;
	interface->getSonarPing_1 = &FPGA_GetSonarPing_1;
	interface->getSonarPing_2 = &FPGA_GetSonarPing_2;
	interface->getSonarPing_3 = &FPGA_GetSonarPing_3;
	interface->getSonarPing_4 = &FPGA_GetSonarPing_4;
	interface->getSonarPing_5 = &FPGA_GetSonarPing_5;
	interface->isGPSNew = &FPGA_IsGPSNew;
	interface->isCompassNew = &FPGA_IsCompassNew;
	interface->getCompassHeading=&FPGA_GetCompassHeading;
        interface->getFPGAVersion = &FPGA_GetVersion;
        interface->getCRIOVLineRead = &FPGA_GetCRIOVLineRead;
        interface->getESTOPVLineRead = &FPGA_GetESTOPVLineRead;
        interface->get5VLineRead = &FPGA_Get5VLineRead;
        interface->get13VLineRead = &FPGA_Get13VLineRead;
        interface->get24VLineRead = &FPGA_Get24VLineRead;
        interface->getRCCH1 = &FPGA_GetRCCH1;
        interface->getRCCH2 = &FPGA_GetRCCH2;
        interface->getRCCH3 = &FPGA_GetRCCH3;
        interface->getRCESTOP = &FPGA_GetRCESTOP;
        interface->getYawRef = &FPGA_GetYawRef;
        interface->getYawSwing = &FPGA_GetYawSwing;
        interface->getYawTemp = &FPGA_GetYawTemp;
	return interface;
}

//  Initializes the FPGA, the sabertooth and the mototrs
void FPGA_Boot(void)
{
	LOG.INFO("Initializing FPGA...");
	FPGA_Status = NiFpga_Initialize();
	if (NiFpga_IsNotError(FPGA_Status))
	{
		// opens a session, downloads the bitstream, and runs the FPGA.
		LOG.INFO("Opening a session FPGA...");

		NiFpga_MergeStatus(&FPGA_Status, NiFpga_Open(NiFpga_mainFPGA_Bitfile,
					NiFpga_mainFPGA_Signature,
					"RIO0",
					NiFpga_OpenAttribute_NoRun,
					&FPGA_Session));
		if (NiFpga_IsNotError(FPGA_Status))
		{
			LOG.INFO("ReDownloading the FPGA");
			NiFpga_MergeStatus(&FPGA_Status,NiFpga_Download(FPGA_Session));
			if (NiFpga_IsNotError(FPGA_Status))
			{
				LOG.INFO("Restarting the FPGA");
				NiFpga_MergeStatus(&FPGA_Status,NiFpga_Reset(FPGA_Session));
				if (NiFpga_IsNotError(FPGA_Status))
				{
					LOG.INFO("Running the FPGA");
					NiFpga_MergeStatus(&FPGA_Status,NiFpga_Run(FPGA_Session, 0));

					if (NiFpga_IsNotError(FPGA_Status))
					{
					}
					else
					{
						LOG.ERR("FPGA Fail to run  fpga %d ",FPGA_Status);
					}

				}
				else
				{
					LOG.ERR("FPGA Fail to redownload fpga %d ",FPGA_Status);
				}
			}
			else
			{
				LOG.ERR("FPGA Fail to redownload fpga %d ",FPGA_Status);
			}
		}
		else
		{
			LOG.ERR("FPGA Fail to  open a session. Error Code %d ",FPGA_Status);
		}
	}
	LOG.VERBOSE("Reading Constants for the fpga");

	if(fileExists("config/odometry.ini"))
	{
		dictionary* config = iniparser_load("config/odometry.ini");
		leftWheelConversionConstant  = 1/(iniparser_getdouble(config,"MotorEncoderConstant:MeterPerTickLeft",0)) * PIDUpdateRateInMs/1000;
		rightWheelConversionConstant = 1/(iniparser_getdouble(config,"MotorEncoderConstant:MeterPerTickRight",0)) * PIDUpdateRateInMs/1000;
		iniparser_freedict(config);
		LOG.VERBOSE("Odometry Ini file loaded for fpga!");

	}
	else
	{
		LOG.ERR("!!!!!!!!!!!!!!!!odomentry.ini was not found!!!!!!!!!!!!!!!!!!!!!!");
	}

        if(fileExists("config/pid.ini")) {
            dictionary* config = iniparser_load("config/pid.ini");
            max_pid_speed = (uint16_t) iniparser_getint(config, "Both:MaxSpeedTicksPerDt", 0);
            left_pid_pro_gain = iniparser_getint(config, "LeftPID:ProportionalGain",0);
            left_pid_int_gain = iniparser_getint(config, "LeftPID:IntegralGain",0);
            left_pid_der_gain = iniparser_getint(config, "LeftPID:DerivativeGain", 0);
            right_pid_pro_gain = iniparser_getint(config, "RightPID:ProportionalGain",0);
            right_pid_int_gain = iniparser_getint(config, "RightPID:IntegralGain",0);
            right_pid_der_gain = iniparser_getint(config, "RightPID:DerivativeGain",0);
            iniparser_freedict(config);
            LOG.VERBOSE("PID INI file loaded for FPGA");
        }
        else {
            LOG.ERR("PID.ini was not found");
        }

        if(fileExists("config/CRIO.ini")) {
            dictionary* config = iniparser_load("config/CRIO.ini");
            sabertooth_address = (uint8_t) iniparser_getint(config, "Sabertooth:Address", 130);
            iniparser_freedict(config);
            LOG.VERBOSE("Sabertooth address loaded for FPGA");            
        }
        else {
            LOG.ERR("Unable to load Sabertooth address");
        }

        NiFpga_MergeStatus(&FPGA_Status,NiFpga_WriteU8(FPGA_Session,NiFpga_mainFPGA_ControlU8_SlewRateControl, 10));
        if (NiFpga_IsError(FPGA_Status))
        {
                LOG.ERR("Failed to set Sabertooth slew rate");
        }

        FPGA_SetPIDdt(PIDUpdateRateInMs * 1000);
        FPGA_setMaxPIDSpeed(max_pid_speed);
        FPGA_setLPIDProGain(left_pid_pro_gain);
        FPGA_setLPIDIntGain(left_pid_int_gain);
        FPGA_setLPIDDerGain(left_pid_der_gain);
        FPGA_setRPIDProGain(right_pid_pro_gain);
        FPGA_setRPIDIntGain(right_pid_int_gain);
        FPGA_setRPIDDerGain(right_pid_der_gain);
        FPGA_setSabertoothAddress(sabertooth_address);

	    // This logs the version number
        LOG.INFO("FPGA VERSION =  %d",FPGA_GetVersion());

	LOG.INFO("Turning on the motors");
	FPGA_SetMotorStatus(1);
}

// Sets the sabertooth address
void FPGA_setSabertoothAddress(uint8_t address) {
    NiFpga_MergeStatus(&FPGA_Status,NiFpga_WriteU8(FPGA_Session,NiFpga_mainFPGA_ControlU8_SerialAddress, address));
    if (NiFpga_IsError(FPGA_Status))
    {
            LOG.ERR("Failed to set Sabertooth Address");
    }
}

// Closes the FPGA session
void FPGA_Close(void)
{
	// close the session now that we're done 
	printf("Closing the session...");
	if (NiFpga_IsNotError(FPGA_Status))
	{
		NiFpga_MergeStatus(&FPGA_Status, NiFpga_Close(FPGA_Session, 0));
	}
	// must be called after all other calls
	printf("Finalizing...");

	NiFpga_MergeStatus(&FPGA_Status, NiFpga_Finalize());
}

// Sets the right wheel velocity
void FPGA_SetRightWheelVelocity(float MetersPerSecond)
{
	/* Check if NAN */
	if (MetersPerSecond != MetersPerSecond)
	{
		MetersPerSecond = 0.0;
		LOG.ERR("Right Wheel command was NaN. Sending 0 instead");
	}
	int16_t ticks = MetersPerSecond * rightWheelConversionConstant;
	LOG.DATA("RwT Commanded: %d for %f", ticks, MetersPerSecond);

	NiFpga_MergeStatus(&FPGA_Status,NiFpga_WriteI16(FPGA_Session,NiFpga_mainFPGA_ControlI16_PIDRCmdTicksPerDt,ticks));
	if (NiFpga_IsError(FPGA_Status))
	{
		LOG.ERR("Setting Right Wheel Velocity Failed.");
	}

}

// Sets the left wheel velocity
void FPGA_SetLeftWheelVelocity(float MetersPerSecond)
{
	/* Check if NAN */
	if (MetersPerSecond != MetersPerSecond)
	{
		MetersPerSecond = 0.0;
		LOG.ERR("Left Wheel command was NaN. Sending 0 instead");
	}

	int16_t ticks = MetersPerSecond * leftWheelConversionConstant;
	LOG.DATA("LwT Commanded: %d for %f", ticks, MetersPerSecond);
	NiFpga_MergeStatus(&FPGA_Status,NiFpga_WriteI16(FPGA_Session,NiFpga_mainFPGA_ControlI16_PIDLCmdTicksPerDt,ticks));
	if (NiFpga_IsError(FPGA_Status))
	{
		LOG.ERR("Setting Left Wheel Velocity Failed.");
	}
}

// Returns the tick value of the left motor
int32_t FPGA_GetLeftMotorTicks()
{
	int32_t ticks;
	NiFpga_MergeStatus(&FPGA_Status,NiFpga_ReadI32(FPGA_Session,NiFpga_mainFPGA_IndicatorI32_LPosMotor,&ticks));
	if (NiFpga_IsError(FPGA_Status))
	{
		LOG.ERR("Get Left Motor Ticks Failed.");
	}
	return ticks;
}

// Returns the tick value of the right motor
int32_t FPGA_GetRightMotorTicks()
{
	int32_t ticks;
	NiFpga_MergeStatus(&FPGA_Status,NiFpga_ReadI32(FPGA_Session,NiFpga_mainFPGA_IndicatorI32_RPosMotor,&ticks));
	if (NiFpga_IsError(FPGA_Status))
	{
		LOG.ERR("Get Right Motor Ticks Failed.");
	}
	return ticks;
}

// Returns the velocity of the right motor
float FPGA_GetRightMotorVelocity()
{
	int16_t ticks;
	NiFpga_MergeStatus(&FPGA_Status,NiFpga_ReadI16(FPGA_Session,NiFpga_mainFPGA_IndicatorI16_RightMotorVTicksPerDt,&ticks));
	if (NiFpga_IsError(FPGA_Status))
	{
		LOG.ERR("Get Right Motor Velocity Failed.");
	}
	float Speed = ticks /rightWheelConversionConstant;
	LOG.DATA("RMV = %f",Speed);
	return Speed;
}

// Returns the velocity of the left motor
float FPGA_GetLeftMotorVelocity()
{
	int16_t ticks;
	NiFpga_MergeStatus(&FPGA_Status,NiFpga_ReadI16(FPGA_Session,NiFpga_mainFPGA_IndicatorI16_LeftMotorVTicksPerDt,&ticks));
	if (NiFpga_IsError(FPGA_Status))
	{
		LOG.ERR("Get Left Motor Velocity Failed.");
	}
	float Speed =  ticks /leftWheelConversionConstant;
	LOG.DATA("LMV = %f",Speed);
	return Speed;
}

// Returns the tick value on the left wheel
int32_t FPGA_GetLeftWheelTicks()
{
	int32_t ticks;
	NiFpga_MergeStatus(&FPGA_Status,NiFpga_ReadI32(FPGA_Session,NiFpga_mainFPGA_IndicatorI32_LPosWheel,&ticks));
	if (NiFpga_IsError(FPGA_Status))
	{
		LOG.ERR("Get Left Wheel Ticks Failed.");
	}
	LOG.DATA("LwT =  %d",ticks);
	return ticks;
}

// Returns the tick value on the right wheel
int32_t FPGA_GetRightWheelTicks()
{
	int32_t ticks;
	NiFpga_MergeStatus(&FPGA_Status,NiFpga_ReadI32(FPGA_Session,NiFpga_mainFPGA_IndicatorI32_RPosWheel,&ticks));
	if (NiFpga_IsError(FPGA_Status))
	{
		LOG.ERR("Get Right Wheel Ticks Failed.");
	}
	LOG.DATA("RwT  = %d",ticks);
	return ticks;
}

// Returns the yaw rate
int16_t FPGA_GetYawRate()
{
	int16_t value;
	NiFpga_MergeStatus(&FPGA_Status,NiFpga_ReadI16(FPGA_Session,NiFpga_mainFPGA_IndicatorI16_YawRatemV,&value));
        if (NiFpga_IsError(FPGA_Status))
	{
		LOG.ERR("Get Yaw Rate Failed.");
	}
	LOG.DATA("YawRate  = %d",value);
	return value;
}

// Returns the yaw swing
int16_t FPGA_GetYawSwing()
{
	int16_t value;
	NiFpga_MergeStatus(&FPGA_Status,NiFpga_ReadI16(FPGA_Session,NiFpga_mainFPGA_IndicatorI16_YawSwingmV,&value));
        if (NiFpga_IsError(FPGA_Status))
	{
		LOG.ERR("Get Yaw Swing Failed.");
	}
	LOG.DATA("YawSwing  = %d",value);
	return value;
}

int16_t FPGA_GetYawTemp()
{
	int16_t value;
	NiFpga_MergeStatus(&FPGA_Status,NiFpga_ReadI16(FPGA_Session,NiFpga_mainFPGA_IndicatorI16_YawTempmV,&value));
        if (NiFpga_IsError(FPGA_Status))
	{
		LOG.ERR("Get Yaw Temp Failed.");
	}
	LOG.DATA("YawTemp  = %d",value);
	return value;
}

int16_t FPGA_GetYawRef()
{
	int16_t value;
	NiFpga_MergeStatus(&FPGA_Status,NiFpga_ReadI16(FPGA_Session,NiFpga_mainFPGA_IndicatorI16_YawRefmV,&value));
        if (NiFpga_IsError(FPGA_Status))
	{
		LOG.ERR("Get Yaw Ref Failed.");
	}
	LOG.DATA("YawRef = %d",value);
	return value;
}

/*
void FPGA_SetYawRateReset(int value)
{
	if(value)
	{
		NiFpga_MergeStatus(&FPGA_Status,NiFpga_WriteBool(FPGA_Session,NiFpga_mainFPGA_ControlBool_YawRateReset,NiFpga_True));
		
                if (NiFpga_IsError(FPGA_Status))
		{
			LOG.ERR("Set Yaw Rate Reset Failed.");
		}
	}
	else
	{
                
		NiFpga_MergeStatus(&FPGA_Status,NiFpga_WriteBool(FPGA_Session,NiFpga_mainFPGA_ControlBool_YawRateReset,NiFpga_False));
		
                if (NiFpga_IsError(FPGA_Status))
		{
			LOG.ERR("Set Yaw Rate Reset Failed.");
		}
	}
}
*/

uint32_t FPGA_GetSonarPing_1() {
	uint32_t value;
	NiFpga_MergeStatus(&FPGA_Status, NiFpga_ReadU32(FPGA_Session, NiFpga_mainFPGA_IndicatorU32_Sonar1PWM, &value)); 
	LOG.DATA("SonarPing 1= %d", value);
	return value;
}

uint32_t FPGA_GetSonarPing_2() {
	uint32_t value;
	NiFpga_MergeStatus(&FPGA_Status, NiFpga_ReadU32(FPGA_Session, NiFpga_mainFPGA_IndicatorU32_Sonar2PWM, &value)); 
	LOG.DATA("SonarPing 2= %d", value);
	return value;
}

uint32_t FPGA_GetSonarPing_3() {
	uint32_t value;
	NiFpga_MergeStatus(&FPGA_Status, NiFpga_ReadU32(FPGA_Session, NiFpga_mainFPGA_IndicatorU32_Sonar3PWM, &value)); 
	LOG.DATA("SonarPing 3= %d", value);
	return value;
}


uint32_t FPGA_GetSonarPing_4() {
	uint32_t value;
	NiFpga_MergeStatus(&FPGA_Status, NiFpga_ReadU32(FPGA_Session, NiFpga_mainFPGA_IndicatorU32_Sonar4PWM, &value)); 
	LOG.DATA("SonarPing 4= %d", value);
	return value;
}


uint32_t FPGA_GetSonarPing_5() {
	uint32_t value;
	NiFpga_MergeStatus(&FPGA_Status, NiFpga_ReadU32(FPGA_Session, NiFpga_mainFPGA_IndicatorU32_Sonar5PWM, &value)); 
	LOG.DATA("SonarPing 5= %d", value);
	return value;
}

int32_t FPGA_GetGPSArrayLength()
{
	return 100;
}

uint32_t FPGA_IsGPSNew(void)
{
	uint32_t current;
	NiFpga_MergeStatus(&FPGA_Status,NiFpga_ReadU32(FPGA_Session,NiFpga_mainFPGA_IndicatorU32_GPSPOSCount,&current));
	if(NiFpga_IsError(FPGA_Status))
	{
		LOG.ERR("Get GPS Counter Failed");
	}
	if(current != FPGA_IsGPSNewPrevious)
	{
            FPGA_IsGPSNewPrevious = current;
            return 1;
	}
	return 0;
}

int32_t FPGA_GetGPSArrayData(uint8_t * data, uint32_t length)
{

	NiFpga_MergeStatus(&FPGA_Status,NiFpga_ReadArrayU8(FPGA_Session, NiFpga_mainFPGA_IndicatorArrayU8_GPSPOSBytes, data, length));
	if (NiFpga_IsError(FPGA_Status))
	{
		LOG.ERR("GetArrayFailed");
	}
	return 0;
}

uint32_t FPGA_IsCompassNew(void)
{
	uint32_t current;
	NiFpga_MergeStatus(&FPGA_Status,NiFpga_ReadU32(FPGA_Session,NiFpga_mainFPGA_IndicatorU32_CompassResponseCount,&current));
	if(NiFpga_IsError(FPGA_Status))
	{
		LOG.ERR("Get Compass Counter Failed");
	}
	if(current != FPGA_IsCompassNewPrevious)
	{
		current = FPGA_IsCompassNewPrevious;
		return 1;
	}
	return 0;
}

float FPGA_GetCompassHeading(void)
{
	uint32_t current;
	NiFpga_MergeStatus(&FPGA_Status,NiFpga_ReadU32(FPGA_Session,NiFpga_mainFPGA_IndicatorU32_HeadingFloat,&current));
	if(NiFpga_IsError(FPGA_Status))
	{
		LOG.ERR("Get Compass Heading Failed");
	}
	float out;
	out = *((float *)&current);
	LOG.DATA("Compass %f",out);
	return out;
}

void FPGA_SetMotorStatus(int value)
{
	if(value)
	{
		NiFpga_MergeStatus(&FPGA_Status,NiFpga_WriteBool(FPGA_Session,NiFpga_mainFPGA_ControlBool_enableMotors,NiFpga_True));
		if (NiFpga_IsError(FPGA_Status))
		{
			LOG.ERR("Set Motor Status Failed.");
		}
	}
	else
	{
		NiFpga_MergeStatus(&FPGA_Status,NiFpga_WriteBool(FPGA_Session,NiFpga_mainFPGA_ControlBool_enableMotors,NiFpga_False));
		if (NiFpga_IsError(FPGA_Status))
		{
			LOG.ERR("Set Motor Status Failed.");
		}
	}
}

uint8_t FPGA_GetRCOn()
{
	NiFpga_Bool  RC;
	NiFpga_MergeStatus(&FPGA_Status,NiFpga_ReadBool(FPGA_Session,NiFpga_mainFPGA_IndicatorBool_RCOn,&RC));
	if (NiFpga_IsError(FPGA_Status))
	{
		LOG.ERR("Get RCon Status Failed.");
	}
	if(RC == NiFpga_False)
	{
		LOG.DATA("RCOn  = False");

		return 0;
	}
	else
	{
		LOG.DATA("RCOn  = True");
		return 1;
	}
	return 0;
}

uint8_t FPGA_GetESTOPTriggered()
{
	NiFpga_Bool  ReStopTriggered;
	NiFpga_MergeStatus(&FPGA_Status,NiFpga_ReadBool(FPGA_Session,NiFpga_mainFPGA_IndicatorBool_eSTOPTriggered,&ReStopTriggered));
	if (NiFpga_IsError(FPGA_Status))
	{
		LOG.ERR("Get EStop Status Failed.");
	}
	if(ReStopTriggered == NiFpga_False)
	{
		LOG.DATA("EStopTriggered  = False");
		return 0;
	}
	else
	{
		LOG.DATA("EStopTriggered  = True");
		return 1;
	}
	return 0;
}

/*
void FPGA_SetYawRateOffset(int32_t value)
{
        NiFpga_MergeStatus(&FPGA_Status,NiFpga_WriteI32(FPGA_Session,NiFpga_mainFPGA_ControlI32_YawRateOffsetuV,value))
        
        if (NiFpga_IsError(FPGA_Status))
	{
		LOG.ERR("Set Yaw Rate Offset Failed.");
	}
}

void FPGA_SetYawRateSlope(int32_t value)
{
        NiFpga_MergeStatus(&FPGA_Status,NiFpga_WriteI32(FPGA_Session,NiFpga_mainFPGA_ControlI32_YawRateSlopeuVperV,value));

        if (NiFpga_IsError(FPGA_Status))
	{
		LOG.ERR("Set Yaw Rate Slope failed");
	}
}
*/

int16_t FPGA_GetVersion()
{
	int16_t version;
	NiFpga_MergeStatus(&FPGA_Status,NiFpga_ReadI16(FPGA_Session,NiFpga_mainFPGA_IndicatorI16_FPGAVersion,&version));
	if (NiFpga_IsError(FPGA_Status))
	{
		LOG.ERR("Get FPGA Version Failed.");
	}
	return version;
}

void FPGA_SetPIDdt(uint16_t dt) {
    NiFpga_MergeStatus(&FPGA_Status,NiFpga_WriteU16(FPGA_Session,NiFpga_mainFPGA_ControlU16_PIDdt, dt));
    if (NiFpga_IsError(FPGA_Status))
    {
            LOG.ERR("Set PID dt failed");
    }
}

void FPGA_setMaxPIDSpeed(uint16_t max) {
    NiFpga_MergeStatus(&FPGA_Status,NiFpga_WriteU16(FPGA_Session,NiFpga_mainFPGA_ControlU16_maxPIDSpeedTicksPerDt, max));
    if (NiFpga_IsError(FPGA_Status))
    {
            LOG.ERR("Set max PID speed failed");
    }
}

void FPGA_setLPIDProGain(int16_t gain) {
    NiFpga_MergeStatus(&FPGA_Status,NiFpga_WriteI16(FPGA_Session,NiFpga_mainFPGA_ControlI16_LPIDProGain,gain));
    if (NiFpga_IsError(FPGA_Status))
    {
            LOG.ERR("Set left PID proportional gain failed");
    }
}

void FPGA_setRPIDProGain(int16_t gain) {
    NiFpga_MergeStatus(&FPGA_Status,NiFpga_WriteI16(FPGA_Session,NiFpga_mainFPGA_ControlI16_RPIDProGain,gain));
    if (NiFpga_IsError(FPGA_Status))
    {
            LOG.ERR("Set right PID proportional gain failed");
    }
}

void FPGA_setLPIDIntGain(int16_t gain) {
    NiFpga_MergeStatus(&FPGA_Status,NiFpga_WriteI16(FPGA_Session,NiFpga_mainFPGA_ControlI16_LPIDIntGain,gain));
    if (NiFpga_IsError(FPGA_Status))
    {
            LOG.ERR("Set left PID integral gain failed");
    }
}

void FPGA_setRPIDIntGain(int16_t gain) {
    NiFpga_MergeStatus(&FPGA_Status,NiFpga_WriteI16(FPGA_Session,NiFpga_mainFPGA_ControlI16_RPIDIntGain,gain));
    if (NiFpga_IsError(FPGA_Status))
    {
            LOG.ERR("Set right PID integral gain failed");
    }
}

void FPGA_setLPIDDerGain(int16_t gain) {
    NiFpga_MergeStatus(&FPGA_Status,NiFpga_WriteI16(FPGA_Session,NiFpga_mainFPGA_ControlI16_LPIDDerGain,gain));
    if (NiFpga_IsError(FPGA_Status))
    {
            LOG.ERR("Set left PID derivative gain failed");
    }
}

void FPGA_setRPIDDerGain(int16_t gain) {
    NiFpga_MergeStatus(&FPGA_Status,NiFpga_WriteI16(FPGA_Session,NiFpga_mainFPGA_ControlI16_RPIDDerGain,gain));
    if (NiFpga_IsError(FPGA_Status))
    {
            LOG.ERR("Set right PID derivative gain failed");
    }
}

int16_t FPGA_Get24VLineRead() {
    int16_t retval;
    NiFpga_MergeStatus(&FPGA_Status,NiFpga_ReadI16(FPGA_Session,NiFpga_mainFPGA_IndicatorI16_24VMonitormV,&retval));
    if (NiFpga_IsError(FPGA_Status))
    {
            LOG.ERR("Get 24V Line Read Failed.");
    }
    return retval;
}

int16_t FPGA_Get13VLineRead() {
    int16_t retval;
    NiFpga_MergeStatus(&FPGA_Status,NiFpga_ReadI16(FPGA_Session,NiFpga_mainFPGA_IndicatorI16_13VMonitormV,&retval));
    if (NiFpga_IsError(FPGA_Status))
    {
            LOG.ERR("Get 13V Line Read Failed.");
    }
    return retval;
}

int16_t FPGA_Get5VLineRead() {
    int16_t retval;
    NiFpga_MergeStatus(&FPGA_Status,NiFpga_ReadI16(FPGA_Session,NiFpga_mainFPGA_IndicatorI16_5VMonitormV,&retval));
    if (NiFpga_IsError(FPGA_Status))
    {
            LOG.ERR("Get 5V Line Read Failed.");
    }
    return retval;
}

int16_t FPGA_GetESTOPVLineRead() {
    int16_t retval;
    NiFpga_MergeStatus(&FPGA_Status,NiFpga_ReadI16(FPGA_Session,NiFpga_mainFPGA_IndicatorI16_eSTOPVMonitor,&retval));
    if (NiFpga_IsError(FPGA_Status))
    {
            LOG.ERR("Get eStopV Line Read Failed.");
    }
    return retval;
}

int16_t FPGA_GetCRIOVLineRead() {
    int16_t retval;
    NiFpga_MergeStatus(&FPGA_Status,NiFpga_ReadI16(FPGA_Session,NiFpga_mainFPGA_IndicatorI16_cRIOVMonitormV,&retval));
    if (NiFpga_IsError(FPGA_Status))
    {
            LOG.ERR("Get cRIOV Line Read Failed.");
    }
    return retval;
}

uint16_t FPGA_GetRCCH1() {
    uint16_t retval;
    NiFpga_MergeStatus(&FPGA_Status,NiFpga_ReadU16(FPGA_Session,NiFpga_mainFPGA_IndicatorU16_C1Steering,&retval));
    if (NiFpga_IsError(FPGA_Status))
    {
            LOG.ERR("Get RC CH1  Read Failed.");
    }
    return retval;
}

uint16_t FPGA_GetRCCH2() {
    uint16_t retval;
    NiFpga_MergeStatus(&FPGA_Status,NiFpga_ReadU16(FPGA_Session,NiFpga_mainFPGA_IndicatorU16_C2Throttle,&retval));
    if (NiFpga_IsError(FPGA_Status))
    {
            LOG.ERR("Get RC CH2  Read Failed.");
    }
    return retval;
}

uint16_t FPGA_GetRCCH3() {
    uint16_t retval;
    NiFpga_MergeStatus(&FPGA_Status,NiFpga_ReadU16(FPGA_Session,NiFpga_mainFPGA_IndicatorU16_C3Mode,&retval));
    if (NiFpga_IsError(FPGA_Status))
    {
            LOG.ERR("Get RC CH3  Read Failed.");
    }
    return retval;
}

uint8_t FPGA_GetRCESTOP() {
    NiFpga_Bool rcestop;
    NiFpga_MergeStatus(&FPGA_Status,NiFpga_ReadBool(FPGA_Session,NiFpga_mainFPGA_IndicatorBool_RCeSTOP,&rcestop));
    if (NiFpga_IsError(FPGA_Status))
    {
            LOG.ERR("Get RCEStop Failed.");
    }
    if(rcestop == NiFpga_False)
    {
            LOG.DATA("RCEstop  = False");
            return 0;
    }
    else
    {
            LOG.DATA("RCEStop  = True");
            return 1;
    }
}
