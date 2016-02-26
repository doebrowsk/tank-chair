/*
 * Generated with the FPGA Interface C API Generator 1.2.0
 * for NI-RIO 3.5.0 or later.
 */

#ifndef __NiFpga_mainFPGA_h__
#define __NiFpga_mainFPGA_h__

#ifndef NiFpga_Version
   #define NiFpga_Version 120
#endif

#include "NiFpga.h"

/**
 * The filename of the FPGA bitfile.
 *
 * This is a #define to allow for string literal concatenation. For example:
 *
 *    static const char* const Bitfile = "C:\\" NiFpga_mainFPGA_Bitfile;
 */
#define NiFpga_mainFPGA_Bitfile "NiFpga_mainFPGA.lvbitx"

/**
 * The signature of the FPGA bitfile.
 */
static const char* const NiFpga_mainFPGA_Signature = "11C3842D8DACD58B4D7C9B81312C59F4";

typedef enum
{
   NiFpga_mainFPGA_IndicatorBool_PassCRC = 0x815A,
   NiFpga_mainFPGA_IndicatorBool_RCOn = 0x81A2,
   NiFpga_mainFPGA_IndicatorBool_RCeSTOP = 0x819E,
   NiFpga_mainFPGA_IndicatorBool_SerialHold = 0x81FE,
   NiFpga_mainFPGA_IndicatorBool_eSTOPTriggered = 0x8222,
   NiFpga_mainFPGA_IndicatorBool_flushIntegral = 0x81DE,
} NiFpga_mainFPGA_IndicatorBool;

typedef enum
{
   NiFpga_mainFPGA_IndicatorI8_LOutput = 0x81C2,
   NiFpga_mainFPGA_IndicatorI8_ROutput = 0x81C6,
} NiFpga_mainFPGA_IndicatorI8;

typedef enum
{
   NiFpga_mainFPGA_IndicatorU8_SatelittesComputed = 0x8156,
   NiFpga_mainFPGA_IndicatorU8_SatelittesTracked = 0x8136,
} NiFpga_mainFPGA_IndicatorU8;

typedef enum
{
   NiFpga_mainFPGA_IndicatorI16_13VMonitormV = 0x822E,
   NiFpga_mainFPGA_IndicatorI16_24VMonitormV = 0x822A,
   NiFpga_mainFPGA_IndicatorI16_5VMonitormV = 0x8232,
   NiFpga_mainFPGA_IndicatorI16_FPGAVersion = 0x821E,
   NiFpga_mainFPGA_IndicatorI16_LeftMotorVTicksPerDt = 0x81CA,
   NiFpga_mainFPGA_IndicatorI16_RightMotorVTicksPerDt = 0x81D6,
   NiFpga_mainFPGA_IndicatorI16_YawRatemV = 0x8186,
   NiFpga_mainFPGA_IndicatorI16_YawRefmV = 0x8182,
   NiFpga_mainFPGA_IndicatorI16_YawSwingmV = 0x817A,
   NiFpga_mainFPGA_IndicatorI16_YawTempmV = 0x817E,
   NiFpga_mainFPGA_IndicatorI16_cRIOVMonitormV = 0x8226,
   NiFpga_mainFPGA_IndicatorI16_eSTOPVMonitor = 0x8236,
} NiFpga_mainFPGA_IndicatorI16;

typedef enum
{
   NiFpga_mainFPGA_IndicatorU16_C1Steering = 0x818A,
   NiFpga_mainFPGA_IndicatorU16_C2Throttle = 0x818E,
   NiFpga_mainFPGA_IndicatorU16_C3Mode = 0x8192,
} NiFpga_mainFPGA_IndicatorU16;

typedef enum
{
   NiFpga_mainFPGA_IndicatorI32_LPosMotor = 0x8218,
   NiFpga_mainFPGA_IndicatorI32_LPosWheel = 0x820C,
   NiFpga_mainFPGA_IndicatorI32_RPosMotor = 0x8214,
   NiFpga_mainFPGA_IndicatorI32_RPosWheel = 0x8210,
} NiFpga_mainFPGA_IndicatorI32;

typedef enum
{
   NiFpga_mainFPGA_IndicatorU32_CompassResponseCount = 0x8128,
   NiFpga_mainFPGA_IndicatorU32_ComputedCRC = 0x8138,
   NiFpga_mainFPGA_IndicatorU32_DifferentialAgeFloat = 0x8160,
   NiFpga_mainFPGA_IndicatorU32_GPSPOSCount = 0x815C,
   NiFpga_mainFPGA_IndicatorU32_HeadingFloat = 0x8118,
   NiFpga_mainFPGA_IndicatorU32_LatStdDevFloat = 0x8148,
   NiFpga_mainFPGA_IndicatorU32_LongStdDevFloat = 0x8140,
   NiFpga_mainFPGA_IndicatorU32_PitchFloat = 0x8114,
   NiFpga_mainFPGA_IndicatorU32_PositionType = 0x813C,
   NiFpga_mainFPGA_IndicatorU32_RollFloat = 0x8110,
   NiFpga_mainFPGA_IndicatorU32_SolutionAgeFloat = 0x8164,
   NiFpga_mainFPGA_IndicatorU32_SolutionStatus = 0x8150,
   NiFpga_mainFPGA_IndicatorU32_Sonar1PWM = 0x8168,
   NiFpga_mainFPGA_IndicatorU32_Sonar2PWM = 0x816C,
   NiFpga_mainFPGA_IndicatorU32_Sonar3PWM = 0x8170,
   NiFpga_mainFPGA_IndicatorU32_Sonar4PWM = 0x8174,
   NiFpga_mainFPGA_IndicatorU32_Sonar5PWM = 0x8260,
} NiFpga_mainFPGA_IndicatorU32;

typedef enum
{
   NiFpga_mainFPGA_IndicatorI64_LMotorTicks = 0x81B8,
   NiFpga_mainFPGA_IndicatorI64_RMotorTicks = 0x81BC,
} NiFpga_mainFPGA_IndicatorI64;

typedef enum
{
   NiFpga_mainFPGA_IndicatorU64_LattitudeDouble = 0x814C,
   NiFpga_mainFPGA_IndicatorU64_LongitudeDouble = 0x8144,
} NiFpga_mainFPGA_IndicatorU64;

typedef enum
{
   NiFpga_mainFPGA_ControlBool_FilterRC = 0x81B6,
   NiFpga_mainFPGA_ControlBool_RCCh3Enable = 0x81B2,
   NiFpga_mainFPGA_ControlBool_enableMotors = 0x820A,
} NiFpga_mainFPGA_ControlBool;

typedef enum
{
   NiFpga_mainFPGA_ControlI8_PIDMaxOutput = 0x8202,
} NiFpga_mainFPGA_ControlI8;

typedef enum
{
   NiFpga_mainFPGA_ControlU8_BaudRateCode = 0x825E,
   NiFpga_mainFPGA_ControlU8_NumberOfFPGALEDBlinks = 0x823A,
   NiFpga_mainFPGA_ControlU8_SerialAddress = 0x824E,
   NiFpga_mainFPGA_ControlU8_SerialDeadband = 0x8256,
   NiFpga_mainFPGA_ControlU8_SerialTimeout = 0x825A,
   NiFpga_mainFPGA_ControlU8_SlewRateControl = 0x8252,
} NiFpga_mainFPGA_ControlU8;

typedef enum
{
   NiFpga_mainFPGA_ControlI16_13VScale = 0x8246,
   NiFpga_mainFPGA_ControlI16_24VScale = 0x823E,
   NiFpga_mainFPGA_ControlI16_5VScale = 0x824A,
   NiFpga_mainFPGA_ControlI16_LPIDDerGain = 0x81EA,
   NiFpga_mainFPGA_ControlI16_LPIDIntGain = 0x81E6,
   NiFpga_mainFPGA_ControlI16_LPIDProGain = 0x81E2,
   NiFpga_mainFPGA_ControlI16_PIDLCmdTicksPerDt = 0x81D2,
   NiFpga_mainFPGA_ControlI16_PIDRCmdTicksPerDt = 0x81DA,
   NiFpga_mainFPGA_ControlI16_RPIDDerGain = 0x81FA,
   NiFpga_mainFPGA_ControlI16_RPIDIntGain = 0x81F6,
   NiFpga_mainFPGA_ControlI16_RPIDProGain = 0x81F2,
   NiFpga_mainFPGA_ControlI16_cRIOVScale = 0x8242,
   NiFpga_mainFPGA_ControlI16_eStopThresholdmV = 0x8206,
} NiFpga_mainFPGA_ControlI16;

typedef enum
{
   NiFpga_mainFPGA_ControlU16_PIDdt = 0x81CE,
   NiFpga_mainFPGA_ControlU16_RCCh3Hysteresis = 0x81AE,
   NiFpga_mainFPGA_ControlU16_RCOnLimit = 0x81AA,
   NiFpga_mainFPGA_ControlU16_RCSteeringGain = 0x8196,
   NiFpga_mainFPGA_ControlU16_RCThrottleGain = 0x819A,
   NiFpga_mainFPGA_ControlU16_RCeSTOPLimit = 0x81A6,
   NiFpga_mainFPGA_ControlU16_maxPIDSpeedTicksPerDt = 0x81EE,
} NiFpga_mainFPGA_ControlU16;

typedef enum
{
   NiFpga_mainFPGA_ControlI32_CompassTimeoutms = 0x8124,
   NiFpga_mainFPGA_ControlI32_GPSTimeoutms = 0x812C,
} NiFpga_mainFPGA_ControlI32;

typedef enum
{
   NiFpga_mainFPGA_ControlU32_CompassPollRateus = 0x8120,
   NiFpga_mainFPGA_ControlU32_TabControl = 0x8264,
} NiFpga_mainFPGA_ControlU32;

typedef enum
{
   NiFpga_mainFPGA_IndicatorArrayU8_Checksum = 0x8130,
   NiFpga_mainFPGA_IndicatorArrayU8_CompassBytes = 0x811C,
   NiFpga_mainFPGA_IndicatorArrayU8_GPSPOSBytes = 0x810C,
} NiFpga_mainFPGA_IndicatorArrayU8;

typedef enum
{
   NiFpga_mainFPGA_IndicatorArrayU8Size_Checksum = 4,
   NiFpga_mainFPGA_IndicatorArrayU8Size_CompassBytes = 21,
   NiFpga_mainFPGA_IndicatorArrayU8Size_GPSPOSBytes = 100,
} NiFpga_mainFPGA_IndicatorArrayU8Size;

#endif
