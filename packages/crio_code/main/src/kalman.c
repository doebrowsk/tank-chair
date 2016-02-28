#include "basicfusion.h"
#include "statevariable.h"
#include "harlielog.h"
#include "MainCRIO.h"
#include "harliealloc.h"
#include <stdio.h>
#include <vxWorks.h>
#include <math.h>
#include <string.h>
#include "sensorfusion.h"
#include "kalman.h"
#include "matrixLib.h"

matrix _H;
matrix _HT;
matrix _G;
matrix _GT;
matrix _P;
matrix _Y;
matrix _S;
matrix _K;
matrix _StateEye;
matrix _ModelCovarianceMatrix;
matrix _SensorMatrix;
matrix _SensorCovarianceMatrix;
matrix _State;
matrix _Temp1m3x3;
matrix _Temp2m3x3;
matrix _Temp3m3x3;
matrix _Temp4m4x1;
matrix _Temp5m4x3;
matrix _Temp6m4x4;
matrix _Temp7m3x4;
matrix _Temp8m3x1;
matrix _Temp9m3x3;
matrix _Temp10m3x3;
matrix _Temp11m3x3;
matrix _Temp12m3x3;
matrix _Temp13m3x1;

float _LeftWheelSpeedPrediction;
float _RightWheelSpeedPrediction;
float _RightWheelConstant;
float _LeftWheelConstant;
float _DthetaConstant;
float _PSOUpdateRateInS;
float _Track;

// Initializes the Kalman filter
SensorFusion * Kalman_Init()
{
	 // TODO: Fix allocation locations for varaiables

	SensorFusion * fusion = hMalloc(sizeof(SensorFusion));

    _P=Matrix_Create(3,3);
    Matrix_SetValue(_P,1,1,1);
    Matrix_SetValue(_P,2,2,1);
    Matrix_SetValue(_P,3,3,1);

    _ModelCovarianceMatrix = Matrix_Create(3,3);
    _SensorMatrix = Matrix_Create(4,1);
    _SensorCovarianceMatrix = Matrix_Create(4,4);
    _HT = Matrix_Create(3,4);
    _GT = Matrix_Create(3,3);
    _S = Matrix_Create(4,4);
    _K = Matrix_Create(3,4);
    _Y = Matrix_Create(4,1);
    _State = Matrix_Create(3,1);
    _StateEye = Matrix_Create(3,3);
    _StateEye = Matrix_MakeI(_StateEye);

    _H=Matrix_Create(4,3);
    Matrix_SetValue(_H,1,1,1);
    Matrix_SetValue(_H,2,2,1);
    Matrix_SetValue(_H,3,3,1);
    Matrix_SetValue(_H,4,3,1);

    _G=Matrix_Create(3,3);
    Matrix_SetValue(_G,1,1,1);
    Matrix_SetValue(_G,2,2,1);
    Matrix_SetValue(_G,3,3,1);

	_Temp1m3x3 = Matrix_Create(3,3);
	_Temp2m3x3 = Matrix_Create(3,3);
	_Temp3m3x3 = Matrix_Create(3,3);
	_Temp4m4x1 = Matrix_Create(4,1);
	_Temp5m4x3 = Matrix_Create(4,3);
	_Temp6m4x4 = Matrix_Create(4,4);
	_Temp7m3x4 = Matrix_Create(3,4);
	_Temp8m3x1 = Matrix_Create(3,1);
	_Temp9m3x3 = Matrix_Create(3,3);
	_Temp10m3x3 = Matrix_Create(3,3);
	_Temp11m3x3 = Matrix_Create(3,3);
	_Temp12m3x3 = Matrix_Create(3,3);
	_Temp13m3x1 = Matrix_Create(3,1);

	_LeftWheelSpeedPrediction = 0;
	_RightWheelSpeedPrediction = 0;
    // Fix this to be  read froma file
    // Fix all of these
	_Track = .5677;

	float constant = .0525;
	float e = 2.71828183;
	_RightWheelConstant = (1-1/pow(e,constant));
	_LeftWheelConstant = (1-1/pow(e,constant));
	_PSOUpdateRateInS = PSOUpdateRateInMs/1000;

	_DthetaConstant = (PSOUpdateRateInMs/1000)/_Track;

	SensorFusion_Init(fusion);
	fusion->updateFilter = &Kalman_UpdateFilter;
	return fusion;
}

// Updates the filter values and determines the wheel speed prediction values
void Kalman_UpdateFilter(SensorFusion * fusion,WheelSpeedCommand speeds)
{
	int i;
	StateVariable location = fusion->currentState;
    LOG.KALMAN("Starting sensor update");
	for(i = 0; i < fusion->sensorCount; i++)
	{
		fusion->sensorList[i]->update(fusion->sensorList[i],location);
	}
    LOG.KALMAN("Sensors updated");

	float LastX = fusion->currentState.x;
	float LastY = fusion->currentState.y;
	float LastTheta = fusion->currentState.theta;

    LOG.KALMAN("Making the speed predictions");

	_LeftWheelSpeedPrediction = (speeds.leftWheelSpeed - _LeftWheelSpeedPrediction) * (_LeftWheelConstant) + _LeftWheelSpeedPrediction;
	_RightWheelSpeedPrediction = (speeds.rightWheelSpeed - _RightWheelSpeedPrediction) * (_RightWheelConstant) + _RightWheelSpeedPrediction;

    LOG.KALMAN("Left Wheel Speed Prediction %f",_LeftWheelSpeedPrediction);
    LOG.KALMAN("Right Wheel Speed Prediction %f",_RightWheelSpeedPrediction);
	
	float predictedDtheta = (_LeftWheelSpeedPrediction - _RightWheelSpeedPrediction)*_DthetaConstant;
	float modelTheta = predictedDtheta + LastTheta;

	float modelX = 0;
	float modelY = 0;

	if(_RightWheelSpeedPrediction-_LeftWheelSpeedPrediction >=.0000001 && _RightWheelSpeedPrediction - _LeftWheelSpeedPrediction <= -.0000001)
	{
        LOG.KALMAN("!=0 path");
		float ChangeFactor = _Track * (_LeftWheelSpeedPrediction + _RightWheelSpeedPrediction)/(2*(_LeftWheelSpeedPrediction - _RightWheelSpeedPrediction));
        LOG.KALMAN("Changefactor = %f",ChangeFactor);
		modelX = LastX + ChangeFactor*(sin(modelTheta)-sin(LastTheta));
		modelY = LastY + ChangeFactor*(cos(modelTheta)-cos(LastTheta));
        Matrix_SetValue(_G,1,3,ChangeFactor * (-cos(LastTheta)+cos(modelTheta)));
        Matrix_SetValue(_G,2,3,ChangeFactor * (-sin(LastTheta)+sin(modelTheta)));
        LOG.KALMAN("_M_G");
        Matrix_Print(_G);
		ChangeFactor * (-cos(LastTheta)+cos(modelTheta));
        LOG.KALMAN("1,3 = %f",ChangeFactor*(-cos(LastTheta)+cos(modelTheta)));
        LOG.KALMAN("2,3 = %f",ChangeFactor*(-sin(LastTheta)+sin(modelTheta)));
	}
	else
	{
        LOG.KALMAN("==0 path");
		float distance = _RightWheelSpeedPrediction*_PSOUpdateRateInS;
		modelX = LastX + distance * cos(LastTheta);
		modelY = LastY - distance*sin(LastTheta);
        Matrix_SetValue(_G,1,3,-1 * distance *sin(LastTheta));
        Matrix_SetValue(_G,2,3,distance * cos(LastTheta));
        LOG.KALMAN("_M_G");
        Matrix_Print(_G);
        LOG.KALMAN("1,3 = %f",-1*distance*sin(LastTheta));
        LOG.KALMAN("2,3 = %f",distance*cos(LastTheta));
	}

    Matrix_SetValue(_State,1,1,modelX);
    Matrix_SetValue(_State,2,1,modelY);
    Matrix_SetValue(_State,3,1,limitTheta(modelTheta));

    LOG.KALMAN("MODELX = %f, MODELY = %f, MODELTHETA = %f",modelX,modelY,modelTheta);
    Matrix_Print(_State);

    Matrix_SetValue(_ModelCovarianceMatrix,1,1,.03);
    Matrix_SetValue(_ModelCovarianceMatrix,2,2,.03);
    Matrix_SetValue(_ModelCovarianceMatrix,3,3,1);

    Matrix_SetValue(_SensorMatrix,1,1,fusion->sensorList[0]->state->x);
    Matrix_SetValue(_SensorMatrix,2,1,fusion->sensorList[0]->state->y);
    Matrix_SetValue(_SensorMatrix,3,1,fusion->sensorList[0]->state->theta);
    Matrix_SetValue(_SensorMatrix,4,1,fusion->sensorList[1]->state->theta);
    LOG.KALMAN("%f",fusion->sensorList[0]->state->theta);
    LOG.KALMAN("%f",fusion->sensorList[1]->state->theta);
    LOG.KALMAN("Semsor Marix");
    Matrix_Print(_SensorMatrix);

	// Create the Covarince matrix for the sensors
    Matrix_SetValue(_SensorCovarianceMatrix,1,1,fusion->sensorList[0]->standardDeviation->x);
    Matrix_SetValue(_SensorCovarianceMatrix,2,2,fusion->sensorList[0]->standardDeviation->y);
    Matrix_SetValue(_SensorCovarianceMatrix,3,3,fusion->sensorList[0]->standardDeviation->theta);
    Matrix_SetValue(_SensorCovarianceMatrix,4,4,fusion->sensorList[0]->standardDeviation->theta);

    Matrix_SetValue(_SensorCovarianceMatrix,1,1,.01);
    Matrix_SetValue(_SensorCovarianceMatrix,2,2,.01);
    Matrix_SetValue(_SensorCovarianceMatrix,3,3,.1);
    Matrix_SetValue(_SensorCovarianceMatrix,4,4,.001);

	// The kalman filter equations.
    LOG.KALMAN("Running the Kalman filter");

    LOG.KALMAN("Finding _P");
    Matrix_Print(_G);
    Matrix_Print(_P);
    Matrix_Print(_ModelCovarianceMatrix);

    _P=Matrix_Add(Matrix_Product(Matrix_Product(_G,_P,_Temp1m3x3),Matrix_Transpose(_G,_GT),_Temp3m3x3),_ModelCovarianceMatrix,_P);

    Matrix_Print(_P);
    Matrix_Print(_SensorMatrix);
    Matrix_Print(_State);
    Matrix_Print(_H);

    LOG.KALMAN("Finding _Y");
    _Y=Matrix_Subtract(_SensorMatrix,Matrix_Product(_H,_State,_Temp4m4x1),_Y);

    Matrix_Print(_State);
    LOG.KALMAN("Finding _S");
    Matrix_Print(_H);
    Matrix_Print(_P);
    Matrix_Print(_SensorCovarianceMatrix);
    _S=Matrix_Add(Matrix_Product(Matrix_Product(_H,_P,_Temp5m4x3),Matrix_Transpose(_H,_HT),_Temp6m4x4),_SensorCovarianceMatrix,_S);
    LOG.KALMAN("Finding _K");
    Matrix_Print(_S);
    Matrix_Print(_P);
    Matrix_Print(_HT);
    Matrix_Print(_H);

    _K=Matrix_Product(Matrix_Product(_P,_HT,_Temp7m3x4),Matrix_Inverse(_S),_K);
    LOG.KALMAN("Finding _State");
    Matrix_Print(_State);
    Matrix_Print(_K);
    Matrix_Print(_Y);
    _State = Matrix_Copy(Matrix_Add(_State,Matrix_Product(_K,_Y,_Temp8m3x1),_Temp13m3x1),_State);

    LOG.KALMAN("Finding _P");

    Matrix_Print(_State);
    _P=Matrix_Copy(Matrix_Product(Matrix_Subtract(_StateEye,Matrix_Product(_K,_H,_Temp9m3x3),_Temp10m3x3),_P,_Temp11m3x3),_P);

    Matrix_Print(_State);

    LOG.KALMAN("Writing out the data to the location");
    location.x = Matrix_GetValue(_State,1,1);
    location.y = Matrix_GetValue(_State,2,1);
    location.theta = limitTheta(Matrix_GetValue(_State,3,1));
	fusion->currentState = location;

    LOG.INFO("Location after update\n\tx=%f\n\ty=%f\n\ttheta=%f",location.x,location.y,location.theta);
}
