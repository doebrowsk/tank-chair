#include "basicfusion.h"
#include "statevariable.h"
#include "harlielog.h"
#include "MainCRIO.h"
#include "harliealloc.h"
#include "matrixLib.h"
#include <stdio.h>
#include <vxWorks.h>
#include <math.h>
#include <string.h>
#include "sensorfusion.h"
#include "dkalman.h"
#include "sysutils.h"
#include "matrixLibStack.h"
#include "odometry.h"
#include "iniparser.h"

// Matricies 
// @TODO: figure out what each matrix is
matrix _G;
matrix _P;
matrix _State;
matrix _Q;

// Other file scoped variables
float _LeftWheelSpeedPrediction;
float _RightWheelSpeedPrediction;
float _RightWheelConstant;
float _LeftWheelConstant;
float _DthetaConstant;
float _Track;
SensorVariable _Previous;

// TODO: figure out what this function does
matrix _zeroBiasXYThetaCov(matrix covariance);

/**
 * Initializes the Kalman sensor and sets the _Q matrix with initial values
 */
SensorFusion * Kalman_Init()
{
	// TODO Fix allocation locations for varaiables
	float modelXY_var = .001;
	float modelTheta_var = .001;
	float modelVel_var = .001;
	float modelOmega_var = .001;
    float modelYawBias_var = 0.001;

	LOG.ODOMETRY("Initalizing the Kalman filter");
	if(fileExists("config/kalman.ini"))
	{
		dictionary* config = iniparser_load("config/kalman.ini");

		modelXY_var = iniparser_getdouble(config,"Variance:XY",1);
		modelTheta_var = iniparser_getdouble(config,"Variance:Theta",1);
		modelVel_var = iniparser_getdouble(config,"Variance:Velocity",1);
		modelOmega_var = iniparser_getdouble(config,"Variance:Omega",1);
        modelYawBias_var = iniparser_getdouble(config,"Variance:YawBias",1);

		iniparser_freedict(config);
		LOG.ODOMETRY("kalman.ini file loaded");
	}
	else
	{
		LOG.ERR("kalman.ini was not found");
	}

    // @TODO: change this to static const
	float constant = .0525;
	float e = 2.71828183;

	_RightWheelConstant = (1-1/pow(e,constant));
	_LeftWheelConstant = (1-1/pow(e,constant));

	_DthetaConstant = (PSOUpdateRateInMs/1000.0)/_Track;

	SensorFusion * fusion = hMalloc(sizeof(SensorFusion));

        LOG.KALMAN("About to initialize the P matrix");
	_P=Matrix_MakeI(6);
        LOG.KALMAN("P matrix make next");
	_P=Matrix_SetValue(_P,1,1,0.001);
        LOG.KALMAN("P matrix make next");
	_P=Matrix_SetValue(_P,2,2,0.001);
        LOG.KALMAN("P matrix make next");
	_P=Matrix_SetValue(_P,3,3,0.001);
        LOG.KALMAN("P matrix make next");
	_P=Matrix_SetValue(_P,4,4,10);
        LOG.KALMAN("P matrix make next");
	_P=Matrix_SetValue(_P,5,5,10);
        LOG.KALMAN("P matrix make next");
        _P=Matrix_SetValue(_P,6,6,.25);
        LOG.KALMAN("P matrix make done");

	fusion->currentState.State = Matrix_Create(6,1);
	fusion->currentState.Variance = Matrix_MakeI(6);
	_State = Matrix_Create(6,1);
        LOG.KALMAN("Initialized P matrix and setup the fusion's current state");
	_G=Matrix_MakeI(6);
	_G = Matrix_SetValue(_G,3,5,PSOUpdateRateInS);

	_Q = Matrix_MakeI(6);
	_Q = Matrix_SetValue(_Q,1,1,modelXY_var);
	_Q = Matrix_SetValue(_Q,2,2,modelXY_var);
	_Q = Matrix_SetValue(_Q,3,3,modelTheta_var);
	_Q = Matrix_SetValue(_Q,4,4,modelVel_var);
	_Q = Matrix_SetValue(_Q,5,5,modelOmega_var);
    _Q = Matrix_SetValue(_Q,6,6,modelYawBias_var);

	_Track = .5677;

	SensorFusion_Init(fusion);
	fusion->updateFilter = &Kalman_UpdateFilter;
	return fusion;
}

/**
 * Some sort of fancy math with magic numbers
 */
matrix _zeroBiasXYThetaCov(matrix covariance) {
    covariance = Matrix_SetValue(covariance, 1, 6, 0.);
    covariance = Matrix_SetValue(covariance, 2, 6, 0.);
    covariance = Matrix_SetValue(covariance, 3, 6, 0.);
    covariance = Matrix_SetValue(covariance, 6, 1, 0.);
    covariance = Matrix_SetValue(covariance, 6, 2, 0.);
    covariance = Matrix_SetValue(covariance, 6, 3, 0.);
    return covariance;
}

/**
 * Prints out the current state in a matrix format and recaculates the _P value of the Kalman filter
 */
void _ModelUpdate(SensorVariable currentState)
{
	float LastX = Matrix_GetX(currentState.State);
	float LastY = Matrix_GetY(currentState.State);
	float LastTheta = Matrix_GetTheta(currentState.State);
	float LastVel = Matrix_GetVel(currentState.State);
	float LastOmega = Matrix_GetOmega(currentState.State);

	float cosLast = cos(LastTheta);
	float sinLast = sin(LastTheta);

	// Place the found values into the _State Matrix
	_State = Matrix_SetValue(_State,1,1,LastX  + LastVel*PSOUpdateRateInS*cosLast);
	_State = Matrix_SetValue(_State,2,1,LastY + LastVel*PSOUpdateRateInS*sinLast);
	_State = Matrix_SetValue(_State,3,1,LastTheta+LastOmega*PSOUpdateRateInS);

	_G = Matrix_SetValue(_G,1,3,-1*PSOUpdateRateInS*sinLast*LastVel);
	_G = Matrix_SetValue(_G,2,3,1*PSOUpdateRateInS*cosLast*LastVel);
	_G = Matrix_SetValue(_G,1,4,1*PSOUpdateRateInS*cosLast);
	_G = Matrix_SetValue(_G,2,4,1*PSOUpdateRateInS*sinLast);

	LOG.KALMAN("Updated State matrix");
	Matrix_Print(_State);

	LOG.KALMAN("Finding _P");
	LOG.KALMAN("P =");
	Matrix_Print(_P);

	LOG.KALMAN("Finding G");
	Matrix_Print(_G);

	LOG.KALMAN("Print Q");
	Matrix_Print(_Q);

	LOG.KALMAN("Finding the _P Value of the Kalman filter");
	_P=Matrix_Add(Matrix_Product(Matrix_Product(_G,_P),Matrix_Transpose(_G)),_Q);
    _P = _zeroBiasXYThetaCov(_P);
	LOG.KALMAN("Printing K");
	Matrix_Print(_P);
}

/**
 *
 */
void Kalman_UpdateFilter(SensorFusion * fusion,WheelSpeedCommand speeds)
{
	SensorVariable location = fusion->currentState;
    // TODO: Update the Model. Perform predict state based on the model of the robot
	// TODO: FIX THIS HACK!!!!!! Cannot be sure that sensorlist[0] exists!!!!
	_ModelUpdate(location);

	_State=Matrix_SetValue(_State, 3, 1, limitTheta(Matrix_GetValue(_State, 3, 1)));

	// Update based on the sensors 
	int i;
	for(i = 0; i < fusion->sensorCount; i++)
	{
		fusion->sensorList[i]->update(fusion->sensorList[i],location);

		if(fusion->sensorList[i]->hasBeenUpdated)
		{
			matrix H = fusion->sensorList[i]->H;
			matrix Residual = fusion->sensorList[i]->state->State;
			matrix SensedVariance = fusion->sensorList[i]->state->Variance;
			// The kalman filter equations.
			LOG.KALMAN("Sensed then Variance then H");
			Matrix_Print(SensedVariance);
			Matrix_Print(H);
			LOG.KALMAN("Finding Y");
			matrix Y = Residual;
			Matrix_Print(Y);

            // TODO: Create temporaries so that the values are computed once and then reused. 
			LOG.KALMAN("MidS HPH'");
			Matrix_Print(Matrix_Product(Matrix_Product(H,_P),Matrix_Transpose(H)));
			LOG.KALMAN("MidS HP");
			Matrix_Print(Matrix_Product(H,_P));

			LOG.KALMAN("MidS H'");
			Matrix_Print(Matrix_Transpose(H));

			LOG.KALMAN("Finding S");
			matrix S=Matrix_Add(Matrix_Product(Matrix_Product(H,_P),Matrix_Transpose(H)),SensedVariance);
			Matrix_Print(S);
			matrix SInverse = Matrix_Inverse(S);
			Matrix_Print(SInverse);
			LOG.KALMAN("Finding K");

			matrix K = Matrix_Product(Matrix_Product(_P,Matrix_Transpose(H)),SInverse);
			Matrix_Print(K);
			LOG.KALMAN("Finding the _State");
			_State = Matrix_Add(_State,Matrix_Product(K,Y));
			Matrix_Print(_State);

			LOG.KALMAN("Finding _P");
			_P = Matrix_Product(Matrix_Subtract(Matrix_MakeI(6),Matrix_Product(K,H)),_P);
            _P = _zeroBiasXYThetaCov(_P);
			Matrix_Print(_P);
			fusion->sensorList[i]->hasBeenUpdated = 0;

			// Make sure we stay with in 2pi
			_State=Matrix_SetValue(_State,3,1,limitTheta(Matrix_GetValue(_State,3,1)));
		}
	}
	_Previous = fusion->currentState;
	fusion->currentState.State = _State;
	fusion->currentState.Variance = _P;
	LOG.KALMAN("Updated Kalman State");
	LOG.KALMAN("X=,%2.4f, Y=,%2.4f, Theta=,%2.4f", Matrix_GetX(_State), Matrix_GetY(_State), Matrix_GetTheta(_State));
	LOG.KALMAN("Xvar=,%2.4f, Yvar=,%2.4f, Thetavar=,%2.4f", Matrix_GetXVar(_P), Matrix_GetYVar(_P), Matrix_GetThetaVar(_P));
	Matrix_Print(_State);
	Matrix_Print(_P);
}
