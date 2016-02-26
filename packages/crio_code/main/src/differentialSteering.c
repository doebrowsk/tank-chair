#include "differentialSteering.h"
#include "packets.h"
#include "iniparser.h"
#include "harlielog.h"
#include "sonar.h"
#include "MainCRIO.h"
#include <math.h>

// PID error and gains
PIDErrors real_errors;
PIDGains real_gains;
PIDErrors test_errors;
PIDGains test_gains;

// called by the differentialsteering function.... not sure why there is an underscore.  Maybe a compiler thing??
STATUS _differentialSteering(const HeadingSpeedCommand *commands, WheelSpeedCommand * const speeds, const float currentHeading, PIDGains * const gains, PIDErrors * const errors);

// limits the steering. The underscore is a compiler thing?
STATUS _limitTurningSpeed(WheelSpeedCommand * const input, WheelSpeedCommand * const output);

/**
 * Creates a dictionary from config/steering.ini and loads the gain values into the gain struct.  Also initializes the
 * real_errors too.
 */
STATUS initDifferentialSteering() {
	 // TODO: use file exists shit!! Check if file exists first
	dictionary *config = iniparser_load("config/steering.ini");
	real_gains.proportionalGain = (float) iniparser_getdouble(config, "DifferentialSteering:proportionalGain", 0.2);
	real_gains.derivativeGain = (float) iniparser_getdouble(config, "DifferentialSteering:derivativeGain", 0.0);
	real_gains.integralGain = (float) iniparser_getdouble(config, "DifferentialSteering:integralGain", 0.0);
	iniparser_freedict(config);

    //Initialize error values for real error struct
	real_errors.derivativeError = 0.0;
	real_errors.integralError = 0.0;
	real_errors.propotionalError = 0.0;

	return(OK);
}

/**
 * Steers the robot using the real gains and error and the calculated speed and current heading
 */
STATUS differentialSteering(const HeadingSpeedCommand *commands, WheelSpeedCommand * const speeds, const float currentHeading) {
	return _differentialSteering(commands, speeds, currentHeading, &real_gains, &real_errors);
}

/**
 * Takes the twist commands and sets the desired speed to the wheels and doesn't use currentHeading....???
 */
STATUS twistSteering(const AngularRateSpeedCommand* twist_commands, WheelSpeedCommand* const speeds, const float currentHeading) {
    LOG.VERBOSE("Twist command rads/s = %.5f", twist_commands->desiredAngularRate);
    LOG.VERBOSE("Twist command m/s = %.5f", twist_commands->desiredSpeed);

    float change = twist_commands->desiredAngularRate * trackInMeters / 2.0;

    LOG.VERBOSE("Twist change = %.5f", change);

    float desiredSpeed = twist_commands->desiredSpeed;
    
    speeds->leftWheelSpeed = desiredSpeed + change;
    speeds->rightWheelSpeed = desiredSpeed - change;

    LOG.VERBOSE("Limiting Turning speed");
    STATUS s = _limitTurningSpeed(speeds, speeds);

    return(s);
}

/**
 * Calculates the error and adjusts for the error
 */
STATUS _differentialSteering(const HeadingSpeedCommand *commands, WheelSpeedCommand * const speeds, const float currentHeading, PIDGains * const gains, PIDErrors * const errors) {
	float correct_angle = 0;
	correct_angle = calculateMinimalSteeringAngle(commands->desiredHeading, currentHeading);

	LOG.VERBOSE("Commanded angle: %f, current angle: %f", commands->desiredHeading, currentHeading);
	LOG.VERBOSE("Minimal steering angle is %f", correct_angle);

	errors->propotionalError = correct_angle;
	errors->integralError = (errors->integralError + correct_angle) * gains->integralGain;
	float derivativeChange = errors->propotionalError - errors->derivativeError;
	errors->derivativeError = correct_angle;

	float change = (errors->propotionalError * gains->proportionalGain) + errors->integralError + (derivativeChange * gains->derivativeGain);

	if (fabs(errors->integralError) > 7.0) {
		errors->integralError = 0.0;
	}

	LOG.VERBOSE("Wheel speed change is %f", change);

	// Sonar O shit detection/
    //float dist = getFrontSonarPing(); 
	float dist = 1.0;
	float desiredSpeed = commands->desiredSpeed;

	if (dist < 0.5) {
		desiredSpeed = 0.0;
		LOG.ERR("Sonar Found something!!!!! ESTOPING!!!");
	}

	speeds->leftWheelSpeed = desiredSpeed + change;
	speeds->rightWheelSpeed = desiredSpeed - change;

	LOG.VERBOSE("Limiting Turning speed");
	STATUS s = _limitTurningSpeed(speeds, speeds);

	return(s);
}

/**
 * Doesn't allow the turning speed to go over the wheel speed limit
 */
STATUS _limitTurningSpeed(WheelSpeedCommand * const input, WheelSpeedCommand * const output) {
	float speedA = input->leftWheelSpeed;
	float speedB = input->rightWheelSpeed;

	float x = speedA - speedB;
	float y = (float) (fabs(speedA) + fabs(speedB));

	if(fabs(x) >= MAX_TURNING) {
		speedA = (speedA / y) * MAX_TURNING;
		speedB = (speedB / y) * MAX_TURNING;
	}

	output->leftWheelSpeed = speedA;
	output->rightWheelSpeed = speedB;

	return(OK);
}

/*
* This function takes the desired heading angle and the current heading
* angle and calculates the minimal steering angle. This means that it
* calculates whether it is better to turn right or left to reach a given
* heading based on the current heading.
*
* Ported from Calc_Correct_Steering_Angle.vi
*/
float calculateMinimalSteeringAngle(const float desiredHeading, const float currentHeading) {
    // Needed because apparently VxWorks doesn't have an M_PI constant 
	const float pi = (float) acos(-1.0); 

	float difference = desiredHeading - currentHeading;

	float high = difference + (2.0 * pi);
	float low = difference - (2.0 * pi);

	float angle = difference;
	if (fabs(high) < fabs(angle)) {
		angle = high;
	}
	if (fabs(low) < fabs(angle)) {
		angle = low;
	}

	return angle;
}
