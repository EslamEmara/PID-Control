#include "PID.h"

void PIDController_Init(PIDController *pid) {
	/* Clear controller variables */
	pid->integrator = 0.0f;
	pid->prevError  = 0.0f;

	pid->differentiator  = 0.0f;
	pid->prevMeasurement = 0.0f;

	pid->out = 0.0f;
}

float PIDController_Update(PIDController *pid, float setpoint, float actual) {

	/*
	* Error signal
	*/


	/*
	* Proportional
	*/


	/*
	* Derivative (band-limited differentiator)
	*/



	/*
	* Integral
	*/


	/* Anti-wind-up via integrator clamping */



	/*
	* Compute output and apply limits
	*/


	/* Store error and measurement for later use */

	/* Return controller output */

}
