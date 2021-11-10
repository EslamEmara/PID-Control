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
    float error = setpoint - actual;

	/*
	* Proportional
	*/
    float proportional = pid->Kp * error;


	/*
	* Derivative (band-limited differentiator)
	*/

		pid->differentiator = -(2.0f * pid->Kd * (actual - pid->prevMeasurement)	/* Note: derivative on measurement, therefore minus sign in front of equation! */
												+ (2.0f * pid->tau - pid->T) * pid->differentiator)
												/ (2.0f * pid->tau + pid->T);

	/*
	* Integral
	*/

		pid->integrator = pid->integrator + 0.5f * pid->Ki * pid->T * (error + pid->prevError);

	/* Anti-wind-up via integrator clamping */
		if (pid->integrator > pid->limMaxInt) {

				pid->integrator = pid->limMaxInt;

		} else if (pid->integrator < pid->limMinInt) {

				pid->integrator = pid->limMinInt;

		}


	/*
	* Compute output and apply limits
	*/
    pid->out = proportional + pid->integrator + pid->differentiator;
    if (pid->out > pid->limMax) {

        pid->out = pid->limMax;

    } else if (pid->out < pid->limMin) {

        pid->out = pid->limMin;
    }

	/* Store error and measurement for later use */
    pid->prevMeasurement = actual;
		pid->prevError       = error;
	/* Return controller output */
    return pid->out;
}

typedef struct{
	short unsigned int ROV_ROTATING
	short unsigned int ROV_ROLLING
	short unsigned int ROV_PITCHING
}ROV_MOTION;

typedef struct{
	short unsigned int YAW_PID_STATE
	short unsigned int PITCH_PID_STATE
	short unsigned int ROLL_PID_STATE
}PID_STATES;

typedef struct{
	float YAW_SETPOINT;
	float ROLL_SETPOINT;
	float PITCH_SETPOINT;
}SETPOINTS;

ROV_MOTION MOTION_STATES;
PID_STATES	ON_OFF_STATES;
SETPOINTS ANGLE_SETPOINTS;

float IMU_READING[3] ={0}; 

void Set_setpoint(float *axes){

	if (axes[YAW_AXIS] != 0){
		MOTION_STATES.ROV_ROTATING = 1;
	}
	else{
		MOTION_STATES.ROV_ROTATING = 0;
	}
	if (axes[ROLL_AXIS] != 0){
		MOTION_STATES.ROV_ROLLING = 1;
	}
	else{
		MOTION_STATES.ROV_ROLLING = 0;
	}
	if (axes[PITCH_AXIS] != 0){
		MOTION_STATES.ROV_PITCHING = 1;
	}
	else{
		MOTION_STATES.ROV_PITCHING = 0;
	}

	if(MOTION_STATES.ROV_ROTATING ==1){
		PID_STATES.YAW_PID_STATE =0;
		ANGLE_SETPOINTS.YAW_SETPOINT = IMU_READING[YAW_READING];
	}
	else{
		PID_STATES.YAW_PID_STATE = 1;
	}

	if(MOTION_STATES.ROV_PITCHING ==1){
		PID_STATES.PITCH_PID_STATE =0;
		ANGLE_SETPOINTS.PITCH_SETPOINT = IMU_READING[PITCH_READING];
	}
	else{
		PID_STATES.PITCH_PID_STATE = 1;
	}

	if(MOTION_STATES.ROV_ROLLING ==1){
		PID_STATES.ROLL_PID_STATE =0;
		ANGLE_SETPOINTS.ROLL_SETPOINT = IMU_READING[ROLL_READING];
	}
	else{
		PID_STATES.ROLL_PID_STATE = 1;
	}





}
