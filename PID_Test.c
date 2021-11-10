#include <stdio.h>
#include <stdlib.h>

#include "PID.h"

/* Controller parameters */
#define PID_KP  7.0f
#define PID_KI  0.0f
#define PID_KD  0.0f

#define PID_TAU 0.01f

#define PID_LIM_MIN -100.0f
#define PID_LIM_MAX  100.0f

#define PID_LIM_MIN_INT -100.0f
#define PID_LIM_MAX_INT  100.0f

#define SAMPLE_TIME_S 0.01f

float Get_sensor_measurment();

int main()
{
    /* Initialise PID controller */
    PIDController pid = { PID_KP, PID_KI, PID_KD,
                          PID_TAU,
                          PID_LIM_MIN, PID_LIM_MAX,
			  PID_LIM_MIN_INT, PID_LIM_MAX_INT,
                          SAMPLE_TIME_S };

    PIDController_Init(&pid);

    /* Simulate response using test system */
    float setpoint = 0.0f;

    printf("System Output\tControllerOutput\r\n");
    while (1) {

        /* Get measurement from system */
        float measurement = Get_sensor_measurment();

        /* Compute new control signal */
        PIDController_Update(&pid, setpoint, measurement);

        printf("%f\t%f\n",measurement, pid.out);
    }

    return 0;
}

float Get_sensor_measurment() {
    float out;
    scanf("%f",&out);
    return out;
}
