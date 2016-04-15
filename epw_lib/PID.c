#include "stm32f4xx.h"
#include "PID.h"

void Init_pid(pid_struct *pid, float p, float i, float d){	
	pid -> Kp = p;
	pid -> Ki = i;
	pid -> Kd = d;
	pid -> Kp_temp = 0.0f;
	pid -> Ki_temp = 0.0f;
	pid -> Kd_temp = 0.0f;
	pid -> err = 0.0f;
	pid -> prev_err = 0.0f;
	pid -> int_err = 0.0f;
	pid -> diff = 0.0f;
	pid -> output = 0.0f;
}

float pid_cal(pid_struct *pid, float x, float y){
	/*------------------------------------------------------------------------
	 * The Formula of Incremental PID controller:
	 * e(n) = x(n) - y(n)
	 * u(n) = Kp * e(n) + Ki * SUM(e(i), i: from 0 to n) + Kd * (e(n) - e(n-1))
	 *-----------------------------------------------------------------------*/

	 pid -> err = x - y;
	 pid -> int_err += pid -> err;

	 if (pid ->int_err >255) pid -> int_err = 255;
	 else if (pid -> int_err <0) pid -> int_err = 0;

	 pid -> diff = pid -> err - pid -> prev_err;

	 pid -> Kp_temp = pid -> Kp * pid -> err;
	 pid -> Ki_temp = pid -> Ki * pid -> int_err;
	 pid -> Kd_temp = pid -> Kd * pid -> diff;

	 //Calculate the output
	 pid -> output = pid -> Kp_temp + pid -> Ki_temp + pid -> Kd_temp;

 	 if(pid -> output >255.0f) pid -> output = 255.0f;
	 else if(pid -> output <0.0f) pid -> output = 0.0f;

	 //Update the error
	 pid -> prev_err = pid -> err;

	 return pid -> output;
}