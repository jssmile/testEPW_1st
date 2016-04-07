
#ifndef __PID__
#define __PID__
typedef struct PID_struct{
	float Kp;
	float Ki;
	float Kd;
	float Kp_temp;
	float Ki_temp;
	float Kd_temp;
	float err;
	float prev_err;
	float int_err;
	float diff;
	float output;
} pid_struct;

extern inline void Init_pid(pid_struct *pid, float p, float i, float d);
extern inline float pid_cal(pid_struct *pid, float x, float y);

#endif