#ifndef __CONTROL_H__
#define __CONTROL_H__

#include "motor.h"
#include "encoder.h"
#include "FreeRTOS.h"
#include "timers.h"

#include "uart.h"

void getData(void);

void test_PID_forward(void);
void test_PID_backward(void);
void test_PID_left(void);
void test_PID_right(void);

void PID_forward(void);
void PID_backward(void);
void PID_left(void);
void PID_right(void);

void motor_Stop(void);

void motor_SpeedUp(void);
void motor_SpeedDown(void);
void motor_SpeedReset(void);

#endif
