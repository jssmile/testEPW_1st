#ifndef __CONTROL_H__
#define __CONTROL_H__

#include "motor.h"
#include "encoder.h"
#include "FreeRTOS.h"
#include "timers.h"

#include "uart.h"

void getData(void);
void test_forward(void);
void move_forward(void);
void test_PID(void);
void PID_forward(void);
void tic(void);
void toc(void);
void SysTick_Handler(void);

#endif
