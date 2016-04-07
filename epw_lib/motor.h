/* 20150708 wei han for the test of EPW2 */
#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "stm32f4xx.h"

/* Define all pins of the motor of EPW2 */
#define MOTOR_PWM_PORT				GPIOD

/* The switch of the motor drivers */
#define MOTOR_RELAY_PIN				GPIO_Pin_9

/* The switch to toggle constant voltage and control signal */
#define MOTOR_LEFT_SW_PIN			GPIO_Pin_10
#define MOTOR_RIGHT_SW_PIN			GPIO_Pin_11 

/* Constant Voltage for motor driver initialization */
#define MOTOR_LEFT_CV_PIN			GPIO_Pin_12 //Green  TIM4_CH1
#define MOTOR_RIGHT_CV_PIN			GPIO_Pin_13 //Orange TIM4_CH2

/* Control Signal */
#define MOTOR_LEFT_PWM_PIN			GPIO_Pin_13 //Red  TIM4_CH3
#define MOTOR_RIGHT_PWM_PIN			GPIO_Pin_15 //Blue TIM4_CH4


void mPowerON(void);
void mPowerOFF(void);
void mStop(void);
void mMove(uint32_t SpeedValue_left, uint32_t SpeedValue_right);
void mForward(void);
void mBackward(void);
void mLeft(void);
void mRight(void);

void init_motor(void);

#endif 
/* __MOTOR_H__ */
