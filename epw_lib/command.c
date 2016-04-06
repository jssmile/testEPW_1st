#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "stm32f4xx_usart.h"
#include "uart.h"
#include "command.h"
#include "motor.h"
#include "linear_actuator.h"
#include "control.h"
/*parse*/
#include "string.h"

extern uint32_t SpeedValue_left;
extern uint32_t SpeedValue_right;
uint32_t inc = 1;

struct receive_cmd_list * receive_cmd_type;

void receive_task(){

		if(Receive_String_Ready){

			//forward step by step 
			if(received_string[0] == '+'){
				SpeedValue_left += inc;
				SpeedValue_right += inc;
				mMove(SpeedValue_left,SpeedValue_right);
				USART_puts(USART3, "left:");
				USART_putd(USART3, SpeedValue_left);
				USART_puts(USART3, " right:");
				USART_putd(USART3, SpeedValue_right);
				USART_puts(USART3, "\r\n");
			}

			//backward step by step
			else if(received_string[0] == '-'){
				SpeedValue_left -= inc;
				SpeedValue_right -= inc;
				mMove(SpeedValue_left, SpeedValue_right);
				USART_puts(USART3, "left:");
				USART_putd(USART3, SpeedValue_left);
				USART_puts(USART3, " right:");
				USART_putd(USART3, SpeedValue_right);
				USART_puts(USART3, "\r\n");
			}

			//forward
			if(received_string[0] == 'f'){
				forward();
			}

			//backward
			else if(received_string[0] == 'b'){
				backward();
			}

			//left
			else if(received_string[0] == 'l'){
				left();
			}

			//right
			else if(received_string[0] == 'r'){
				right();
			}

			//stop
			else if(received_string[0] == 's'){
				stop();
			}

			//get encoder
			else if(received_string[0] == 'e'){
				getEncoder();
			}

			//test
			else if(received_string[0] == 't'){
				test_forward();
			}
		}
}

void forward(){
	SpeedValue_left = 140;
	SpeedValue_right = 140;
	mMove(SpeedValue_left,SpeedValue_right);
	USART_puts(USART3, "left:");
	USART_putd(USART3, SpeedValue_left);
	USART_puts(USART3, " right:");
	USART_putd(USART3, SpeedValue_right);
	USART_puts(USART3, "\r\n");
}

void backward(){
	SpeedValue_left = 105;
	SpeedValue_right = 105;
	mMove(SpeedValue_left, SpeedValue_right);
	USART_puts(USART3, "left:");
	USART_putd(USART3, SpeedValue_left);
	USART_puts(USART3, " right:");
	USART_putd(USART3, SpeedValue_right);
	USART_puts(USART3, "\r\n");
}

void left(){
	SpeedValue_left = 105;
	SpeedValue_right = 140;
	mMove(SpeedValue_left, SpeedValue_right);
	USART_puts(USART3, "left");
	USART_putd(USART3, SpeedValue_left);
	USART_puts(USART3, " right");
	USART_putd(USART3, SpeedValue_right);
	USART_puts(USART3, "\r\n");
}

void right(){
	SpeedValue_left = 140;
	SpeedValue_right = 105;
	mMove(SpeedValue_left, SpeedValue_right);
	USART_puts(USART3, "left");
	USART_putd(USART3, SpeedValue_left);
	USART_puts(USART3, " right");
	USART_putd(USART3, SpeedValue_right);
	USART_puts(USART3, "\r\n");
}

void stop(){
	SpeedValue_left = 120;
	SpeedValue_right = 120;
	mMove(SpeedValue_left, SpeedValue_right);
	USART_puts(USART3, "left");
	USART_putd(USART3, SpeedValue_left);
	USART_puts(USART3, " right");
	USART_putd(USART3, SpeedValue_right);
	USART_puts(USART3, "\r\n");
}