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
	//int i, j;
	//while(1){
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
				SpeedValue_left = 140;
				SpeedValue_right = 140;
				mMove(SpeedValue_left,SpeedValue_right);
				USART_puts(USART3, "left:");
				USART_putd(USART3, SpeedValue_left);
				USART_puts(USART3, " right:");
				USART_putd(USART3, SpeedValue_right);
				USART_puts(USART3, "\r\n");
			}

			//backward
			else if(received_string[0] == 'b'){
				SpeedValue_left = 105;
				SpeedValue_right = 105;
				mMove(SpeedValue_left, SpeedValue_right);
				USART_puts(USART3, "left:");
				USART_putd(USART3, SpeedValue_left);
				USART_puts(USART3, " right:");
				USART_putd(USART3, SpeedValue_right);
				USART_puts(USART3, "\r\n");
			}

			//left
			else if(received_string[0] == 'l'){
				SpeedValue_left = 105;
				SpeedValue_right = 140;
				mMove(SpeedValue_left, SpeedValue_right);
				USART_puts(USART3, "left");
				USART_putd(USART3, SpeedValue_left);
				USART_puts(USART3, " right");
				USART_putd(USART3, SpeedValue_right);
				USART_puts(USART3, "\r\n");
			}

			//right
			else if(received_string[0] == 'r'){
				SpeedValue_left = 140;
				SpeedValue_right = 105;
				mMove(SpeedValue_left, SpeedValue_right);
				USART_puts(USART3, "left");
				USART_putd(USART3, SpeedValue_left);
				USART_puts(USART3, " right");
				USART_putd(USART3, SpeedValue_right);
				USART_puts(USART3, "\r\n");
			}

			//stop
			else if(received_string[0] == 's'){
				SpeedValue_left = 120;
				SpeedValue_right = 120;
				mMove(SpeedValue_left, SpeedValue_right);
				USART_puts(USART3, "left");
				USART_putd(USART3, SpeedValue_left);
				USART_puts(USART3, " right");
				USART_putd(USART3, SpeedValue_right);
				USART_puts(USART3, "\r\n");
			}

			else if(received_string[0] == 'e'){
				getEncoder();
			}
			
			//}
			/*Receive_String_Ready = 0;
			for( i = 0 ; i< MAX_STRLEN ; i++){
				received_string[i]= 0;*/
		}
	//}
}
/*
void parseString(unsigned char *received_string){
	char *word = strtok(received_string,' ');
	receive_cmd_type->Identifier[0] = word;
	if (word = 'c'){
			word = strtok(NULL,' ');
			//receive_cmd_type->DIR_cmd = word;
			//word = strtok(NULL,' ');
			//receive_cmd_type->pwm_value = word;
	}
}*/
