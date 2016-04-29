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

extern int SpeedValue_left;
extern int SpeedValue_right;
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
			else if(received_string[0] == 'p'){
				forward();
			}

			//PID
			else if(received_string[0] == 'f'){
				test_PID_forward();
			}

			//backward
			else if(received_string[0] == 'b'){
				test_PID_backward();
			}

			//left
			else if(received_string[0] == 'l'){
				test_PID_left();
			}

			//right
			else if(received_string[0] == 'r'){
				test_PID_right();
			}

			//stop
			else if(received_string[0] == 's'){
				stop();
			}

			//Linear Acturator
			else if(received_string[0] == 'a'){
			    	USART_puts(USART3, "Actu_A_up");
			    	set_linearActuator_A_cmd(LINEAR_ACTU_CW);
			    	USART_puts(USART3, "\r\n");
			    }

			else if(received_string[0] == 'n'){
			    	USART_puts(USART3, "Actu_A_down");
			    	//set_linearActuator_A_cmd(LINEAR_ACTU_CCW);
			    	USART_puts(USART3, "\r\n");
			    	set_linearActuator_A_cmd(LINEAR_ACTU_CCW);
			    }
			else if(received_string[0] == 'd'){
			    	USART_puts(USART3, "Actu_A_stop");
			    	//set_linearActuator_A_cmd(LINEAR_ACTU_CCW);
			    	USART_puts(USART3, "\r\n");
			    	set_linearActuator_A_cmd(LINEAR_ACTU_STOP);
			    }

			else if(received_string[0] == 'u'){
			    	USART_puts(USART3, "Actu_B_up");
			    	set_linearActuator_B_cmd(LINEAR_ACTU_CW);
			    	USART_puts(USART3, "\r\n");
				}

				
			else if(received_string[0] == 'k'){
			    	USART_puts(USART3, "Actu_B_down");
			    	//set_linearActuator_B_cmd(LINEAR_ACTU_CCW);
			    	USART_puts(USART3, "\r\n");
			    	set_linearActuator_B_cmd(LINEAR_ACTU_CCW);
				}

			else if(received_string[0] == 'w'){
			    	USART_puts(USART3, "Actu_B_stop");
			    	//set_linearActuator_B_cmd(LINEAR_ACTU_CCW);
			    	USART_puts(USART3, "\r\n");
			    	set_linearActuator_B_cmd(LINEAR_ACTU_STOP);
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