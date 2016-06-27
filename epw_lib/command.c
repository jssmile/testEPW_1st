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

void receive_task()
{

    if(Receive_String_Ready) {

        //forward
        if(received_string[0] == 'f') {
            test_PID_forward();
        }

        //backward
        else if(received_string[0] == 'b') {
            test_PID_backward();
        }

        //left
        else if(received_string[0] == 'l') {
            test_PID_left();
        }

        //right
        else if(received_string[0] == 'r') {
            test_PID_right();
        }

        //stop
        else if(received_string[0] == 's') {
            motor_Stop();
        }

        //Linear Acturator
        else if(received_string[0] == 'n') {
            USART_puts(USART3, "Actu_A_up");
            set_linearActuator_A_cmd(LINEAR_ACTU_CW);
            USART_puts(USART3, "\r\n");
        }

        else if(received_string[0] == 'd') {
            USART_puts(USART3, "Actu_A_down");
            USART_puts(USART3, "\r\n");
            set_linearActuator_A_cmd(LINEAR_ACTU_CCW);
        } else if(received_string[0] == 'a') {
            USART_puts(USART3, "Actu_A_stop");
            USART_puts(USART3, "\r\n");
            set_linearActuator_A_cmd(LINEAR_ACTU_STOP);
        }

        //Due to the
        else if(received_string[0] == 'u'){
            	USART_puts(USART3, "Actu_B_up");
            	set_linearActuator_B_cmd(LINEAR_ACTU_CW);
            	USART_puts(USART3, "\r\n");
        	}


        else if(received_string[0] == 'k'){
            	USART_puts(USART3, "Actu_B_down");
            	USART_puts(USART3, "\r\n");
            	set_linearActuator_B_cmd(LINEAR_ACTU_CCW);
        	}

        else if(received_string[0] == 'w'){
            	USART_puts(USART3, "Actu_B_stop");
            	USART_puts(USART3, "\r\n");
            	set_linearActuator_B_cmd(LINEAR_ACTU_STOP);
        	}

    }
}