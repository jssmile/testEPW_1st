#include "control.h"
#include "PID.h"
#include "clib.h"

#define Vset		100
#define Period		100 //ms
#define cmd_times	50 //times
xTimerHandle ctrlTimer;
xTimerHandle PIDTimer;

extern Encoder_t ENCODER_L;
extern Encoder_t ENCODER_R;

extern uint32_t SpeedValue_left;
extern uint32_t SpeedValue_right;
uint32_t cmd_cnt = 0;
static float set_encoder_count = 100.0f;

/*create the pid struct for use*/
pid_struct PID_Motor_L;
pid_struct PID_Motor_R;

//PWM vlaue tuned by PID algorithm
int pwm_value_left_pid = 0;
int pwm_value_right_pid = 0;

/*calculate Position PID of two motor*/
void test_PID(){
	cmd_cnt = 50;
	PIDTimer = xTimerCreate("pid forward test", (Period), pdTRUE, (void *) 1, PID_fprward);
	xTimerStart(PIDTimer, 0);
}

void PID_fprward(){
	int cnt[2];
	cnt[0] = getEncoderLeft();
	cnt[1] = getEncoderRight();

	if(cmd_cnt){
		if (cnt[0] || cnt[1]) --cmd_cnt;
		pwm_value_left_pid = round(pid_cal(&PID_Motor_L , set_encoder_count , cnt[0]));
		pwm_value_right_pid = round(pid_cal(&PID_Motor_R , set_encoder_count , cnt[1]));

		//Set the safe range of the epw,or the epw may go to the hell
		if(pwm_value_left_pid > 140)
			pwm_value_left_pid = 140;
		if(pwm_value_left_pid < 110)
			pwm_value_left_pid = 110;

		if(pwm_value_right_pid > 140)
			pwm_value_right_pid =140;
		if(pwm_value_right_pid < 110)
			pwm_value_right_pid = 110;

		//use the tuned value for pwm.
		SpeedValue_left = pwm_value_left_pid;
		SpeedValue_right = pwm_value_right_pid;
		mMove (SpeedValue_left, SpeedValue_right);
	}

	else{
		mStop();
		if(!(cnt[0] || cnt[1])){
			xTimerDelete(PIDTimer, 0);
		}
	}

	//record the data
	USART_puts(USART3, "fl:");
	USART_putd(USART3, SpeedValue_left);
	USART_puts(USART3, " fr:");
	USART_putd(USART3, SpeedValue_right);
	USART_puts(USART3, "\r\nel:");
	USART_putd(USART3, cnt[0]);
	USART_puts(USART3, " rl:");
	USART_putd(USART3, cnt[1]);
	USART_puts(USART3, "\r\n");

	recControlData(SpeedValue_left, SpeedValue_right, cnt[0], cnt[1]);

}

uint32_t mvl, mvr;

void test_forward(){
	cmd_cnt = 50;

	if(mvl && mvr){
		SpeedValue_left = mvl;
		SpeedValue_right = mvr;
	}

		ctrlTimer = xTimerCreate("forward control", (Period), pdTRUE, (void *) 1, move_forward);
		xTimerStart(ctrlTimer, 0);
}

void move_forward(){
	int cnt[2];
	cnt[0] = getEncoderLeft();
	cnt[1] = getEncoderRight();

	if(cmd_cnt){
		/* start counting only if encoder get data(motor moving)
		 * moving period = cmd_cnt * Period */
		if(cnt[0] || cnt[1]) --cmd_cnt;
		SpeedValue_left += (cnt[0] < 90)? 1: (cnt[0] > 100)? -1: 0;
		SpeedValue_right += (cnt[1] < 90)? 1: (cnt[1] > 100)? -1: 0;

		mMove(SpeedValue_left, SpeedValue_right);
		/* record the value for next forward command */
		mvl = SpeedValue_left;
		mvr = SpeedValue_right;
	}
	else{
		mStop();
		if(!(cnt[0] || cnt[1])){
			xTimerDelete(ctrlTimer, 0);
		}
	}

	USART_puts(USART3, "fl:");
	USART_putd(USART3, SpeedValue_left);
	USART_puts(USART3, " fr:");
	USART_putd(USART3, SpeedValue_right);
	USART_puts(USART3, "\r\nel:");
	USART_putd(USART3, cnt[0]);
	USART_puts(USART3, " rl:");
	USART_putd(USART3, cnt[1]);
	USART_puts(USART3, "\r\n");

	recControlData(SpeedValue_left, SpeedValue_right, cnt[0], cnt[1]);
}


                    /*calculate Position PID of two motor*/    
//pwm_value_left_pid = math_round(PID_Pos_Calc(&PID_Motor_L , set_encoder_count , encoder_left_counter));
//pwm_value_right_pid = math_round(PID_Pos_Calc(&PID_Motor_R , set_encoder_count , encoder_right_counter));
