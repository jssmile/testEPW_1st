#include "control.h"
#include "PID.h"
#include "clib.h"


#define Vset		100
#define Period		75 //ms
#define cmd_times	50 //times
xTimerHandle ctrlTimer;
xTimerHandle PIDTimer;

extern Encoder_t ENCODER_L;
extern Encoder_t ENCODER_R;

extern uint32_t SpeedValue_left;
extern uint32_t SpeedValue_right;
uint32_t cmd_cnt = 0;
static float set_encoder_count = 50.0f;

/*create the pid struct for use*/
pid_struct PID_Motor_L;
pid_struct PID_Motor_R;

/*pid alg premeter.*/
float Kp_left,Ki_left,Kd_left;
float Kp_right, Ki_right, Kd_right;

//PWM vlaue tuned by PID algorithm
int pwm_value_left_pid = 0;
int pwm_value_right_pid = 0;

void Motor_init(){
	/*Initialization the right motor of pid paremeter.*/
    /*Original value p = 5.0f, i = 0.5f, d = 5.0f*/
	Kp_left=5.0f; Ki_left=0.5f; Kd_left=5.0f;
	Kp_right=5.0f; Ki_right=0.5f; Kd_right=5.0f;

	Init_pid(&PID_Motor_L, Kp_left, Ki_left, Kd_left);
	Init_pid(&PID_Motor_R, Kp_right, Ki_right, Kd_right);
}

uint16_t OverFlowTimes=0;
void tic(void)                            //程序开始计时
{
SysTick->CTRL |= (1<<2);   //时钟选择，HCLK
SysTick->CTRL |= (1<<1);   //中断使能
SysTick->VAL=0X00;            //当前数值寄存器清零，并清除溢出标志位
SysTick->LOAD=0XFFFFFF;     //计数器赋初值
SysTick->CTRL |= (1<<0);     //开启计数器
}

void toc(void)                            //结束计时
{
float ElaspTime;
uint32_t ClkNum;
SysTick->CTRL &= ~(1<<0); //关闭计数器
ClkNum=SysTick->VAL; //读取计数器的值
ElaspTime=(OverFlowTimes*((float)0xffffff/SystemCoreClock)+(float)(0xffffff-ClkNum)/SystemCoreClock); //计算时间
OverFlowTimes=0;
USART_puts(USART3,"Escaple time is");
USART_putd(USART3,ElaspTime);
USART_puts(USART3,"\r\n");
SysTick->CTRL &= ~(0<<1);
}
/*溢出的次数*/
void SysTick_Handler1(void)
{
OverFlowTimes++;
}

/*calculate Position PID of two motor*/
void test_PID(){
	Motor_init();
	cmd_cnt = 50;
	PIDTimer = xTimerCreate("pid forward test", (Period), pdTRUE, (void *) 1, PID_forward);
	xTimerStart(PIDTimer, 0);
	USART_putd(USART3, cmd_cnt);
}

void PID_forward(){
	int cnt[2];
	cnt[0] = getEncoderLeft();
	cnt[1] = getEncoderRight();

	if(cmd_cnt){
		if (cnt[0] || cnt[1]) --cmd_cnt;
		pwm_value_left_pid = round(pid_cal(&PID_Motor_L , set_encoder_count , cnt[0]));
		pwm_value_right_pid = round(pid_cal(&PID_Motor_R , set_encoder_count , cnt[1]));
		
		//To see the original pid left and right value which are not tuned in saferange
		USART_puts(USART3, "pid_left:");
		USART_putd(USART3, pwm_value_left_pid);
		USART_puts(USART3, "pid_right:");
		USART_putd(USART3, pwm_value_right_pid);


		//Set the safe range of the epw, or the epw may go to the hell
		if(pwm_value_left_pid > 145)
			pwm_value_left_pid = 145;
		if(pwm_value_left_pid < 110)
			pwm_value_left_pid = 110;

		if(pwm_value_right_pid > 145)
			pwm_value_right_pid =145;
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
			USART_puts(USART3, "Delete complete!!!");
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
	USART_puts(USART3, " cmd_cnt : ");
	USART_putd(USART3, cmd_cnt);

	USART_puts(USART3, "\r\n");

	recControlData(pwm_value_left_pid, pwm_value_right_pid, cnt[0], cnt[1]);

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