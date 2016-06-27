#include "control.h"
#include "PID.h"
#include "clib.h"
#include "record.h"


#define Vset		100
#define Period		50 //ms
#define cmd_times	50 //times
xTimerHandle ctrlTimer;
xTimerHandle PIDTimer;

extern Encoder_t ENCODER_L;
extern Encoder_t ENCODER_R;

extern int SpeedValue_left;
extern int SpeedValue_right;
uint32_t cmd_cnt = 0;
static float set_encoder_count = 100.0f;

/*create the pid struct for use*/
pid_struct PID_Motor_L;
pid_struct PID_Motor_R;

/*pid alg premeter.*/
float Kp_left,Ki_left,Kd_left;
float Kp_right, Ki_right, Kd_right;

//PWM vlaue tuned by PID algorithm
int pwm_value_left_pid = 0;
int pwm_value_right_pid = 0;
int pwm_value_left, pwm_value_right, pwm_value_left_err, pwm_value_right_err, pwm_value_left_mcu, pwm_value_right_mcu;

//State struct
State_t EPW_State = EPW_IDLE;
/*
typedef enum {
    EPW_IDLE,
    EPW_STOP,
    EPW_FORWARD,
    EPW_BACKWARD,
    EPW_LEFT,
    EPW_RIGHT,
    EPW_UNREADY,
    EPW_BUSY,
    EPW_ERROR
} State_t;
*/

void Motor_init()
{
    /*Initialization the right motor of pid paremeter.*/
    /*Original value p = 5.0f, i = 0.5f, d = 5.0f*/
    Kp_left=2.5f;
    Ki_left=0.5f;
    Kd_left=2.5f;
    Kp_right=2.5f;
    Ki_right=0.5f;
    Kd_right=2.5f;

    Init_pid(&PID_Motor_L, Kp_left, Ki_left, Kd_left);
    Init_pid(&PID_Motor_R, Kp_right, Ki_right, Kd_right);
}

/*calculate Position PID of two motor*/
void test_PID_forward()
{
    Motor_init();
    cmd_cnt = 50;
    EPW_State = EPW_FORWARD;

    if(xTimerIsTimerActive(PIDTimer) != pdTRUE || (PIDTimer == NULL)){
    PIDTimer = xTimerCreate("pid forward test", (Period), pdTRUE, (void *) 1, PID_forward);
    xTimerStart(PIDTimer, 0);
    USART_putd(USART3, cmd_cnt);
    }
    else{
        xTimerReset(PIDTimer, 0);
    }
}

//PID_forward
void PID_forward()
{
    int cnt[2];
    cnt[0] = getEncoderLeft();
    cnt[1] = getEncoderRight();

    if(cmd_cnt && EPW_State != EPW_STOP) {
        EPW_State = EPW_FORWARD;
        if (cnt[0] || cnt[1]) --cmd_cnt;
        pwm_value_left_pid = round(pid_cal(&PID_Motor_L , set_encoder_count , cnt[0]));
        pwm_value_right_pid = round(pid_cal(&PID_Motor_R , set_encoder_count , cnt[1]));

        //Set the safe range of the epw, or the epw may go to the hell
        if(pwm_value_left_pid > 145)	pwm_value_left_pid = 145;
        if(pwm_value_left_pid < 130)	pwm_value_left_pid = 130;

        if(pwm_value_right_pid > 145)	pwm_value_right_pid =145;
        if(pwm_value_right_pid < 130)	pwm_value_right_pid = 130;

        //use the tuned value for pwm.
        SpeedValue_left = pwm_value_left_pid;
        SpeedValue_right = pwm_value_right_pid;
        mMove (SpeedValue_left, SpeedValue_right);
    }

    else {
        motor_Stop();
        if(!(cnt[0] || cnt[1])) {
            xTimerDelete(PIDTimer, 0);
            USART_puts(USART3, "complete f");
            EPW_State = EPW_IDLE;
            //endofRecord();
        }
    }

    //record the data
    //recControlData(pwm_value_left_pid, pwm_value_right_pid, cnt[0], cnt[1]);

}

/*calculate Position PID of two motor*/
void test_PID_backward()
{
    Motor_init();
    cmd_cnt = 30;
    if(xTimerIsTimerActive(PIDTimer) != pdTRUE || (PIDTimer == NULL)){
    PIDTimer = xTimerCreate("pid forward test", (Period), pdTRUE, (void *) 1, PID_backward);
    xTimerStart(PIDTimer, 0);
    USART_putd(USART3, cmd_cnt);
    }
    else{
        xTimerReset(PIDTimer, 0);
    }
}

//PID_backward
void PID_backward()
{
    int cnt[2];
    cnt[0] = getEncoderLeft();
    cnt[1] = getEncoderRight();

    if(cmd_cnt && EPW_State != EPW_STOP) {
        EPW_State = EPW_BACKWARD;
        if (cnt[0] || cnt[1]) --cmd_cnt;
        pwm_value_left_pid = round(pid_cal(&PID_Motor_L , set_encoder_count , cnt[0]));
        pwm_value_right_pid = round(pid_cal(&PID_Motor_R , set_encoder_count , cnt[1]));

        //Set the safe range of the epw, or the epw may go to the hell
        if(pwm_value_left_pid > 107)	pwm_value_left_pid = 107;
        if(pwm_value_left_pid < 102)	pwm_value_left_pid = 102;

        if(pwm_value_right_pid > 107)	pwm_value_right_pid =107;
        if(pwm_value_right_pid < 102)	pwm_value_right_pid = 102;

        //use the tuned value for pwm.
        SpeedValue_left = pwm_value_left_pid;
        SpeedValue_right = pwm_value_right_pid;
        mMove (SpeedValue_left, SpeedValue_right);
    }

    else {
        mStop();
        if(!(cnt[0] || cnt[1])) {
            xTimerDelete(PIDTimer, 0);
            USART_puts(USART3, "complete b");
            EPW_State = EPW_IDLE;
            //endofRecord();
        }
    }

    //record the data
    //recControlData(pwm_value_left_pid, pwm_value_right_pid, cnt[0], cnt[1]);
}

/*calculate Position PID of two motor*/
void test_PID_left()
{
    Motor_init();
    cmd_cnt = 20;
    if(xTimerIsTimerActive(PIDTimer) != pdTRUE || (PIDTimer == NULL)){
    PIDTimer = xTimerCreate("pid forward test", (Period), pdTRUE, (void *) 1, PID_left);
    xTimerStart(PIDTimer, 0);
    USART_putd(USART3, cmd_cnt);
    }
    else{
        xTimerReset(PIDTimer, 0);
    }
}

//PID_left
void PID_left()
{
    int cnt[2];
    cnt[0] = getEncoderLeft();
    cnt[1] = getEncoderRight();

    if(cmd_cnt && EPW_State != EPW_STOP) {
        EPW_State = EPW_LEFT;
        if (cnt[0] || cnt[1]) --cmd_cnt;
        pwm_value_left_pid = round(pid_cal(&PID_Motor_L , set_encoder_count , cnt[0]));
        pwm_value_right_pid = round(pid_cal(&PID_Motor_R , set_encoder_count , cnt[1]));

        //Set the safe range of the epw, or the epw may go to the hell
        if(pwm_value_left_pid > 105)	pwm_value_left_pid = 105;
        if(pwm_value_left_pid < 100)	pwm_value_left_pid = 100;

        if(pwm_value_right_pid > 140)	pwm_value_right_pid =140;
        if(pwm_value_right_pid < 130)	pwm_value_right_pid = 130;

        //use the tuned value for pwm.
        SpeedValue_left = pwm_value_left_pid;
        SpeedValue_right = pwm_value_right_pid;
        mMove (SpeedValue_left, SpeedValue_right);
    }

    else {
        mStop();
        if(!(cnt[0] || cnt[1])) {
            xTimerDelete(PIDTimer, 0);
            USART_puts(USART3, "complete l");
            EPW_State = EPW_IDLE;
            //endofRecord();
        }
    }

    //record the data
    //recControlData(pwm_value_left_pid, pwm_value_right_pid, cnt[0], cnt[1]);
}

/*calculate Position PID of two motor*/
void test_PID_right()
{
    Motor_init();
    cmd_cnt = 20;
    if(xTimerIsTimerActive(PIDTimer) != pdTRUE || (PIDTimer == NULL)){
    PIDTimer = xTimerCreate("pid forward test", (Period), pdTRUE, (void *) 1, PID_right);
    xTimerStart(PIDTimer, 0);
    USART_putd(USART3, cmd_cnt);
    }
    else{
        xTimerReset(PIDTimer, 0);
    }
}

//PID_right
void PID_right()
{
    int cnt[2];
    cnt[0] = getEncoderLeft();
    cnt[1] = getEncoderRight();

    if(cmd_cnt && EPW_State != EPW_STOP) {
        EPW_State = EPW_RIGHT;
        if (cnt[0] || cnt[1]) --cmd_cnt;
        pwm_value_left_pid = round(pid_cal(&PID_Motor_L , set_encoder_count , cnt[0]));
        pwm_value_right_pid = round(pid_cal(&PID_Motor_R , set_encoder_count , cnt[1]));

        //Set the safe range of the epw, or the epw may go to the hell
        if(pwm_value_left_pid > 140)	pwm_value_left_pid = 140;
        if(pwm_value_left_pid < 130)	pwm_value_left_pid = 130;

        if(pwm_value_right_pid > 105)	pwm_value_right_pid =105;
        if(pwm_value_right_pid < 100)	pwm_value_right_pid = 100;

        //use the tuned value for pwm.
        SpeedValue_left = pwm_value_left_pid;
        SpeedValue_right = pwm_value_right_pid;
        mMove (SpeedValue_left, SpeedValue_right);
    }

    else {
        mStop();
        if(!(cnt[0] || cnt[1])) {
            xTimerDelete(PIDTimer, 0);
            USART_puts(USART3, "complete r");
            EPW_State = EPW_IDLE;
            //endofRecord();
        }
    }

    //record the data
    //recControlData(pwm_value_left_pid, pwm_value_right_pid, cnt[0], cnt[1]);
}

void motor_Stop()
{
    SpeedValue_left = 120;
    SpeedValue_right = 120;
    EPW_State = EPW_STOP;
    mMove (SpeedValue_left, SpeedValue_right);
}