#include "ultrasonic.h"

uint32_t distance = 0;
int sense = 0;

void delay(uint32_t us)
{
    us *= 6;
    while(us--) {
        __NOP();
    }
}

void init_ultrasonic()
{

    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    /*Trigger Pin*/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /*Echo Pin*/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}

uint32_t Read_Distance()
{
    init_ultrasonic();

    //let trigger pin sends signal
    GPIO_ResetBits(GPIOA, GPIO_Pin_7);
    delay(2);
    GPIO_SetBits(GPIOA, GPIO_Pin_7);
    delay(10);
    GPIO_ResetBits(GPIOA, GPIO_Pin_7);


    // Give some time for response
    uint32_t timeout = 1000000;
    while(!GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_6)) {
        if(timeout -- == 0x00) {
            return -1;
        }
    }

    //if echo pin receive signal, start counting time.
    uint32_t wait_time = 0;
    while(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_6) == 1) {
        wait_time ++;
        delay(1);
    }
    return wait_time/58;
}