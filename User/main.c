#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"

int main(void)
{
	GPIO_InitTypeDef gpio;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	
	gpio.GPIO_Mode=GPIO_Pin_1;
	gpio.GPIO_Pin=GPIO_Mode_Out_PP;
	gpio.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&gpio);
	
	while(1)
	{
	GPIO_ResetBits(GPIOA,GPIO_Pin_1);
	Delay_ms(500);
	GPIO_SetBits(GPIOA,GPIO_Pin_1);
	Delay_ms(500);
	
	}
	
	
	return 0;
	
}
