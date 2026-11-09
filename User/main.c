/*
adc-tmi定时采样->dma搬运数据->usart串口与电脑通讯
do--pin0
rx--pin10
tx--pin9
*/




#include "stm32f10x.h"
#include "OLED.h"
#include "Delay.h"
#include <stdio.h>


volatile uint16_t adc_buf[1] = {0};

static void dma_init_adc(void) 
{
   DMA_InitTypeDef dma;
   
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE);
   DMA_DeInit(DMA1_Channel1);//开启通道

	dma.DMA_PeripheralBaseAddr=(uint32_t)&ADC1->DR;//数据地址
    dma.DMA_MemoryBaseAddr= (uint32_t)adc_buf;//数据去除地址
    dma.DMA_DIR=DMA_DIR_PeripheralSRC;//数据流向adc->stm32外设到内存
    dma.DMA_BufferSize=1;//搬几个
    
	dma.DMA_PeripheralInc=DMA_PeripheralInc_Disable;//地址指向是否增减
	dma.DMA_MemoryInc=DMA_MemoryInc_Disable;
	
	dma.DMA_MemoryDataSize= DMA_MemoryDataSize_HalfWord;//数据大小16位二进制
	dma.DMA_PeripheralDataSize= DMA_PeripheralDataSize_HalfWord;
	
	dma.DMA_Mode=DMA_Mode_Circular;//循环搬运模式
	dma.DMA_Priority=DMA_Priority_Medium;
	dma.DMA_M2M=DMA_M2M_Disable;//非内存
	 DMA_Init(DMA1_Channel1, &dma);

    DMA_Cmd(DMA1_Channel1, ENABLE);//打开dma->dac1通道
	
}


static void adc_init(void)
{
    GPIO_InitTypeDef gpio;
    ADC_InitTypeDef  adc;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_ADC1, ENABLE);
    RCC_ADCCLKConfig(RCC_PCLK2_Div6);          

    gpio.GPIO_Pin  = GPIO_Pin_0;
    gpio.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &gpio);

    adc.ADC_Mode               = ADC_Mode_Independent;
    adc.ADC_ScanConvMode       = DISABLE;                        /* 多通道 */
    adc.ADC_ContinuousConvMode = DISABLE;                       /* 单次，靠触发 */
    adc.ADC_ExternalTrigConv   = ADC_ExternalTrigConv_T3_TRGO;  /* TIM3 触发 */
    adc.ADC_DataAlign          = ADC_DataAlign_Right;
    adc.ADC_NbrOfChannel       = 1;                             /* 两个通道 */
    ADC_Init(ADC1, &adc);
	
	 ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_55Cycles5);//设定优先级
	
     ADC_Cmd(ADC1, ENABLE);//使能

    ADC_ResetCalibration(ADC1);//校准
    while (ADC_GetResetCalibrationStatus(ADC1) == SET);
    ADC_StartCalibration(ADC1);
    while (ADC_GetCalibrationStatus(ADC1) == SET);

    ADC_DMACmd(ADC1, ENABLE);//开启adc-dma
    ADC_ExternalTrigConvCmd(ADC1, ENABLE);//开启adc-tmi
}
	static void tmi_init(void)
	{
	
	TIM_TimeBaseInitTypeDef tb;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    
    tb.TIM_Prescaler         = 72 - 1;
    tb.TIM_Period            = 1000 - 1;
    tb.TIM_ClockDivision     = TIM_CKD_DIV1;
    tb.TIM_CounterMode       = TIM_CounterMode_Up;
    tb.TIM_RepetitionCounter = 0;
	
	  TIM_TimeBaseInit(TIM3, &tb);

    TIM_SelectOutputTrigger(TIM3, TIM_TRGOSource_Update);//开启溢出事件输出到trgo-adc

    TIM_Cmd(TIM3, ENABLE);
	
	}

	static void led_init(void)
	{
	
	GPIO_InitTypeDef  gpio;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
		
		
    gpio.GPIO_Pin   = GPIO_Pin_7;
    gpio.GPIO_Mode  = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    GPIO_SetBits(GPIOA, GPIO_Pin_7);     
	
	}
	
	static void USART1_Init(uint32_t baud)
{
    GPIO_InitTypeDef  gpio;
    USART_InitTypeDef usart;

    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);

   
    gpio.GPIO_Pin   = GPIO_Pin_9;
    gpio.GPIO_Mode  = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

  
    gpio.GPIO_Pin  = GPIO_Pin_10;
    gpio.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &gpio);

   
    usart.USART_BaudRate            = baud;
    usart.USART_WordLength          = USART_WordLength_8b;
    usart.USART_StopBits            = USART_StopBits_1;
    usart.USART_Parity              = USART_Parity_No;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart.USART_Mode                = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(USART1, &usart);

  
    USART_Cmd(USART1, ENABLE);
}


int fputc(int ch, FILE *f)
{
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
    USART_SendData(USART1, (uint8_t)ch);
    return ch;
}
	
	int main(void)
{
	unsigned int led=0;
	uint16_t l;
	uint8_t ch;
	 
	OLED_Init();
	USART1_Init(115200);
	led_init();
	dma_init_adc(); 
	adc_init();
	tmi_init();
	
	while(1)
	{
	l=adc_buf[0];

	
	printf("L=%u\r\n",l);

	 if (led) { GPIO_SetBits(GPIOA, GPIO_Pin_1);   led = 0; }
     else{ GPIO_ResetBits(GPIOA, GPIO_Pin_1); led = 1; }
	
	 	 	
	if (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) != RESET)
    {
     ch = (uint8_t)USART_ReceiveData(USART1);  
    }
	 
	 OLED_ShowChar(1, 1,ch);

	/* ② 限速：每 200ms 发一次（5Hz），否则会把串口刷爆 */
	Delay_ms(200);
	}
}

