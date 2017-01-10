#include "gd32f1x0.h"
#include "drv_adc.h"
#include "util.h"
#include "hardware.h"
#include "stdio.h"

uint16_t adcarray[10];

struct ADC_SETTINGS
{
	uint8_t id;
	uint8_t pin;
	GPIO_TypeDef* port;
	uint8_t channel;
	float adc_readout;
	float adc_value;

};

struct ADC_SETTINGS adc_settings[] = 
{
#ifdef ADC_PA0
	{ADC_PA0, GPIO_PIN_0, GPIOA,ADC_CHANNEL_0, ADC_PA0_READOUT, ADC_PA0_VALUE},
#endif
#ifdef ADC_PA1
	{ADC_PA1, GPIO_PIN_1, GPIOA,ADC_CHANNEL_1, ADC_PA1_READOUT, ADC_PA1_VALUE},
#endif
#ifdef ADC_PA2
	{ADC_PA2, GPIO_PIN_2, GPIOA,ADC_CHANNEL_2, ADC_PA2_READOUT, ADC_PA2_VALUE},
#endif
#ifdef ADC_PA3
	{ADC_PA3, GPIO_PIN_3, GPIOA,ADC_CHANNEL_3, ADC_PA3_READOUT, ADC_PA3_VALUE},
#endif
#ifdef ADC_PA4
	{ADC_PA4, GPIO_PIN_4, GPIOA,ADC_CHANNEL_4, ADC_PA4_READOUT, ADC_PA4_VALUE},
#endif
#ifdef ADC_PA5
	{ADC_PA5, GPIO_PIN_5, GPIOA,ADC_CHANNEL_5, ADC_PA5_READOUT, ADC_PA5_VALUE},
#endif
#ifdef ADC_PA6
	{ADC_PA6, GPIO_PIN_6, GPIOA,ADC_CHANNEL_6, ADC_PA6_READOUT, ADC_PA6_VALUE},
#endif
#ifdef ADC_PA7
	{ADC_PA7, GPIO_PIN_7, GPIOA,ADC_CHANNEL_7, ADC_PA7_READOUT, ADC_PA7_VALUE},
#endif
#ifdef ADC_PB0
	{ADC_PB0, GPIO_PIN_0, GPIOB,ADC_CHANNEL_8, ADC_PB0_READOUT, ADC_PB0_VALUE},
#endif
#ifdef ADC_PB1
	{ADC_PB1, GPIO_PIN_1, GPIOB,ADC_CHANNEL_9, ADC_PB1_READOUT, ADC_PB1_VALUE},
#endif
};

void adc_init(void)
{

	int size = sizeof(adc_settings) / sizeof(adc_settings[0]);

	for (int i = 0; i < size; ++i)
	{
		GPIO_InitPara    GPIO_InitStructure;

		GPIO_InitStructure.GPIO_Pin = adc_settings[i].pin ;
		GPIO_InitStructure.GPIO_Mode = GPIO_MODE_AN;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PUPD_NOPULL ;
		GPIO_Init(adc_settings[i].port, &GPIO_InitStructure);
	}

	DMA_InitPara DMA_InitStructure;
	ADC_InitPara ADC_InitStructure;

	RCC_APB2PeriphClock_Enable(RCC_APB2PERIPH_ADC1, ENABLE);

	RCC_ADCCLKConfig(RCC_ADCCLK_APB2_DIV6);

	RCC_AHBPeriphClock_Enable(RCC_AHBPERIPH_DMA1, ENABLE);

	DMA_InitStructure.DMA_PeripheralBaseAddr = 0x4001244C;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t) adcarray;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PERIPHERALSRC;
	DMA_InitStructure.DMA_BufferSize = size;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PERIPHERALINC_DISABLE;
	DMA_InitStructure.DMA_MemoryInc = DMA_MEMORYINC_ENABLE;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PERIPHERALDATASIZE_HALFWORD;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MEMORYDATASIZE_HALFWORD;
	DMA_InitStructure.DMA_Mode = DMA_MODE_CIRCULAR;
	DMA_InitStructure.DMA_Priority = DMA_PRIORITY_HIGH;
	DMA_InitStructure.DMA_MTOM = DMA_MEMTOMEM_DISABLE;

	DMA_Init(DMA1_CHANNEL1, &DMA_InitStructure);

	DMA_Enable(DMA1_CHANNEL1, ENABLE);

	//  ADC_DeInit(&ADC_InitStructure);

	ADC_InitStructure.ADC_Mode_Scan = ENABLE;
	ADC_InitStructure.ADC_Mode_Continuous = ENABLE;
	ADC_InitStructure.ADC_Trig_External = ADC_EXTERNAL_TRIGGER_MODE_NONE;
	ADC_InitStructure.ADC_Data_Align = ADC_DATAALIGN_RIGHT;
	ADC_InitStructure.ADC_Channel_Number = size; 
	ADC_Init(&ADC_InitStructure);

	for (int i = 0; i < size; ++i)
	{
		ADC_RegularChannel_Config(adc_settings[i].channel, i+1, ADC_SAMPLETIME_239POINT5);
	}

	ADC_DMA_Enable(ENABLE);

	ADC_Enable(ENABLE);

	ADC_Calibration();

	ADC_SoftwareStartConv_Enable(ENABLE);
}

float adc_read(int id)
{
	for (int i = 0; i < sizeof(adc_settings) / sizeof(adc_settings[0]); ++i)
	{
		if (id == adc_settings[i].id)
		{
			return (float) adcarray[i] * ((float)(adc_settings[i].adc_value)/(float) (adc_settings[i].adc_readout)) ;
		}
	}
	return 0.f;
}
