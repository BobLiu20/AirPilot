/*电池电量检测
	PA5
*/
#include "AP_Battery.h"

//#define ADC1_DR_Address    ((u32)0x4001244C) //即ADC1_DR寄存器的地址

u16 Battery_Val;				//这里保存着单节电池的真实电压值，真实电压值=Battery_Val/100;  也就是这个100倍
__IO u16 ADC_ConvertedValue;
u8 Battery_LowPower=1;			//电量不足指示，如果为1表示充足，为0表示不足了

static void ADC1_GPIO_Config()
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* Enable ADC3, DMA2 and GPIO clocks ****************************************/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2 | RCC_AHB1Periph_GPIOA, ENABLE);//
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

static void ADC1_Mode_Config(void)
{
	DMA_InitTypeDef DMA_InitStructure;
	ADC_InitTypeDef       ADC_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	
// 	/* DMA2 Stream0 channel0 configuration **************************************/
// 	DMA_InitStructure.DMA_Channel = DMA_Channel_0; 
// 	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)ADC1->DR;
// 	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&ADC_ConvertedValue;
// 	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
// 	DMA_InitStructure.DMA_BufferSize = 1;
// 	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
// 	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
// 	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
// 	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
// 	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
// 	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
// 	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
// 	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
// 	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
// 	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
// 	DMA_Init(DMA2_Stream0, &DMA_InitStructure);
// 	DMA_Cmd(DMA2_Stream0, ENABLE);
	
	/* ADC Common Init **********************************************************/
	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
	ADC_CommonInit(&ADC_CommonInitStructure);
	
	/* ADC1 Init ****************************************************************/
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfConversion = 1;
	ADC_Init(ADC1, &ADC_InitStructure);

	/* ADC1 regular channel11 configuration */ 
	ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 1, ADC_SampleTime_3Cycles);
	
	/* Enable ADC1 DMA */
	//ADC_DMACmd(ADC1, ENABLE);
	
	/* Enable ADC1 */
	ADC_Cmd(ADC1, ENABLE);
	
// 	/* Enable ADC1 reset calibaration register */   
// 	ADC_ResetCalibration(ADC1);
// 	/* Check the end of ADC1 reset calibration register */
// 	while(ADC_GetResetCalibrationStatus(ADC1));
// 	
// 	/* Start ADC1 calibaration */
// 	ADC_StartCalibration(ADC1);
// 	/* Check the end of ADC1 calibration */
// 	while(ADC_GetCalibrationStatus(ADC1));
	
	/* Start ADC1 Software Conversion */ 
	ADC_SoftwareStartConv(ADC1);
}

// void Battery_Check(void)
// {
// 	GPIO_InitTypeDef GPIO_InitStructure;
// 	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
//     GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
// 	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
//     GPIO_Init(GPIOC, &GPIO_InitStructure);
// }

/*就是用个AD采集电压。。。。*/
void Battery_Init(void)
{
	ADC1_GPIO_Config();
	ADC1_Mode_Config();
//	Battery_Check();
}

/*获取温度值*/
void Battery_Read(void)
{
	float val;
	u8 i;
	static uint16_t vbatRawArray[8];
	static uint8_t ind = 0;
	uint16_t vbatRaw = 0;
//    if (!(++vbatTimer % VBATFREQ)) 
//	{
        vbatRawArray[(ind++) % 8] = ADC1->DR;//滑动平均滤波
        for (i = 0; i < 8; i++)
            vbatRaw += vbatRawArray[i];
//  }


	val = (float)(vbatRaw / 8)/4096.0f * 3.3f;	 //
	val = val/(3.14f/4.11f);
	Battery_Val = val * 100;

// 	if(GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_1))  //高电平为电量足
// 		Battery_LowPower=1;
// 	else
// 		Battery_LowPower=0;
}


