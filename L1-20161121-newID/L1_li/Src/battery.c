#include "stm32f0xx_hal.h"
#include "battery.h"
#include "sim908.h"

extern ADC_HandleTypeDef hadc;
extern float g_main_battery_voltage;

void battery_update(void)
{
	uint32_t aResultDMA=0;
	uint32_t sum=0;
	float invertedvalue=0;
	
	HAL_ADC_PollForConversion(&hadc,10);
	HAL_ADCEx_Calibration_Start(&hadc);
	HAL_ADC_Start_DMA(&hadc,&aResultDMA,1);
	HAL_ADC_PollForConversion(&hadc,10);

//there 'i<1' meams we just collect once everytime 'ADCread()' is	called,------ 
//-------for minimal time to let thread 'thread_quickAdd(ADCread)' run regulaly.
	for (int i = 0; i<1; i++)
	{
		sum += aResultDMA;
//			usart_printf(&huart1,"voltage is %d\r\n",aResultDMA);
	}

	invertedvalue = (float)(sum*6600)/4096;
	float voltage = invertedvalue/1;
//voltage'unit is 'mV',which reasonable range is '3000mV' to '4200mV'.
	g_main_battery_voltage = voltage;

//this printf just wants to test wether thread 'thread_quickAdd(ADCread)' run regulaly or not.
//	usart_printf(&huart1,"voltage of battery is %5.2f mV.\r\n",g_main_battery_voltage);
}





