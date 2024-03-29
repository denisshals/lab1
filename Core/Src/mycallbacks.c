#include <mytypes.h>
#include "stm32f4xx_hal.h"
#include <stdint.h>
#include "FreeRTOSConfig.h"
#include "main.h"
#include "cmsis_os.h"

extern SemaphoreHandle_t buttonMutex;
extern TaskHandle_t buttonHandle;
extern TaskHandle_t pollHandle;

inline void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	UNUSED(GPIO_Pin);
	if(xSemaphoreTakeFromISR(buttonMutex, pdFALSE))
		xTaskResumeFromISR(buttonHandle);
}
inline void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
	UNUSED(hadc);
	xTaskResumeFromISR(pollHandle);
}
