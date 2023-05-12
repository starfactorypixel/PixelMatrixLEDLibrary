#pragma once

#include <LEDLibrary.h>

namespace Leds
{
	InfoLeds<4> ledsObj;
	
	inline void Setup()
	{
		ledsObj.AddLed( {GPIOB, GPIO_PIN_6}, ledsObj.LED_RED );
		ledsObj.AddLed( {GPIOB, GPIO_PIN_5}, ledsObj.LED_YELLOW );
		ledsObj.AddLed( {GPIOB, GPIO_PIN_4}, ledsObj.LED_GREEN );
		ledsObj.AddLed( {GPIOB, GPIO_PIN_3}, ledsObj.LED_BLUE );

		return;
	}
	
	inline void Loop(uint32_t &current_time)
	{
		ledsObj.Processing(current_time);

		current_time = HAL_GetTick();

		return;
	}
}
