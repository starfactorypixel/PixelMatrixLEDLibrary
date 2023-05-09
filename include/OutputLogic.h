#pragma once

#include  <PowerOutputs.h>

namespace Output
{
	/* Настройки */
	static constexpr uint8_t CFG_PortCount = 6;			// Кол-во портов управления.
	static constexpr uint32_t CFG_RefVoltage = 3337000;	// Опорное напряжение, микровольты.
	static constexpr uint8_t CFG_INA180_Gain = 20;		// Усиление микросхемы INA180.
	static constexpr uint8_t CFG_ShuntResistance = 5;	// Сопротивление шунта, миллиомы.
	/* */
	
	PowerOutputs<CFG_PortCount> OutObj(CFG_RefVoltage, CFG_INA180_Gain, CFG_ShuntResistance);
	
	void OnShortCircuit(uint8_t num, uint16_t current)
	{

	}
	
	inline void Setup()
	{
		OutObj.AddPort( {GPIOB, GPIO_PIN_11, ADC_CHANNEL_6} );
		OutObj.AddPort( {GPIOB, GPIO_PIN_10, ADC_CHANNEL_5} );
		OutObj.AddPort( {GPIOB, GPIO_PIN_2, ADC_CHANNEL_4} );
		OutObj.AddPort( {GPIOB, GPIO_PIN_1, ADC_CHANNEL_3} );
		OutObj.AddPort( {GPIOB, GPIO_PIN_0, ADC_CHANNEL_2} );
		OutObj.AddPort( {GPIOA, GPIO_PIN_7, ADC_CHANNEL_1} );
		OutObj.Init();

		//OutObj.On(4);
		//OutObj.On(6);
		//OutObj.Off(1);
		OutObj.RegShortCircuitEvent(OnShortCircuit);
		//OutObj.Current(1);

		OutObj.SetOn(6, 250, 500);
		OutObj.SetOn(5, 1000, 100);
		
		return;
	}

	uint8_t port_idx = 0;
	uint32_t last_time = 0;

	inline void Loop(uint32_t &current_time)
	{
		OutObj.Processing(current_time);

		if(current_time - last_time > 250)
		{
			last_time = current_time;

			if(++port_idx == 7) port_idx = 1;

			for(uint8_t i = 1; i < 7; ++i)
			{
				//if(port_idx == i) OutObj.On(i);
				//else OutObj.Off(i);

				Serial::Print("P");
				Serial::Print((uint32_t)i);
				Serial::Print(": ");
				Serial::Print( (uint32_t)OutObj.GetCurrent(i) );
				Serial::Println();
			}
			Serial::Println();

		}
		
		// Обновляем текущее время, чтобы все последующий вызовы текущего loop получили его актуальным.
		current_time = HAL_GetTick();
		
		return;
	}

}