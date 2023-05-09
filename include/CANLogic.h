#pragma once

#include <CANLibrary.h>
#include "can_abstarction.h"

void HAL_CAN_Send(can_object_id_t id, uint8_t *data, uint8_t length);

extern CAN_HandleTypeDef hcan;
extern UART_HandleTypeDef huart1;

namespace CANLib
{
	/* Настройки */
	static constexpr uint8_t CFG_CANObjects = 6;	// Кол-во объектов CAN.
	/* */
	
	// structure for all data fields of CANObject
	rear_light_can_data_t light_ecu_can_data;
	
	CANManager<CFG_CANObjects> can_manager(&HAL_CAN_Send);
	
	// common blocks
	CANObject<uint8_t, 7> obj_block_info(REAR_LIGHT_CANO_ID_BLOCK_INFO, 15000, 300);
	CANObject<uint8_t, 7> obj_block_health(REAR_LIGHT_CANO_ID_BLOCK_HEALTH, CAN_TIMER_DISABLED, 300);
	CANObject<uint8_t, 7> obj_block_cfg(REAR_LIGHT_CANO_ID_BLOCK_CFG, CAN_TIMER_DISABLED, CAN_ERROR_DISABLED);
	CANObject<uint8_t, 7> obj_block_error(REAR_LIGHT_CANO_ID_BLOCK_ERROR, CAN_TIMER_DISABLED, 300);
	
	// specific blocks
	CANObject<uint8_t, 1> obj_side_beam(REAR_LIGHT_CANO_ID_SIDE_BEAM, CAN_TIMER_DISABLED, 300);
	CANObject<uint8_t, 1> obj_brake_light(REAR_LIGHT_CANO_ID_BRAKE_LIGHT, CAN_TIMER_DISABLED, 300);
	CANObject<uint8_t, 1> obj_reverse_light(REAR_LIGHT_CANO_ID_REVERSE_LIGHT, CAN_TIMER_DISABLED, 300);
	CANObject<uint8_t, 1> obj_left_indicator(REAR_LIGHT_CANO_ID_LEFT_INDICATOR, CAN_TIMER_DISABLED, 300);
	CANObject<uint8_t, 1> obj_right_indicator(REAR_LIGHT_CANO_ID_RIGHT_INDICATOR, CAN_TIMER_DISABLED, 300);
	CANObject<uint8_t, 1> obj_hazard_beam(REAR_LIGHT_CANO_ID_HAZARD_BEAM, CAN_TIMER_DISABLED, 300);
	CANObject<uint8_t, 1> obj_custom_beam(REAR_LIGHT_CANO_ID_CUSTOM_BEAM, CAN_TIMER_DISABLED, 300);
	CANObject<uint8_t, 1> obj_custom_image(REAR_LIGHT_CANO_ID_CUSTOM_IMAGE, CAN_TIMER_DISABLED, 300);
	
	
	// вызывается, если по CAN пришла команда включения/выключения габаритов
	void side_beam_set_handler(can_frame_t &can_frame, can_error_t &error)
	{
		/*
		can_frame.initialized = false;
		error.error_section = ERROR_SECTION_CAN_OBJECT;
		error.error_code = ERROR_CODE_OBJECT_SET_FUNCTION_IS_MISSING;
		error.function_id = CAN_FUNC_SET_OUT_ERR;
		return;
		*/

		can_frame.initialized = true;
		can_frame.function_id = CAN_FUNC_SET_OUT_OK;
		can_frame.raw_data_length = 1;

		light_ecu_can_data.side_beam.brightness = can_frame.data[0];

		if (light_ecu_can_data.side_beam.brightness == 0)
		{
			//OUT1_OFF;
			Matrix::matrix.HideLayer(2);
			Output::OutObj.SetOff(3);
		}
		else
		{
			//OUT1_ON;
			Matrix::matrix.ShowLayer(2);
			Output::OutObj.SetOn(3);
			// TODO: установка яркости не корректна, так как задаётся яркость всей панели, а не только одних огней
			// matrix.SetBrightness(light_ecu_can_data.side_beam.brightness);
		}
	}

	// вызывается, если по CAN пришла команда включения/выключения стоп-сигналов
	void brake_light_set_handler(can_frame_t &can_frame, can_error_t &error)
	{
		can_frame.initialized = false;
		error.error_section = ERROR_SECTION_CAN_OBJECT;
		error.error_code = ERROR_CODE_OBJECT_SET_FUNCTION_IS_MISSING;
		error.function_id = CAN_FUNC_SET_OUT_ERR;
		return;

		light_ecu_can_data.brake_light.brightness = can_frame.data[0];

		if (light_ecu_can_data.brake_light.brightness == 0)
		{
			//OUT2_OFF;
			Matrix::matrix.HideLayer(4);
		}
		else
		{
			//OUT2_ON;
			Matrix::matrix.ShowLayer(4);
			// TODO: установка яркости не корректна, так как задаётся яркость всей панели, а не только одних огней
			// matrix.SetBrightness(light_ecu_can_data.brake_light.brightness);
		}
	}

	// вызывается, если по CAN пришла команда включения/выключения заднего хода
	void reverse_light_set_handler(can_frame_t &can_frame, can_error_t &error)
	{
		can_frame.initialized = false;
		error.error_section = ERROR_SECTION_CAN_OBJECT;
		error.error_code = ERROR_CODE_OBJECT_SET_FUNCTION_IS_MISSING;
		error.function_id = CAN_FUNC_SET_OUT_ERR;
		return;

		light_ecu_can_data.reverse_light.brightness = can_frame.data[0];

		if (light_ecu_can_data.reverse_light.brightness == 0)
		{
			//OUT3_OFF;
			Matrix::matrix.HideLayer(3);
		}
		else
		{
			//OUT3_ON;
			Matrix::matrix.ShowLayer(3);
			// TODO: установка яркости не корректна, так как задаётся яркость всей панели, а не только одних огней
			// matrix.SetBrightness(light_ecu_can_data.reverse_light.brightness);
		}
	}

	// вызывается, если по CAN пришла команда включения/выключения левого поворотника
	void turn_left_set_handler(can_frame_t &can_frame, can_error_t &error)
	{
		can_frame.initialized = false;
		error.error_section = ERROR_SECTION_CAN_OBJECT;
		error.error_code = ERROR_CODE_OBJECT_SET_FUNCTION_IS_MISSING;
		error.function_id = CAN_FUNC_SET_OUT_ERR;
		return;

		light_ecu_can_data.left_indicator.brightness = can_frame.data[0];

		// TODO: надо же ещё мигать выводами OUTx?
		if (light_ecu_can_data.left_indicator.brightness == 0)
		{
			//OUT4_OFF;
			Matrix::matrix.HideLayer(5);
		}
		else
		{
			//OUT4_ON;
			Matrix::matrix.ShowLayer(5);
			// TODO: установка яркости не корректна, так как задаётся яркость всей панели, а не только одних огней
			// matrix.SetBrightness(light_ecu_can_data.left_indicator.brightness);
		}
	}

	// вызывается, если по CAN пришла команда включения/выключения правкого поворотника
	void turn_right_set_handler(can_frame_t &can_frame, can_error_t &error)
	{
		can_frame.initialized = false;
		error.error_section = ERROR_SECTION_CAN_OBJECT;
		error.error_code = ERROR_CODE_OBJECT_SET_FUNCTION_IS_MISSING;
		error.function_id = CAN_FUNC_SET_OUT_ERR;
		return;

		light_ecu_can_data.right_indicator.brightness = can_frame.data[0];

		// TODO: надо же ещё мигать выводами OUTx?
		if (light_ecu_can_data.right_indicator.brightness == 0)
		{
			//OUT5_OFF;
			Matrix::matrix.HideLayer(6);
		}
		else
		{
			//OUT5_ON;
			Matrix::matrix.ShowLayer(6);
			// TODO: установка яркости не корректна, так как задаётся яркость всей панели, а не только одних огней
			// matrix.SetBrightness(light_ecu_can_data.right_indicator.brightness);
		}
	}

	// вызывается, если по CAN пришла команда включения/выключения аварийного сигнала
	void hazard_beam_set_handler(can_frame_t &can_frame, can_error_t &error)
	{
		can_frame.initialized = false;
		error.error_section = ERROR_SECTION_CAN_OBJECT;
		error.error_code = ERROR_CODE_OBJECT_SET_FUNCTION_IS_MISSING;
		error.function_id = CAN_FUNC_SET_OUT_ERR;
		return;

		light_ecu_can_data.hazard_beam.brightness = can_frame.data[0];

		// TODO: надо же ещё мигать выводами OUTx?
		if (light_ecu_can_data.hazard_beam.brightness == 0)
		{
			//OUT4_OFF;
			//OUT5_OFF;
			Matrix::matrix.HideLayer(5);
			Matrix::matrix.HideLayer(6);
		}
		else
		{
			//OUT4_ON;
			//OUT5_ON;
			Matrix::matrix.ShowLayer(5);
			Matrix::matrix.ShowLayer(6);
			// TODO: установка яркости не корректна, так как задаётся яркость всей панели, а не только одних огней
			// matrix.SetBrightness(light_ecu_can_data.hazard_beam.brightness);
		}
	}

	// вызывается, если по CAN пришла команда включения/выключения пользовательского света
	void custom_beam_set_handler(can_frame_t &can_frame, can_error_t &error)
	{
		can_frame.initialized = false;
		error.error_section = ERROR_SECTION_CAN_OBJECT;
		error.error_code = ERROR_CODE_OBJECT_SET_FUNCTION_IS_MISSING;
		error.function_id = CAN_FUNC_SET_OUT_ERR;
		return;

		light_ecu_can_data.custom_beam.brightness = can_frame.data[0];

		if (light_ecu_can_data.custom_beam.brightness == 0)
		{
			//OUT6_OFF;
		}
		else
		{
			//OUT6_ON;
			// TODO: включение - это любое значение больше 0?
			// Или жестко 255 будет включение, а 1..254 не будут влиять ни на что?
		}
	}

	// вызывается, если по CAN пришла команда включения/выключения пользовательского изображения на панели
	void custom_image_set_handler(can_frame_t &can_frame, can_error_t &error)
	{
		can_frame.initialized = false;
		error.error_section = ERROR_SECTION_CAN_OBJECT;
		error.error_code = ERROR_CODE_OBJECT_SET_FUNCTION_IS_MISSING;
		error.function_id = CAN_FUNC_SET_OUT_ERR;
		return;

		light_ecu_can_data.custom_image.brightness = can_frame.data[0];

		// TODO: надо же ещё мигать выводами OUTx?
		if (light_ecu_can_data.custom_image.brightness == 0)
		{
			Matrix::matrix.HideLayer(1);
		}
		else
		{
			Matrix::matrix.ShowLayer(1);
			// TODO: установка яркости не корректна, так как задаётся яркость всей панели, а не только одних огней
			// matrix.SetBrightness(light_ecu_can_data.custom_image.brightness);
		}
	}
	
	inline void Setup()
	{
		// set CAN data structure to zero
		memset(&light_ecu_can_data, 0, sizeof(light_ecu_can_data));
		
		obj_side_beam.RegisterFunctionSet(&side_beam_set_handler);
		obj_brake_light.RegisterFunctionSet(&brake_light_set_handler);
		obj_reverse_light.RegisterFunctionSet(&reverse_light_set_handler);
		obj_left_indicator.RegisterFunctionSet(&turn_left_set_handler);
		obj_right_indicator.RegisterFunctionSet(&turn_right_set_handler);
		obj_hazard_beam.RegisterFunctionSet(&hazard_beam_set_handler);
		obj_custom_beam.RegisterFunctionSet(&custom_beam_set_handler);
		obj_custom_image.RegisterFunctionSet(&custom_image_set_handler);
		
		// common blocks
		can_manager.RegisterObject(obj_block_info);
		can_manager.RegisterObject(obj_block_health);
		can_manager.RegisterObject(obj_block_cfg);
		can_manager.RegisterObject(obj_block_error);
		
		// specific blocks
		can_manager.RegisterObject(obj_side_beam);
		can_manager.RegisterObject(obj_brake_light);
		can_manager.RegisterObject(obj_reverse_light);
		can_manager.RegisterObject(obj_left_indicator);
		can_manager.RegisterObject(obj_right_indicator);
		can_manager.RegisterObject(obj_hazard_beam);
		can_manager.RegisterObject(obj_custom_beam);
		can_manager.RegisterObject(obj_custom_image);
		
		// Tests
		obj_block_info.SetValue(0, 0x66, CAN_TIMER_TYPE_NORMAL);
		
		return;
	}
	
	inline void Loop(uint32_t &current_time)
	{
		can_manager.Process(current_time);
		
		current_time = HAL_GetTick();
		
		return;
	}
}
