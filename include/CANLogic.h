#pragma once

#include <CANLibrary.h>
#include "can_abstarction.h"

void HAL_CAN_Send(can_object_id_t id, uint8_t *data, uint8_t length);

extern CAN_HandleTypeDef hcan;
extern UART_HandleTypeDef huart1;

namespace CANLib
{
	//*********************************************************************
	// CAN Library settings
	//*********************************************************************

	/// @brief Number of CANObjects in CANManager
	static constexpr uint8_t CFG_CANObjectsCount = 12;

	/// @brief The size of CANManager's internal CAN frame buffer
	static constexpr uint8_t CFG_CANFrameBufferSize = 16;
	//*********************************************************************
	//*********************************************************************

	// structure for all data fields of CANObject
	// rear_light_can_data_t light_ecu_can_data;

	CANManager<CFG_CANObjectsCount, CFG_CANFrameBufferSize> can_manager(&HAL_CAN_Send);

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
		// light_ecu_can_data.side_beam.brightness = can_frame.data[0];

		// if (light_ecu_can_data.side_beam.brightness == 0)
		if (can_frame.data[0] == 0)
		{
			Matrix::matrixObj.HideLayer(2);
			Outputs::outObj.SetOff(2);
		}
		else
		{
			Matrix::matrixObj.ShowLayer(2);
			Outputs::outObj.SetOn(2);
		}

		can_frame.initialized = true;
		can_frame.function_id = CAN_FUNC_SET_OUT_OK;
		// can_frame.data[0] doesn't change
		// TODO: может читать установленное значение с порта и его присваивать в can_frame.data[0]?
		// читать вот этой функцией: HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9)
		can_frame.raw_data_length = 2;
	}

	// вызывается, если по CAN пришла команда включения/выключения стоп-сигналов
	void brake_light_set_handler(can_frame_t &can_frame, can_error_t &error)
	{
		// light_ecu_can_data.brake_light.brightness = can_frame.data[0];

		// if (light_ecu_can_data.brake_light.brightness == 0)
		if (can_frame.data[0] == 0)
		{
			Matrix::matrixObj.HideLayer(4);
			Outputs::outObj.SetOff(4);
		}
		else
		{
			Matrix::matrixObj.ShowLayer(4);
			Outputs::outObj.SetOn(4);
		}

		can_frame.initialized = true;
		can_frame.function_id = CAN_FUNC_SET_OUT_OK;
		// can_frame.data[0] doesn't change
		can_frame.raw_data_length = 2;
	}

	// вызывается, если по CAN пришла команда включения/выключения заднего хода
	void reverse_light_set_handler(can_frame_t &can_frame, can_error_t &error)
	{
		// light_ecu_can_data.reverse_light.brightness = can_frame.data[0];

		// if (light_ecu_can_data.reverse_light.brightness == 0)
		if (can_frame.data[0] == 0)
		{
			Matrix::matrixObj.HideLayer(3);
			Outputs::outObj.SetOff(3);
		}
		else
		{
			Matrix::matrixObj.ShowLayer(3);
			Outputs::outObj.SetOn(3);
		}

		can_frame.initialized = true;
		can_frame.function_id = CAN_FUNC_SET_OUT_OK;
		// can_frame.data[0] doesn't change
		can_frame.raw_data_length = 2;
	}

	// вызывается, если по CAN пришла команда включения/выключения левого поворотника
	void turn_left_set_handler(can_frame_t &can_frame, can_error_t &error)
	{
		// light_ecu_can_data.left_indicator.brightness = can_frame.data[0];

		// if (light_ecu_can_data.left_indicator.brightness == 0)
		if (can_frame.data[0] == 0)
		{
			Matrix::matrixObj.HideLayer(5);
			Outputs::outObj.SetOff(5);
		}
		else
		{
			Matrix::matrixObj.ShowLayer(5);
			Outputs::outObj.SetOn(5, 750, 750);
		}

		can_frame.initialized = true;
		can_frame.function_id = CAN_FUNC_SET_OUT_OK;
		// can_frame.data[0] doesn't change
		can_frame.raw_data_length = 2;
	}

	// вызывается, если по CAN пришла команда включения/выключения правого поворотника
	void turn_right_set_handler(can_frame_t &can_frame, can_error_t &error)
	{
		// light_ecu_can_data.right_indicator.brightness = can_frame.data[0];

		// if (light_ecu_can_data.right_indicator.brightness == 0)
		if (can_frame.data[0] == 0)
		{
			Matrix::matrixObj.HideLayer(6);
			Outputs::outObj.SetOff(6);
		}
		else
		{
			Matrix::matrixObj.ShowLayer(6);
			Outputs::outObj.SetOn(6, 750, 750);
		}

		can_frame.initialized = true;
		can_frame.function_id = CAN_FUNC_SET_OUT_OK;
		// can_frame.data[0] doesn't change
		can_frame.raw_data_length = 2;
	}

	// вызывается, если по CAN пришла команда включения/выключения аварийного сигнала
	void hazard_beam_set_handler(can_frame_t &can_frame, can_error_t &error)
	{
		// light_ecu_can_data.hazard_beam.brightness = can_frame.data[0];

		// if (light_ecu_can_data.hazard_beam.brightness == 0)
		if (can_frame.data[0] == 0)
		{
			Matrix::matrixObj.HideLayer(7);
			Outputs::outObj.SetOff(5);
			Outputs::outObj.SetOff(6);
		}
		else
		{
			Matrix::matrixObj.ShowLayer(7);
			Outputs::outObj.SetOn(5, 750, 750);
			Outputs::outObj.SetOn(6, 750, 750);
		}

		can_frame.initialized = true;
		can_frame.function_id = CAN_FUNC_SET_OUT_OK;
		// can_frame.data[0] doesn't change
		can_frame.raw_data_length = 2;
	}

	// вызывается, если по CAN пришла команда включения/выключения пользовательского света
	void custom_beam_set_handler(can_frame_t &can_frame, can_error_t &error)
	{
		// light_ecu_can_data.custom_beam.brightness = can_frame.data[0];

		// if (light_ecu_can_data.custom_beam.brightness == 0)
		if (can_frame.data[0] == 0)
		{
			Outputs::outObj.SetOff(1);
		}
		else
		{
			Outputs::outObj.SetOn(1);
		}

		can_frame.initialized = true;
		can_frame.function_id = CAN_FUNC_SET_OUT_OK;
		// can_frame.data[0] doesn't change
		can_frame.raw_data_length = 2;
	}

	// вызывается, если по CAN пришла команда включения/выключения пользовательского изображения на панели
	void custom_image_set_handler(can_frame_t &can_frame, can_error_t &error)
	{
		// light_ecu_can_data.custom_image.brightness = can_frame.data[0];

		// if (light_ecu_can_data.custom_image.brightness == 0)
		if (can_frame.data[0] == 0)
		{
			Matrix::matrixObj.HideLayer(1);
		}
		else
		{
			// TODO: 
			// 1. Прочитать ID файла из пакета;
			// 2. Загрузить слой ("customXXX.pxl");
			// 3. Включить слой;
			//Matrix::matrixObj.ShowLayer(1);
		}

		can_frame.initialized = true;
		can_frame.function_id = CAN_FUNC_SET_OUT_OK;
		// can_frame.data[0] doesn't change
		can_frame.raw_data_length = 2;
	}

	inline void Setup()
	{
		// set CAN data structure to zero
		// memset(&light_ecu_can_data, 0, sizeof(light_ecu_can_data));

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

		//*************************************************************
		// TODO: CANManager experiments
		// init normal timer for Block_Info
		obj_block_info.SetValue(0, 0x66, CAN_TIMER_TYPE_NORMAL);
		//*************************************************************

		return;
	}

	inline void Loop(uint32_t &current_time)
	{
		can_manager.Process(current_time);

		//*************************************************************
		// TODO: CANManager experiments
		static uint32_t iter = 0;
		if (current_time - iter > 1000)
		{
			iter = current_time;

			uint8_t *data = (uint8_t *)&current_time;
			obj_block_info.SetValue(3, data[0], CAN_TIMER_TYPE_NORMAL);
			obj_block_info.SetValue(4, data[1], CAN_TIMER_TYPE_NORMAL);
			obj_block_info.SetValue(5, data[2], CAN_TIMER_TYPE_NORMAL);
			obj_block_info.SetValue(6, data[3], CAN_TIMER_TYPE_NORMAL);
		}
		//*************************************************************

		current_time = HAL_GetTick();

		return;
	}
}
