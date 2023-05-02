
#pragma once

#include <inttypes.h>
#include <string.h>
#include <PXLParser.h>

template <uint8_t _max_layers, uint8_t _width, uint8_t _height>
class MatrixLed
{
	typedef struct
	{
		bool active;			  // только для active = true изображение будет отображаться
		bool is_set;			  // is_set = true только для слоёв, которым присвоены файлы

		PXLParser<_width, _height> parser;
	} layers_t;

	typedef struct __attribute__((__packed__))
	{
		uint8_t G;
		uint8_t R;
		uint8_t B;
	} color_t;

public:
	MatrixLed(uint16_t fps) : _fps(fps), _brightness(255)
	{
		memset(_layers, 0x00, sizeof(_layers));
		
		_ClearBuffer();
	}

	bool RegLayer(const char *filename, uint8_t idx)
	{
		if (idx < _max_layers)
		{
			_layers[idx].active = false;
			_layers[idx].is_set = true;

			return _layers[idx].parser.OpenFile(filename);
		}

		return false;
	}

	void ShowLayer(uint8_t idx)
	{
		if (idx < _max_layers && _layers[idx].is_set == true)
		{
			_layers[idx].active = true;
		}

		return;
	}

	void HideLayer(uint8_t idx)
	{
		if (idx < _max_layers && _layers[idx].is_set == true)
		{
			_layers[idx].active = false;
			_layers[idx].parser.ReInit();
		}

		return;
	}

	void SetBrightness(uint8_t brightness)
	{
		_brightness = brightness;

		return;
	}

	void Processing(uint32_t time)
	{
		if (time - _last_screen_time <= _fps)
			return;

		_last_screen_time = time;

		for (layers_t &layer : _layers)
		{
			if (!layer.active)
				continue;
			
			layer.parser.GetAutoFrame(time, [&](pixel_data_t &pixel_data)
			{
				if (pixel_data.color4 == 255)
				{
					memcpy(_frame_buff + pixel_data.index, &pixel_data.color1, 3);
				}
			});
		}

		// cumulative brightness correction
		uint8_t *buffer = (uint8_t *)_frame_buff;
		uint16_t frame_buff_idx = sizeof(_frame_buff);
		while (frame_buff_idx != 0)
		{
			--frame_buff_idx;
			_BrightnessConvert(buffer[frame_buff_idx]);
		}

		_frame_buff_ready = true;

		return;
	}
	
	/*
		Возвращает указатель на кадровый буфер и его размер.
			uint8_t *&buffer - Указатель на буфер;
			uint16_t &length - Длина массиа.
	*/
	void GetFrameBuffer(uint8_t *&buffer, uint16_t &length)
	{
		buffer = (uint8_t *)_frame_buff;
		length = sizeof(_frame_buff);

		return;
	}
	
	/*
		Возвращает true если кадровый буфер готов для вывода.
	*/
	bool IsBufferReady()
	{
		bool result = false;
		
		if(_frame_buff_ready == true && _frame_is_draw == false)
		{
			result = true;
		}
		
		return result;
	}
	
	/*
		Вызывается при инициализации передачи данных из буфера.
	*/
	void SetFrameDrawStart()
	{
		_frame_is_draw = true;

		return;
	}

	/*
		Вызывается при завершении передачи данных из буфера.
	*/
	void SetFrameDrawEnd()
	{
		_ClearBuffer();

		return;
	}
	
private:
	void _ClearBuffer()
	{
		memset(_frame_buff, 0x00, sizeof(_frame_buff));
		
		_frame_buff_ready = false;
		_frame_is_draw = false;
		
		return;
	}
	
	// Расчёт яркости пикселя
	inline void _BrightnessConvert(uint8_t &val)
	{
		if(val == 0) return;

		val = ((val * _brightness) + 255) >> 8;
	}

	// Расчёт яркости пикселя + гамма-коррекция по параболе x^2
	inline void _BrightnessGammaConvert(uint16_t &val)
	{
		if(val == 0) return;

		val = ((val * val * _brightness) + 130305) >> 16;
	}
	
	layers_t _layers[_max_layers];
	
	color_t _frame_buff[(_width * _height)];
	bool _frame_buff_ready;
	bool _frame_is_draw = false;
	
	uint8_t _brightness;
	
	uint32_t _last_screen_time = 0;
	uint16_t _fps;
};
