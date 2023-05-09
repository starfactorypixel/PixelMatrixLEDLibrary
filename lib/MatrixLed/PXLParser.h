
#pragma once

#include <inttypes.h>
#include <string.h>
#include <functional>
#include <ReaderWrapper.h>

typedef struct __attribute__((__packed__))
{
	uint16_t index;
	uint8_t color1;
	uint8_t color2;
	uint8_t color3;
	uint8_t color4;
} pixel_data_t;

/************************************************************************************************************************
 *
 * PXLParser
 *
 ************************************************************************************************************************/
template <uint8_t _width, uint8_t _height>
class PXLParser
{
	struct __attribute__((__packed__)) header_file_t
	{
		uint8_t version;		  // версия формата файла
		uint8_t size_x;			  // Полный размер изображения в пикселях по оси X
		uint8_t size_y;			  // Полный размер изображения в пикселях по оси Y
		uint8_t format_strip : 4; // Формат данных: способ подключения ленты
		uint8_t format_color : 4; // Формат данных: порядок цветов
		uint16_t img_frames;	  // Число кадров анимации, от 1 до 65535
		uint8_t img_repeats;	  // Кол-во повторов анимации, от 0 до 255. 0 - статичная картинка, 255 - бесконечно.
	};

	struct __attribute__((__packed__)) header_frame_t
	{
		uint16_t frame_timeout; // Время (мс) отображения текущего кадра, от 0 до 65535
		uint16_t frame_pixels;	// Число пикселей в текущем кадре (по сути кол-во структур pixel_data_t)
	};

	static const uint8_t _file_version = 1;
	
	static const uint8_t _file_header_size_bytes = 3 + sizeof(header_file_t);
	static const uint8_t _frame_header_size_bytes = sizeof(header_frame_t);
	static const uint16_t _frame_chank_size_bytes = 504;
	static const uint8_t _frame_chank_size_pixels = 84;
	static const uint16_t _reader_buffer_size_bytes = 512;

public:

	typedef std::function<void(pixel_data_t &pixel_data)> pixel_callback_t;

	enum error_t : uint8_t
	{
		ERROR_OK,
		ERROR_INVALID_FILE,
		ERROR_VERSION,
		ERROR_SIZE,
		ERROR_FORMAT,
		ERROR_NOFRAMES,
		ERROR_OPEN_FILE
	};


	error_t OpenFile(const char *filename)
	{
		if( _reader.Open(filename) == 0 )
		{
			if( _reader.Read(_file_offset, _file_header_size_bytes) == _file_header_size_bytes )
			{
				_ParseFileHeader( _reader.GetBufferPtr() );
				
				ReInit();
			}
		}
		else
		{
			_SetError(ERROR_OPEN_FILE);
		}
		
		return _error;
	}

	/*

	*/
	bool GetAutoFrame(uint32_t current_time, pixel_callback_t callback)
	{
		if (_error != ERROR_OK)
			return false;

		// Если пришло время рисовать следующий кадр.
		if (_layer_params.has_animation && current_time - _frame_last_draw_time > _frame_delay_time)
		{
			_frame_last_draw_time = current_time;	// сохраняем время начала отрисовки кадра
			_file_offset = _file_offset_next_frame; // устанавливаем текущее смещение в файле на следующий кадр
			++_frame_current;						// увеличиваем номер кадра
		}
		// Если рано рисовать следующий кадр, то рисовать будем текущий кадр.
		else
		{
			_file_offset = _file_offset_current_frame; // текущее смещение в файле равно текущему кадру
													   // номер кадра не увеличиваем
		}

		// если текущий фрейм — это последний в анимации, то сохраняем на будущее его смещение в файле
		if (_header_file.img_frames == _frame_current)
		{
			_file_offset_last_frame = _file_offset;
		}

		// если достигли последнего кадра в анимации...
		if (_header_file.img_frames + 1 == _frame_current)
		{
			// если анимация должна быть проиграна конечное количество раз
			// и последняя итерация еще не достигнута, то...
			if (_layer_params.animation_finite_repeatable && _img_repeats_count < _header_file.img_repeats)
			{
				// увеличиваем счетчик повторов анимации
				++_img_repeats_count;
			}

			// если в анимации достигли заданного количества повторов,
			// то можно останавливаться
			if (_img_repeats_count == _header_file.img_repeats)
			{
				--_frame_current;						// уменьшаем счетчик текущего кадра, потому что до этого его зря увеличили, видимо....
				_file_offset = _file_offset_last_frame; // текущее смещение в файле устанавливаем на последний кадр
			}
			// если же повторяем дальше,
			// то стартовать надо с первого кадра
			else
			{
				_frame_current = 1;						// счетчик кадров на начало
				_file_offset = _file_header_size_bytes; // смещение в файле на позицию сразу после хедера
			}
		}

		// сохраняем смещение в файле как текущий кадр
		_file_offset_current_frame = _file_offset;

		uint32_t frame_bytes_left = 0; // Расчётное кол-во байт кадра.

		uint16_t frame_pixels;
		uint16_t pixel_skip = 0;

		uint8_t *buffer = nullptr;
		header_frame_t *frame_header = nullptr;
		
		uint16_t bytes_need = (_frame_chank_size_bytes + _frame_header_size_bytes);

		// читаем из файла данные, начиная со смещения _file_offset
		// количество данных не более _frame_chank_size_bytes + _frame_header_size_bytes
		uint16_t read = _reader.Read(_file_offset, bytes_need);

		// не прочитали, уходим
		if (read == 0)
			return false;

		// если что-то прочитали, то получаем указатель на буфер
		buffer = _reader.GetBufferPtr();

		frame_header = (header_frame_t *)buffer;
		frame_pixels = frame_header->frame_pixels;

		_frame_delay_time = frame_header->frame_timeout;

		// если пикселей в кадре больше, чем влезает в наш чанк
		if (frame_pixels > _frame_chank_size_pixels)
		{
			// смещаемся на количество прочитанных байт
			_file_offset += read;
			// подсчитываем, сколько байт из кадра осталось прочитать
			frame_bytes_left = (frame_pixels * 6) - _frame_chank_size_bytes; // 6 - количество байт на пиксель
		}
		// если же прочитано больше, чем один фрейм
		else
		{
			// то смещаемся только на размер кадра + его заголовок
			_file_offset += (frame_pixels * 6) + 4;
			// получается, всё, что за пределами кадра прочитано, будем читать повторно...
			// выигрыш от чтения всего буфера сразу будет только при условии, что чтение "лишней" части буфера
			// по времени будет занимать меньше, чем дополнительное обращение к файлу с запросом чтения.
			// Но по идее закрывать файл между чтением хедера кадра и чтением пикселей кадра не потребуется,
			// а значит по времени больших расходов не должно быть.
			// В общем, надо тестировать. Возможно, выгоднее читать заголовок фрейма и его пиксели по отдельности
		}

		pixel_data_t pixel_data = {0};
		pixel_data_t *pixel_data_pointer = (pixel_data_t *)&buffer[_frame_header_size_bytes];

		// шагаем по всем пикселям кадра
		for (uint16_t pixel_idx = 0; pixel_idx < frame_pixels; ++pixel_idx)
		{
			// если пиксель упёрся в границу буфера, то надо бы подгрузить остаток кадра
			if (pixel_idx > 0 && pixel_idx % _frame_chank_size_pixels == 0)
			{
				// определяем, сколько нам еще надо загрузить
				bytes_need = (frame_bytes_left > _frame_chank_size_bytes) ? _frame_chank_size_bytes : frame_bytes_left;

				// читаем очередную порцию данных
				read = _reader.Read(_file_offset, bytes_need);
				// не прочитали, уходим
				if (read == 0)
					return false;

				frame_bytes_left -= read;	// уменьшаем непрочитанный остаток кадра
				_file_offset += read; // и увеличиваем смещение в файле

				// обнуляем указатель на пиксели
				pixel_data_pointer = (pixel_data_t *)buffer;
			}

			// считываем количество пропускаемых пикселей
			pixel_skip += pixel_data_pointer->index;

			// и заполняем данные в матрице с помощью коллбэка
			memcpy(&pixel_data, pixel_data_pointer, 6);
			pixel_data.index = (pixel_skip + pixel_idx);
			callback(pixel_data);

			pixel_data_pointer++;
		}

		// сохранение текущего смещения в файле как смещение следующего кадра
		_file_offset_next_frame = _file_offset;

		return true;
	}

	void ReInit()
	{
		_file_offset = _file_header_size_bytes;
		_frame_current = 0;
		_img_repeats_count = 0;

		_file_offset_current_frame = _file_header_size_bytes;
		_file_offset_next_frame = _file_header_size_bytes;
		_file_offset_last_frame = _file_header_size_bytes;

		_frame_last_draw_time = 0;
		_frame_delay_time = 0;

		return;
	}

private:
	void _ParseFileHeader(uint8_t *data)
	{
		if (memcmp(data, "PXL", 3) != 0)
			return _SetError(ERROR_INVALID_FILE);

		_header_file.version = data[3];
		_header_file.size_x = data[4];
		_header_file.size_y = data[5];
		_header_file.format_strip = data[6] & 0x0F;
		_header_file.format_color = (data[6] >> 4) & 0x0F;
		_header_file.img_frames = data[7] | (data[8] << 8);
		_header_file.img_repeats = data[9];

		_AnalysisFile();

		return;
	}

	void _AnalysisFile()
	{
		if (_header_file.version != _file_version)
			return _SetError(ERROR_VERSION);

		if (_header_file.size_x != _width || _header_file.size_y != _height)
			return _SetError(ERROR_SIZE);

		if (_header_file.format_strip != 2 || _header_file.format_color != 2)
			return _SetError(ERROR_FORMAT);

		if (_header_file.img_frames == 0)
			return _SetError(ERROR_NOFRAMES);

		_layer_params.animation_finite_repeatable = _header_file.img_repeats > 0 && _header_file.img_repeats < 255;
		_layer_params.has_animation = _header_file.img_frames > 1 && _header_file.img_repeats != 0;

		return _SetError(ERROR_OK);
	}

	error_t _error;
	void _SetError(error_t error)
	{
		_error = error;
	}

	ReaderWrapper<_reader_buffer_size_bytes> _reader;

	uint32_t _file_offset;				 // Смещение чтения файла, байт.
	uint32_t _file_offset_last_frame;	 // Смещение от начала файла до последнего кадра (для отрисовки последнего кадра в не бесконечной анимации).
	uint32_t _file_offset_current_frame; // Смещение текущего кадра в файле
	uint32_t _file_offset_next_frame;	 // смещение следующего кадра в файле

	uint16_t _frame_current;

	uint16_t _img_repeats_count;

	uint32_t _frame_last_draw_time; // Время рисование последнего кадра.
	uint16_t _frame_delay_time;		// задержка между кадрами анимации

	header_file_t _header_file;

	header_frame_t _header_frame;

	struct __attribute__((__packed__)) layer_params_t
	{
		bool animation_finite_repeatable;
		bool has_animation;
	} _layer_params;
};