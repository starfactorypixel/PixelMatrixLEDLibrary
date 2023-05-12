#pragma once

#include <inttypes.h>
#include <functional>
#include <ff.h>

template <uint16_t _buffer_size> 
class ReaderWrapper
{
	static const uint8_t _filename_length = 13; // Формат 8.3.
	
	typedef std::function<uint16_t(const char *filename, const uint32_t offset, const uint16_t length, uint8_t *buffer, const uint16_t buffer_size)> request_callback_t;
	
	public:
		
		void SetRequestCallback(request_callback_t callback)
		{
			_request_callback = callback;
			
			return;
		}
		
		FRESULT Open(const char *filename)
		{
			FRESULT result = f_open(&_file_obj, filename, FA_READ);
			
			_file_is_open = (result == FR_OK);
			
			return result;
		}
		
		uint16_t Read(const uint32_t offset, const uint16_t length)
		{
			if(_file_is_open == false) return 0;
			
			uint16_t read_size = 0;
			f_lseek(&_file_obj, offset);
			f_read(&_file_obj, _buffer, length, (UINT*)&read_size);
			
			return read_size;
		}
		
		void Close()
		{


			return;
		}
		
		uint8_t *GetBufferPtr()
		{
			return _buffer;
		}
		
	private:
		
		request_callback_t _request_callback = nullptr;
		
		static inline uint8_t _buffer[_buffer_size];
		uint16_t _buffer_idx;
		
		FIL _file_obj;
		bool _file_is_open;
};
