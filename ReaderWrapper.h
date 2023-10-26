#pragma once

#include <inttypes.h>
#include <functional>
#include <ff.h>

template <uint16_t _buffer_size> 
class ReaderWrapper
{
	public:
		
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
		
		FRESULT Close()
		{
			_file_is_open = false;
			
			return f_close(&_file_obj);
		}
		
		uint8_t *GetBufferPtr()
		{
			return _buffer;
		}
		
	private:
		
		FIL _file_obj;
		bool _file_is_open;
		
		static uint8_t _buffer[_buffer_size];
		
};

#warning Temporary static initialization.
template<> uint8_t ReaderWrapper<512>::_buffer[512] = {0};
