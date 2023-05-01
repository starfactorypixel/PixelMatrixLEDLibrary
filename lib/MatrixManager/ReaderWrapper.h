
#pragma once

#include <inttypes.h>
#include <functional>
#include <ff.h>
#include <string.h>

void SerialPrint(const char *str, uint16_t len);

//FIL MyFile;

template <uint16_t _buffer_size> 
class ReaderWrapper
{
	typedef std::function<uint16_t(const char *filename, const uint32_t offset, const uint16_t length, uint8_t *buffer, const uint16_t buffer_size)> request_callback_t;
	
	public:
		
		void SetRequestCallback(request_callback_t callback)
		{
			_request_callback = callback;
			
			return;
		}
		
		bool Open(const char *filename)
		{
			f_open(&MyFile, filename, FA_READ);
			
			return true;
		}
		
		uint16_t Read(const char *filename, const uint32_t offset, const uint16_t length)
		{
			uint16_t read_size = 0;
			
			//uint8_t resultF = f_open(&MyFile, filename, FA_READ);
			//if(resultF == FR_OK)
			//{
				f_lseek(&MyFile, offset);
				f_read(&MyFile, _buffer, length, (UINT*)&read_size);
				//f_close(&MyFile);
			//}
			
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
		
		// changed to non static
		static inline uint8_t _buffer[_buffer_size];
		uint32_t _buffer_idx;
		
		request_callback_t _request_callback = nullptr;

		FIL MyFile;
		
};
