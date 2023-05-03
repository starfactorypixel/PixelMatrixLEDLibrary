#include <inttypes.h>

extern UART_HandleTypeDef huart1;

namespace Serial
{
	HAL_StatusTypeDef _PrintNumber(uint32_t num, uint8_t radix);
	
	
	HAL_StatusTypeDef Print(const char *str)
	{
		return HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), 1000);
	}

	HAL_StatusTypeDef Print(const uint8_t *str, uint16_t length)
	{
		return HAL_UART_Transmit(&huart1, (uint8_t*)str, length, 1000);
	}
	
	HAL_StatusTypeDef Print(uint32_t num, uint8_t radix = 10)
	{
		return _PrintNumber(num, radix);
	}
	
	HAL_StatusTypeDef Print(int32_t num, uint8_t radix = 10)
	{
		if(radix == 10)
		{
			if(num < 0)
			{
				Print("-");
				
				return _PrintNumber(abs(num), radix);
			}
			
			return _PrintNumber(num, radix);
		}
		else
		{
			return _PrintNumber(num, radix);
		}
	}

	HAL_StatusTypeDef Println()
	{
		return Print("\r\n");
	}
	
	HAL_StatusTypeDef _PrintNumber(uint32_t num, uint8_t radix)
	{
		if(radix < 2) radix = 10;
		
		char buf[8 * sizeof(long) + 1];
		char *str = &buf[sizeof(buf) - 1];
		
		*str = '\0';
		
		do
		{
			char c = num % radix;
			num /= radix;
			
			*--str = c < 10 ? c + '0' : c + 'A' - 10;
		} while(num);
		
		return Print(str);
	}

}
