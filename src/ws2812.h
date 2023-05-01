#ifndef WS2812_H_
#define WS2812_H_
//--------------------------------------------------
// #include "libs.h"
#include "stm32f1xx_hal.h"
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
//--------------------------------------------------
#define HIGH 43	//65
#define LOW	21 //26
// #define PWM_HI	0x2B	//43
// #define PWM_LO	0x15	//21
//--------------------------------------------------
#define TIM_NUM	   2  ///< Timer number
#define TIM_CH	   TIM_CHANNEL_2  ///< Timer's PWM channel
#define DMA_HANDLE hdma_tim2_ch2_ch4  ///< DMA Channel
#define DMA_SIZE_BYTE     ///< DMA Memory Data Width: {.._BYTE, .._HWORD, .._WORD}

//--------------------------------------------------


//--------------------------------------------------
void ws2812_init(void);
void ws2812_pixel_rgb_to_buf_dma(uint8_t Rpixel , uint8_t Gpixel, uint8_t Bpixel, uint16_t posX);
void ws2812_set(uint8_t G, uint8_t B, uint8_t R, uint16_t len);
void ws2812_clear(uint16_t len);
void ws2812_setValue(void);
void ws2812_light(void);

//--------------------------------------------------
#endif /* WS2812_H_ */
