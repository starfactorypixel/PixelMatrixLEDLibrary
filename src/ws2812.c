#include "ws2812.h"
//----------------------------------------------------------------------------
extern TIM_HandleTypeDef htim2;

static void TIM_DMAPulseCplt(DMA_HandleTypeDef *hdma);
static void TIM_DMAPulseHalfCplt(DMA_HandleTypeDef *hdma);
//----------------------------------------------------------------------------
/// Timer handler
#if TIM_NUM == 1
#define TIM_HANDLE  htim1
#elif TIM_NUM == 2
#define TIM_HANDLE  htim2
#elif TIM_NUM == 3
#define TIM_HANDLE  htim3
#elif TIM_NUM == 4
#define TIM_HANDLE  htim4
#elif TIM_NUM == 5
#define TIM_HANDLE  htim5
#elif TIM_NUM == 8
#define TIM_HANDLE  htim8
#else
#warning If you shure, set TIM_HANDLE
#endif

//volatile ARGB_STATE ARGB_LOC_ST; ///< Buffer send status

#define ARRAY_LEN (3 * 8 * 4)    ///< Pack len * 8 bit * 4 LEDs	
volatile uint8_t PWM_HI = 0x2B;    ///< PWM Code HI Log.1 period
volatile uint8_t PWM_LO = 0x15;    ///< PWM Code LO Log.1 period
//----------------------------------------------------------------------------
uint16_t BUF_DMA [ARRAY_LEN] = {0};
uint8_t rgb_temp[12][3];

//uint16_t DMA_BUF_TEMP[24];
//------------------------------------------------------------------
void ws2812_init(void)
{
  int i;
  for(i=0;i<ARRAY_LEN;i++) BUF_DMA[i] = LOW;
	
    /* Auto-calculation! */
    uint32_t APBfq; // Clock freq
#ifdef APB1
    APBfq = HAL_RCC_GetPCLK1Freq();
    APBfq *= (RCC->CFGR & RCC_CFGR_PPRE1) == 0 ? 1 : 2;
#endif
#ifdef APB2
    APBfq = HAL_RCC_GetPCLK2Freq();
    APBfq *= (RCC->CFGR & RCC_CFGR_PPRE2) == 0 ? 1 : 2;
#endif
#ifdef WS2811S
    APBfq /= (uint32_t) (400 * 1000);  // 400 KHz - 2.5us
#else
    APBfq /= (uint32_t) (800 * 1000);  // 800 KHz - 1.25us
#endif
    TIM_HANDLE.Instance->PSC = 0;                        // dummy hardcode now
    TIM_HANDLE.Instance->ARR = (uint16_t) (APBfq - 1);   // set timer prescaler
    TIM_HANDLE.Instance->EGR = 1;                        // update timer registers
#if defined(WS2811F) || defined(WS2811S)
    PWM_HI = (u8_t) (APBfq * 0.48) - 1;     // Log.1 - 48% - 0.60us/1.2us
    PWM_LO = (u8_t) (APBfq * 0.20) - 1;     // Log.0 - 20% - 0.25us/0.5us
#endif
#ifdef WS2812
    PWM_HI = (u8_t) (APBfq * 0.56) - 1;     // Log.1 - 56% - 0.70us
    PWM_LO = (u8_t) (APBfq * 0.28) - 1;     // Log.0 - 28% - 0.35us
#endif
#ifdef SK6812
    PWM_HI = (u8_t) (APBfq * 0.48) - 1;     // Log.1 - 48% - 0.60us
    PWM_LO = (u8_t) (APBfq * 0.24) - 1;     // Log.0 - 24% - 0.30us
#endif

    TIM_CCxChannelCmd(TIM_HANDLE.Instance, TIM_CH, TIM_CCx_ENABLE); // Enable GPIO to IDLE state
    HAL_Delay(1); // Make some delay
	
}
//------------------------------------------------------------------
void ws2812_pixel_rgb_to_buf_dma(uint8_t Rpixel , uint8_t Gpixel, uint8_t Bpixel, uint16_t posX)
{
  // volatile uint16_t i;
  // for(i=0;i<8;i++)
  // {
  //   if (BitIsSet(Rpixel,(7-i)) == 1)
  //   {
  //     BUF_DMA[posX*24+i+8] = HIGH;
  //   }else
  //   {
  //     BUF_DMA[posX*24+i+8] = LOW;
  //   }
  //   if (BitIsSet(Gpixel,(7-i)) == 1)
  //   {
  //     BUF_DMA[posX*24+i+0] = HIGH;
  //   }else
  //   {
  //     BUF_DMA[posX*24+i+0] = LOW;
  //   }
  //   if (BitIsSet(Bpixel,(7-i)) == 1)
  //   {
  //     BUF_DMA[posX*24+i+16] = HIGH;
  //   }else
  //   {
  //     BUF_DMA[posX*24+i+16] = LOW;
  //   }
  // }
}
//------------------------------------------------------------------
void ws2812_set(uint8_t G, uint8_t B, uint8_t R, uint16_t len){
	//  uint8_t n=0;
  // for(n=0;n<len;n++)
  // {
	// 	ws2812_pixel_rgb_to_buf_dma( G, B, R, n);
	// }
	
}
void ws2812_clear(uint16_t len){
	//  uint8_t n=0;
  // for(n=0;n<len;n++)
  // {
	// 	ws2812_pixel_rgb_to_buf_dma( 0, 0, 0, n);
	// }
	
}
//------------------------------------------------------------------


//-----------------------------------------------  
static void TIM_DMAPulseCplt(DMA_HandleTypeDef *hdma) {
	
	
	TIM_HandleTypeDef *htim = (TIM_HandleTypeDef *) ((DMA_HandleTypeDef *) hdma)->Parent;
	

//	TIM_CHANNEL_STATE_SET(htim, TIM_CHANNEL_2, HAL_TIM_CHANNEL_STATE_READY);
	
	HAL_TIM_PWM_Stop_DMA(&htim2,TIM_CHANNEL_2);
	

        // STOP DMA:
#if TIM_CH == TIM_CHANNEL_1
        __HAL_TIM_DISABLE_DMA(htim, TIM_DMA_CC1);
        (void) HAL_DMA_Abort_IT(htim->hdma[TIM_DMA_ID_CC1]);
#endif
#if TIM_CH == TIM_CHANNEL_2
        __HAL_TIM_DISABLE_DMA(htim, TIM_DMA_CC2);
        (void) HAL_DMA_Abort_IT(htim->hdma[TIM_DMA_ID_CC2]);
#endif
#if TIM_CH == TIM_CHANNEL_3
        __HAL_TIM_DISABLE_DMA(htim, TIM_DMA_CC3);
        (void) HAL_DMA_Abort_IT(htim->hdma[TIM_DMA_ID_CC3]);
#endif
#if TIM_CH == TIM_CHANNEL_4
        __HAL_TIM_DISABLE_DMA(htim, TIM_DMA_CC4);
        (void) HAL_DMA_Abort_IT(htim->hdma[TIM_DMA_ID_CC4]);
#endif
        if (IS_TIM_BREAK_INSTANCE(htim->Instance) != RESET) {
            /* Disable the Main Output */
            __HAL_TIM_MOE_DISABLE(htim);
        }
        /* Disable the Peripheral */
        __HAL_TIM_DISABLE(htim);
        /* Set the TIM channel state */
        TIM_CHANNEL_STATE_SET(htim, TIM_CH, HAL_TIM_CHANNEL_STATE_READY);
//        ARGB_LOC_ST = ARGB_READY;
 
    htim->Channel = HAL_TIM_ACTIVE_CHANNEL_CLEARED;

}
//------------------------------------------------ 
static void TIM_DMAPulseHalfCplt(DMA_HandleTypeDef *hdma) {
	

}
//------------------------------------------------------------------
void ws2812_light(void)
{
//	HAL_StatusTypeDef DMA_Send_Stat = HAL_ERROR;
	HAL_StatusTypeDef DMA_Send_Stat;
	while (DMA_Send_Stat != HAL_OK) {
			if (TIM_CHANNEL_STATE_GET(&TIM_HANDLE, TIM_CH) == HAL_TIM_CHANNEL_STATE_BUSY) {
					DMA_Send_Stat = HAL_BUSY;
					continue;
			} else if (TIM_CHANNEL_STATE_GET(&TIM_HANDLE, TIM_CH) == HAL_TIM_CHANNEL_STATE_READY) {
					TIM_CHANNEL_STATE_SET(&TIM_HANDLE, TIM_CH, HAL_TIM_CHANNEL_STATE_BUSY);
			} else {
					DMA_Send_Stat = HAL_ERROR;
					continue;
			}
	
#if TIM_CH == TIM_CHANNEL_1
#define RGB_TIM_DMA_ID TIM_DMA_ID_CC1
#define RGB_TIM_DMA_CC TIM_DMA_CC1
#define RGB_TIM_CCR CCR1
#elif TIM_CH == TIM_CHANNEL_2
#define RGB_TIM_DMA_ID TIM_DMA_ID_CC2
#define RGB_TIM_DMA_CC TIM_DMA_CC2
#define RGB_TIM_CCR CCR2
#elif TIM_CH == TIM_CHANNEL_3
#define RGB_TIM_DMA_ID TIM_DMA_ID_CC3
#define RGB_TIM_DMA_CC TIM_DMA_CC3
#define RGB_TIM_CCR CCR3
#elif TIM_CH == TIM_CHANNEL_4
#define RGB_TIM_DMA_ID TIM_DMA_ID_CC4
#define RGB_TIM_DMA_CC TIM_DMA_CC4
#define RGB_TIM_CCR CCR4
#endif
	
	TIM_HANDLE.hdma[RGB_TIM_DMA_ID]->XferCpltCallback = TIM_DMAPulseCplt;
  TIM_HANDLE.hdma[RGB_TIM_DMA_ID]->XferHalfCpltCallback = TIM_DMAPulseHalfCplt;
  TIM_HANDLE.hdma[RGB_TIM_DMA_ID]->XferErrorCallback = TIM_DMAError;
	
	
  HAL_TIM_PWM_Start_DMA(&TIM_HANDLE,TIM_CHANNEL_2,(uint32_t*)&BUF_DMA,ARRAY_LEN);
	
	
			__HAL_TIM_ENABLE_DMA(&TIM_HANDLE, RGB_TIM_DMA_CC);
		if (IS_TIM_BREAK_INSTANCE(TIM_HANDLE.Instance) != RESET)
				__HAL_TIM_MOE_ENABLE(&TIM_HANDLE);
		if (IS_TIM_SLAVE_INSTANCE(TIM_HANDLE.Instance)) {
				uint32_t tmpsmcr = TIM_HANDLE.Instance->SMCR & TIM_SMCR_SMS;
				if (!IS_TIM_SLAVEMODE_TRIGGER_ENABLED(tmpsmcr))
						__HAL_TIM_ENABLE(&TIM_HANDLE);
		} else
				__HAL_TIM_ENABLE(&TIM_HANDLE);
		DMA_Send_Stat = HAL_OK;
	}
}
//----------------------------------------------------------------------------
