/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */

#include "main.h"
#include "fatfs.h"

#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include "sd.h"
#include <SerialUtils.h>
#include <MatrixLogic.h>
#include <CANLibrary.h>
#include "can_abstarction.h"

#define Button1 HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8)
#define Button2 HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9)

#define LedGreen_ON HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);     // Вкл
#define LedGreen_OFF HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);  // Выкл
#define LedBlue_ON HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);      // Вкл
#define LedBlue_OFF HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);   // Выкл
#define LedRed_ON HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);       // Вкл
#define LedRed_OFF HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);    // Выкл
#define LedYellow_ON HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);    // Вкл
#define LedYellow_OFF HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET); // Выкл

// SideBeam (габариты), layer 2 of the Matrix
#define OUT1_ON HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);    // Вкл
#define OUT1_OFF HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET); // Выкл

// BrakeLight (стопы), layer 4 of the Matrix
#define OUT2_ON HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);    // Вкл
#define OUT2_OFF HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET); // Выкл

// ReverseLight (задний ход), layer 3 of the Matrix
#define OUT3_ON HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);    // Вкл
#define OUT3_OFF HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET); // Выкл

// LeftIndicator (левый поворотник), layer 5 of the Matrix
#define OUT4_ON HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);    // Вкл
#define OUT4_OFF HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET); // Выкл

// RightIndicator (правый поворотник), layer 6 of the Matrix
#define OUT5_ON HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);    // Вкл
#define OUT5_OFF HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET); // Выкл

// CustomBeam (пользовательский свет), NO Matrix layer!
#define OUT6_ON HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);    // Вкл
#define OUT6_OFF HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET); // Выкл

#define cntRead 1024 // 6200

#define ARGB_LIB
#define MATRIX_LIB

#define ClearBit(reg, bit) reg &= (~(1 << (bit)))        // пример: ClearBit(PORTB, 1); //сбросить 1-й бит PORTB
#define SetBit(reg, bit) reg |= (1 << (bit))             // пример: SetBit(PORTB, 3); //установить 3-й бит PORTB
#define BitIsClear(reg, bit) ((reg & (1 << (bit))) == 0) // пример: if (BitIsClear(PORTB,1)) {...} //если бит очищен
#define BitIsSet(reg, bit) ((reg & (1 << (bit))) != 0)   // пример: if(BitIsSet(PORTB,2)) {...} //если бит установлен

#define DELAY_VAL 100

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
DMA_HandleTypeDef hdma_tim2_ch1;

UART_HandleTypeDef huart1;

// For SD
volatile uint16_t Timer1 = 0;
#ifndef MATRIX_LIB
uint16_t cntBytesStat = 0;
uint8_t sect[512];
uint8_t readBuff[cntRead];
#endif

/*
UINT cntReadBytes;
extern char str1[60];
uint32_t byteswritten, bytesread;
uint8_t result;
uint8_t readCAN = 0;

FILINFO fileInfo;
char *fn;
DIR dir;
uint8_t resultF = 0;
FRESULT res; // результат выполнения
DWORD fre_clust, fre_sect, tot_sect;
*/
FATFS SDFatFs;
// FATFS *fs;

#ifndef MATRIX_LIB
FIL MyFile;
#endif

// structure for all data fields of CANObject
rear_light_can_data_t light_ecu_can_data;
// HAL_CAN_Send forward declaration
void HAL_CAN_Send(can_object_id_t id, uint8_t *data, uint8_t length);
CANManager<12> can_manager(&HAL_CAN_Send);

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

struct __attribute__((__packed__)) raw_can_frame_t
{
    can_frame_t can_frame;
    can_object_id_t object_id;
};

#define FRAME_BUFFER_SIZE 64
raw_can_frame_t frame_buffer[FRAME_BUFFER_SIZE];

uint8_t frame_buffer_index = 0;
uint8_t frames_count = 0;

// For RGB
// extern uint8_t *RGB_BUF;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_CAN_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);
/* Private function prototypes -----------------------------------------------*/

void init_can_manager_and_objects()
{
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
}

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
        OUT1_OFF;
        Matrix::matrix.HideLayer(2);
    }
    else
    {
        OUT1_ON;
        Matrix::matrix.ShowLayer(2);
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
        OUT2_OFF;
        Matrix::matrix.HideLayer(4);
    }
    else
    {
        OUT2_ON;
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
        OUT3_OFF;
        Matrix::matrix.HideLayer(3);
    }
    else
    {
        OUT3_ON;
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
        OUT4_OFF;
        Matrix::matrix.HideLayer(5);
    }
    else
    {
        OUT4_ON;
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
        OUT5_OFF;
        Matrix::matrix.HideLayer(6);
    }
    else
    {
        OUT5_ON;
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
        OUT4_OFF;
        OUT5_OFF;
        Matrix::matrix.HideLayer(5);
        Matrix::matrix.HideLayer(6);
    }
    else
    {
        OUT4_ON;
        OUT5_ON;
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
        OUT6_OFF;
    }
    else
    {
        OUT6_ON;
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

/*
    Колбек для приёма данных (для буфера RX_FIFO_0)
    При приёме любого кадра мы тут же забираем его из почтового ящика с помощью функции HAL_CAN_GetRxMessage(...)
    и мигаем светиком.
    ВАЖНО!!!!!!  для приема нужно в MX_CAN_Init() прописать настройки приема
*/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[8] = {0};

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
    {

        memcpy(frame_buffer[frame_buffer_index].can_frame.raw_data, RxData, RxHeader.DLC);
        frame_buffer[frame_buffer_index].can_frame.raw_data_length = RxHeader.DLC;
        frame_buffer[frame_buffer_index].can_frame.initialized = true;
        frame_buffer[frame_buffer_index].object_id = RxHeader.StdId & 0xFFFF;
        frame_buffer_index++;
        frames_count++;
        if (frame_buffer_index >= FRAME_BUFFER_SIZE)
            frame_buffer_index = 0;

        // can_manager.IncomingCANFrame(RxHeader.StdId, RxData, RxHeader.DLC);

        LOG("RX: CAN 0x%04lX", RxHeader.StdId);
    }
}

/*
    Колбек для ошибок
*/
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
    uint32_t er = HAL_CAN_GetError(hcan);
    LOG("CAN ERROR: %lu %08lX", (unsigned long)er, (unsigned long)er);
}

void HAL_CAN_Send(can_object_id_t id, uint8_t *data, uint8_t length)
{
    /*
      Заполняем структуру отвечающую за отправку кадров
      StdId — это идентификатор стандартного кадра.
      ExtId — это идентификатор расширенного кадра. Мы будем отправлять стандартный поэтому сюда пишем 0.
      RTR =
          CAN_RTR_DATA    — отправляем кадр с данными (Data Frame)
          CAN_RTR_REMOTE  — отправляем Remote Frame.
      IDE =
          CAN_ID_STD — отправляем стандартный кадр.
          CAN_ID_EXT — расширенный кадр. В этом случае в StdId нужно будет указать 0,
                       а в ExtId записать расширенный идентификатор.
      DLC = 8 — количество полезных байт передаваемых в кадре (от 1 до 8).
      TransmitGlobalTime — относится к Time Triggered Communication Mode, мы это не используем поэтому пишем 0.
    */
    CAN_TxHeaderTypeDef TxHeader;
    uint8_t TxData[8] = {0};
    memcpy(TxData, data, length);
    uint32_t TxMailbox = 0;
    TxHeader.StdId = id;
    TxHeader.ExtId = 0;
    TxHeader.RTR = CAN_RTR_DATA; // CAN_RTR_REMOTE
    TxHeader.IDE = CAN_ID_STD;   // CAN_ID_EXT
    TxHeader.DLC = length;
    TxHeader.TransmitGlobalTime = DISABLE;

    while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0)
    {
        LedYellow_ON
    }
    LedYellow_OFF

    if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK)
    {
        LOG("CAN TX ERROR: 0x%04lX", TxHeader.StdId);
    }
}

//========================== for SD Card:
#ifndef MATRIX_LIB

FRESULT ReadFile(void)
{
    uint16_t i = 0, i1 = 0;
    uint32_t ind = 0;
    uint32_t f_size = MyFile.fsize;
    cntBytesStat = 0;

    ind = 0;
    do
    {
        if (f_size < 512)
        {
            i1 = f_size;
        }
        else
        {
            i1 = 512;
        }
        f_size -= i1;
        f_lseek(&MyFile, ind);
        f_read(&MyFile, sect, i1, (UINT *)&bytesread); //( , Указатель на буфер данных, кол-во байт для чтения, Указатель на кол-во прочитанных байтов) чтение по 512 байт

        for (i = 0; i < bytesread; i++)
        {
            //			readBuff[cntBytesStat] = *sect+i;
            readBuff[cntBytesStat] = sect[i];
            cntBytesStat++;
        }
        ind += i1;
    } while (f_size > 0);

    return FR_OK;
}

FRESULT ReadFileOUT(void)
{
    uint16_t i = 0, i1 = 0, it = 0;
    uint32_t ind = 0;
    uint32_t f_size = MyFile.fsize;
    cntBytesStat = 0;           // общее количество прочитанных байт (глобальная переменная)
    uint16_t cntBytesFrame = 0; //
    uint16_t k = 0;

    ind = 0;
    do
    {
        if (f_size < 512)
        {
            i1 = f_size;
        }
        else
        {
            i1 = 512;
        }
        f_size -= i1;
        f_lseek(&MyFile, ind);
        f_read(&MyFile, sect, i1, (UINT *)&bytesread); //( , Указатель на буфер данных, кол-во байт для чтения, Указатель на кол-во прочитанных байтов) чтение по 512 байт

        // переносим считанные байты из файла в буфер

        if (cntBytesStat == 0)
        { // первый сектор с инфой 1байт
            for (i = 1; i < bytesread; i += 3)
            {
                ARGB_SetRGB(k, sect[i + 2], sect[i + 1], sect[i]); // Set LED № with R, G, B
                k++;
                cntBytesStat += 3;
                cntBytesFrame += 3;
            }
        }
        else
        { // следующие сектора  с чистыми данными
            for (it = 0; it < bytesread; it += 3)
            {
                ARGB_SetRGB(k, sect[it + 2], sect[it + 1], sect[it]); // Set LED № with R, G, B
                k++;
                cntBytesStat += 3;
                cntBytesFrame += 3;
                // if read frame
                if (cntBytesFrame >= 6143)
                {
                    cntBytesFrame = 0;
                    k = 0;
                    it++;
                    while (!RGB_Show())
                        ;
                }
            }
        }
        ind += i1;
    } while (f_size > 0);

    return FR_OK;
}

FRESULT ReadFilePXL(void)
{
    uint16_t i = 0, i1 = 0, it = 0;
    uint32_t ind = 0;
    uint32_t f_size = MyFile.fsize;
    cntBytesStat = 0;           // общее количество прочитанных байт (глобальная переменная)
    uint16_t cntBytesFrame = 0; //
    uint16_t k = 0;

    ind = 0;
    do
    {
        if (f_size < 512)
        {
            i1 = f_size;
        }
        else
        {
            i1 = 512;
        }
        f_size -= i1;
        f_lseek(&MyFile, ind);
        f_read(&MyFile, sect, i1, (UINT *)&bytesread); //( , Указатель на буфер данных, кол-во байт для чтения, Указатель на кол-во прочитанных байтов) чтение по 512 байт

        // переносим считанные байты из файла в буфер

        if (cntBytesStat == 0)
        { // первый сектор с инфой 1байт
            for (i = 1; i < bytesread; i += 3)
            {
                RGB_SetRGB(k, sect[i + 2], sect[i + 1], sect[i], (u8_t *)&buffer); // Set LED № with R, G, B
                k++;
                cntBytesStat += 3;
                cntBytesFrame += 3;
            }
        }
        else
        { // следующие сектора  с чистыми данными
            for (it = 0; it < bytesread; it += 3)
            {
                RGB_SetRGB(k, sect[it + 2], sect[it + 1], sect[it], (u8_t *)&buffer); // Set LED № with R, G, B
                k++;
                cntBytesStat += 3;
                cntBytesFrame += 3;
                // if read frame
                if (cntBytesFrame >= 6143)
                {
                    cntBytesFrame = 0;
                    k = 0;
                    it++;
                    while (!RGB_Show())
                        ;
                }
            }
        }
        ind += i1;
    } while (f_size > 0);

    return FR_OK;
}
#endif

void InitFlash(void)
{
    disk_initialize(SDFatFs.drv);

    // f_mount(&SDFatFs, "", 0);

    if (f_mount(&SDFatFs, "", 1) != FR_OK)
    {
        LedRed_ON
        //		Error_Handler();
        // HAL_UART_Transmit(&huart1,(uint8_t*)"mount error",12,0x1000);
    }
    /*
    else
    {
        fileInfo.lfname = (char *)sect;
        fileInfo.lfsize = sizeof(sect);
        result = f_opendir(&dir, "/");
        if (result == FR_OK)
        {
            while (1)
            {
                result = f_readdir(&dir, &fileInfo); // 4200 байт
                if (result == FR_OK && fileInfo.fname[0])
                {
                    fn = fileInfo.lfname;
                    if (strlen(fn))
                    {
                        // HAL_UART_Transmit(&huart1,(uint8_t*)fn,strlen(fn),0x1000);
                    }
                    else
                    {
                        // HAL_UART_Transmit(&huart1,(uint8_t*)fileInfo.fname,strlen((char*)fileInfo.fname),0x1000);

                        //						BMPtoBIN();
                        //						PtoBIN();
                    }
                    if (fileInfo.fattrib & AM_DIR)
                    {
                        // HAL_UART_Transmit(&huart1,(uint8_t*)"  [DIR]",7,0x1000);
                    }
                }
                else
                    break;
                // HAL_UART_Transmit(&huart1,(uint8_t*)"\r\n",2,0x1000);
            }
            f_closedir(&dir);
        }
    }*/
}

uint32_t current_time = 0;

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* Configure the system clock */
    SystemClock_Config();

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_TIM2_Init();
    MX_CAN_Init();
    MX_SPI2_Init();
    MX_FATFS_Init();
    MX_USART1_UART_Init();
    MX_TIM1_Init();
    MX_ADC1_Init();

    HAL_TIM_Base_Start_IT(&htim1);

    OUT1_OFF;
    OUT2_OFF;
    OUT3_OFF;
    OUT4_OFF;
    OUT5_OFF;
    OUT6_OFF;

    LedGreen_OFF;
    LedBlue_OFF;
    LedRed_OFF;
    LedYellow_OFF;

    LedRed_ON;
    HAL_Delay(DELAY_VAL);
    LedRed_OFF;

    LedYellow_ON;
    HAL_Delay(DELAY_VAL);
    LedYellow_OFF;

    LedGreen_ON;
    HAL_Delay(DELAY_VAL);
    LedGreen_OFF;

    LedBlue_ON;
    HAL_Delay(DELAY_VAL);
    LedBlue_OFF;

    /* активируем события которые будут вызывать прерывания  */
    HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_ERROR | CAN_IT_BUSOFF | CAN_IT_LAST_ERROR_CODE);

    HAL_CAN_Start(&hcan);

#ifndef MATRIX_LIB
#ifdef ARGB_LIB
    ARGB_Init();            // Initialization
    ARGB_SetBrightness(50); // Set global brightness to 30%
    ARGB_FillRGB(0, 20, 0); // Fill all the strip with Red
                            //	ARGB_Clear(); // Clear stirp
    while (RGB_Show() != ARGB_OK)
        ; // Update - Option 1
#else
    ws2812_init();
#endif
#endif

    InitFlash();

#ifdef MATRIX_LIB
    Matrix::Setup();
#endif

    // set CAN data structure to zero
    memset(&light_ecu_can_data, 0, sizeof(light_ecu_can_data));
    // init CANManager
    init_can_manager_and_objects();

    uint32_t can_manager_last_tick = HAL_GetTick();
    while (1)
    {
        current_time = HAL_GetTick();

        // this function is still empty
        // update_block_can_data(light_ecu_can_data);

#ifdef MATRIX_LIB
        Matrix::Loop(current_time);
#endif

        // CAN Manager checks data every 300 ms
        if (current_time - can_manager_last_tick > 100)
        {
            can_manager.Process(current_time);

            can_manager_last_tick = HAL_GetTick();
        }

        if (frames_count > 0)
        {
            for (uint8_t i = 0; i < FRAME_BUFFER_SIZE; i++)
            {
                if (!frame_buffer[i].can_frame.initialized)
                    continue;
                
                can_manager.IncomingCANFrame(frame_buffer[i].object_id, frame_buffer[i].can_frame.raw_data, frame_buffer[i].can_frame.raw_data_length);
                frame_buffer[i].can_frame.initialized = false;
                frames_count--;
            }
        }

        if (Button1 == 0)
        {
            // do something here
        }
        if (Button2 == 0)
        {
            // do something here
        }
    }
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL8;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }
    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        Error_Handler();
    }
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
    PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void)
{

    /* USER CODE BEGIN ADC1_Init 0 */

    /* USER CODE END ADC1_Init 0 */

    ADC_ChannelConfTypeDef sConfig = {0};

    /* USER CODE BEGIN ADC1_Init 1 */

    /* USER CODE END ADC1_Init 1 */
    /** Common config
     */
    hadc1.Instance = ADC1;
    hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 1;
    if (HAL_ADC_Init(&hadc1) != HAL_OK)
    {
        Error_Handler();
    }
    /** Configure Regular Channel
     */
    sConfig.Channel = ADC_CHANNEL_1;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN ADC1_Init 2 */

    /* USER CODE END ADC1_Init 2 */
}

/**
 * @brief CAN Initialization Function
 * @param None
 * @retval None
 */
static void MX_CAN_Init(void)
{

    /* USER CODE BEGIN CAN_Init 0 */
    CAN_FilterTypeDef sFilterConfig;
    /* USER CODE END CAN_Init 0 */

    /* USER CODE BEGIN CAN_Init 1 */

    /* USER CODE END CAN_Init 1 */
    hcan.Instance = CAN1;
    hcan.Init.Prescaler = 4;
    hcan.Init.Mode = CAN_MODE_NORMAL;
    hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
    hcan.Init.TimeSeg1 = CAN_BS1_13TQ;
    hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
    hcan.Init.TimeTriggeredMode = DISABLE;
    hcan.Init.AutoBusOff = DISABLE;
    hcan.Init.AutoWakeUp = DISABLE;
    hcan.Init.AutoRetransmission = DISABLE;
    hcan.Init.ReceiveFifoLocked = DISABLE;
    hcan.Init.TransmitFifoPriority = DISABLE;
    if (HAL_CAN_Init(&hcan) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN CAN_Init 2 */
    sFilterConfig.FilterBank = 0;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterIdHigh = 0x0000;
    sFilterConfig.FilterIdLow = 0x0000;
    sFilterConfig.FilterMaskIdHigh = 0x0000;
    sFilterConfig.FilterMaskIdLow = 0x0000;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    sFilterConfig.FilterActivation = ENABLE;
    // sFilterConfig.SlaveStartFilterBank = 14;

    if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE END CAN_Init 2 */
}

/**
 * @brief SPI2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI2_Init(void)
{

    /* USER CODE BEGIN SPI2_Init 0 */

    /* USER CODE END SPI2_Init 0 */

    /* USER CODE BEGIN SPI2_Init 1 */

    /* USER CODE END SPI2_Init 1 */
    /* SPI2 parameter configuration*/
    hspi2.Instance = SPI2;
    hspi2.Init.Mode = SPI_MODE_MASTER;
    hspi2.Init.Direction = SPI_DIRECTION_2LINES;
    hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi2.Init.NSS = SPI_NSS_HARD_OUTPUT;
    hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
    hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi2.Init.CRCPolynomial = 10;
    if (HAL_SPI_Init(&hspi2) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN SPI2_Init 2 */

    /* USER CODE END SPI2_Init 2 */
}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void)
{

    /* USER CODE BEGIN TIM1_Init 0 */

    /* USER CODE END TIM1_Init 0 */

    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    /* USER CODE BEGIN TIM1_Init 1 */

    /* USER CODE END TIM1_Init 1 */
    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 63999;
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = 10;
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
    {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
    {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM1_Init 2 */

    /* USER CODE END TIM1_Init 2 */
}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void)
{

    /* USER CODE BEGIN TIM2_Init 0 */

    /* USER CODE END TIM2_Init 0 */

    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};

    /* USER CODE BEGIN TIM2_Init 1 */

    /* USER CODE END TIM2_Init 1 */
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 0;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 89;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
    {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
    {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM2_Init 2 */

    /* USER CODE END TIM2_Init 2 */
    HAL_TIM_MspPostInit(&htim2);
}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void)
{

    /* USER CODE BEGIN USART1_Init 0 */

    /* USER CODE END USART1_Init 0 */

    /* USER CODE BEGIN USART1_Init 1 */

    /* USER CODE END USART1_Init 1 */
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 500000;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart1) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN USART1_Init 2 */

    /* USER CODE END USART1_Init 2 */
}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void)
{

    /* DMA controller clock enable */
    __HAL_RCC_DMA1_CLK_ENABLE();

    /* DMA interrupt init */
    /* DMA1_Channel5_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_15, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6, GPIO_PIN_RESET);

    /*Configure GPIO pins : PC13 PC14 PC15 */
    GPIO_InitStruct.Pin = GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /*Configure GPIO pins : PA7 PA15 */
    GPIO_InitStruct.Pin = GPIO_PIN_7 | GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pins : PB0 PB1 PB2 PB10
                             PB11 PB12 PB3 PB4
                             PB5 PB6 */
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /*Configure GPIO pin : PA8 */
    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pin : PB7 */
    GPIO_InitStruct.Pin = GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /*Configure GPIO pins : PB8 PB9 */
    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim1)
    {
        Timer1++;
    }
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    LedGreen_ON
    while (1)
    {
    }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
