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
uint16_t cntBytesStat = 0;
uint16_t strLen = 160;
uint16_t strLenData = 160;
uint8_t fileName[16];
volatile uint16_t Timer1 = 0;
uint8_t sect[512];
#ifndef MATRIX_LIB
uint8_t readBuff[cntRead];
#else
uint32_t FrameBufferLen = 0;
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
//FATFS *fs;


#ifndef MATRIX_LIB
FIL MyFile;
#endif

// structure for all data fields of CANObject
rear_light_can_data_t light_ecu_can_data;
CANManager can_manager(&HAL_GetTick);

// For RGB
extern uint8_t *RGB_BUF;

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

// вызывается, если по CAN пришла команда включения/выключения габаритов
CAN_function_result_t side_beam_set_handler(CANObject &parent_object, CANFunctionBase &parent_function, CANFrame *can_frame)
{
    if (can_frame == nullptr)
        return CAN_RES_NEXT_ERR;

    light_ecu_can_data.side_beam.brightness = can_frame->get_data_pointer()[1];

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

    // если всё хорошо, возвращаем CAN_RES_NEXT_OK (в CAN уйдёт ОК-сообщение)
    // если что-то пошло не так, то возвращаем CAN_RES_NEXT_ERR (в CAN уйдёт сообщение об ошибке)
    // если всё обработали, но ничего в CAN сообщать не хотим, то возвращаем CAN_RES_FINAL
    // если сейчас не можем обработать, то возвращаем CAN_RES_NONE
    return CAN_RES_NEXT_OK;
}

// вызывается, если по CAN пришла команда включения/выключения стоп-сигналов
CAN_function_result_t brake_light_set_handler(CANObject &parent_object, CANFunctionBase &parent_function, CANFrame *can_frame)
{
    if (can_frame == nullptr)
        return CAN_RES_NEXT_ERR;

    light_ecu_can_data.brake_light.brightness = can_frame->get_data_pointer()[1];

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

    // если всё хорошо, возвращаем CAN_RES_NEXT_OK (в CAN уйдёт ОК-сообщение)
    // если что-то пошло не так, то возвращаем CAN_RES_NEXT_ERR (в CAN уйдёт сообщение об ошибке)
    // если всё обработали, но ничего в CAN сообщать не хотим, то возвращаем CAN_RES_FINAL
    // если сейчас не можем обработать, то возвращаем CAN_RES_NONE
    return CAN_RES_NEXT_OK;
}

// вызывается, если по CAN пришла команда включения/выключения заднего хода
CAN_function_result_t reverse_light_set_handler(CANObject &parent_object, CANFunctionBase &parent_function, CANFrame *can_frame)
{
    if (can_frame == nullptr)
        return CAN_RES_NEXT_ERR;

    light_ecu_can_data.reverse_light.brightness = can_frame->get_data_pointer()[1];

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

    // если всё хорошо, возвращаем CAN_RES_NEXT_OK (в CAN уйдёт ОК-сообщение)
    // если что-то пошло не так, то возвращаем CAN_RES_NEXT_ERR (в CAN уйдёт сообщение об ошибке)
    // если всё обработали, но ничего в CAN сообщать не хотим, то возвращаем CAN_RES_FINAL
    // если сейчас не можем обработать, то возвращаем CAN_RES_NONE
    return CAN_RES_NEXT_OK;
}

// вызывается, если по CAN пришла команда включения/выключения левого поворотника
CAN_function_result_t turn_left_set_handler(CANObject &parent_object, CANFunctionBase &parent_function, CANFrame *can_frame)
{
    if (can_frame == nullptr)
        return CAN_RES_NEXT_ERR;

    light_ecu_can_data.left_indicator.brightness = can_frame->get_data_pointer()[1];

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

    // если всё хорошо, возвращаем CAN_RES_NEXT_OK (в CAN уйдёт ОК-сообщение)
    // если что-то пошло не так, то возвращаем CAN_RES_NEXT_ERR (в CAN уйдёт сообщение об ошибке)
    // если всё обработали, но ничего в CAN сообщать не хотим, то возвращаем CAN_RES_FINAL
    // если сейчас не можем обработать, то возвращаем CAN_RES_NONE
    return CAN_RES_NEXT_OK;
}

// вызывается, если по CAN пришла команда включения/выключения правкого поворотника
CAN_function_result_t turn_right_set_handler(CANObject &parent_object, CANFunctionBase &parent_function, CANFrame *can_frame)
{
    if (can_frame == nullptr)
        return CAN_RES_NEXT_ERR;

    light_ecu_can_data.right_indicator.brightness = can_frame->get_data_pointer()[1];

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

    // если всё хорошо, возвращаем CAN_RES_NEXT_OK (в CAN уйдёт ОК-сообщение)
    // если что-то пошло не так, то возвращаем CAN_RES_NEXT_ERR (в CAN уйдёт сообщение об ошибке)
    // если всё обработали, но ничего в CAN сообщать не хотим, то возвращаем CAN_RES_FINAL
    // если сейчас не можем обработать, то возвращаем CAN_RES_NONE
    return CAN_RES_NEXT_OK;
}

// вызывается, если по CAN пришла команда включения/выключения аварийного сигнала
CAN_function_result_t hazard_beam_set_handler(CANObject &parent_object, CANFunctionBase &parent_function, CANFrame *can_frame)
{
    if (can_frame == nullptr)
        return CAN_RES_NEXT_ERR;

    light_ecu_can_data.hazard_beam.brightness = can_frame->get_data_pointer()[1];

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

    // если всё хорошо, возвращаем CAN_RES_NEXT_OK (в CAN уйдёт ОК-сообщение)
    // если что-то пошло не так, то возвращаем CAN_RES_NEXT_ERR (в CAN уйдёт сообщение об ошибке)
    // если всё обработали, но ничего в CAN сообщать не хотим, то возвращаем CAN_RES_FINAL
    // если сейчас не можем обработать, то возвращаем CAN_RES_NONE
    return CAN_RES_NEXT_OK;
}

// вызывается, если по CAN пришла команда включения/выключения пользовательского света
CAN_function_result_t custom_beam_set_handler(CANObject &parent_object, CANFunctionBase &parent_function, CANFrame *can_frame)
{
    if (can_frame == nullptr)
        return CAN_RES_NEXT_ERR;

    light_ecu_can_data.custom_beam.brightness = can_frame->get_data_pointer()[1];

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

    // если всё хорошо, возвращаем CAN_RES_NEXT_OK (в CAN уйдёт ОК-сообщение)
    // если что-то пошло не так, то возвращаем CAN_RES_NEXT_ERR (в CAN уйдёт сообщение об ошибке)
    // если всё обработали, но ничего в CAN сообщать не хотим, то возвращаем CAN_RES_FINAL
    // если сейчас не можем обработать, то возвращаем CAN_RES_NONE
    return CAN_RES_NEXT_OK;
}

// вызывается, если по CAN пришла команда включения/выключения пользовательского изображения на панели
CAN_function_result_t custom_image_set_handler(CANObject &parent_object, CANFunctionBase &parent_function, CANFrame *can_frame)
{
    if (can_frame == nullptr)
        return CAN_RES_NEXT_ERR;

    light_ecu_can_data.custom_image.brightness = can_frame->get_data_pointer()[1];

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

    // если всё хорошо, возвращаем CAN_RES_NEXT_OK (в CAN уйдёт ОК-сообщение)
    // если что-то пошло не так, то возвращаем CAN_RES_NEXT_ERR (в CAN уйдёт сообщение об ошибке)
    // если всё обработали, но ничего в CAN сообщать не хотим, то возвращаем CAN_RES_FINAL
    // если сейчас не можем обработать, то возвращаем CAN_RES_NONE
    return CAN_RES_NEXT_OK;
}

// CAN Send Raw Function Callback
// вызывается при проверке наличия свободного места
// возвращает:
//    true, если для указанного количества байт есть место
//    false, если места нет
bool free_space(uint32_t size_needed)
{
    LOG("callback: free_space()");

    // TODO: в качестве заглушки всегда говорим, что места нет
    // но надо прикрутить функцию из FAT
    return false;
}

// CAN Send Raw Function Callback
// вызывается при создании нового ВРЕМЕННОГО файла для записи
// возвращает:
//    true, если всё успешно
//    false, если не удалось
bool open_tmp_file()
{
    LOG("callback: open_tmp_file()");

    // TODO: в качестве заглушки ничего не делаем и возвращаем "не шмогла"
    // но надо прикрутить функцию из FAT
    return false;
}

// CAN Send Raw Function Callback
// вызывается при записи во временный файл очередного чанка с данными
// возвращает:
//    true, если всё успешно
//    false, если не удалось
bool write_chunk(uint8_t chunk_size, uint8_t *chunk_data)
{
    LOG("callback: write_chunk()");

    // TODO: в качестве заглушки ничего не делаем и возвращаем "не шмогла"
    // но надо прикрутить функцию из FAT
    return false;
}

// CAN Send Raw Function Callback
// вызывается при окончании записи во временный файл
// сюда передаётся цифровой код файла, под которым требуется сохранить этот временный файл
// возвращает:
//    true, если всё успешно
//    false, если не удалось
bool close_file(uint8_t file_code)
{
    LOG("callback: close_file()");

    // TODO: в качестве заглушки ничего не делаем и возвращаем "не шмогла"
    // но надо прикрутить функцию из FAT
    return false;
}

// CAN Send Raw Function Callback
// вызывается, если запись файла была прервана
// нужно удалить временный файл, очистить память и т.п.
// возвращает:
//    true, если всё успешно
//    false, если не удалось (по факту, вызывающему коду на это уже плевать)
bool abort_callback()
{
    LOG("callback: abort_callback()");

    // TODO: в качестве заглушки ничего не делаем и возвращаем "не шмогла"
    // но надо прикрутить функцию из FAT
    return false;
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
        CANFrame new_frame(RxHeader.StdId, &RxData[0], RxHeader.DLC);
        can_manager.take_new_rx_frame(new_frame);

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

void HAL_CAN_Send(CANFrame &can_frame)
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
    uint32_t TxMailbox = 0;
    TxHeader.StdId = can_frame.get_id();
    TxHeader.ExtId = 0;
    TxHeader.RTR = CAN_RTR_DATA; // CAN_RTR_REMOTE
    TxHeader.IDE = CAN_ID_STD;   // CAN_ID_EXT
    TxHeader.DLC = can_frame.get_data_length();
    TxHeader.TransmitGlobalTime = DISABLE;

    while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0)
        ;

    can_frame.copy_frame_data_to(TxData, 8);

    if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK)
    {
        LOG("CAN TX ERROR: 0x%04lX", TxHeader.StdId);
    }
}

void CAN_Send_All_Frames(CANManager &can_manager)
{
    CANFrame can_frame;
    while (can_manager.has_tx_frames_for_transmission())
    {
        can_manager.give_tx_frame(can_frame);
        HAL_CAN_Send(can_frame);
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

	//f_mount(&SDFatFs, "", 0);
	
    if(f_mount(&SDFatFs, "", 1) != FR_OK)
    {
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
    //init_can_manager(can_manager, light_ecu_can_data);

    uint32_t can_manager_last_tick = HAL_GetTick();
    CANFrame can_frame;
    uint8_t can_frame_data[8];
    while (1)
    {
        current_time = HAL_GetTick();

        // this function is still empty
        // update_block_can_data(light_ecu_can_data);

#ifdef MATRIX_LIB
        Matrix::Loop(current_time);
#endif

        // CAN Manager checks data every 300 ms
        if (current_time - can_manager_last_tick > 300)
        {
            // do all stuff and process RX frames
            can_manager.process();

            // send TX frames if there are any
            if (can_manager.has_tx_frames_for_transmission())
                CAN_Send_All_Frames(can_manager);

            can_manager_last_tick = HAL_GetTick();
        }

        if (Button1 == 0)
        {
            // do something here
        }
        if (Button2 == 0)
        {
            can_frame_data[0] = 0x44;
            can_frame_data[1] = 0x53;
            can_frame_data[2] = 0x46;
            can_frame_data[3] = 0x30;
            can_frame_data[4] = 0x30;
            can_frame_data[5] = 0x30;
            can_frame_data[6] = 0x32;
            can_frame_data[7] = 0x00;
            can_frame.set_frame(0x07B0, can_frame_data, 8);
        }
        if (can_frame.is_initialized())
        {
            HAL_CAN_Send(can_frame);
            can_frame.clear_frame();
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
