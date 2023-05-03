#ifndef CAN_ABSTRACTION_H
#define CAN_ABSTRACTION_H

#include <stdint.h>
#include <stdlib.h>
#include <assert.h>
#include <cstring>

#include <CANLibrary.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define CAN_SEND_RAW_FUNCTION_BUFFER_SIZE 512

    // ***************************************************************************************************
    // Rear Light CAN related types and constants
    enum rear_light_can_object_id_t : uint16_t
    {
        // 0x00E0	BlockInfo
        // request | timer:15000
        // byte	1 + 7	{ type[0] data[1..7] }
        // Основная информация о блоке. См. "Системные параметры".
        REAR_LIGHT_CANO_ID_BLOCK_INFO = 0x00E0,

        // 0x00E1	BlockHealth
        // request | event
        // byte	1 + 7	{ type[0] data[1..7] }
        // Информация о здоровье блока. См. "Системные параметры".
        REAR_LIGHT_CANO_ID_BLOCK_HEALTH = 0x00E1,

        // 0x00E2	BlockCfg
        // request
        // byte	1 + 1 + X	{ type[0] param[1] data[2..7] }
        // Чтение и запись настроек блока. См. "Системные параметры".
        REAR_LIGHT_CANO_ID_BLOCK_CFG = 0x00E2,

        // 0x00E3	BlockError
        // request | event
        // byte	1 + X	{ type[0] data[1..7] }
        // Ошибки блока. См. "Системные параметры".
        REAR_LIGHT_CANO_ID_BLOCK_ERROR = 0x00E3,

        // 0x00E4	SideBeam
        // set | request | event
        // uint8_t	0..255	1 + 1	{ type[0] val[1] }
        // Управление габаритами.
        REAR_LIGHT_CANO_ID_SIDE_BEAM = 0x00E4,

        // 0x00E5	BrakeLight
        // set | request | event
        // uint8_t	0..255	1 + 1	{ type[0] val[1] }
        // Управление стоп-сигналами.
        REAR_LIGHT_CANO_ID_BRAKE_LIGHT = 0x00E5,

        // 0x00E6	ReverseLight
        // set | request | event
        // uint8_t	0..255	1 + 1	{ type[0] val[1] }
        // Управление задним ходом.
        REAR_LIGHT_CANO_ID_REVERSE_LIGHT = 0x00E6,

        // 0x00E7	LeftIndicator
        // set | request | event
        // uint8_t	0..255	1 + 1	{ type[0] val[1] }
        // Управление левым поворотником.
        REAR_LIGHT_CANO_ID_LEFT_INDICATOR = 0x00E7,

        // 0x00E8	RightIndicator
        // set | request | event
        // uint8_t	0..255	1 + 1	{ type[0] val[1] }
        // Управление правым поворотником.
        REAR_LIGHT_CANO_ID_RIGHT_INDICATOR = 0x00E8,

        // 0x00E9	HazardBeam
        // set | request | event
        // uint8_t	0..255	1 + 1	{ type[0] val[1] }
        // Управление аварийным сигналом.
        REAR_LIGHT_CANO_ID_HAZARD_BEAM = 0x00E9,

        // 0x00EA	CustomBeam
        // set | request | event
        // uint8_t	0..255	1 + 1	{ type[0] val[1] }
        // Управление пользовательским светом.
        REAR_LIGHT_CANO_ID_CUSTOM_BEAM = 0x00EA,

        // 0x00EB	CustomImage
        // set | request | event
        // uint8_t	0..255	1 + 1	{ type[0] val[1] }
        // Управление пользовательскими изображениями ( для WS2812b ).
        REAR_LIGHT_CANO_ID_CUSTOM_IMAGE = 0x00EB,

        // 0x00EC	ImageTransfer
        // send raw
        // Link 1 + X	{ type[0] data[1..7] }
        // Для передачи изображений
        REAR_LIGHT_CANO_ID_IMAGE_TRANSFER = 0x00EC,
    };

    // 0x00E0	BlockInfo
    typedef block_info_t rear_light_block_info_t;

    // 0x00E1	BlockHealth
    typedef block_health_t rear_light_block_health_t;

    // 0x00E2	BlockCfg
    typedef block_cfg_t rear_light_block_cfg_t;

    // 0x00E3	BlockError
    typedef block_error_t rear_light_block_error_t;

    // 0x00E4	SideBeam
    struct __attribute__((__packed__)) rear_light_side_beam_t
    {
        // byte 1
        uint8_t brightness;
    };

    // 0x00E5	BrakeLight
    struct __attribute__((__packed__)) rear_light_brake_light_t
    {
        // byte 1
        uint8_t brightness;
    };

    // 0x00E6	ReverseLight
    struct __attribute__((__packed__)) rear_light_reverse_lite_t
    {
        // byte 1
        uint8_t brightness;
    };

    // 0x00E7	LeftIndicator
    struct __attribute__((__packed__)) rear_light_left_indicator_t
    {
        // byte 1
        uint8_t brightness;
    };

    // 0x00E8	RightIndicator
    struct __attribute__((__packed__)) rear_light_right_indicator_t
    {
        // byte 1
        uint8_t brightness;
    };

    // 0x00E9	HazardBeam
    struct __attribute__((__packed__)) rear_light_hazard_beam_t
    {
        // byte 1
        uint8_t brightness;
    };

    // 0x00EA	CustomBeam
    struct __attribute__((__packed__)) rear_light_custom_beam_t
    {
        // byte 1
        uint8_t brightness;
    };

    // 0x00EB	CustomImage
    struct __attribute__((__packed__)) rear_light_custom_image_t
    {
        // byte 1
        uint8_t brightness;
    };

    // 0x00EC	ImageTransfer
    struct __attribute__((__packed__)) rear_light_image_transfer_t
    {
        // byte 1
        uint8_t chunk[CAN_SEND_RAW_FUNCTION_BUFFER_SIZE];
        // TODO: check it after initial configuratiion comletion
    };

    // ************ Block data ************
    struct __attribute__((__packed__)) rear_light_can_data_t
    {
        // 0x00E0	BlockInfo
        rear_light_block_info_t block_info;

        // 0x00E1	BlockHealth
        rear_light_block_health_t block_health;

        // 0x00E2	BlockCfg
        rear_light_block_cfg_t block_cfg;

        // 0x00E3	BlockError
        rear_light_block_error_t block_error;

        // 0x00E4	SideBeam
        rear_light_side_beam_t side_beam;

        // 0x00E5	BrakeLight
        rear_light_brake_light_t brake_light;

        // 0x00E6	ReverseLight
        rear_light_reverse_lite_t reverse_light;

        // 0x00E7	LeftIndicator
        rear_light_left_indicator_t left_indicator;

        // 0x00E8	RightIndicator
        rear_light_right_indicator_t right_indicator;

        // 0x00E9	HazardBeam
        rear_light_hazard_beam_t hazard_beam;

        // 0x00EA	CustomBeam
        rear_light_custom_beam_t custom_beam;

        // 0x00EB	CustomImage
        rear_light_custom_image_t custom_image;

        // 0x00EC	ImageTransfer
        rear_light_image_transfer_t image_transfer;
    };

    // ***************************************************************************************************
    // Rear Light ECU initialization and data conversion functions
    void update_block_can_data(rear_light_can_data_t &light_ecu_can_data);
    bool init_can_manager(CANManager &cm, rear_light_can_data_t &light_ecu_can_data);

    // external callbacks, should be implemented in main.cpp
    CAN_function_result_t side_beam_set_handler(CANObject &parent_object, CANFunctionBase &parent_function, CANFrame *can_frame);
    CAN_function_result_t brake_light_set_handler(CANObject &parent_object, CANFunctionBase &parent_function, CANFrame *can_frame);
    CAN_function_result_t reverse_light_set_handler(CANObject &parent_object, CANFunctionBase &parent_function, CANFrame *can_frame);
    CAN_function_result_t turn_left_set_handler(CANObject &parent_object, CANFunctionBase &parent_function, CANFrame *can_frame);
    CAN_function_result_t turn_right_set_handler(CANObject &parent_object, CANFunctionBase &parent_function, CANFrame *can_frame);
    CAN_function_result_t hazard_beam_set_handler(CANObject &parent_object, CANFunctionBase &parent_function, CANFrame *can_frame);
    CAN_function_result_t custom_beam_set_handler(CANObject &parent_object, CANFunctionBase &parent_function, CANFrame *can_frame);
    CAN_function_result_t custom_image_set_handler(CANObject &parent_object, CANFunctionBase &parent_function, CANFrame *can_frame);
    bool free_space(uint32_t size_needed);
    bool open_tmp_file();
    bool write_chunk(uint8_t chunk_size, uint8_t *chunk_data);
    bool close_file(uint8_t file_code);
    bool abort_callback();

#ifdef __cplusplus
}
#endif

#endif // CAN_ABSTRACTION_H