#include "can_abstarction.h"

void update_block_can_data(rear_light_can_data_t &light_ecu_can_data)
{
    // TODO: do something here
    // here we should copy data from external variables to the LightECU structure
    // It is the intemediate step. In the future it will be better to work with LightECU structure directly
    // in the hardware dependent part of code
}

bool init_can_manager(CANManager &cm, rear_light_can_data_t &light_ecu_can_data)
{
    CANObject *co = nullptr;

    // 0x00E0	BlockInfo
    // request | timer:15000
    // byte	1 + 7	{ type[0] data[1..7] }
    // Основная информация о блоке. См. "Системные параметры".
    init_block_info(cm, REAR_LIGHT_CANO_ID_BLOCK_INFO, light_ecu_can_data.block_info);

    // 0x00E1	BlockHealth
    // request | event
    // byte	1 + 7	{ type[0] data[1..7] }
    // Информация о здоровье блока. См. "Системные параметры".
    init_block_health(cm, REAR_LIGHT_CANO_ID_BLOCK_HEALTH, light_ecu_can_data.block_health);

    // 0x00E2	BlockCfg
    // request
    // byte	1 + 1 + X	{ type[0] param[1] data[2..7] }
    // Чтение и запись настроек блока. См. "Системные параметры".
    init_block_cfg(cm, REAR_LIGHT_CANO_ID_BLOCK_CFG, light_ecu_can_data.block_cfg);

    // 0x00E3	BlockError
    // request | event
    // byte	1 + X	{ type[0] data[1..7] }
    // Ошибки блока. См. "Системные параметры".
    init_block_error(cm, REAR_LIGHT_CANO_ID_BLOCK_ERROR, light_ecu_can_data.block_error);

    // 0x00E4	SideBeam
    // set | request | event
    // uint8_t	0..255	1 + 1	{ type[0] val[1] }
    // Управление габаритами.
    co = init_common_light_can_object(cm, REAR_LIGHT_CANO_ID_SIDE_BEAM, "SideBeam",
                                      light_ecu_can_data.side_beam.brightness,
                                      &side_beam_set_handler, 3000);

    // 0x00E5	BrakeLight
    // set | request | event
    // uint8_t	0..255	1 + 1	{ type[0] val[1] }
    // Управление стоп-сигналами.
    co = init_common_light_can_object(cm, REAR_LIGHT_CANO_ID_BRAKE_LIGHT, "BrakeLight",
                                      light_ecu_can_data.brake_light.brightness,
                                      &brake_light_set_handler, 3000);

    // 0x00E6	ReverseLight
    // set | request | event
    // uint8_t	0..255	1 + 1	{ type[0] val[1] }
    // Управление задним ходом.
    co = init_common_light_can_object(cm, REAR_LIGHT_CANO_ID_REVERSE_LIGHT, "ReverseLight",
                                      light_ecu_can_data.reverse_light.brightness,
                                      &reverse_light_set_handler, 3000);

    // 0x00E7	LeftIndicator
    // set | request | event
    // uint8_t	0..255	1 + 1	{ type[0] val[1] }
    // Управление левым поворотником.
    co = init_common_light_can_object(cm, REAR_LIGHT_CANO_ID_LEFT_INDICATOR, "LeftIndicator",
                                      light_ecu_can_data.left_indicator.brightness,
                                      &turn_left_set_handler, 3000);

    // 0x00E8	RightIndicator
    // set | request | event
    // uint8_t	0..255	1 + 1	{ type[0] val[1] }
    // Управление правым поворотником.
    co = init_common_light_can_object(cm, REAR_LIGHT_CANO_ID_RIGHT_INDICATOR, "RightIndicator",
                                      light_ecu_can_data.right_indicator.brightness,
                                      &turn_right_set_handler, 3000);

    // 0x00E9	HazardBeam
    // set | request | event
    // uint8_t	0..255	1 + 1	{ type[0] val[1] }
    // Управление аварийным сигналом.
    co = init_common_light_can_object(cm, REAR_LIGHT_CANO_ID_HAZARD_BEAM, "HazardBeam",
                                      light_ecu_can_data.hazard_beam.brightness,
                                      &hazard_beam_set_handler, 3000);

    // 0x00EA	CustomBeam
    // set | request | event
    // uint8_t	0..255	1 + 1	{ type[0] val[1] }
    // Управление пользовательским светом.
    co = init_common_light_can_object(cm, REAR_LIGHT_CANO_ID_CUSTOM_BEAM, "CustomBeam",
                                      light_ecu_can_data.custom_beam.brightness,
                                      &custom_beam_set_handler, 3000);

    // 0x00EB	CustomImage
    // set | request | event
    // uint8_t	0..255	1 + 1	{ type[0] val[1] }
    // Управление пользовательскими изображениями ( для WS2812b ).
    co = init_common_light_can_object(cm, REAR_LIGHT_CANO_ID_CUSTOM_IMAGE, "CustomImage",
                                      light_ecu_can_data.custom_image.brightness,
                                      &custom_image_set_handler, 3000);

    // 0x00EC	ImageTransfer
    // send raw
    // Link 1 + X	{ type[0] data[1..7] }
    // Для передачи изображений
    /*
    co = cm.add_can_object(REAR_LIGHT_CANO_ID_IMAGE_TRANSFER, "ImageTransfer");
    DataFieldRawData *df_raw = (DataFieldRawData *)co->add_data_field(DF_RAW_DATA_ARRAY, light_ecu_can_data.image_transfer.chunk, CAN_SEND_RAW_FUNCTION_BUFFER_SIZE);
    co->add_function(CAN_FUNC_SEND_RAW_INIT_IN);
    co->add_function(CAN_FUNC_SEND_RAW_CHUNK_START_IN);
    co->add_function(CAN_FUNC_SEND_RAW_CHUNK_DATA_IN);
    co->add_function(CAN_FUNC_SEND_RAW_CHUNK_END_IN);
    co->add_function(CAN_FUNC_SEND_RAW_FINISH_IN);

    df_raw->set_external_handler_abort(&abort_callback);
    df_raw->set_external_handler_close_file(&close_file);
    df_raw->set_external_handler_free_space_checker(&free_space);
    df_raw->set_external_handler_open_file(&open_tmp_file);
    df_raw->set_external_handler_write_chunk(&write_chunk);
    */
    return true;
};