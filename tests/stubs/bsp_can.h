#pragma once
#include <stdint.h>
typedef uint32_t basic_data_t;
typedef void (*rx_callback)(uint8_t, uint32_t, uint8_t *, basic_data_t);
void BSP_CAN_Send(uint8_t bus, uint32_t id, uint8_t *data, basic_data_t len);
void BSP_CAN_RegisterRxCallback(uint8_t bus, rx_callback callback);
void BSP_CAN_AddFilter(uint8_t bus, uint32_t id);
