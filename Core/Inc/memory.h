//
// Created by Benjamin Scholar on 5/11/20.
//

#ifndef MAGENCODERRMC_MEMORY_H
#define MAGENCODERRMC_MEMORY_H

#include "stm32f1xx_hal.h"

#define RESET_EEPROM_MEMORY // uncomment to wipe i2c memory on startup

#ifdef RESET_EEPROM_MEMORY
#warning "RESET_EEPROM_MEMORY option set"
#endif

// defined in the data sheet (last bit determines read or write)
#define EEPROM_ADDRESS 0b10101110
#define MIN_MEMORY_LOCATION 0   // bytes
#define MAX_MEMORY_LOCATION 127 // bytes

#define IS_VALID_MEMORY_LOCATION(x) (((x) >= MIN_MEMORY_LOCATION) && ((x)<= MAX_MEMORY_LOCATION))

// memory addresses
#define ENCODER_TICKS_LOCATION             0x00 // 4 bytes wide
#define ENCODER_POLARITY_LOCATION          0x01 // 1 bytes wide
#define ENCODER_ABSOLUTE_OFFSET_LOCATION   0x02 // 4 bytes wide
#define FEEDBACK_PERIOD_LOCATION           0x03 // 2 bytes wide


HAL_StatusTypeDef eeprom_clear_all(I2C_HandleTypeDef* hi2c);
HAL_StatusTypeDef eeprom_read_page(I2C_HandleTypeDef* hi2c, uint8_t memory_address, uint64_t* data);
HAL_StatusTypeDef eeprom_write_page(I2C_HandleTypeDef* hi2c, uint8_t memory_address, uint64_t* data);

#endif //MAGENCODERRMC_MEMORY_H
