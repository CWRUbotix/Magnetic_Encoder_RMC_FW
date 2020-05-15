//
// Created by Benjamin Scholar on 5/14/20.
//
#include "memory.h"

#include "utils.h"
#include "string.h"

HAL_StatusTypeDef eeprom_clear_all(I2C_HandleTypeDef* hi2c) {
    uint8_t buf[8];
    memset(buf, 0, 8);
    uint8_t i = 0;
    HAL_StatusTypeDef status = {0};
    while(i < 128) {
        while(hi2c->State != HAL_I2C_STATE_READY);
        status |= HAL_I2C_Mem_Write(hi2c, EEPROM_ADDRESS, i, 1, buf, 8, 100);
        i++;
    }
    return status;
}

HAL_StatusTypeDef eeprom_read_page(I2C_HandleTypeDef* hi2c, uint8_t memory_address, uint64_t* data) {
    assert_param(IS_VALID_MEMORY_LOCATION(memory_address));
    uint8_t buf[8];
    memset(buf, 0, 8);
    while(hi2c->State != HAL_I2C_STATE_READY);
    auto status = HAL_I2C_Mem_Read(hi2c, EEPROM_ADDRESS, memory_address, 1, buf, 8, 100);
    memcpy(data, buf, 8);
    return status;
}

HAL_StatusTypeDef eeprom_write_page(I2C_HandleTypeDef* hi2c, uint8_t memory_address, uint64_t* data) {
    assert_param(IS_VALID_MEMORY_LOCATION(memory_address));
    uint8_t buf[8];
    memset(buf, 0, 8);
    memcpy(buf, data, 8);
    while(hi2c->State != HAL_I2C_STATE_READY);
    auto status = HAL_I2C_Mem_Write(hi2c, EEPROM_ADDRESS, memory_address, 1, buf, 8, 100);
    return status;
}
