//
// Created by Benjamin Scholar on 5/11/20.
//

#ifndef MAGENCODERRMC_ENCODER_H
#define MAGENCODERRMC_ENCODER_H

#include <inttypes.h>
#include <stdbool.h>

#include "stm32f1xx_hal.h"

#include "utils.h"

#define ADJ_ANG // uncomment use adjusted angle
#define SPI_TIMEOUT 100

// volatile addresses
#define NOP      0x0000 // no operation
#define ERRFL    0x0001 // error register
#define PROG     0x0003 // prog register
#define DIAAGC   0x3FFC // diagnostics
#define MAG      0x3FFD // cordiac magnitude
#define ANGLEUNC 0x3FFE // unadjusted angle
#define ANGLECOM 0x3FFF // adjusted angle

// non-volatile addresses
#define ZPOSM     0x0016 // zero position msb
#define ZPOSL     0x0017 // zero position lsb/mag diagnostic
#define SETTINGS1 0x0018 // custom settings register 1
#define SETTINGS2 0x0019 // custom settings register 2
#define RED       0x001A // redundancy register (we probably dont need this at all)

typedef enum EncoderStatus {
    ENCODER_STATUS_OK = 0x0,
    ENCODER_STATUS_INCREMENT_ERROR = 0x01,
    ENCODER_STATUS_SPI_ERROR = 0x02
} EncoderStatus_t;

typedef struct EncoderSettings {
    uint16_t zero_offset;
    struct {
        bool error_low;
        bool error_high;
    } error_en;
    struct {
        bool iwidth;
        bool noiseset;
        bool dir;
        bool uvw_abi;
        bool daecdis;
        bool dataselect;
        bool pwmon;
    } settings1;
    uint16_t settings2; // probably dont need to config anything here.
} EncoderSettings_t;

typedef struct EncoderDiagnostics {
    bool parity_error;
    bool invalid_command_error;
    bool framing_error;
    bool mag_low;
    bool mag_high;
    bool cordiac_overflow;
    bool loops_finished;
    uint8_t agc_value;
} EncoderDiagnostics_t;

typedef struct Encoder {
    volatile int32_t count;
    volatile bool inverted;
    volatile int32_t absolute_position;
    volatile int32_t absolute_offset;
    volatile EncoderStatus_t status;
    volatile uint8_t last_gpio_value;
} Encoder_t;

extern HAL_StatusTypeDef spi_write_to_register(SPI_HandleTypeDef* hspi, uint16_t address, uint16_t* data);
extern HAL_StatusTypeDef spi_read_from_register(SPI_HandleTypeDef* hspi, uint16_t address, uint16_t* data);

extern HAL_StatusTypeDef encoder_config_settings1(SPI_HandleTypeDef* hspi, Encoder_t* encoder, EncoderSettings_t* settings);
extern HAL_StatusTypeDef encoder_config_settings2(SPI_HandleTypeDef* hspi, Encoder_t* encoder, EncoderSettings_t* settings);
extern HAL_StatusTypeDef encoder_config_zpos(SPI_HandleTypeDef* hspi, Encoder_t* encoder, EncoderSettings_t* settings);
extern HAL_StatusTypeDef encoder_config_all_settings(SPI_HandleTypeDef* hspi, Encoder_t* encoder, EncoderSettings_t* settings);

extern HAL_StatusTypeDef encoder_get_absolute_position(SPI_HandleTypeDef* hspi, Encoder_t* encoder);
extern HAL_StatusTypeDef encoder_get_errors(SPI_HandleTypeDef* hspi, Encoder_t* encoder, EncoderDiagnostics_t* diag);
extern HAL_StatusTypeDef encoder_get_diagnostics(SPI_HandleTypeDef* hspi, Encoder_t* encoder, EncoderDiagnostics_t* diag);

extern void encoder_update_count(Encoder_t* encoder, bool a_high, bool b_high);

extern void encoder_init(Encoder_t* encoder);
extern void encoder_reset(Encoder_t* encoder);
extern void encoder_set_ticks(Encoder_t* encoder, int32_t count);
extern void encoder_set_inverted(Encoder_t* encoder, bool inverted);

#endif //MAGENCODERRMC_ENCODER_H
