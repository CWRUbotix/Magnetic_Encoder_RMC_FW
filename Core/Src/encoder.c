//
// Created by Benjamin Scholar on 5/11/20.
//
#include "encoder.h"

#include <string.h>
#include <encoder.h>

#include "utils.h"

void encoder_init(Encoder_t* encoder) {
    encoder->last_gpio_value = 0;
    encoder->count = 0;
    encoder->status = ENCODER_STATUS_OK;
}

void encoder_reset(Encoder_t* encoder) {
    encoder_init(encoder);
}

void encoder_set_ticks(Encoder_t* encoder, int32_t ticks) {
    encoder->count = ticks;
}

bool calculate_even_parity(uint16_t data) {
    uint8_t parity = 0;
    while(data) {
        parity ^= (data & 1u);
        data >>= 1u;
    }
    return (bool) parity;
}

uint16_t generate_control_frame(uint16_t address, bool read) {
    uint16_t frame = 0;
    frame |= (read << 14u); // write operation bit
    frame |= address; // write address
    frame |= (calculate_even_parity(frame) << 15u); // write parity
    return frame;
}


HAL_StatusTypeDef spi_write_to_register(SPI_HandleTypeDef* hspi, uint16_t address, uint16_t* data) {
    static uint8_t tx_buf[4], rx_buf[4];
    memset(rx_buf, 0, 2);
    uint16_t frame = generate_control_frame(address, false);
    memcpy(tx_buf, &frame, 2);
    memcpy(&tx_buf[2], &data, 2);
    while(hspi->State != HAL_SPI_STATE_READY);
    auto status = HAL_SPI_Transmit(hspi, tx_buf, 2, SPI_TIMEOUT);
    // recieve 2 frames just to clear rx buffer
    HAL_SPI_Receive(hspi, rx_buf, 2, SPI_TIMEOUT);
    return status;
}

HAL_StatusTypeDef spi_read_from_register(SPI_HandleTypeDef* hspi, uint16_t address, uint16_t* data) {
    static uint8_t tx_buf[2], rx_buf[2];
    memset(rx_buf, 0, 2);
    uint16_t frame = generate_control_frame(address, true);
    memcpy(tx_buf, &frame, 2);
    while(hspi->State != HAL_SPI_STATE_READY);

    HAL_SPI_Transmit(hspi, tx_buf, 1, SPI_TIMEOUT);
    auto status = HAL_SPI_Receive(hspi, rx_buf, 1, SPI_TIMEOUT);
    *data = rx_buf[0] << 8u;
    *data |= rx_buf[1];
    CLEAR_BIT(*data, 15u);
    CLEAR_BIT(*data, 14u);
    return status;
}

void encoder_update_count(Encoder_t* encoder, bool a_high, bool b_high) {
    const static uint8_t encoder_lut[] = {0,-1,1,2,1,0,2,-1,-1,2,0,1,2,1,-1,0};
    uint8_t current_gpio_value = 0;
    current_gpio_value |= (a_high << 1u);
    current_gpio_value |= (b_high << 0u);
    uint8_t increment = encoder_lut[encoder->last_gpio_value * 4 + current_gpio_value];
    if(increment == 2) {
        // error, either we arent reading the encoder fast enough or theres another issue
        encoder->status = ENCODER_STATUS_INCREMENT_ERROR;
    } else {
        encoder->count += (!encoder->inverted ? increment : -increment);
        encoder->status = ENCODER_STATUS_OK;
    }
    encoder->last_gpio_value = current_gpio_value;
}

HAL_StatusTypeDef encoder_config_settings1(SPI_HandleTypeDef* hspi, Encoder_t* encoder, EncoderSettings_t* settings) {
    uint16_t data1 = 0;
    auto s = settings->settings1;
    data1 |= (s.iwidth << 0u);
    data1 |= (s.noiseset << 1u);
    data1 |= (s.dir << 2u);
    data1 |= (s.uvw_abi << 3u);
    data1 |= (s.daecdis << 4u);
    data1 |= (s.dataselect << 6u);
    data1 |= (s.pwmon << 7u);
    return spi_write_to_register(hspi, SETTINGS1, &data1);
}

HAL_StatusTypeDef encoder_config_settings2(SPI_HandleTypeDef* hspi, Encoder_t* encoder, EncoderSettings_t* settings) {
    return spi_write_to_register(hspi, SETTINGS2, &settings->settings2);
}

HAL_StatusTypeDef encoder_config_zpos(SPI_HandleTypeDef* hspi, Encoder_t* encoder, EncoderSettings_t* settings) {
    uint16_t zposm_data = 0;
    uint16_t zposl_data = 0;
    // TODO(Ben): add offset here

    zposl_data |= (settings->error_en.error_low << 6u);
    zposl_data |= (settings->error_en.error_high << 7u);

    auto status = spi_write_to_register(hspi, ZPOSL, &zposl_data);
    status = spi_write_to_register(hspi, ZPOSM, &zposm_data);
    return status;
}

HAL_StatusTypeDef encoder_config_all_settings(SPI_HandleTypeDef* hspi, Encoder_t* encoder, EncoderSettings_t* settings) {
    auto status1 = encoder_config_settings1(hspi, encoder, settings);
    auto status2 = encoder_config_settings2(hspi, encoder, settings);
    auto status3 = encoder_config_zpos(hspi, encoder, settings);
    return status2;
}

HAL_StatusTypeDef encoder_get_absolute_position(SPI_HandleTypeDef* hspi, Encoder_t* encoder) {
    uint16_t address;
    #ifdef ADJ_ANG
        address = ANGLECOM;
    #else
        address = ANGLEUNC;
    #endif
    uint16_t data;
    auto status = spi_read_from_register(hspi, address, &data);
    encoder->absolute_position = data;
    return status;
}

HAL_StatusTypeDef encoder_get_diagnostics(SPI_HandleTypeDef* hspi, Encoder_t* encoder, EncoderDiagnostics_t* diag) {
    uint16_t data;
    auto status = spi_read_from_register(hspi, DIAAGC, &data);
    diag->mag_low = (data >> 11u) & 1u;
    diag->mag_low = (data >> 10u) & 1u;
    diag->coriac_overflow = (data >> 9u) & 1u;
    diag->loops_finished = (data >> 8u) & 1u;
    diag->agc_value = (data) & 0b1111u;
    return status;
}

HAL_StatusTypeDef encoder_get_errors(SPI_HandleTypeDef* hspi, Encoder_t* encoder, EncoderDiagnostics_t* diag) {
    uint16_t data;
    auto status = spi_read_from_register(hspi, ERRFL, &data);
    diag->parity_error = (data >> 2u) & 1u;
    diag->invalid_command_error = (data >> 1u) & 1u;
    diag->framing_error = (data >> 0u) & 1u;
    return status;
}