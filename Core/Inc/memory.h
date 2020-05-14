//
// Created by Benjamin Scholar on 5/11/20.
//

#ifndef MAGENCODERRMC_MEMORY_H
#define MAGENCODERRMC_MEMORY_H

// defined in the data sheet (last bit determines read or write)
#define EEPROM_ADDRESS 0b10100000
#define MIN_MEMORY_LOCATION 0   // bytes
#define MAX_MEMORY_LOCATION 127 // bytes

// memory addresses
#define ENCODER_TICKS_LOCATION    0x00 // 4 bytes wide
#define ENCODER_POLARITY_LOCATION 0x04 // 1 bytes wide
#define ENCODER_ABSOLUTE_OFFSET_LOCATION   0x05 // 4 bytes wide
#define FEEDBACK_PERIOD_LOCATION  0x09 // 2 bytes wide

#endif //MAGENCODERRMC_MEMORY_H
