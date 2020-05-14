//
// Created by Benjamin Scholar on 5/11/20.
//

#ifndef MAGENCODERRMC_UTILS_H
#define MAGENCODERRMC_UTILS_H

#include <stddef.h>
#include <stdint.h>


// helpful macros
#define STR(x) #x
#define XSTR(x) STR(x)

///* a=target variable, b=bit number to act upon 0-n */
//#define BIT_SET(a,b) ((a) |= (1ULL<<(b)))
//#define BIT_CLEAR(a,b) ((a) &= ~(1ULL<<(b)))
//#define BIT_FLIP(a,b) ((a) ^= (1ULL<<(b)))
//#define BIT_CHECK(a,b) (!!((a) & (1ULL<<(b))))        // '!!' to make sure this returns 0 or 1
//
/* x=target variable, y=mask */
#define BITMASK_SET(x,y) ((x) |= (y))
#define BITMASK_CLEAR(x,y) ((x) &= (~(y)))
#define BITMASK_FLIP(x,y) ((x) ^= (y))
#define BITMASK_CHECK_ALL(x,y) (((x) & (y)) == (y))   // warning: evaluates y twice
#define BITMASK_CHECK_ANY(x,y) ((x) & (y))


// keyword defines
#define auto     __auto_type
#define volatile __volatile

extern uint16_t swap_order(uint16_t num);

extern volatile void* memcpy_v(volatile void *restrict, const volatile void *restrict, size_t);

#endif //MAGENCODERRMC_UTILS_H
