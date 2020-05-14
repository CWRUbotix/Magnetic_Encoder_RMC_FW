//
// Created by Benjamin Scholar on 5/11/20.
//

#include "utils.h"

uint16_t swap_order(uint16_t num) {
    return (num >> 8u) | (num << 8u);
}

__volatile void* memcpy_v(__volatile void *restrict dest, const __volatile void *restrict src, size_t n) {
    const __volatile unsigned char *src_c = src;
    __volatile unsigned char *dest_c      = dest;

    while(n > 0) {
        n--;
        dest_c[n] = src_c[n];
    }
    return dest;
}

