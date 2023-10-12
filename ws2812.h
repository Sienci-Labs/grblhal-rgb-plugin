#ifndef WS2812_H
#define WS2812_H

#include <stdint.h>

#define FRAME_SIZE 24
#define OFF 0
#define GLOBAL 1
#define PER_PIXEL 2

typedef struct {
    int size;
    int* transmitBuf;
    int use_II;
    uint8_t II;
    int outPin;
    int zeroHigh;
    int zeroLow;
    int oneHigh;
    int oneLow;
    int* gpo;
} WS2812;

void WS2812_setDelays(WS2812* ws2812, int zeroHigh, int zeroLow, int oneHigh, int oneLow);
void WS2812_loadBuf(WS2812* ws2812, int* buf, int r_offset, int g_offset, int b_offset);
void WS2812_write(WS2812* ws2812, int* buf);
void WS2812_write_offsets(WS2812* ws2812, int* buf, int r_offset, int g_offset, int b_offset);
void WS2812_useII(WS2812* ws2812, int bc);
void WS2812_setII(WS2812* ws2812, uint8_t II);

#endif /* WS2812_H */