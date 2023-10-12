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

void WS2812_setDelays(WS2812* ws2812, int zeroHigh, int zeroLow, int oneHigh, int oneLow) {
    ws2812->zeroHigh = zeroHigh;
    ws2812->zeroLow = zeroLow;
    ws2812->oneHigh = oneHigh;
    ws2812->oneLow = oneLow;
}

void WS2812_loadBuf(WS2812* ws2812, int* buf, int r_offset, int g_offset, int b_offset) {
    for (int i = 0; i < ws2812->size; i++) {
        int color = 0;

        color |= ((buf[(i + g_offset) % ws2812->size] & 0x0000FF00));
        color |= ((buf[(i + r_offset) % ws2812->size] & 0x00FF0000));
        color |= (buf[(i + b_offset) % ws2812->size] & 0x000000FF);
        color |= (buf[i] & 0xFF000000);

        unsigned char agrb[4] = {0x0, 0x0, 0x0, 0x0};

        unsigned char sf;
        agrb[0] = (color & 0x0000FF00) >> 8;
        agrb[1] = (color & 0x00FF0000) >> 16;
        agrb[2] = color & 0x000000FF;
        agrb[3] = (color & 0xFF000000) >> 24;

        if (ws2812->use_II == GLOBAL) {
            sf = ws2812->II;
        } else if (ws2812->use_II == PER_PIXEL) {
            sf = agrb[3];
        } else {
            sf = 0xFF;
        }

        for (int clr = 0; clr < 3; clr++) {
            agrb[clr] = ((agrb[clr] * sf) >> 8);

            for (int j = 0; j < 8; j++) {
                if (((agrb[clr] << j) & 0x80) == 0x80) {
                    ws2812->transmitBuf[(i * FRAME_SIZE) + (clr * 8) + j] = 1;
                } else {
                    ws2812->transmitBuf[(i * FRAME_SIZE) + (clr * 8) + j] = 0;
                }
            }
        }
    }
}

void WS2812_write(WS2812* ws2812, int* buf) {
    WS2812_write_offsets(ws2812, buf, 0, 0, 0);
}

void WS2812_write_offsets(WS2812* ws2812, int* buf, int r_offset, int g_offset, int b_offset) {
    int i, j;
    WS2812_loadBuf(ws2812, buf, r_offset, g_offset, b_offset);

    // Entering timing critical section, so disabling interrupts
    // Assuming __disable_irq() and __enable_irq() are custom functions
    // that disable and enable interrupts respectively
    __disable_irq();

    for (i = 0; i < FRAME_SIZE * ws2812->size; i++) {
        j = 0;
        if (ws2812->transmitBuf[i]) {
            *(ws2812->gpo) = 1;
            for (; j < ws2812->oneHigh; j++) {
                __nop();
            }
            *(ws2812->gpo) = 0;
            for (; j < ws2812->oneLow; j++) {
                __nop();
            }
        } else {
            *(ws2812->gpo) = 1;
            for (; j < ws2812->zeroHigh; j++) {
                __nop();
            }
            *(ws2812->gpo) = 0;
            for (; j < ws2812->zeroLow; j++) {
                __nop();
            }
        }
    }

    // Exiting timing critical section, so enabling interrupts
    __enable_irq();
}

void WS2812_useII(WS2812* ws2812, int bc) {
    if (bc > OFF) {
        ws2812->use_II = bc;
    } else {
        ws2812->use_II = OFF;
    }
}

void WS2812_setII(WS2812* ws2812, uint8_t II) {
    ws2812->II = II;
}