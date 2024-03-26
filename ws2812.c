/*
#include "WS2812.h"
#include "PixelArray.h"

#define WS2812_BUF 150
#define NUM_COLORS 6
#define NUM_LEDS_PER_COLOR 10

PixelArray px(WS2812_BUF);

// See the program page for information on the timing numbers
// The given numbers are for the K64F
WS2812 ws(D9, WS2812_BUF, 0, 5, 5, 0);

int main()
{

    ws.useII(WS2812::PER_PIXEL); // use per-pixel intensity scaling
    
    // set up the colours we want to draw with
    int colorbuf[NUM_COLORS] = {0x2f0000,0x2f2f00,0x002f00,0x002f2f,0x00002f,0x2f002f};

    // for each of the colours (j) write out 10 of them
    // the pixels are written at the colour*10, plus the colour position
    // all modulus 60 so it wraps around
    for (int i = 0; i < WS2812_BUF; i++) {
        px.Set(i, colorbuf[(i / NUM_LEDS_PER_COLOR) % NUM_COLORS]);
    }

    // now all the colours are computed, add a fade effect using intensity scaling
    // compute and write the II value for each pixel
    for (int j=0; j<WS2812_BUF; j++) {
        // px.SetI(pixel position, II value)
        px.SetI(j%WS2812_BUF, 0xf+(0xf*(j%NUM_LEDS_PER_COLOR)));
    }


    // Now the buffer is written, rotate it
    // by writing it out with an increasing offset
    while (1) {
        for (int z=WS2812_BUF; z >= 0 ; z--) {
            ws.write_offsets(px.getBuf(),z,z,z);
            wait(0.075);
        }
    }

}

*/

#include <stdint.h>
#include "driver.h"
#include "grbl/hal.h"

#include "ws2812.h"

#define FRAME_SIZE 24
#define OFF 0
#define GLOBAL 1
#define PER_PIXEL 2

void WS2812_setDelays(WS2812* ws2812, int zeroHigh, int zeroLow, int oneHigh, int oneLow, int latch) {
    ws2812->zeroHigh = zeroHigh;
    ws2812->zeroLow = zeroLow;
    ws2812->oneHigh = oneHigh;
    ws2812->oneLow = oneLow;
    ws2812->latch = latch;
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
    int i, j, k;

    WS2812_loadBuf(ws2812, buf, r_offset, g_offset, b_offset);

    // Entering timing critical section, so disabling interrupts
    // Assuming __disable_irq() and __enable_irq() are custom functions
    // that disable and enable interrupts respectively
    //__disable_irq();

    for (i = 0; i < ws2812->size; i++) {
        for (k = 0; k<FRAME_SIZE; k++){
            if ((ws2812->transmitBuf[i]>>k)&0x01) {
                //need to set the output high
                //*(ws2812->gpo) = 1;
                hal.port.digital_out(ws2812->gpo, true);
                for (j = 0; j < ws2812->oneHigh; j++) {
                    __ASM volatile ("nop");
                }
                //need to set the output low
                //*(ws2812->gpo) = 0;
                hal.port.digital_out(ws2812->gpo, false);
                for (j = 0; j < ws2812->oneLow; j++) {
                    __ASM volatile ("nop");
                }
            } else {
                //need to set the output high
                //*(ws2812->gpo) = 1;
                hal.port.digital_out(ws2812->gpo, true);
                for (j = 0; j < ws2812->zeroHigh; j++) {
                    __ASM volatile ("nop");
                }
                //need to set the output low
                //*(ws2812->gpo) = 0;
                hal.port.digital_out(ws2812->gpo, false);
                for (j = 0; j < ws2812->zeroLow; j++) {
                    __ASM volatile ("nop");
                }
            }
        }
    }
    // Exiting timing critical section, so enabling interrupts
    //__enable_irq();
    for (j = 0; j < ws2812->latch; j++) {
        __ASM volatile ("nop");
    }    

}

void WS2812_write_simple(WS2812* ws2812, int color) {
    int i, j, k;

    // Entering timing critical section, so disabling interrupts
    // Assuming __disable_irq() and __enable_irq() are custom functions
    // that disable and enable interrupts respectively
    __disable_irq();

    for (i = 0; i < ws2812->size; i++) {

        for (k = FRAME_SIZE-1; k>=0; k--){
            if ((color>>k)&0x01) {
                //need to set the output high
                //*(ws2812->gpo) = 1;
                hal.port.digital_out(ws2812->gpo, true);
                for (j = 0; j < ws2812->oneHigh; j++) {
                    __ASM volatile ("nop");
                }
                //need to set the output low
                //*(ws2812->gpo) = 0;
                hal.port.digital_out(ws2812->gpo, false);
                for (j = 0; j < ws2812->oneLow; j++) {
                    __ASM volatile ("nop");
                }
            } else {
                //need to set the output high
                //*(ws2812->gpo) = 1;
                hal.port.digital_out(ws2812->gpo, true);
                //for (j = 0; j < ws2812->zeroHigh; j++) {
                //    __ASM volatile ("nop");
                //}
                //need to set the output low
                //*(ws2812->gpo) = 0;
                hal.port.digital_out(ws2812->gpo, false);
                for (j = 0; j < ws2812->zeroLow; j++) {
                    __ASM volatile ("nop");
                }
            }
        }
    }

    // Exiting timing critical section, so enabling interrupts
    __enable_irq();

    for (j = 0; j < ws2812->latch; j++) {
        __ASM volatile ("nop");
    }
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