/*

   Copyright (C) Sienci Labs Inc.
  
   This file is part of the SuperLongBoard family of products.
  
   This source describes Open Hardware and is licensed under the "CERN-OHL-S v2"

   You may redistribute and modify this source and make products using
   it under the terms of the CERN-OHL-S v2 (https://ohwr.org/cern_ohl_s_v2.t). 
   This source is distributed WITHOUT ANY EXPRESS OR IMPLIED WARRANTY,
   INCLUDING OF MERCHANTABILITY, SATISFACTORY QUALITY AND FITNESS FOR A 
   PARTICULAR PURPOSE. Please see the CERN-OHL-S v2 for applicable conditions.
   
   As per CERN-OHL-S v2 section 4, should You produce hardware based on this 
   source, You must maintain the Source Location clearly visible on the external
   case of the CNC Controller or other product you make using this source.
  
   You should have received a copy of the CERN-OHL-S v2 license with this source.
   If not, see <https://ohwr.org/project/cernohl/wikis/Documents/CERN-OHL-version-2>.
   
   Contact for information regarding this program and its license
   can be sent through gSender@sienci.com or mailed to the main office
   of Sienci Labs Inc. in Waterloo, Ontario, Canada.

*/

#ifndef WS2812_H
#define WS2812_H

#include <stdint.h>

#define FRAME_SIZE 24
#define OFF 0
#define GLOBAL 1
#define PER_PIXEL 2

typedef struct {
    int size;
    int *transmitBuf;
    int use_II;
    uint8_t II;
    int outPin;
    int zeroHigh;
    int zeroLow;
    int oneHigh;
    int oneLow;
    int latch;
    uint8_t gpo;
} WS2812;

void WS2812_setDelays(WS2812* ws2812, int zeroHigh, int zeroLow, int oneHigh, int oneLow, int latch);
void WS2812_loadBuf(WS2812* ws2812, int* buf, int r_offset, int g_offset, int b_offset);
void WS2812_write(WS2812* ws2812, int* buf);
void WS2812_write_simple(WS2812* ws2812, int color);
void WS2812_write_offsets(WS2812* ws2812, int* buf, int r_offset, int g_offset, int b_offset);
void WS2812_useII(WS2812* ws2812, int bc);
void WS2812_setII(WS2812* ws2812, uint8_t II);

#endif /* WS2812_H */
