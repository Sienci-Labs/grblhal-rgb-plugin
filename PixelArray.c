#include <stdlib.h>

#define CHANNEL_COUNT 4

typedef struct {
    int pbufsize;
    int* pbuf;
} PixelArray;

PixelArray* PixelArray_create(int size) {
    PixelArray* pixelArray = (PixelArray*)malloc(sizeof(PixelArray));
    if (pixelArray != NULL) {
        pixelArray->pbufsize = size;
        pixelArray->pbuf = (int*)malloc(size * sizeof(int));
        if (pixelArray->pbuf == NULL) {
            free(pixelArray);
            return NULL; // Allocation failure
        }
        // Initialize memory to zeros
        for (int i = 0; i < size; i++) {
            pixelArray->pbuf[i] = 0;
        }
    }
    return pixelArray;
}

void PixelArray_destroy(PixelArray* pixelArray) {
    if (pixelArray != NULL) {
        free(pixelArray->pbuf);
        free(pixelArray);
    }
}

void PixelArray_setAll(PixelArray* pixelArray, unsigned int value) {
    // for each pixel
    for (int i = 0; i < pixelArray->pbufsize; i++) {
        PixelArray_setPixel(pixelArray, i, value);
    }
}

void PixelArray_setAllI(PixelArray* pixelArray, unsigned char value) {
    // for each pixel
    for (int i = 0; i < pixelArray->pbufsize; i++) {
        PixelArray_setPixelComponent(pixelArray, i, 3, value);
    }
}

void PixelArray_setAllR(PixelArray* pixelArray, unsigned char value) {
    // for each pixel
    for (int i = 0; i < pixelArray->pbufsize; i++) {
        PixelArray_setPixelComponent(pixelArray, i, 2, value);
    }
}

void PixelArray_setAllG(PixelArray* pixelArray, unsigned char value) {
    // for each pixel
    for (int i = 0; i < pixelArray->pbufsize; i++) {
        PixelArray_setPixelComponent(pixelArray, i, 1, value);
    }
}

void PixelArray_setAllB(PixelArray* pixelArray, unsigned char value) {
    // for each pixel
    for (int i = 0; i < pixelArray->pbufsize; i++) {
        PixelArray_setPixelComponent(pixelArray, i, 0, value);
    }
}

void PixelArray_setPixel(PixelArray* pixelArray, int i, unsigned int value) {
    if ((i >= 0) && (i < pixelArray->pbufsize)) {
        PixelArray_setPixelComponent(pixelArray, i, 0, (value & 0xFF));           // Blue
        PixelArray_setPixelComponent(pixelArray, i, 1, ((value >> 8) & 0xFF));    // Green
        PixelArray_setPixelComponent(pixelArray, i, 2, ((value >> 16) & 0xFF));   // Red
        PixelArray_setPixelComponent(pixelArray, i, 3, ((value >> 24) & 0xFF));   // Intensity
    }
}

void PixelArray_setPixelComponent(PixelArray* pixelArray, int index, int channel, unsigned char value) {
    if ((index >= 0) && (index < pixelArray->pbufsize) && (channel >= 0) && (channel < CHANNEL_COUNT)) {
        // AND with 0x00 shifted to the right location to clear the bits
        pixelArray->pbuf[index] &= ~(0xFF << (8 * channel));

        // Set the bits with an OR
        pixelArray->pbuf[index] |= (value << (8 * channel));
    }
}

int* PixelArray_getBuf(PixelArray* pixelArray) {
    return (pixelArray->pbuf);
}