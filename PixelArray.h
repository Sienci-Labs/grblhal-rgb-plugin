#ifndef PIXELARRAY_H
#define PIXELARRAY_H

typedef struct {
    int pbufsize;
    int* pbuf;
} PixelArray;

PixelArray* PixelArray_create(int size);
void PixelArray_destroy(PixelArray* pixelArray);
void PixelArray_setAll(PixelArray* pixelArray, unsigned int value);
void PixelArray_setAllI(PixelArray* pixelArray, unsigned char value);
void PixelArray_setAllR(PixelArray* pixelArray, unsigned char value);
void PixelArray_setAllG(PixelArray* pixelArray, unsigned char value);
void PixelArray_setAllB(PixelArray* pixelArray, unsigned char value);
void PixelArray_setPixel(PixelArray* pixelArray, int i, unsigned int value);
void PixelArray_setPixelComponent(PixelArray* pixelArray, int index, int channel, unsigned char value);
int* PixelArray_getBuf(PixelArray* pixelArray);

#endif /* PIXELARRAY_H */