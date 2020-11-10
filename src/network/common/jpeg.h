#pragma once

#include <jpeglib.h>

#include <stdint.h>

#define FRAME_SIZE (640*480*2)

class jpeg {
public:
    jpeg() {};
   ~jpeg() {};

    static int compress(uint8_t* in, int width, int height, uint8_t* out);
    static int decompress(unsigned char* in, int t_compressedSize, unsigned char* out);
};
