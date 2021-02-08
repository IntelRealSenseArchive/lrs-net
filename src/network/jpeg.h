#pragma once

#include <jpeglib.h>

#include <stdint.h>

class jpeg {
public:
    jpeg() {};
   ~jpeg() {};

    static int compress(uint8_t* in, int width, int height, uint8_t* out, uint32_t size);
    static int decompress(uint8_t* in, int in_size, uint8_t* out, uint32_t out_size);
};
