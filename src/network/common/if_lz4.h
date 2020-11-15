#pragma once

#include <lz4.h>
#include <zstd.h>
#include <zstd_errors.h>

#include <stdint.h>

#define FRAME_SIZE (640*480*2)

#define ZSTD_MAGIC (0xFD2FB528)

class lz4 {
public:
    lz4();
   ~lz4();

    size_t compress(uint8_t* in, size_t size_in, uint8_t* out, size_t size_out);
    size_t decompress(uint8_t* in, size_t in_size, uint8_t* out, size_t out_size);

private:
    void print_error(size_t err);

    ZSTD_CCtx* cctx;
    ZSTD_DCtx* dctx;

    ZSTD_CDict* cdict;
    ZSTD_DDict* ddict;
};
